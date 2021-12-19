/* SPDX-License-Identifier: GPL-2.0
 *
 * Based on xlnx-pcie-xvc.c
 * Author: Pirea Radu <pirea.radu@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/socket.h>

#include <helper/bits.h>
#include <helper/replacements.h>
#include <jtag/interface.h>
#include <jtag/commands.h>

#define MAP_SIZE		0x10000
#define PROGRESS_BEGIN		(1024 * 512 * 8)
#define XLNX_XVC_REG_SIZE	0x20

struct jtag_xvc_uio {
  uint32_t  length;
  uint32_t  tms;
  uint32_t  tdi;
  uint32_t  tdo;
  uint32_t  enable;
};

enum xvc_interface {
	XVC_IF_NONE,
	XVC_IF_TCP,
	XVC_IF_UIO
};

struct xlnx_tcp_xvc {
	int fd;
	int port;
	int uio_dev;
	char *ip_addr;
	uint8_t *command;
	size_t vector_len;
	size_t command_len;
	size_t command_size;
	size_t custom_vector_len;
	struct sockaddr_in serv_addr;
	enum xvc_interface interface;
	volatile struct jtag_xvc_uio *jtag_mmap;
	int (*transact)(size_t num_bits, uint32_t *tms,
			   uint32_t *tdi, uint32_t *tdo);
};

static struct xlnx_tcp_xvc xlnx_tcp_xvc_state;
static struct xlnx_tcp_xvc *xlnx_tcp_xvc = &xlnx_tcp_xvc_state;

static size_t xlnx_tcp_xvc_num_u8(size_t num_bits)
{
	return (num_bits + 7) / 8;
}

static void xlnx_tcp_xvc_cmd_append(size_t num_bytes, const void *bytes)
{
	size_t begin = xlnx_tcp_xvc->command_len;
	memcpy(&xlnx_tcp_xvc->command[begin], bytes, num_bytes);
	begin += num_bytes;
	xlnx_tcp_xvc->command_len = begin;
}

static int xlnx_tcp_xvc_transact(size_t num_bits, uint32_t *tms, uint32_t *tdi,
				 uint32_t *tdo)
{
	size_t num_bytes = xlnx_tcp_xvc_num_u8(num_bits);
	uint8_t tdo_r[xlnx_tcp_xvc_num_u8(xlnx_tcp_xvc->vector_len)];
	size_t bytes_r = 0;
	int ret;

	xlnx_tcp_xvc->command_len = 0;
	memset(xlnx_tcp_xvc->command, 0, xlnx_tcp_xvc->command_size);

	xlnx_tcp_xvc_cmd_append(6, "shift:");
	xlnx_tcp_xvc_cmd_append(4, &num_bits);
	xlnx_tcp_xvc_cmd_append(num_bytes, tms);
	xlnx_tcp_xvc_cmd_append(num_bytes, tdi);

	send(xlnx_tcp_xvc->fd, xlnx_tcp_xvc->command, xlnx_tcp_xvc->command_len, 0);
	while (bytes_r != num_bytes) {
		ret = read(xlnx_tcp_xvc->fd, &tdo_r[bytes_r], num_bytes - bytes_r);
		if (ret < 0) {
			LOG_ERROR("%s", strerror(errno));
			return ERROR_FAIL;
		}
		bytes_r += ret;
	}

	memcpy(tdo, tdo_r, num_bytes);

	return ERROR_OK;
}

static int xlnx_uio_xvc_transact(size_t num_bits, uint32_t *tms, uint32_t *tdi,
				 uint32_t *tdo)
{
	volatile struct jtag_xvc_uio *jtag_mmap = xlnx_tcp_xvc->jtag_mmap;

	if (!num_bits) {
		LOG_WARNING("Can not shift 0 bits. Skip.");
		return ERROR_FAIL;
	}

	if (num_bits > XLNX_XVC_REG_SIZE) {
		LOG_ERROR("Too many bits to shift %zu, %s can shift only %d",
			  num_bits, __func__, XLNX_XVC_REG_SIZE);
		return ERROR_FAIL;
	}

	jtag_mmap->length = num_bits;
	jtag_mmap->tms = *tms;
	jtag_mmap->tdi = *tdi;
	jtag_mmap->enable = 1;

	while (jtag_mmap->enable) { };

	*tdo = jtag_mmap->tdo;

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_stableclocks(struct jtag_command *cmd)
{
	uint32_t tms = tap_get_state() == TAP_RESET ? 1 : 0;
	size_t left = cmd->cmd.stableclocks->num_cycles;
	uint32_t tdi = 0;
	uint32_t tdo;
	size_t write;
	int err;

	LOG_DEBUG("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);

	while (left) {
		write = MIN(XLNX_XVC_REG_SIZE, left);
		err = xlnx_tcp_xvc->transact(write, &tms, &tdi, &tdo);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_statemove(size_t skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
					    tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
					     tap_get_end_state());
	uint32_t tms = tms_scan >> skip;
	uint32_t tdi = 0;
	uint32_t tdo;
	int err;

	LOG_DEBUG("statemove starting at (skip: %zu) %s end in %s", skip,
		  tap_state_name(tap_get_state()),
		  tap_state_name(tap_get_end_state()));

	err = xlnx_tcp_xvc->transact(tms_count - skip, &tms, &tdi, &tdo);
	if (err != ERROR_OK)
		return err;

	tap_set_state(tap_get_end_state());

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_runtest(struct jtag_command *cmd)
{
	int err = ERROR_OK;
	uint32_t tms = 0;
	uint32_t tdi = 0;
	uint32_t tdo;

	LOG_DEBUG("runtest %i cycles, end in %i",
		  cmd->cmd.runtest->num_cycles,
		  cmd->cmd.runtest->end_state);

	tap_state_t tmp_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		err = xlnx_tcp_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			return err;
	};

	size_t left = cmd->cmd.runtest->num_cycles;
	size_t write;

	while (left) {
		write = MIN(XLNX_XVC_REG_SIZE, left);
		err = xlnx_tcp_xvc->transact(write, &tms, &tdi, &tdo);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	tap_set_end_state(tmp_state);
	if (tap_get_state() != tap_get_end_state())
		err = xlnx_tcp_xvc_execute_statemove(0);

	return err;
}

static int xlnx_tcp_xvc_execute_pathmove(struct jtag_command *cmd)
{
	size_t num_states = cmd->cmd.pathmove->num_states;
	tap_state_t *path = cmd->cmd.pathmove->path;
	int err = ERROR_OK;
	uint32_t tms = 0;
	uint32_t tdi = 0;
	uint32_t tdo;
	size_t i;

	LOG_DEBUG("pathmove: %i states, end in %i",
		  cmd->cmd.pathmove->num_states,
		  cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false)) {
			tms = 1;
			err = xlnx_tcp_xvc->transact(1, &tms, &tdi, &tdo);
		} else if (path[i] == tap_state_transition(tap_get_state(), true)) {
			err = xlnx_tcp_xvc->transact(1, &tms, &tdi, &tdo);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()),
				  tap_state_name(path[i]));
			err = ERROR_JTAG_QUEUE_FAILED;
		}
		if (err != ERROR_OK)
			return err;
		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_scan(struct jtag_command *cmd)
{
	enum scan_type type = jtag_scan_type(cmd->cmd.scan);
	tap_state_t saved_end_state = cmd->cmd.scan->end_state;
	bool ir_scan = cmd->cmd.scan->ir_scan;
	uint8_t tms_buff[xlnx_tcp_xvc_num_u8(xlnx_tcp_xvc->vector_len)];
	uint8_t *buf, *rd_ptr;
	int err, scan_size;
	size_t last_byte;
	size_t last_bit;
	float percent;
	size_t write;
	float i = 0;
	size_t left;

	memset(tms_buff, 0, sizeof(tms_buff));
	scan_size = jtag_build_buffer(cmd->cmd.scan, &buf);
	rd_ptr = buf;
	LOG_DEBUG("%s scan type %d %d bits; starts in %s end in %s",
		 (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		 tap_state_name(tap_get_state()),
		 tap_state_name(cmd->cmd.scan->end_state));

	/* If we're in TAP_DR_SHIFT state but need to do a IR_SCAN or
	 * vice-versa, do a statemove to corresponding other state, then restore
	 * end state
	 */
	if (ir_scan && tap_get_state() != TAP_IRSHIFT) {
		tap_set_end_state(TAP_IRSHIFT);
		err = xlnx_tcp_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	} else if (!ir_scan && (tap_get_state() != TAP_DRSHIFT)) {
		tap_set_end_state(TAP_DRSHIFT);
		err = xlnx_tcp_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	}

	left = scan_size;
	percent = scan_size / 100.0;
	while (left) {
		write = MIN(xlnx_tcp_xvc->vector_len, left);
		if (left < xlnx_tcp_xvc->vector_len) {
			last_byte = left / 8;
			last_bit = left % 8;
			if (last_bit == 0) {
				last_byte -= 1;
				last_bit = 7;
			} else {
				last_bit = last_bit - 1;
			}
			tms_buff[last_byte] = BIT(last_bit);
			LOG_DEBUG("write %zu last_byte %zu last_bit %zu", write, last_byte, last_bit);
		}
		err = xlnx_tcp_xvc->transact(write, (uint32_t *) tms_buff, (uint32_t *) rd_ptr, (uint32_t *) rd_ptr);
		if (err != ERROR_OK)
			goto out_err;
		left -= write;
		rd_ptr += xlnx_tcp_xvc_num_u8(write);

		if (((uint32_t)(percent * i)) < (scan_size - left) &&
		    scan_size >= PROGRESS_BEGIN) {
			while ((percent * i) < (scan_size - left))
			i += 1;
			LOG_INFO("Progress %.2f%%", i);
		}
	};

	err = jtag_read_buffer(buf, cmd->cmd.scan);

	free(buf);
	if (tap_get_state() != tap_get_end_state())
		err = xlnx_tcp_xvc_execute_statemove(1);

	return err;

out_err:
	free(buf);
	return err;
}

static void xlnx_tcp_xvc_execute_reset(struct jtag_command *cmd)
{
	LOG_DEBUG("reset trst: %i srst: %i", cmd->cmd.reset->trst,
		  cmd->cmd.reset->srst);
}

static void xlnx_tcp_xvc_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG("sleep %"
	PRIu32, cmd->cmd.sleep->us);
	usleep(cmd->cmd.sleep->us);
}

static int xlnx_tcp_xvc_execute_tms(struct jtag_command *cmd)
{
	const size_t num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	size_t left, write;
	uint32_t tdi = 0;
	uint32_t tms;
	uint32_t tdo;
	int err;

	LOG_DEBUG("execute tms %zu", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(XLNX_XVC_REG_SIZE, left);
		tms = buf_get_u32(bits, 0, write);
		err = xlnx_tcp_xvc->transact(write, &tms, &tdi, &tdo);
		if (err != ERROR_OK)
			return err;
		left -= write;
		bits += 4;
	};

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG("%s: cmd->type: %u", __func__, cmd->type);
	switch (cmd->type) {
		case JTAG_STABLECLOCKS:
			return xlnx_tcp_xvc_execute_stableclocks(cmd);
		case JTAG_RUNTEST:
			return xlnx_tcp_xvc_execute_runtest(cmd);
		case JTAG_TLR_RESET:
			tap_set_end_state(cmd->cmd.statemove->end_state);
			return xlnx_tcp_xvc_execute_statemove(0);
		case JTAG_PATHMOVE:
			return xlnx_tcp_xvc_execute_pathmove(cmd);
		case JTAG_SCAN:
			return xlnx_tcp_xvc_execute_scan(cmd);
		case JTAG_RESET:
			xlnx_tcp_xvc_execute_reset(cmd);
			break;
		case JTAG_SLEEP:
			xlnx_tcp_xvc_execute_sleep(cmd);
			break;
		case JTAG_TMS:
			return xlnx_tcp_xvc_execute_tms(cmd);
		default:
			LOG_ERROR("BUG: Unknown JTAG command type encountered.");
			return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int xlnx_tcp_xvc_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		ret = xlnx_tcp_xvc_execute_command(cmd);

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int xlnx_tcp_xvc_init(void)
{
	char *message = "getinfo:";
	char buffer[1024] = {0};
	char *vector_len_begin;
	int nbytes;
	int fd_uio;

	if (xlnx_tcp_xvc->interface != XVC_IF_TCP &&
	    xlnx_tcp_xvc->interface != XVC_IF_UIO) {
	    LOG_ERROR("xlnx_xvc_interface_config must be TCP or UIO");
	    return ERROR_JTAG_INIT_FAILED;
	}

	if (xlnx_tcp_xvc->interface == XVC_IF_UIO)
		goto xvc_init_uio;

	xlnx_tcp_xvc->fd = socket(AF_INET, SOCK_STREAM, 0);
	if ((xlnx_tcp_xvc->fd) < 0) {
		LOG_ERROR("Socket creation failed.");
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_tcp_xvc->serv_addr.sin_family = AF_INET;
	xlnx_tcp_xvc->serv_addr.sin_port = htons(xlnx_tcp_xvc->port);

	if (inet_pton(AF_INET, xlnx_tcp_xvc->ip_addr, &xlnx_tcp_xvc->serv_addr.sin_addr) <= 0) {
		LOG_ERROR("Invalid IP address.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (connect(xlnx_tcp_xvc->fd, (struct sockaddr *) &xlnx_tcp_xvc->serv_addr, sizeof(xlnx_tcp_xvc->serv_addr)) <
	    0) {
		LOG_ERROR("Cannot connect to server.");
		return ERROR_JTAG_INIT_FAILED;
	}

	send(xlnx_tcp_xvc->fd, message, strlen(message), 0);
	nbytes = read(xlnx_tcp_xvc->fd, buffer, 1024);
	buffer[nbytes - 1] = 0;
	LOG_INFO("Successfully connected to XVC server.");
	LOG_INFO("Server info: %s", buffer);
	vector_len_begin = strchr(buffer, ':') + 1;
	sscanf(vector_len_begin, "%zu", &xlnx_tcp_xvc->vector_len);
	LOG_INFO("Server vector length: %zu", xlnx_tcp_xvc->vector_len);
	if (xlnx_tcp_xvc->custom_vector_len) {
		xlnx_tcp_xvc->vector_len  = xlnx_tcp_xvc->custom_vector_len;
		LOG_INFO("Override vector length: %zu", xlnx_tcp_xvc->vector_len);
	}
	xlnx_tcp_xvc->command_size = xlnx_tcp_xvc->vector_len / 8 * 2 + 10;
	xlnx_tcp_xvc->command = malloc(xlnx_tcp_xvc->command_size);

	xlnx_tcp_xvc->transact = xlnx_tcp_xvc_transact;

	return ERROR_OK;

xvc_init_uio:
	sprintf(buffer,"/dev/uio%d", xlnx_tcp_xvc->uio_dev);
	fd_uio = open(buffer, O_RDWR );
		if (fd_uio < 1) {
		LOG_ERROR("Failed to Open UIO device");
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_tcp_xvc->jtag_mmap = (volatile struct jtag_xvc_uio*)
		mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
		     MAP_SHARED, fd_uio, 0);

	if (xlnx_tcp_xvc->jtag_mmap == MAP_FAILED) {
		LOG_ERROR("Failed to MMAP UIO device");
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_tcp_xvc->vector_len = XLNX_XVC_REG_SIZE;
	xlnx_tcp_xvc->transact = xlnx_uio_xvc_transact;

	close(fd_uio);

	return ERROR_OK;
}

static int xlnx_tcp_xvc_quit(void)
{
	int err;

	free(xlnx_tcp_xvc->ip_addr);
	free(xlnx_tcp_xvc->command);
	err = close(xlnx_tcp_xvc->fd);
	if (err)
		return err;

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_xvc_ip_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(xlnx_tcp_xvc->ip_addr);

	xlnx_tcp_xvc->ip_addr = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_xvc_port_command)
{
	int ret;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ret = sscanf(CMD_ARGV[0], "%i", &xlnx_tcp_xvc->port);
	if (ret != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_xvc_vector_length_command)
{
	int custom_vector_len;
	int ret;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ret = sscanf(CMD_ARGV[0], "%i", &custom_vector_len);
	if (ret != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (custom_vector_len <= 0) {
		LOG_ERROR("xlnx_xvc_vector_length should be greather than 0.");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	xlnx_tcp_xvc->custom_vector_len = custom_vector_len;

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_xvc_uio_dev_command)
{
	int ret;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ret = sscanf(CMD_ARGV[0], "%i", &xlnx_tcp_xvc->uio_dev);
	if (ret != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (xlnx_tcp_xvc->uio_dev < 0) {
		LOG_ERROR("xlnx_xvc_uio_dev_config should be greater than 0");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_xvc_interface_command)
{
	char interface[4];
	size_t i;
	int ret;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strlen(CMD_ARGV[0]) != 3)
		goto xlnx_xvc_interface_config_err;

	ret = sscanf(CMD_ARGV[0], "%s", interface);
	if (ret != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for(i=0; i < strlen(interface); i++)
		interface[i] = toupper(interface[i]);

	xlnx_tcp_xvc->interface = XVC_IF_NONE;

	if (!strcmp(interface, "UIO"))
		xlnx_tcp_xvc->interface = XVC_IF_UIO;

	if (!strcmp(interface, "TCP"))
		xlnx_tcp_xvc->interface = XVC_IF_TCP;

	if (xlnx_tcp_xvc->interface == XVC_IF_NONE)
		goto xlnx_xvc_interface_config_err;

	return ERROR_OK;

xlnx_xvc_interface_config_err:
	LOG_ERROR("xlnx_xvc_interface_config must be TCP or UIO");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration xlnx_tcp_xvc_command_handlers[] = {
	{
		.name = "xlnx_xvc_ip",
		.handler = xlnx_xvc_ip_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC server IP address",
		.usage = "device",
	},
	{
		.name = "xlnx_xvc_port",
		.handler = xlnx_xvc_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC server TCP port",
		.usage = "device",
	},
	{
		.name = "xlnx_xvc_vector_length",
		.handler = xlnx_xvc_vector_length_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC server vector length(in bits) and override the reported one.",
		.usage = "device",
	},
	{
		.name = "xlnx_xvc_uio_dev",
		.handler = xlnx_xvc_uio_dev_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC uio dev id",
		.usage = "device",
	},
	{
		.name = "xlnx_xvc_interface",
		.handler = xlnx_xvc_interface_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC interface(UIO or TCP)",
		.usage = "device",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface xlnx_tcp_xvc_jtag_ops = {
	.execute_queue = &xlnx_tcp_xvc_execute_queue,
};

static const char *const xlnx_tcp_xvc_transports[] = {"jtag", NULL};

struct adapter_driver xlnx_tcp_xvc_adapter_driver = {
	.name = "xlnx_tcp_xvc",
	.transports = xlnx_tcp_xvc_transports,
	.commands = xlnx_tcp_xvc_command_handlers,

	.init = &xlnx_tcp_xvc_init,
	.quit = &xlnx_tcp_xvc_quit,

	.jtag_ops = &xlnx_tcp_xvc_jtag_ops,
};
