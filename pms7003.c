#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>

#include <gio/gio.h>

#include "pms7003-generated.h"

#define die(...) do { \
	fprintf(stderr, ##__VA_ARGS__); \
	exit(EXIT_FAILURE); \
} while (0)

#define PMS7003_DBUS_OBJECT "/local/pms7003"
#define PMS7003_DBUS_IFACE "local.pms7003"
#define PMS7003_DBUS_NAME "local.pms7003"

#define PMS7003_CMD_LEN 7
#define PMS7003_CMD_RSP_LEN 8

enum pms7003_state {
	SLEEP,
	PASSIVE,
	ACTIVE
};

enum pms7003_cmd {
	SLEEP_CMD,
	WAKEUP_CMD,
	ENTER_PASSIVE_MODE_CMD,
	ENTER_ACTIVE_MODE_CMD,
	READ_PASSIVE_CMD,
	READ_ACTIVE_CMD /* not a sensor command per se */
};

struct pms7003_frame {
	uint8_t magic0;
	uint8_t magic1;
	uint8_t dummy; /* always zero */
	uint8_t length;
	/*
	 * if frame contains measurements then data has following content:
	 *
	 * data[0]: pm1.0 cf1
	 * data[1]: pm2.5 cf1
	 * data[2]: pm10 cf1
	 * data[3]: pm1.0 std
	 * data[4]: pm2.5 std
	 * data[5]: pm10 std
	 * data[6]: over 0.3um
	 * data[7]: over 0.5um
	 * data[8]: over 1um
	 * data[9]: over 2.5um
	 * data[10]: over 5um
	 * data[11]: over 10um
	 * data[12]: version + error code
	 * data[13]: crc
	 */
	uint16_t data[14];
};

struct fd_source {
	GSource source;
	GIOCondition condition;
	GPollFD pollfd;
};

static int fd;
static enum pms7003_state state = SLEEP;
static bool dump_measuremenets;

static GMainLoop *loop;
static GDBusConnection *conn;
static GSource *source;
static Pms7003 *pms7003;

/*
 * command has following format:
 * +------+------+-----+-------+-------+------+------+
 * | 0x42 | 0x4d | cmd | datah | datal | lrch | lrcl |
 * +------+------+-----+-------+-------+------+------+
 */
static const char pms7003_cmd_tbl[][PMS7003_CMD_LEN] = {
	[SLEEP_CMD] = { 0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73 },
	[WAKEUP_CMD] = { 0x42, 0x4d, 0xe4, 0x00, 0x01, 0x01, 0x74 },
	[ENTER_PASSIVE_MODE_CMD] = { 0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70 },
	[ENTER_ACTIVE_MODE_CMD] = { 0x42, 0x4d, 0xe1, 0x00, 0x01, 0x01, 0x71 },
	[READ_PASSIVE_CMD] = { 0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71 }
};

/*
 * command response has following format:
 * +------+------+------+--------+-----+-------+------+------+
 * | 0x42 | 0x4d | 0x00 | length | cmd | datal | lrch | lrcl |
 * +------+------+------+--------+-----+-------+------+------+
 */
static const char pms7003_cmd_rsp_tbl[][PMS7003_CMD_RSP_LEN] = {
	[SLEEP_CMD] = { 0x42, 0x4d, 0x00, 0x04, 0xe4, 0x00, 0x01, 0x77 },
	/* on WAKEUP_CMD sensor returns a whole frame */
	[ENTER_PASSIVE_MODE_CMD] = { 0x42, 0x4d, 0x00, 0x04, 0xe1, 0x00, 0x01, 0x74 },
	[ENTER_ACTIVE_MODE_CMD] = { 0x42, 0x4d, 0x00, 0x04, 0xe1, 0x01, 0x01, 0x75 },
	/* on READ_PASSIVE_CMD sensor returns a whole frame */
};

static void usage(const char *name)
{
	printf("Usage: %s -p <device path> [-d] [-v] [-s]\n", name);
	printf("\n");
	printf("  -p device path\n");
	printf("  -d run as daemon\n");
	printf("  -v dump measurements\n");
	printf("  -s set initial state (0 - sleep, 1 - passive, 2 - active)\n");
	printf("\n");
}

static void daemonize(void)
{
	switch (fork()) {
	case -1:
		die("Failed to fork\n");
	case 0:
		break;
	default:
		exit(EXIT_SUCCESS);
	}

	if (setsid() == -1)
		die("Failed start a new session\n");

	umask(0);

	if (chdir("/") == -1)
		die("Failed to switch to root directory\n");

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	if (open("/dev/null", O_RDWR) != STDIN_FILENO)
		exit(EXIT_FAILURE);

	if (dup2(STDIN_FILENO, STDOUT_FILENO) != STDOUT_FILENO)
		exit(EXIT_FAILURE);

	if (dup2(STDIN_FILENO, STDERR_FILENO) != STDERR_FILENO)
		exit(EXIT_FAILURE);
}

static gboolean fd_source_check(GSource *source)
{
	struct fd_source *fds = (struct fd_source *)source;

	if (fds->pollfd.revents & G_IO_ERR) {
		g_main_loop_quit(loop);

		return false;
	}

	return fds->pollfd.revents & fds->condition;
}

static gboolean fd_source_dispatch(GSource *source, GSourceFunc callback,
				   void *user_data)
{
	return callback(user_data);
}

static GSourceFuncs fd_source_funcs = {
	.check = fd_source_check,
	.dispatch = fd_source_dispatch,
};

static GSource *fd_source_new(int fd, GIOCondition condition)
{
  	GSource *s = g_source_new(&fd_source_funcs, sizeof(struct fd_source));
  	struct fd_source *fds = (struct fd_source *)s;

  	fds->condition = condition;
	fds->pollfd.fd = fd;
	fds->pollfd.events = condition;
	fds->pollfd.revents = 0;
	g_source_add_poll(s, &fds->pollfd);

	return s;
}

static int pms7003_open(const char *path)
{
	struct termios tty;
	int ret;

	ret = open(path, O_RDWR | O_NOCTTY);
	if (ret < 0)
		return ret;

	memset (&tty, 0, sizeof(tty));

	tty.c_cflag = CS8 | CREAD | CLOCAL;
	tty.c_cc[VMIN] = 8;
	tty.c_cc[VTIME] = 1;

	tcflush(ret, TCIFLUSH);
	cfsetospeed(&tty, B9600);
	tcsetattr(ret, TCSANOW, &tty);

	return ret;
}

static int pms7003_frame_ok(const struct pms7003_frame *frame)
{
	const uint8_t *p;
	int i, checksum;

	if (frame->magic0 != 0x42 && frame->magic1 != 0x4d)
		return 0;

	if (frame->length != 4 && frame->length != 28)
		return 0;

	checksum = frame->magic0;
	checksum += frame->magic1;
	checksum += frame->length;

	p = &frame->length + 1;
	for (i = 0; i < frame->length - 2; i++)
		checksum += *p++;

	return checksum == be16toh(*(uint16_t *)p);
}

static void pms7003_be16data_to_host(struct pms7003_frame *frame)
{
	int i;

	for (i = 0; i < (frame->length - 4) / 2; i++)
		frame->data[i] = be16toh(frame->data[i]);
}

static void pms7003_dump_measuremets(struct pms7003_frame *frame)
{
	printf("\033c");
	printf("pm1.0 cf1 [ug/m3]: %d\n", frame->data[0]);
	printf("pm2.5 cf1 [ug/m3]: %d\n", frame->data[1]);
	printf("pm10  cf1 [ug/m3]: %d\n", frame->data[2]);
	printf("\n");
	printf("pm1.0 std [ug/m3]: %d\n", frame->data[3]);
	printf("pm2.5 std [ug/m3]: %d\n", frame->data[4]);
	printf("pm10  std [ug/m3]: %d\n", frame->data[5]);
	printf("\n");
	printf("over 0.3um [1/0.1L]: %d\n", frame->data[6]);
	printf("over 0.5um [1/0.1L]: %d\n", frame->data[7]);
	printf("over 1.0um [1/0.1L]: %d\n", frame->data[8]);
	printf("over 2.5um [1/0.1L]: %d\n", frame->data[9]);
	printf("over 5um   [1/0.1L]: %d\n", frame->data[10]);
	printf("over 10um  [1/0.1L]: %d\n", frame->data[11]);
}

static int pms7003_do_cmd(enum pms7003_cmd cmd, struct pms7003_frame *frame)
{
	void *p = frame;
	int ret;

	if (cmd == READ_ACTIVE_CMD)
		goto skip_write;

	ret = write(fd, pms7003_cmd_tbl[cmd], PMS7003_CMD_LEN);
	if (ret != PMS7003_CMD_LEN)
		return -1;

skip_write:
	ret = read(fd, p, 8);
	if (ret != 8)
		return -1;

	if (frame->length == 4)
		goto skip_read;

	ret += read(fd, p + 8, 8);
	ret += read(fd, p + 16, 8);
	ret += read(fd, p + 24, 8);
	if (ret != 32)
		return -1;

skip_read:
	if (!pms7003_frame_ok(frame))
		return -1;

	switch (cmd) {
	case SLEEP_CMD:
	case ENTER_PASSIVE_MODE_CMD:
	case ENTER_ACTIVE_MODE_CMD:
		if (memcmp((void *)frame, pms7003_cmd_rsp_tbl[cmd],
			   PMS7003_CMD_RSP_LEN))
			return -1;

		return 0;
	case READ_PASSIVE_CMD:
	case READ_ACTIVE_CMD:
		pms7003_be16data_to_host(frame);

		if (dump_measuremenets)
			pms7003_dump_measuremets(frame);

		return 0;
	case WAKEUP_CMD:
		/* On wakeup sensor sends 32 bytes of data which we ignore */
		return 0;
	}
}

static GVariant *variant_from_data(const uint16_t *data, int len)
{
	GVariantBuilder builder;
	int i;

	g_variant_builder_init(&builder, G_VARIANT_TYPE("aq"));

	for (i = 0; i < len; i++)
		g_variant_builder_add(&builder, "q", data[i]);

	return g_variant_builder_end(&builder);
}

static gboolean read_fd(void *user_data)
{
	struct pms7003_frame frame;
	GVariant *v;
	int ret;

	ret = pms7003_do_cmd(READ_ACTIVE_CMD, &frame);
	if (ret)
		return G_SOURCE_CONTINUE;

	pms7003_emit_new_data(pms7003, variant_from_data(frame.data, 12));

	return G_SOURCE_CONTINUE;
}

static void add_fd_source(void)
{
	source = fd_source_new(fd, G_IO_IN);
	g_source_set_callback(source, read_fd, NULL, NULL);
	g_source_attach(source, NULL);
}

static void remove_fd_source(void)
{
	g_source_destroy(source);
	source = NULL;
}

static int pms7003_change_state(enum pms7003_state new_state)
{
	struct pms7003_frame dummy;
	int ret;

	if (state == new_state)
		return 0;

	/* sensor will enter active state on receiving WAKEUP_CMD */
	if (state == SLEEP && new_state == PASSIVE) {
		ret = pms7003_do_cmd(WAKEUP_CMD, &dummy);
		if (ret)
			return -1;

		ret = pms7003_do_cmd(ENTER_PASSIVE_MODE_CMD, &dummy);
		if (ret)
			return -1;
	} else if (state == SLEEP && new_state == ACTIVE) {
		ret = pms7003_do_cmd(WAKEUP_CMD, &dummy);
		if (ret)
			return -1;

		add_fd_source();
	} else if (state == PASSIVE && new_state == ACTIVE) {
		ret = pms7003_do_cmd(ENTER_ACTIVE_MODE_CMD, &dummy);
		if (ret)
			return -1;

		add_fd_source();
	} else if (state == PASSIVE && new_state == SLEEP) {
		ret = pms7003_do_cmd(SLEEP_CMD, &dummy);
		if (ret)
			return -1;
	} else if (state == ACTIVE && new_state == PASSIVE) {
		ret = pms7003_do_cmd(ENTER_PASSIVE_MODE_CMD, &dummy);
		if (ret)
			return -1;

		remove_fd_source();
	} else if (state == ACTIVE && new_state == SLEEP) {
		ret = pms7003_do_cmd(SLEEP_CMD, &dummy);
		if (ret)
			return -1;

		remove_fd_source();
	} else {

	}

	state = new_state;

	return 0;
}

static void signal_handler(int sig)
{
	if (loop) {
		pms7003_change_state(SLEEP);
		g_main_loop_quit(loop);
	} else {
		exit(EXIT_SUCCESS);
	}
}

static bool handle_set_state(Pms7003 *object, GDBusMethodInvocation *inv,
			     uint32_t state)
{
	int ret;

	if (state > ACTIVE) {
		g_dbus_method_invocation_return_dbus_error(inv,
				PMS7003_DBUS_IFACE".Error.InvalidArguments",
				"Failed to change state");
		goto out;
	}

	ret = pms7003_change_state(state);
	if (ret) {
		g_dbus_method_invocation_return_dbus_error(inv,
				PMS7003_DBUS_IFACE".Error.IO",
				"Failed to change state");
		goto out;
	}

	pms7003_set_state(pms7003, state);
	pms7003_complete_set_state(object, inv);
out:
	return true;
}

static bool handle_read(Pms7003 *object, GDBusMethodInvocation *inv)
{
	struct pms7003_frame frame;
	int ret, i;

	ret = pms7003_do_cmd(READ_PASSIVE_CMD, &frame);
	if (ret) {
		g_dbus_method_invocation_return_dbus_error(inv,
				PMS7003_DBUS_IFACE".Error.IO",
				"Failed to read data");
		return true;
	}

	pms7003_complete_read(object, inv, variant_from_data(frame.data, 12));

	return true;
}

static void on_bus_acquired(GDBusConnection *connection, const char *name,
			    void *user_data)
{
	GError *err = NULL;
	int ret;

	conn = connection;

	pms7003 = pms7003_skeleton_new();
	pms7003_set_state(pms7003, state);
	g_signal_connect(pms7003, "handle-set-state",
			 G_CALLBACK(handle_set_state), NULL);
	g_signal_connect(pms7003, "handle-read",
			 G_CALLBACK(handle_read), NULL);

	g_dbus_interface_skeleton_export(G_DBUS_INTERFACE_SKELETON(pms7003),
					 connection, PMS7003_DBUS_OBJECT, &err);
	if (err) {
		printf("%s\n", err->message);
		g_error_free(err);
	}
}

int main(int argc, char *argv[])
{
	const char *dev_path = NULL;
	bool daemon = false;
	int id, opt, st;

	while ((opt = getopt(argc, argv, "p:d:s:v")) != -1) {
		switch (opt) {
		case 'p':
			dev_path = optarg;
			break;
		case 'd':
			daemon = true;
			break;
		case 'v':
			dump_measuremenets = true;
			break;
		case 's':
			st = strtoul(optarg, NULL, 10);
			if (st > ACTIVE) {
				usage(argv[0]);
				exit(EXIT_FAILURE);
			}

			break;
		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
	}

	if (!dev_path) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	if (daemon)
		daemonize();

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	fd = pms7003_open(dev_path);
	if (fd < 0)
		die("Failed to open %s\n", dev_path);

	pms7003_change_state(PASSIVE);
	pms7003_change_state(st);

	/*
	 * according to documentaion sensor requires ~30s for spinning
	 * up after wake-up
	 */

	loop = g_main_loop_new(NULL, FALSE);

	id = g_bus_own_name(G_BUS_TYPE_SYSTEM, PMS7003_DBUS_NAME,
			    G_BUS_NAME_OWNER_FLAGS_NONE, on_bus_acquired, NULL,
			    NULL, loop, NULL);

	g_main_loop_run(loop);

	if (source)
		g_source_destroy(source);

	g_bus_unown_name(id);

	return 0;
}
