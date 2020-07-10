// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Server process functions.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup ipc_server
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_config_have.h"

#include "os/os_time.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"

#include "ipc_server.h"
#include "ipc_server_utils.h"

#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#ifdef XRT_HAVE_SYSTEMD
#include <systemd/sd-daemon.h>
#endif


/*
 *
 * Defines and helpers.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(exit_on_disconnect, "IPC_EXIT_ON_DISCONNECT", false)

struct _z_sort_data
{
	int32_t index;
	int32_t z_order;
};


/*
 *
 * Static functions.
 *
 */

static void
teardown_all(struct ipc_server *s)
{
	u_var_remove_root(s);

	xrt_comp_destroy(&s->xc);

	for (size_t i = 0; i < IPC_SERVER_NUM_XDEVS; i++) {
		xrt_device_destroy(&s->xdevs[i]);
	}

	xrt_instance_destroy(&s->xinst);

	if (s->listen_socket > 0) {
		// Close socket on exit
		close(s->listen_socket);
		s->listen_socket = -1;
		if (!s->launched_by_socket && s->socket_filename) {
			// Unlink it too, but only if we bound it.
			unlink(s->socket_filename);
			free(s->socket_filename);
			s->socket_filename = NULL;
		}
	}

	os_mutex_destroy(&s->global_state_lock);
}

static int
init_tracking_origins(struct ipc_server *s)
{
	for (size_t i = 0; i < IPC_SERVER_NUM_XDEVS; i++) {
		if (s->xdevs[i] == NULL) {
			continue;
		}

		struct xrt_device *xdev = s->xdevs[i];
		struct xrt_tracking_origin *xtrack = xdev->tracking_origin;
		assert(xtrack != NULL);
		size_t index = 0;

		for (; index < IPC_SERVER_NUM_XDEVS; index++) {
			if (s->xtracks[index] == NULL) {
				s->xtracks[index] = xtrack;
				break;
			} else if (s->xtracks[index] == xtrack) {
				break;
			}
		}
	}

	return 0;
}

static int
init_shm(struct ipc_server *s)
{
	const size_t size = sizeof(struct ipc_shared_memory);

	int fd = shm_open("/monado_shm", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
	if (fd < 0) {
		return -1;
	}

	if (ftruncate(fd, size) < 0) {
		close(fd);
		return -1;
	}

	const int access = PROT_READ | PROT_WRITE;
	const int flags = MAP_SHARED;
	s->ism = mmap(NULL, size, access, flags, fd, 0);
	if (s->ism == NULL) {
		close(fd);
		return -1;
	}

	// we have a filehandle, we will pass this to
	// our client rather than access via filesystem
	shm_unlink("/monado_shm");

	s->ism_fd = fd;


	/*
	 *
	 * Setup the shared memory state.
	 *
	 */

	uint32_t count = 0;
	struct ipc_shared_memory *ism = s->ism;

	count = 0;
	for (size_t i = 0; i < IPC_SERVER_NUM_XDEVS; i++) {
		struct xrt_tracking_origin *xtrack = s->xtracks[i];
		if (xtrack == NULL) {
			continue;
		}

		// The position of the tracking origin matches that in the
		// servers memory.
		assert(i < IPC_SHARED_MAX_DEVICES);

		struct ipc_shared_tracking_origin *itrack =
		    &ism->itracks[count++];
		memcpy(itrack->name, xtrack->name, sizeof(itrack->name));
		itrack->type = xtrack->type;
		itrack->offset = xtrack->offset;
	}

	ism->num_itracks = count;

	count = 0;
	uint32_t input_index = 0;
	uint32_t output_index = 0;
	for (size_t i = 0; i < IPC_SERVER_NUM_XDEVS; i++) {
		struct xrt_device *xdev = s->xdevs[i];
		if (xdev == NULL) {
			continue;
		}

		struct ipc_shared_device *idev = &ism->idevs[count++];

		idev->name = xdev->name;
		memcpy(idev->str, xdev->str, sizeof(idev->str));

		// Is this a HMD?
		if (xdev->hmd != NULL) {
			ism->hmd.views[0].display.w_pixels =
			    xdev->hmd->views[0].display.w_pixels;
			ism->hmd.views[0].display.h_pixels =
			    xdev->hmd->views[0].display.h_pixels;
			ism->hmd.views[0].fov = xdev->hmd->views[0].fov;
			ism->hmd.views[1].display.w_pixels =
			    xdev->hmd->views[1].display.w_pixels;
			ism->hmd.views[1].display.h_pixels =
			    xdev->hmd->views[1].display.h_pixels;
			ism->hmd.views[1].fov = xdev->hmd->views[1].fov;
		}

		// Setup the tracking origin.
		idev->tracking_origin_index = (uint32_t)-1;
		for (size_t k = 0; k < IPC_SERVER_NUM_XDEVS; k++) {
			if (xdev->tracking_origin != s->xtracks[k]) {
				continue;
			}

			idev->tracking_origin_index = k;
			break;
		}

		assert(idev->tracking_origin_index != (uint32_t)-1);

		// Initial update.
		xrt_device_update_inputs(xdev);

		// Copy the initial state and also count the number in inputs.
		size_t input_start = input_index;
		for (size_t k = 0; k < xdev->num_inputs; k++) {
			ism->inputs[input_index++] = xdev->inputs[k];
		}

		// Setup the 'offsets' and number of inputs.
		if (input_start != input_index) {
			idev->num_inputs = input_index - input_start;
			idev->first_input_index = input_start;
		}

		// Copy the initial state and also count the number in outputs.
		size_t output_start = output_index;
		for (size_t k = 0; k < xdev->num_outputs; k++) {
			ism->outputs[output_index++] = xdev->outputs[k];
		}

		// Setup the 'offsets' and number of outputs.
		if (output_start != output_index) {
			idev->num_outputs = output_index - output_start;
			idev->first_output_index = output_start;
		}
	}

	// Finally tell the client how many devices we have.
	s->ism->num_idevs = count;

	return 0;
}

static int
get_systemd_socket(struct ipc_server *s, int *out_fd)
{
#ifdef XRT_HAVE_SYSTEMD
	// We may have been launched with socket activation
	int num_fds = sd_listen_fds(0);
	if (num_fds > 1) {
		fprintf(stderr,
		        "Too many file descriptors passed by systemd.\n");
		return -1;
	}
	if (num_fds == 1) {
		*out_fd = SD_LISTEN_FDS_START + 0;
		s->launched_by_socket = true;
		printf("Got existing socket from systemd.\n");
	}
#endif
	return 0;
}

static int
create_listen_socket(struct ipc_server *s, int *out_fd)
{
	// no fd provided
	struct sockaddr_un addr;
	int fd, ret;

	fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		fprintf(stderr, "Message Socket Create Error!\n");
		return fd;
	}

	memset(&addr, 0, sizeof(addr));

	addr.sun_family = AF_UNIX;
	strcpy(addr.sun_path, IPC_MSG_SOCK_FILE);

	ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		fprintf(stderr,
		        "ERROR: Could not bind socket to path %s: is the "
		        "service running already?\n",
		        IPC_MSG_SOCK_FILE);
#ifdef XRT_HAVE_SYSTEMD
		fprintf(stderr,
		        "Or, is the systemd unit monado.socket or "
		        "monado-dev.socket active?\n");
#endif
		close(fd);
		return ret;
	}
	// Save for later
	s->socket_filename = strdup(IPC_MSG_SOCK_FILE);

	ret = listen(fd, IPC_MAX_CLIENTS);
	if (ret < 0) {
		close(fd);
		return ret;
	}
	printf("Created listening socket.\n");
	*out_fd = fd;
	return 0;
}

static int
init_listen_socket(struct ipc_server *s)
{
	int fd = -1, ret;
	s->listen_socket = -1;

	ret = get_systemd_socket(s, &fd);
	if (ret < 0) {
		return ret;
	}

	if (fd == -1) {
		ret = create_listen_socket(s, &fd);
		if (ret < 0) {
			return ret;
		}
	}
	// All ok!
	s->listen_socket = fd;
	printf("Listening socket is fd %d\n", s->listen_socket);

	return fd;
}

static int
init_epoll(struct ipc_server *s)
{
	int ret = epoll_create1(EPOLL_CLOEXEC);
	if (ret < 0) {
		return ret;
	}

	s->epoll_fd = ret;

	struct epoll_event ev = {0};

	if (!s->launched_by_socket) {
		// Can't do this when launched by systemd socket activation by
		// default
		ev.events = EPOLLIN;
		ev.data.fd = 0; // stdin
		ret = epoll_ctl(s->epoll_fd, EPOLL_CTL_ADD, 0, &ev);
		if (ret < 0) {
			fprintf(stderr, "ERROR: epoll_ctl(stdin) failed '%i'\n",
			        ret);
			return ret;
		}
	}

	ev.events = EPOLLIN;
	ev.data.fd = s->listen_socket;
	ret = epoll_ctl(s->epoll_fd, EPOLL_CTL_ADD, s->listen_socket, &ev);
	if (ret < 0) {
		fprintf(stderr, "ERROR: epoll_ctl(listen_socket) failed '%i'\n",
		        ret);
		return ret;
	}

	return 0;
}

static int
init_all(struct ipc_server *s)
{
	// Yes we should be running.
	s->running = true;
	s->exit_on_disconnect = debug_get_bool_option_exit_on_disconnect();

	int ret = xrt_instance_create(NULL, &s->xinst);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	ret = xrt_instance_select(s->xinst, s->xdevs, IPC_SERVER_NUM_XDEVS);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	if (s->xdevs[0] == NULL) {
		teardown_all(s);
		return -1;
	}

	ret = init_tracking_origins(s);
	if (ret < 0) {
		teardown_all(s);
		return -1;
	}

	ret = xrt_instance_create_fd_compositor(s->xinst, s->xdevs[0], false,
	                                        &s->xcfd);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	ret = init_shm(s);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	ret = init_listen_socket(s);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	ret = init_epoll(s);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	// Init all of the render riming helpers.
	for (size_t i = 0; i < ARRAY_SIZE(s->threads); i++) {
		u_rt_helper_init((struct u_rt_helper *)&s->threads[i].ics.urth);
	}

	ret = os_mutex_init(&s->global_state_lock);
	if (ret < 0) {
		teardown_all(s);
		return ret;
	}

	// Easier to use.
	s->xc = &s->xcfd->base;

	u_var_add_root(s, "IPC Server", false);
	u_var_add_bool(s, &s->print_debug, "print.debug");
	u_var_add_bool(s, &s->print_spew, "print.spew");
	u_var_add_bool(s, &s->exit_on_disconnect, "exit_on_disconnect");
	u_var_add_bool(s, (void *)&s->running, "running");

	return 0;
}

static void
handle_listen(struct ipc_server *vs)
{
	int ret = accept(vs->listen_socket, NULL, NULL);
	if (ret < 0) {
		fprintf(stderr, "ERROR: accept '%i'\n", ret);
		vs->running = false;
	}

	volatile struct ipc_client_state *cs = NULL;
	int32_t cs_index = -1;

	// The return is the new fd;
	int fd = ret;

	os_mutex_lock(&vs->global_state_lock);

	// find the next free thread in our array (server_thread_index is -1)
	// and have it handle this connection
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *_cs = &vs->threads[i].ics;
		if (_cs->server_thread_index < 0) {
			cs = _cs;
			cs_index = i;
			break;
		}
	}
	if (cs == NULL) {
		close(fd);

		// Unlock when we are done.
		os_mutex_unlock(&vs->global_state_lock);

		fprintf(stderr, "ERROR: Max client count reached!\n");
		return;
	}

	struct ipc_thread *it = &vs->threads[cs_index];
	if (it->state != IPC_THREAD_READY && it->state != IPC_THREAD_STOPPING) {
		// we should not get here
		close(fd);

		// Unlock when we are done.
		os_mutex_unlock(&vs->global_state_lock);

		fprintf(stderr, "ERROR: Client state management error!\n");
		return;
	}

	if (it->state != IPC_THREAD_READY) {
		os_thread_join(&it->thread);
		os_thread_destroy(&it->thread);
		it->state = IPC_THREAD_READY;
	}

	it->state = IPC_THREAD_STARTING;
	cs->ipc_socket_fd = fd;
	cs->server = vs;
	cs->server_thread_index = cs_index;
	os_thread_start(&it->thread, ipc_server_client_thread, (void *)cs);

	// Unlock when we are done.
	os_mutex_unlock(&vs->global_state_lock);
}

#define NUM_POLL_EVENTS 8
#define NO_SLEEP 0

static void
check_epoll(struct ipc_server *vs)
{
	int epoll_fd = vs->epoll_fd;

	struct epoll_event events[NUM_POLL_EVENTS] = {0};

	// No sleeping, returns immediately.
	int ret = epoll_wait(epoll_fd, events, NUM_POLL_EVENTS, NO_SLEEP);
	if (ret < 0) {
		fprintf(stderr, "EPOLL ERROR! \"%i\"\n", ret);
		vs->running = false;
		return;
	}

	for (int i = 0; i < ret; i++) {
		// If we get data on stdin, stop.
		if (events[i].data.fd == 0) {
			vs->running = false;
			return;
		}

		// Somebody new at the door.
		if (events[i].data.fd == vs->listen_socket) {
			handle_listen(vs);
		}
	}
}

static uint32_t
find_event_slot(volatile struct ipc_client_state *cs)
{
	uint64_t oldest_event_timestamp = UINT64_MAX;
	uint32_t oldest_event_index = 0;
	for (uint32_t i = 0; i < IPC_EVENT_QUEUE_SIZE; i++) {
		if (cs->queued_events->timestamp < oldest_event_timestamp) {
			oldest_event_index = i;
		}
		if (!cs->queued_events[i].pending) {
			return i;
		}
	}

	fprintf(stderr, "ERROR! event queue full - unconsumed event lost!\n");
	return oldest_event_index;
}

static void
transition_overlay_visibility(volatile struct ipc_client_state *cs,
                              bool visible)
{
	uint32_t event_slot = find_event_slot(cs);
	uint64_t timestamp = os_monotonic_get_ns();

	volatile struct ipc_queued_event *qe = &cs->queued_events[event_slot];

	qe->timestamp = timestamp;
	qe->pending = true;
	qe->event.type = XRT_COMPOSITOR_EVENT_OVERLAY_CHANGE;
	qe->event.overlay.visible = visible;
}

static void
send_client_state(volatile struct ipc_client_state *ics)
{
	uint32_t event_slot = find_event_slot(ics);
	uint64_t timestamp = os_monotonic_get_ns();

	volatile struct ipc_queued_event *qe = &ics->queued_events[event_slot];

	qe->timestamp = timestamp;
	qe->pending = true;
	qe->event.type = XRT_COMPOSITOR_EVENT_STATE_CHANGE;
	qe->event.state.visible = ics->client_state.session_visible;
	qe->event.state.focused = ics->client_state.session_focused;
}

static bool
_update_projection_layer(struct xrt_compositor *xc,
                         volatile struct ipc_client_state *ics,
                         volatile struct ipc_layer_entry *layer,
                         uint32_t i)
{
	// xdev
	uint32_t xdevi = layer->xdev_id;
	// left
	uint32_t lxsci = layer->swapchain_ids[0];
	// right
	uint32_t rxsci = layer->swapchain_ids[1];

	struct xrt_device *xdev = ics->server->xdevs[xdevi];
	struct xrt_swapchain *lxcs = ics->xscs[lxsci];
	struct xrt_swapchain *rxcs = ics->xscs[rxsci];

	if (lxcs == NULL || rxcs == NULL) {
		fprintf(stderr,
		        "ERROR: Invalid swap chain for projection layer.\n");
		return false;
	}

	if (xdev == NULL) {
		fprintf(stderr, "ERROR: Invalid xdev for projection layer.\n");
		return false;
	}

	// Cast away volatile.
	struct xrt_layer_data *data = (struct xrt_layer_data *)&layer->data;

	xrt_comp_layer_stereo_projection(xc, xdev, lxcs, rxcs, data);

	return true;
}

static bool
_update_quad_layer(struct xrt_compositor *xc,
                   volatile struct ipc_client_state *ics,
                   volatile struct ipc_layer_entry *layer,
                   uint32_t i)
{
	uint32_t xdevi = layer->xdev_id;
	uint32_t sci = layer->swapchain_ids[0];

	struct xrt_device *xdev = ics->server->xdevs[xdevi];
	struct xrt_swapchain *xcs = ics->xscs[sci];

	if (xcs == NULL) {
		fprintf(stderr, "ERROR: Invalid swapchain for quad layer.\n");
		return false;
	}

	if (xdev == NULL) {
		fprintf(stderr, "ERROR: Invalid xdev for quad layer.\n");
		return false;
	}

	// Cast away volatile.
	struct xrt_layer_data *data = (struct xrt_layer_data *)&layer->data;

	xrt_comp_layer_quad(xc, xdev, xcs, data);

	return true;
}

static int
_overlay_sort_func(const void *a, const void *b)
{
	struct _z_sort_data *oa = (struct _z_sort_data *)a;
	struct _z_sort_data *ob = (struct _z_sort_data *)b;
	if (oa->z_order < ob->z_order) {
		return -1;
	} else if (oa->z_order > ob->z_order) {
		return 1;
	}
	return 0;
}

static bool
_update_layers(struct ipc_server *s, struct xrt_compositor *xc)
{
	struct _z_sort_data z_data[IPC_MAX_CLIENTS];

	// initialise, and fill in overlay app data
	for (int32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;
		z_data[i].index = -1;
		z_data[i].z_order = -1;
		// we need to create a list of overlay applications, sorted by z
		if (ics->client_state.session_overlay) {
			if (ics->client_state.session_active) {
				z_data[i].index = i;
				z_data[i].z_order = ics->client_state.z_order;
			}
		}
	}

	// ensure our primary application is enabled,
	// and rendered first in the stack
	if (s->active_client_index >= 0) {
		z_data[s->active_client_index].index = s->active_client_index;
		z_data[s->active_client_index].z_order = INT32_MIN;
	}

	// sort the stack array
	qsort(z_data, IPC_MAX_CLIENTS, sizeof(struct _z_sort_data),
	      _overlay_sort_func);

	// render the layer stack
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		struct _z_sort_data *zd = &z_data[i];
		if (zd->index < 0) {
			continue;
		}

		volatile struct ipc_client_state *ics =
		    &s->threads[zd->index].ics;

		for (uint32_t j = 0; j < ics->render_state.num_layers; j++) {
			volatile struct ipc_layer_entry *layer =
			    &ics->render_state.layers[j];

			switch (layer->data.type) {
			case XRT_LAYER_STEREO_PROJECTION:
				if (!_update_projection_layer(xc, ics, layer,
				                              i)) {
					return false;
				}
				break;
			case XRT_LAYER_QUAD:
				if (!_update_quad_layer(xc, ics, layer, i)) {
					return false;
				}
				break;
			default: break;
			}
		}
	}

	return true;
}



static int
main_loop(struct ipc_server *s)
{
	struct xrt_compositor *xc = s->xc;

	// make sure all our client connections have a handle to the
	// compositor and consistent initial state

	while (s->running) {
		int64_t frame_id;
		uint64_t predicted_display_time;
		uint64_t predicted_display_period;

		xrt_comp_wait_frame(xc, &frame_id, &predicted_display_time,
		                    &predicted_display_period);

		uint64_t now = os_monotonic_get_ns();
		uint64_t diff = predicted_display_time - now;

		os_mutex_lock(&s->global_state_lock);

		// Broadcast the new timing information to the helpers.
		for (size_t i = 0; i < ARRAY_SIZE(s->threads); i++) {
			u_rt_helper_new_sample(
			    (struct u_rt_helper *)&s->threads[i].ics.urth,
			    predicted_display_time, diff,
			    predicted_display_period);
		}

		os_mutex_unlock(&s->global_state_lock);


		xrt_comp_begin_frame(xc, frame_id);
		xrt_comp_layer_begin(xc, frame_id, 0);

		_update_layers(s, xc);

		xrt_comp_layer_commit(xc, frame_id);

		// Check polling last, so we know we have valid timing data.
		check_epoll(s);
	}

	return 0;
}


static void
handle_overlay_client_events(volatile struct ipc_client_state *ics,
                             int active_id,
                             int prev_active_id)
{
	// this is an overlay session.
	if (ics->client_state.session_overlay) {

		// switch between main applications
		if (active_id >= 0 && prev_active_id >= 0) {
			transition_overlay_visibility(ics, false);
			transition_overlay_visibility(ics, true);
		}

		// switch from idle to active application
		if (active_id >= 0 && prev_active_id < 0) {
			transition_overlay_visibility(ics, true);
		}

		// switch from active application to idle
		if (active_id < 0 && prev_active_id >= 0) {
			transition_overlay_visibility(ics, false);
		}
	}
}

static void
handle_focused_client_events(volatile struct ipc_client_state *ics,
                             int active_id,
                             int prev_active_id)
{

	// if our prev active id is -1 and our cur active id is -1, we
	// can bail out early

	if (active_id == -1 && prev_active_id == -1) {
		return;
	}

	// set visibility/focus to false on all applications
	ics->client_state.session_focused = false;
	ics->client_state.session_visible = false;

	// do we have a primary application?
	if (active_id >= 0) {

		// if we are an overlay, we are always visible
		// if we have a primary application
		if (ics->client_state.session_overlay) {
			ics->client_state.session_visible = true;
		}

		// set visible + focused if we are the primary
		// application
		if (ics->server_thread_index == active_id) {
			ics->client_state.session_visible = true;
			ics->client_state.session_focused = true;
		}
		send_client_state(ics);
		return;
	}

	// no primary application, set all overlays to synchronised
	// state
	if (ics->client_state.session_overlay) {
		ics->client_state.session_focused = false;
		ics->client_state.session_visible = false;
		send_client_state(ics);
	}
}


void
init_server_state(struct ipc_server *s)
{

	// set up initial state for global vars, and each client state

	s->active_client_index = -1; // we start off with no active client.
	s->last_active_client_index = -1;
	s->current_slot_index = 0;

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *cs = &s->threads[i].ics;
		cs->server = s;
		cs->xc = s->xc;
		cs->server_thread_index = -1;
	}
}

/*
 *
 * Exported functions.
 *
 */

void
update_server_state(struct ipc_server *s)
{
	// multiple threads could call this at the same time.
	os_mutex_lock(&s->global_state_lock);

	// if our client that is set to active is still active,
	// and it is the same as our last active client, we can
	// early-out, as no events need to be sent

	if (s->active_client_index >= 0) {

		volatile struct ipc_client_state *ics =
		    &s->threads[s->active_client_index].ics;

		if (ics->client_state.session_active &&
		    s->active_client_index == s->last_active_client_index) {
			os_mutex_unlock(&s->global_state_lock);
			return;
		}
	}


	// our active application has changed - this would typically be
	// switched by the monado-ctl application or other app making a
	// 'set active application' ipc call, or it could be a
	// connection loss resulting in us needing to 'fall through' to
	// the first active application
	//, or finally to the idle 'wallpaper' images.


	bool set_idle = true;
	int fallback_active_application = -1;

	// do we have a fallback application?
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;
		if (ics->client_state.session_overlay == false &&
		    ics->server_thread_index >= 0 &&
		    ics->client_state.session_active) {
			fallback_active_application = i;
			set_idle = false;
		}
	}

	// if our currently-set active primary application is not
	// actually active/displayable, use the fallback application
	// instead.
	volatile struct ipc_client_state *ics =
	    &s->threads[s->active_client_index].ics;
	if (!(ics->client_state.session_overlay == false &&
	      s->active_client_index >= 0 &&
	      ics->client_state.session_active)) {
		s->active_client_index = fallback_active_application;
	}


	// if we have no applications to fallback to, enable the idle
	// wallpaper.
	if (set_idle) {
		s->active_client_index = -1;
	}

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {

		volatile struct ipc_client_state *ics = &s->threads[i].ics;
		if (ics->server_thread_index >= 0) {

			handle_focused_client_events(
			    ics, s->active_client_index,
			    s->last_active_client_index);

			handle_overlay_client_events(
			    ics, s->active_client_index,
			    s->last_active_client_index);
		}
	}

	s->last_active_client_index = s->active_client_index;

	os_mutex_unlock(&s->global_state_lock);
	return;
}

int
ipc_server_main(int argc, char **argv)
{
	struct ipc_server *s = U_TYPED_CALLOC(struct ipc_server);
	int ret = init_all(s);
	if (ret < 0) {
		free(s);
		return ret;
	}

	init_server_state(s);
	ret = main_loop(s);

	teardown_all(s);
	free(s);

	fprintf(stderr, "SERVER: Exiting! '%i'\n", ret);

	return ret;
}
