// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SLAM tracking code.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frameserver.h"
#include "util/u_debug.h"
#include "os/os_threading.h"
#include "math/m_space.h"

#include <slam_tracker.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/version.hpp>

#define SLAM_TRACE(...) U_LOG_IFL_T(t.ll, __VA_ARGS__)
#define SLAM_DEBUG(...) U_LOG_IFL_D(t.ll, __VA_ARGS__)
#define SLAM_INFO(...) U_LOG_IFL_I(t.ll, __VA_ARGS__)
#define SLAM_WARN(...) U_LOG_IFL_W(t.ll, __VA_ARGS__)
#define SLAM_ERROR(...) U_LOG_IFL_E(t.ll, __VA_ARGS__)
#define SLAM_ASSERT(predicate, ...)                                                                                    \
	do {                                                                                                           \
		bool p = predicate;                                                                                    \
		if (!p) {                                                                                              \
			U_LOG(U_LOGGING_ERROR, __VA_ARGS__);                                                           \
			assert(false && "SLAM_ASSERT failed: " #predicate);                                            \
			exit(EXIT_FAILURE);                                                                            \
		}                                                                                                      \
	} while (false);
#define SLAM_ASSERT_(predicate) SLAM_ASSERT(predicate, "Assertion failed " #predicate)

// Debug assertions, not vital but useful for finding errors
#ifdef NDEBUG
#define SLAM_DASSERT(predicate, ...)
#define SLAM_DASSERT_(predicate)
#else
#define SLAM_DASSERT(predicate, ...) SLAM_ASSERT(predicate, __VA_ARGS__)
#define SLAM_DASSERT_(predicate) SLAM_ASSERT_(predicate)
#endif

//! SLAM tracking logging level
DEBUG_GET_ONCE_LOG_OPTION(slam_log, "SLAM_LOG", U_LOGGING_WARN)

//! Config file path, format is specific to the SLAM implementation in use
DEBUG_GET_ONCE_OPTION(slam_config, "SLAM_CONFIG", NULL)


//! Namespace for the interface to the external SLAM tracking system
namespace xrt::auxiliary::tracking::slam {

using cv::Mat;
using cv::MatAllocator;
using cv::UMatData;
using cv::UMatUsageFlags;

#define USING_OPENCV_3_3_1 (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 3 && CV_VERSION_REVISION == 1)

#if defined(XRT_HAVE_KIMERA_SLAM) && !USING_OPENCV_3_3_1
#warning "Kimera-VIO uses OpenCV 3.3.1, use that to prevent conflicts"
#endif

//! @todo These defs should make OpenCV 4 work but it wasn't tested against a
//! SLAM system that supports that version yet.
#if CV_VERSION_MAJOR < 4
#define ACCESS_RW 0
typedef int AccessFlag;
#define CV_AUTOSTEP 0x7fffffff // From opencv2/core/core_c.h
#else
using cv::ACCESS_RW;
using cv::AccessFlag;
#define CV_AUTOSTEP cv::Mat::AUTO_STEP;
#endif

/*!
 * @brief Wraps a @ref xrt_frame with a `cv::Mat` (conversely to @ref FrameMat).
 *
 * It works by implementing a `cv::MatAllocator` which determines what to do
 * when a `cv::Mat` refcount reaches zero. In that case, it decrements the @ref
 * xrt_frame refcount once the `cv::Mat` own refcount has reached zero.
 *
 * @note a @ref MatFrame `cv::Mat` can wrap a @ref FrameMat @ref xrt_frame,
 * which in turns wraps a `cv::Mat`, with little overhead, and that is precisely
 * how it is being used in this file when the @ref xrt_frame is a @ref FrameMat.
 */
class MatFrame final : public MatAllocator
{
public:
	//! Wraps a @ref xrt_frame in a `cv::Mat`
	Mat
	wrap(struct xrt_frame *frame)
	{
		SLAM_DASSERT_(frame->format == XRT_FORMAT_L8 || frame->format == XRT_FORMAT_R8G8B8);
		auto img_type = frame->format == XRT_FORMAT_L8 ? CV_8UC1 : CV_8UC3;

		// Wrap the frame data into a cv::Mat header
		cv::Mat img{(int)frame->height, (int)frame->width, img_type, frame->data};

		// Enable reference counting for a user-allocated cv::Mat (i.e., using existing frame->data)
		img.u = this->allocate(img.dims, img.size.p, img.type(), img.data, img.step.p, ACCESS_RW,
		                       cv::USAGE_DEFAULT);
		SLAM_DASSERT_(img.u->refcount == 0);
		img.addref();

		// Keep a reference to the xrt_frame in the cv userdata field for when the cv::Mat reference reaches 0
		SLAM_DASSERT_(img.u->userdata == NULL); // Should be default-constructed
		xrt_frame_reference((struct xrt_frame **)&img.u->userdata, frame);

		return img;
	}

	//! Allocates a `cv::UMatData` object which is in charge of reference counting for a `cv::Mat`
	UMatData *
	allocate(
	    int dims, const int *sizes, int type, void *data0, size_t *step, AccessFlag, UMatUsageFlags) const override
	{
		SLAM_DASSERT_(dims == 2 && sizes && data0 && step && step[0] != CV_AUTOSTEP);
		UMatData *u = new UMatData(this);
		uchar *data = (uchar *)data0;
		u->data = u->origdata = data;
		u->size = step[0] * sizes[0];         // Row stride * row count
		u->flags |= UMatData::USER_ALLOCATED; // External data
		return u;
	}

	//! Necessary but unused virtual method for a `cv::MatAllocator`
	bool
	allocate(UMatData *, AccessFlag, UMatUsageFlags) const override
	{
		SLAM_ASSERT(false, "Shouldn't be reached");
		return false;
	}

	//! When `cv::UMatData` refcount reaches zero this method is called, we just
	//! decrement the original @ref xrt_frame refcount as it is the one in charge
	//! of the memory.
	void
	deallocate(UMatData *u) const override
	{
		SLAM_DASSERT_(u->urefcount == 0 && u->refcount == 0);
		SLAM_DASSERT_(u->flags & UMatData::USER_ALLOCATED);
		xrt_frame_reference((struct xrt_frame **)&u->userdata, NULL);
		delete u;
	}
};

/*!
 * Main implementation of @ref xrt_tracked_slam. This is an adapter class for
 * SLAM tracking that wraps an external SLAM implementation.
 *
 * @implements xrt_tracked_slam
 * @implements xrt_frame_node
 * @implements xrt_frame_sink
 * @implements xrt_imu_sink
 */
struct TrackerSlam
{
	struct xrt_tracked_slam base = {};
	struct xrt_frame_node node = {}; //!< Will be called on destruction
	slam_tracker *slam;              //!< Pointer to the external SLAM system implementation

	struct xrt_slam_sinks sinks = {};      //!< Pointers to the sinks below
	struct xrt_frame_sink left_sink = {};  //!< Sends left camera frames to the SLAM system
	struct xrt_frame_sink right_sink = {}; //!< Sends right camera frames to the SLAM system
	struct xrt_imu_sink imu_sink = {};     //!< Sends imu samples to the SLAM system

	enum u_logging_level ll;     //!< Logging level for the SLAM tracker, set by SLAM_LOG var
	struct os_thread_helper oth; //!< Thread where the external SLAM system runs
	MatFrame *cv_wrapper;        //!< Wraps a xrt_frame in a cv::Mat to send to the SLAM system

	// Used for checking that the timestamps come in order
	mutable timepoint_ns last_imu_ts = INT64_MIN;
	mutable timepoint_ns last_left_ts = INT64_MIN;
	mutable timepoint_ns last_right_ts = INT64_MIN;
};

} // namespace xrt::auxiliary::tracking::slam

using namespace xrt::auxiliary::tracking::slam;

//! Receive and send IMU samples to the external SLAM system
extern "C" void
t_slam_imu_sink_push(struct xrt_imu_sink *sink, struct xrt_imu_sample *s)
{
	auto &t = *container_of(sink, TrackerSlam, imu_sink);

	timepoint_ns ts = s->timestamp_ns;
	xrt_vec3_f64 a = s->accel_m_s2;
	xrt_vec3_f64 w = s->gyro_rad_secs;

	//! @todo There are many conversions like these between xrt and
	//! slam_tracker.hpp types. Implement a casting mechanism to avoid copies.
	imu_sample sample{ts, a.x, a.y, a.z, w.x, w.y, w.z};
	t.slam->push_imu_sample(sample);
	SLAM_TRACE("imu t=%ld a=[%f,%f,%f] w=[%f,%f,%f]", ts, a.x, a.y, a.z, w.x, w.y, w.z);

	// Check monotonically increasing timestamps
	SLAM_DASSERT(ts > t.last_imu_ts, "Sample (%ld) is older than last (%ld)", ts, t.last_imu_ts)
	t.last_imu_ts = ts;
}

/*!
 * @brief Get a space relation tracked by a SLAM system at a specified time.
 *
 * @todo This function should do pose prediction, currently it is not using @p
 * when_ns and just returning the latest tracked pose instead.
 */
extern "C" void
t_slam_get_tracked_pose(struct xrt_tracked_slam *xts, timepoint_ns when_ns, struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xts, TrackerSlam, base);
	pose p{};
	bool dequeued = t.slam->try_dequeue_pose(p);
	if (dequeued) {
		SLAM_TRACE("pose p=[%f,%f,%f] r=[%f,%f,%f,%f]", p.px, p.py, p.pz, p.rx, p.ry, p.rz, p.rw);

		// Note that any pose correction should happen in the device consuming the tracking
		out_relation->pose = {{p.rx, p.ry, p.rz, p.rw}, {p.px, p.py, p.pz}};
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	} else {
		SLAM_TRACE("No poses to dequeue");
		out_relation->relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
	}
}

//! Push the frame to the external SLAM system
static void
push_frame(const TrackerSlam &t, struct xrt_frame *frame, bool is_left)
{
	// Construct and send the image sample
	cv::Mat img = t.cv_wrapper->wrap(frame);
	SLAM_DASSERT_(frame->timestamp < INT64_MAX);
	img_sample sample{(int64_t)frame->timestamp, img, is_left};
	t.slam->push_frame(sample);
	SLAM_TRACE("%s frame t=%lu", is_left ? " left" : "right", frame->timestamp);

	// Check monotonically increasing timestamps
	timepoint_ns &last_ts = is_left ? t.last_left_ts : t.last_right_ts;
	SLAM_DASSERT(sample.timestamp > last_ts, "Frame (%ld) is older than last (%ld)", sample.timestamp, last_ts);
	last_ts = sample.timestamp;
}

extern "C" void
t_slam_frame_sink_push_left(struct xrt_frame_sink *sink, struct xrt_frame *frame)
{
	auto &t = *container_of(sink, TrackerSlam, left_sink);
	push_frame(t, frame, true);
}

extern "C" void
t_slam_frame_sink_push_right(struct xrt_frame_sink *sink, struct xrt_frame *frame)
{
	auto &t = *container_of(sink, TrackerSlam, right_sink);
	push_frame(t, frame, false);
}

extern "C" void
t_slam_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerSlam, node);
	t.slam->stop();
	os_thread_helper_stop(&t.oth);
	SLAM_DEBUG("SLAM tracker dismantled");
}

extern "C" void
t_slam_node_destroy(struct xrt_frame_node *node)
{
	auto t_ptr = container_of(node, TrackerSlam, node);
	auto &t = *t_ptr; // Needed by SLAM_DEBUG
	SLAM_DEBUG("Destroying SLAM tracker");
	os_thread_helper_destroy(&t_ptr->oth);
	delete t_ptr->slam;
	delete t_ptr->cv_wrapper;
	delete t_ptr;
}

//! Runs the external SLAM system in a separate thread
extern "C" void *
t_slam_run(void *ptr)
{
	auto &t = *(TrackerSlam *)ptr;
	SLAM_DEBUG("SLAM tracker starting");
	t.slam->start();
	return NULL;
}

//! Starts t_slam_run
extern "C" int
t_slam_start(struct xrt_tracked_slam *xts)
{
	auto &t = *container_of(xts, TrackerSlam, base);
	int ret = os_thread_helper_start(&t.oth, t_slam_run, &t);
	SLAM_ASSERT(ret == 0, "Unable to start thread");
	SLAM_DEBUG("SLAM tracker started");
	return ret;
}

extern "C" int
t_slam_create(struct xrt_frame_context *xfctx, struct xrt_tracked_slam **out_xts, struct xrt_slam_sinks **out_sink)
{
	enum u_logging_level ll = debug_get_log_option_slam_log();
	const char *config_file = debug_get_option_slam_config();
	if (!config_file) {
		U_LOG_IFL_W(ll, "SLAM tracker requires a config file set with the SLAM_CONFIG environment variable");
		return -1;
	}

	auto &t = *(new TrackerSlam{});
	t.ll = ll;
	t.cv_wrapper = new MatFrame();

	t.base.get_tracked_pose = t_slam_get_tracked_pose;

	std::string config_file_string = std::string(config_file);
	t.slam = new slam_tracker{config_file_string};

	t.left_sink.push_frame = t_slam_frame_sink_push_left;
	t.right_sink.push_frame = t_slam_frame_sink_push_right;
	t.imu_sink.push_imu = t_slam_imu_sink_push;

	t.sinks.left = &t.left_sink;
	t.sinks.right = &t.right_sink;
	t.sinks.imu = &t.imu_sink;

	t.node.break_apart = t_slam_node_break_apart;
	t.node.destroy = t_slam_node_destroy;

	int ret = os_thread_helper_init(&t.oth);
	SLAM_ASSERT(ret == 0, "Unable to initialize thread");

	xrt_frame_context_add(xfctx, &t.node);

	*out_xts = &t.base;
	*out_sink = &t.sinks;

	SLAM_DEBUG("SLAM tracker created");
	return 0;
}
