// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver context implementation and entrypoint.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <cmath>
#include <unordered_map>
#include <string_view>
#include <filesystem>
#include <istream>

#include "openvr_driver.h"
#include "vdf_parser.hpp"
#include "steamvr_lh_interface.h"
#include "interfaces/context.hpp"
#include "device.hpp"
#include "util/u_device.h"
#include "util/u_misc.h"
#include "util/u_space_overseer.h"
#include "util/u_builders.h"
#include "util/u_device.h"
#include "util/u_system_helpers.h"
#include "vive/vive_bindings.h"

namespace {

DEBUG_GET_ONCE_LOG_OPTION(lh_log, "LIGHTHOUSE_LOG", U_LOGGING_INFO)

static const size_t MAX_CONTROLLERS = 16;


struct steamvr_lh_system
{
	// System devices wrapper.
	struct xrt_system_devices base;

	//! Origin for all devices.
	struct xrt_tracking_origin origin;

	//! Pointer to driver context
	std::shared_ptr<Context> ctx;

	// Controller as index and value as xdev
	int32_t controller_to_xdev_map[MAX_CONTROLLERS];

	//! Index to the left controller.
	int32_t left_index;

	//! Index to the right controller.
	int32_t right_index;

	//! Index to the gamepad controller.
	int32_t gamepad_index;

	//! Index to the hmd.
	int32_t head_index;
};

struct steamvr_lh_system *svrs = U_TYPED_CALLOC(struct steamvr_lh_system);


// ~/.steam/root is a symlink to where the Steam root is
const std::string STEAM_INSTALL_DIR = std::string(getenv("HOME")) + "/.steam/root";
constexpr auto STEAMVR_APPID = "250820";

// Parse libraryfolder.vdf to find where SteamVR is installed
std::string
find_steamvr_install()
{
	using namespace tyti;
	std::ifstream file(STEAM_INSTALL_DIR + "/steamapps/libraryfolders.vdf");
	auto root = vdf::read(file);
	assert(root.name == "libraryfolders");
	for (auto &[_, child] : root.children) {
		U_LOG_D("Found library folder %s", child->attribs["path"].c_str());
		std::shared_ptr<vdf::object> apps = child->children["apps"];
		for (auto &[appid, _] : apps->attribs) {
			if (appid == STEAMVR_APPID) {
				return child->attribs["path"] + "/steamapps/common/SteamVR";
			}
		}
	}
	return std::string();
}

} // namespace

#define CTX_ERR(...) U_LOG_IFL_E(log_level, __VA_ARGS__)
#define CTX_WARN(...) U_LOG_IFL_W(log_level, __VA_ARGS__)
#define CTX_INFO(...) U_LOG_IFL_I(log_level, __VA_ARGS__)
#define CTX_TRACE(...) U_LOG_IFL_T(log_level, __VA_ARGS__)
#define CTX_DEBUG(...) U_LOG_IFL_D(log_level, __VA_ARGS__)

/**
 * Since only the devices will live after our get_devices function is called, we make our Context
 * a shared ptr that is owned by the devices that exist, so that it is also cleaned up by the
 * devices that exist when they are all destroyed.
 */
std::shared_ptr<Context>
Context::create(const std::string &steam_install,
                const std::string &steamvr_install,
                vr::IServerTrackedDeviceProvider *p)
{
	// xrt_tracking_origin initialization
	Context *c = new Context(steam_install, steamvr_install, debug_get_log_option_lh_log());
	c->provider = p;
	std::strncpy(c->name, "SteamVR Lighthouse Tracking", XRT_TRACKING_NAME_LEN);
	c->type = XRT_TRACKING_TYPE_LIGHTHOUSE;
	c->offset = XRT_POSE_IDENTITY;
	return std::shared_ptr<Context>(c);
}

Context::Context(const std::string &steam_install, const std::string &steamvr_install, u_logging_level level)
    : settings(steam_install, steamvr_install), resources(level, steamvr_install), log_level(level)
{}

Context::~Context()
{
	provider->Cleanup();
}

/***** IVRDriverContext methods *****/

void *
Context::GetGenericInterface(const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
#define MATCH_INTERFACE(version, interface)                                                                            \
	if (std::strcmp(pchInterfaceVersion, version) == 0) {                                                          \
		return interface;                                                                                      \
	}
#define MATCH_INTERFACE_THIS(interface) MATCH_INTERFACE(interface##_Version, static_cast<interface *>(this))

	// Known interfaces
	MATCH_INTERFACE_THIS(vr::IVRServerDriverHost);
	MATCH_INTERFACE_THIS(vr::IVRDriverInput);
	MATCH_INTERFACE_THIS(vr::IVRProperties);
	MATCH_INTERFACE_THIS(vr::IVRDriverLog);
	MATCH_INTERFACE(vr::IVRSettings_Version, &settings);
	MATCH_INTERFACE(vr::IVRResources_Version, &resources);
	MATCH_INTERFACE(vr::IVRIOBuffer_Version, &iobuf);
	MATCH_INTERFACE(vr::IVRDriverManager_Version, &man);
	MATCH_INTERFACE(vr::IVRBlockQueue_Version, &blockqueue);
	MATCH_INTERFACE(vr::IVRPaths_Version, &paths);

	// Internal interfaces
	MATCH_INTERFACE("IVRServer_XXX", &server);
	return nullptr;
}

vr::DriverHandle_t
Context::GetDriverHandle()
{
	return 1;
}


/***** IVRServerDriverHost methods *****/

bool
Context::setup_hmd(const char *serial, vr::ITrackedDeviceServerDriver *driver)
{
	this->hmd = new HmdDevice(DeviceBuilder{this->shared_from_this(), driver, serial, STEAM_INSTALL_DIR});
#define VERIFY(expr, msg)                                                                                              \
	if (!(expr)) {                                                                                                 \
		CTX_ERR("Activating HMD failed: %s", msg);                                                             \
		delete this->hmd;                                                                                      \
		this->hmd = nullptr;                                                                                   \
		return false;                                                                                          \
	}
	vr::EVRInitError err = driver->Activate(0);
	VERIFY(err == vr::VRInitError_None, std::to_string(err).c_str());

	auto *display = static_cast<vr::IVRDisplayComponent *>(driver->GetComponent(vr::IVRDisplayComponent_Version3));
	if (display == NULL) {
		display = static_cast<vr::IVRDisplayComponent *>(driver->GetComponent(vr::IVRDisplayComponent_Version));
	}
	VERIFY(display, "IVRDisplayComponent is null");
#undef VERIFY

	auto hmd_parts = std::make_unique<HmdDevice::Parts>();
	hmd_parts->base.view_count = 2;
	for (size_t idx = 0; idx < 2; ++idx) {
		vr::EVREye eye = (idx == 0) ? vr::Eye_Left : vr::Eye_Right;
		xrt_view &view = hmd_parts->base.views[idx];

		display->GetEyeOutputViewport(eye, &view.viewport.x_pixels, &view.viewport.y_pixels,
		                              &view.viewport.w_pixels, &view.viewport.h_pixels);

		view.display.w_pixels = view.viewport.w_pixels;
		view.display.h_pixels = view.viewport.h_pixels;
		view.rot = u_device_rotation_ident;
	}

	hmd_parts->base.screens[0].w_pixels =
	    hmd_parts->base.views[0].display.w_pixels + hmd_parts->base.views[1].display.w_pixels;
	hmd_parts->base.screens[0].h_pixels = hmd_parts->base.views[0].display.h_pixels;
	// nominal frame interval will be set when lighthouse gives us the display frequency
	// see HmdDevice::handle_property_write

	hmd_parts->base.blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd_parts->base.blend_mode_count = 1;

	auto &distortion = hmd_parts->base.distortion;
	distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	for (size_t idx = 0; idx < 2; ++idx) {
		xrt_fov &fov = distortion.fov[idx];
		float tan_left, tan_right, tan_top, tan_bottom;
		display->GetProjectionRaw((vr::EVREye)idx, &tan_left, &tan_right, &tan_top, &tan_bottom);
		fov.angle_left = atanf(tan_left);
		fov.angle_right = atanf(tan_right);
		fov.angle_up = atanf(tan_bottom);
		fov.angle_down = atanf(tan_top);
	}

	hmd_parts->display = display;
	hmd->set_hmd_parts(std::move(hmd_parts));
	return true;
}

bool
Context::setup_controller(const char *serial, vr::ITrackedDeviceServerDriver *driver)
{
	// Find the first available slot for a new controller
	size_t device_idx = 0;
	for (; device_idx < MAX_CONTROLLERS; ++device_idx) {
		if (!controller[device_idx])
			break;
	}

	// Check if we've exceeded the maximum number of controllers
	if (device_idx == MAX_CONTROLLERS) {
		CTX_WARN("Attempted to activate more than %zu controllers - this is unsupported", MAX_CONTROLLERS);
		return false;
	}

	// Create the new controller
	controller[device_idx] = new ControllerDevice(
	    device_idx + 1, DeviceBuilder{this->shared_from_this(), driver, serial, STEAM_INSTALL_DIR});

	vr::EVRInitError err = driver->Activate(device_idx + 1);
	if (err != vr::VRInitError_None) {
		CTX_ERR("Activating controller failed: error %u", err);
		return false;
	}

	enum xrt_device_name name = controller[device_idx]->name;
	switch (name) {
	case XRT_DEVICE_VIVE_WAND:
		controller[device_idx]->binding_profiles = vive_binding_profiles_wand;
		controller[device_idx]->binding_profile_count = vive_binding_profiles_wand_count;
		break;

		break;
	case XRT_DEVICE_INDEX_CONTROLLER:
		controller[device_idx]->binding_profiles = vive_binding_profiles_index;
		controller[device_idx]->binding_profile_count = vive_binding_profiles_index_count;
		break;
	default: break;
	}

	return true;
}

void
Context::maybe_run_frame(uint64_t new_frame)
{
	if (new_frame > current_frame) {
		++current_frame;
		provider->RunFrame();
	}
}
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
bool
Context::TrackedDeviceAdded(const char *pchDeviceSerialNumber,
                            vr::ETrackedDeviceClass eDeviceClass,
                            vr::ITrackedDeviceServerDriver *pDriver)
{
	CTX_INFO("New device added: %s", pchDeviceSerialNumber);
	switch (eDeviceClass) {
	case vr::TrackedDeviceClass_HMD: {
		return setup_hmd(pchDeviceSerialNumber, pDriver);
		break;
	}
	case vr::TrackedDeviceClass_Controller: {
		return setup_controller(pchDeviceSerialNumber, pDriver);
		break;
	}
	case vr::TrackedDeviceClass_TrackingReference: {
		CTX_INFO("Found lighthouse device: %s", pchDeviceSerialNumber);
		return false;
	}
	case vr::TrackedDeviceClass_GenericTracker: {
		CTX_INFO("Found generic tracker device: %s", pchDeviceSerialNumber);
		return setup_controller(pchDeviceSerialNumber, pDriver);
		break;
	}
	default: {
		CTX_WARN("Attempted to add unsupported device class: %u", eDeviceClass);
		return false;
	}
	}
}

void
Context::TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	assert(sizeof(newPose) == unPoseStructSize);

	// Check for valid device index, allowing for the HMD plus up to 16 controllers
	if (unWhichDevice > 16)
		return;

	Device *dev = nullptr;

	// If unWhichDevice is 0, it refers to the HMD; otherwise, it refers to one of the controllers
	if (unWhichDevice == 0) {
		dev = static_cast<Device *>(this->hmd);
	} else {
		// unWhichDevice - 1 will give the index into the controller array
		dev = static_cast<Device *>(this->controller[unWhichDevice - 1]);
	}

	assert(dev);
	dev->update_pose(newPose);
}

void
Context::VsyncEvent(double vsyncTimeOffsetSeconds)
{}

void
Context::VendorSpecificEvent(uint32_t unWhichDevice,
                             vr::EVREventType eventType,
                             const vr::VREvent_Data_t &eventData,
                             double eventTimeOffset)
{}

bool
Context::IsExiting()
{
	return false;
}

void
Context::add_haptic_event(vr::VREvent_HapticVibration_t event)
{
	vr::VREvent_t e;
	e.eventType = vr::EVREventType::VREvent_Input_HapticVibration;
	e.trackedDeviceIndex = event.containerHandle - 1;
	vr::VREvent_Data_t d;
	d.hapticVibration = event;
	e.data = d;

	std::lock_guard lk(event_queue_mut);
	events.push_back({std::chrono::steady_clock::now(), e});
}

bool
Context::PollNextEvent(vr::VREvent_t *pEvent, uint32_t uncbVREvent)
{
	if (!events.empty()) {
		assert(sizeof(vr::VREvent_t) == uncbVREvent);
		Event e;
		{
			std::lock_guard lk(event_queue_mut);
			e = events.front();
			events.pop_front();
		}
		*pEvent = e.inner;
		using float_sec = std::chrono::duration<float>;
		float_sec event_age = std::chrono::steady_clock::now() - e.insert_time;
		pEvent->eventAgeSeconds = event_age.count();
		return true;
	}
	return false;
}

void
Context::GetRawTrackedDevicePoses(float fPredictedSecondsFromNow,
                                  vr::TrackedDevicePose_t *pTrackedDevicePoseArray,
                                  uint32_t unTrackedDevicePoseArrayCount)
{}

void
Context::RequestRestart(const char *pchLocalizedReason,
                        const char *pchExecutableToStart,
                        const char *pchArguments,
                        const char *pchWorkingDirectory)
{}

uint32_t
Context::GetFrameTimings(vr::Compositor_FrameTiming *pTiming, uint32_t nFrames)
{
	return 0;
}

void
Context::SetDisplayEyeToHead(uint32_t unWhichDevice,
                             const vr::HmdMatrix34_t &eyeToHeadLeft,
                             const vr::HmdMatrix34_t &eyeToHeadRight)
{
	hmd->SetDisplayEyeToHead(unWhichDevice, eyeToHeadLeft, eyeToHeadRight);
}

void
Context::SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t &eyeLeft, const vr::HmdRect2_t &eyeRight)
{}

void
Context::SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight)
{}

/***** IVRDriverInput methods *****/


vr::EVRInputError
Context::create_component_common(vr::PropertyContainerHandle_t container,
                                 const char *name,
                                 vr::VRInputComponentHandle_t *pHandle)
{
	*pHandle = vr::k_ulInvalidInputComponentHandle;
	Device *device = prop_container_to_device(container);
	if (!device) {
		return vr::VRInputError_InvalidHandle;
	}
	if (xrt_input *input = device->get_input_from_name(name); input) {
		CTX_DEBUG("creating component %s", name);
		vr::VRInputComponentHandle_t handle = new_handle();
		handle_to_input[handle] = input;
		*pHandle = handle;
	} else if (device != hmd) {
		auto *controller = static_cast<ControllerDevice *>(device);
		if (IndexFingerInput *finger = controller->get_finger_from_name(name); finger) {
			CTX_DEBUG("creating finger component %s", name);
			vr::VRInputComponentHandle_t handle = new_handle();
			handle_to_finger[handle] = finger;
			*pHandle = handle;
		}
	}
	return vr::VRInputError_None;
}

xrt_input *
Context::update_component_common(vr::VRInputComponentHandle_t handle,
                                 double offset,
                                 std::chrono::steady_clock::time_point now)
{
	xrt_input *input{nullptr};
	if (handle != vr::k_ulInvalidInputComponentHandle) {
		input = handle_to_input[handle];
		std::chrono::duration<double, std::chrono::seconds::period> offset_dur(offset);
		std::chrono::duration offset = (now + offset_dur).time_since_epoch();
		int64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(offset).count();
		input->active = true;
		input->timestamp = timestamp;
	}
	return input;
}

vr::EVRInputError
Context::CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer,
                                const char *pchName,
                                vr::VRInputComponentHandle_t *pHandle)
{
	return create_component_common(ulContainer, pchName, pHandle);
}

vr::EVRInputError
Context::UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset)
{
	xrt_input *input = update_component_common(ulComponent, fTimeOffset);
	if (input) {
		input->value.boolean = bNewValue;
	}
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer,
                               const char *pchName,
                               vr::VRInputComponentHandle_t *pHandle,
                               vr::EVRScalarType eType,
                               vr::EVRScalarUnits eUnits)
{
	std::string_view name{pchName};
	// Lighthouse gives thumbsticks/trackpads as x/y components,
	// we need to combine them for Monado
	auto end = name.back();
	auto second_last = name.at(name.size() - 2);
	if (second_last == '/' && (end == 'x' || end == 'y')) {
		Device *device = prop_container_to_device(ulContainer);
		if (!device) {
			return vr::VRInputError_InvalidHandle;
		}
		bool x = end == 'x';
		name.remove_suffix(2);
		std::string n(name);
		xrt_input *input = device->get_input_from_name(n);
		if (!input) {
			return vr::VRInputError_None;
		}

		// Create the component mapping if it hasn't been created yet
		Vec2Components *components =
		    vec2_input_to_components.try_emplace(input, new Vec2Components).first->second.get();

		vr::VRInputComponentHandle_t new_handle = this->new_handle();
		if (x)
			components->x = new_handle;
		else
			components->y = new_handle;

		handle_to_input[new_handle] = input;
		*pHandle = new_handle;
		return vr::VRInputError_None;
	}
	return create_component_common(ulContainer, pchName, pHandle);
}

vr::EVRInputError
Context::UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset)
{
	if (auto h = handle_to_input.find(ulComponent); h != handle_to_input.end() && h->second) {
		xrt_input *input = update_component_common(ulComponent, fTimeOffset);
		if (XRT_GET_INPUT_TYPE(input->name) == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {
			std::unique_ptr<Vec2Components> &components = vec2_input_to_components.at(input);
			if (components->x == ulComponent) {
				input->value.vec2.x = fNewValue;
			} else if (components->y == ulComponent) {
				input->value.vec2.y = fNewValue;
			} else {
				CTX_WARN(
				    "Attempted to update component with handle %lu"
				    " but it was neither the x nor y "
				    "component of its associated input",
				    ulComponent);
			}

		} else {
			input->value.vec1.x = fNewValue;
		}
	} else {
		if (ulComponent != vr::k_ulInvalidInputComponentHandle) {
			if (auto finger_input = handle_to_finger.find(ulComponent);
			    finger_input != handle_to_finger.end() && finger_input->second) {
				auto now = std::chrono::steady_clock::now();
				std::chrono::duration<double, std::chrono::seconds::period> offset_dur(fTimeOffset);
				std::chrono::duration offset = (now + offset_dur).time_since_epoch();
				int64_t timestamp =
				    std::chrono::duration_cast<std::chrono::nanoseconds>(offset).count();
				finger_input->second->timestamp = timestamp;
				finger_input->second->value = fNewValue;
			} else {
				CTX_WARN("Unmapped component %lu", ulComponent);
			}
		}
	}
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateHapticComponent(vr::PropertyContainerHandle_t ulContainer,
                               const char *pchName,
                               vr::VRInputComponentHandle_t *pHandle)
{
	*pHandle = vr::k_ulInvalidInputComponentHandle;
	Device *d = prop_container_to_device(ulContainer);
	if (!d) {
		return vr::VRInputError_InvalidHandle;
	}

	// Assuming HMDs won't have haptics.
	// Maybe a wrong assumption.
	if (d == hmd) {
		CTX_WARN("Didn't expect HMD with haptics.");
		return vr::VRInputError_InvalidHandle;
	}

	auto *device = static_cast<ControllerDevice *>(d);
	vr::VRInputComponentHandle_t handle = new_handle();
	handle_to_input[handle] = nullptr;
	device->set_haptic_handle(handle);
	*pHandle = handle;

	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateSkeletonComponent(vr::PropertyContainerHandle_t ulContainer,
                                 const char *pchName,
                                 const char *pchSkeletonPath,
                                 const char *pchBasePosePath,
                                 vr::EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
                                 const vr::VRBoneTransform_t *pGripLimitTransforms,
                                 uint32_t unGripLimitTransformCount,
                                 vr::VRInputComponentHandle_t *pHandle)
{
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::UpdateSkeletonComponent(vr::VRInputComponentHandle_t ulComponent,
                                 vr::EVRSkeletalMotionRange eMotionRange,
                                 const vr::VRBoneTransform_t *pTransforms,
                                 uint32_t unTransformCount)
{
	return vr::VRInputError_None;
}

/***** IVRProperties methods *****/

vr::ETrackedPropertyError
Context::ReadPropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                           vr::PropertyRead_t *pBatch,
                           uint32_t unBatchEntryCount)
{
	return vr::TrackedProp_Success;
}

vr::ETrackedPropertyError
Context::WritePropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                            vr::PropertyWrite_t *pBatch,
                            uint32_t unBatchEntryCount)
{
	Device *device = prop_container_to_device(ulContainerHandle);
	if (!device)
		return vr::TrackedProp_InvalidContainer;
	if (!pBatch)
		return vr::TrackedProp_InvalidOperation; // not verified vs steamvr
	device->handle_properties(pBatch, unBatchEntryCount);
	return vr::TrackedProp_Success;
}

const char *
Context::GetPropErrorNameFromEnum(vr::ETrackedPropertyError error)
{
	return nullptr;
}

Device *
Context::prop_container_to_device(vr::PropertyContainerHandle_t handle)
{
	switch (handle) {
	case 1: {
		return hmd;
		break;
	}
	default: {
		// If the handle corresponds to a controller
		if (handle >= 2 && handle <= 17) {
			return controller[handle - 2];
		} else {
			return nullptr;
		}
		break;
	}
	}
}

vr::PropertyContainerHandle_t
Context::TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t nDevice)
{
	size_t container = nDevice + 1;
	if (nDevice == 0 && this->hmd) {
		return container;
	}
	if (nDevice >= 1 && nDevice <= 16 && this->controller[nDevice - 1]) {
		return container;
	}

	return vr::k_ulInvalidPropertyContainer;
}

void
Context::Log(const char *pchLogMessage)
{
	CTX_TRACE("[lighthouse]: %s", pchLogMessage);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

xrt_result_t
get_roles(struct xrt_system_devices *xsysd, struct xrt_system_roles *out_roles)
{
	out_roles->left = svrs->left_index;
	out_roles->right = svrs->right_index;
	out_roles->gamepad = svrs->gamepad_index;
	out_roles->generation_id = 1;

	return XRT_SUCCESS;
}

void
destroy(struct xrt_system_devices *xsysd)
{
	u_system_devices_close(xsysd);
	free(svrs);
}

extern "C" enum xrt_result
steamvr_lh_create_devices(struct xrt_session_event_sink *broadcast,
                          struct xrt_system_devices **out_xsysd,
                          struct xrt_space_overseer **out_xso)
{
	u_logging_level level = debug_get_log_option_lh_log();
	// The driver likes to create a bunch of transient folder - lets make sure they're created where they normally
	// are.
	std::filesystem::current_path(STEAM_INSTALL_DIR + "/config/lighthouse");
	std::string steamvr{};
	if (getenv("STEAMVR_PATH") != nullptr) {
		steamvr = getenv("STEAMVR_PATH");
	} else {
		steamvr = find_steamvr_install();
	}

	if (steamvr.empty()) {
		U_LOG_IFL_E(level, "Could not find where SteamVR is installed!");
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_IFL_I(level, "Found SteamVR install: %s", steamvr.c_str());

	// TODO: support windows?
	auto driver_so = steamvr + "/drivers/lighthouse/bin/linux64/driver_lighthouse.so";

	void *lighthouse_lib = dlopen(driver_so.c_str(), RTLD_LAZY);
	if (!lighthouse_lib) {
		U_LOG_IFL_E(level, "Couldn't open lighthouse lib: %s", dlerror());
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	void *sym = dlsym(lighthouse_lib, "HmdDriverFactory");
	if (!sym) {
		U_LOG_IFL_E(level, "Couldn't find HmdDriverFactory in lighthouse lib: %s", dlerror());
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	using HmdDriverFactory_t = void *(*)(const char *, int *);
	auto factory = reinterpret_cast<HmdDriverFactory_t>(sym);

	vr::EVRInitError err = vr::VRInitError_None;
	auto *driver = static_cast<vr::IServerTrackedDeviceProvider *>(
	    factory(vr::IServerTrackedDeviceProvider_Version, (int *)&err));
	if (err != vr::VRInitError_None) {
		U_LOG_IFL_E(level, "Couldn't get tracked device driver: error %u", err);
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	svrs->ctx = Context::create(STEAM_INSTALL_DIR, steamvr, driver);

	err = driver->Init(svrs->ctx.get());
	if (err != vr::VRInitError_None) {
		U_LOG_IFL_E(level, "Lighthouse driver initialization failed: error %u", err);
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_IFL_I(level, "Lighthouse initialization complete, giving time to setup connected devices...");
	// RunFrame needs to be called to detect controllers
	using namespace std::chrono_literals;
	auto start_time = std::chrono::steady_clock::now();
	while (true) {
		driver->RunFrame();
		auto cur_time = std::chrono::steady_clock::now();
		if (cur_time - start_time >= 3s) {
			break;
		}
	}
	U_LOG_IFL_I(level, "Device search time complete.");

	if (out_xsysd == NULL || *out_xsysd != NULL) {
		U_LOG_IFL_E(level, "Invalid output system pointer");
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct xrt_system_devices *xsysd = NULL;
	xsysd = &svrs->base;

	xsysd->destroy = destroy;
	xsysd->get_roles = get_roles;

	// Do creation.
	// Devices to populate.
	struct xrt_device *head = NULL;
	struct xrt_device *left = NULL, *right = NULL;
	struct xrt_device *left_ht = NULL, *right_ht = NULL;

	svrs->head_index = -1;
	svrs->left_index = -1;
	svrs->right_index = -1;
	svrs->gamepad_index = -1;


	// Include the HMD
	if (svrs->ctx->hmd) {
		xsysd->xdevs[xsysd->xdev_count] = svrs->ctx->hmd;
		head = xsysd->xdevs[xsysd->xdev_count++]; // Always have a head at index 0 and iterate dev count.
		xsysd->static_roles.head = head;
	}

	// Include the controllers (up to 16)
	for (int i = 0; i < 16; i++) {
		if (svrs->ctx->controller[i]) {
			xsysd->xdevs[xsysd->xdev_count] = svrs->ctx->controller[i];
			svrs->controller_to_xdev_map[i] = xsysd->xdev_count++;
		}
	}

	u_device_assign_xdev_roles(xsysd->xdevs, xsysd->xdev_count, &svrs->head_index, &svrs->left_index,
	                           &svrs->right_index);

	if (svrs->left_index >= 0) {
		left = xsysd->xdevs[svrs->left_index];
		left_ht = u_system_devices_get_ht_device_left(xsysd);
		xsysd->static_roles.hand_tracking.left = left_ht;
	}

	if (svrs->right_index >= 0) {
		right = xsysd->xdevs[svrs->right_index];
		right_ht = u_system_devices_get_ht_device_right(xsysd);
		xsysd->static_roles.hand_tracking.right = right_ht;
	}

	if (!head) {
		U_LOG_IFL_E(level, "Unable to find HMD");
		destroy(xsysd);
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	*out_xsysd = xsysd;

	u_builder_create_space_overseer_legacy( //
	    broadcast,                          // broadcast
	    head,                               // head
	    left,                               // left
	    right,                              // right
	    xsysd->xdevs,                       // xdevs
	    xsysd->xdev_count,                  // xdev_count
	    false,                              // root_is_unbounded
	    out_xso);                           // out_xso

	return xrt_result::XRT_SUCCESS;
}
