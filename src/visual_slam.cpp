#include "visual_slam.h"

namespace {
ELBRUS_Pose to_elbrus(const Pose& pose) {
    ELBRUS_Pose e_pose;

    py::buffer_info r_info = pose.r.request();
    py::buffer_info t_info = pose.t.request();
    std::memcpy(e_pose.r, r_info.ptr, 9 * sizeof(float));
    std::memcpy(e_pose.t, t_info.ptr, 3 * sizeof(float));
    return e_pose;
}

Pose from_elbrus(const ELBRUS_Pose& e_pose) {
    Pose pose;

    py::buffer_info r_info = pose.r.request();
    py::buffer_info t_info = pose.t.request();
    std::memcpy(r_info.ptr, e_pose.r, 9 * sizeof(float));
    std::memcpy(t_info.ptr, e_pose.t, 3 * sizeof(float));
    return pose;
}

ELBRUS_PoseEstimate to_elbrus(const PoseEstimate& estimate) {
    ELBRUS_PoseEstimate e_estimate;
    e_estimate.pose = to_elbrus(estimate.pose);
    e_estimate.timestamp_ns = estimate.timestamp_ns;

    py::buffer_info info = estimate.covariance.request();
    std::memcpy(e_estimate.covariance, info.ptr, 6 * 6 * sizeof(float));

    switch (estimate.vo_state) {
        case VoStateUnknown:
            e_estimate.vo_state = ELBRUS_VO_TRACKER_STATE_UNKNOWN;
            break;
        case VoStateSuccess:
            e_estimate.vo_state = ELBRUS_VO_TRACKER_STATE_SUCCESS;
            break;
        case VoStateFailed:
            e_estimate.vo_state = ELBRUS_VO_TRACKER_STATE_FAILED;
            break;
        case VoStateInvalidated:
            e_estimate.vo_state = ELBRUS_VO_TRACKER_STATE_INVALIDATED;
            break;
        default:
            throw std::runtime_error("Invalid VoState");
    }

    switch (estimate.integrator_state) {
        case IntegratorStateUnknown:
            e_estimate.integrator_state = ELBRUS_INTEGRATOR_STATE_UNKNOWN;
            break;
        case IntegratorStateStatic:
            e_estimate.integrator_state = ELBRUS_INTEGRATOR_STATE_STATIC;
            break;
        case IntegratorStateInertial:
            e_estimate.integrator_state = ELBRUS_INTEGRATOR_STATE_INERTIAL;
            break;
        case IntegratorStateImu:
            e_estimate.integrator_state = ELBRUS_INTEGRATOR_STATE_IMU;
            break;
        default:
            throw std::runtime_error("Invalid VoState");
    }
    return e_estimate;
}

PoseEstimate from_elbrus(const ELBRUS_PoseEstimate& e_estimate) {
    PoseEstimate estimate;
    estimate.pose = from_elbrus(e_estimate.pose);
    estimate.timestamp_ns = e_estimate.timestamp_ns;

    py::buffer_info info = estimate.covariance.request();
    std::memcpy(info.ptr, e_estimate.covariance, 6 * 6 * sizeof(float));

    if (e_estimate.vo_state == ELBRUS_VO_TRACKER_STATE_UNKNOWN) {
        estimate.vo_state = VoStateUnknown;
    } else if (e_estimate.vo_state == ELBRUS_VO_TRACKER_STATE_SUCCESS) {
        estimate.vo_state = VoStateSuccess;
    } else if (e_estimate.vo_state == ELBRUS_VO_TRACKER_STATE_FAILED) {
        estimate.vo_state = VoStateFailed;
    } else if (e_estimate.vo_state == ELBRUS_VO_TRACKER_STATE_INVALIDATED) {
        estimate.vo_state = VoStateInvalidated;
    } else {
        throw std::runtime_error("Invalid VO State");
    }

    if (e_estimate.integrator_state == ELBRUS_INTEGRATOR_STATE_UNKNOWN) {
        estimate.integrator_state = IntegratorStateUnknown;
    } else if (e_estimate.integrator_state == ELBRUS_INTEGRATOR_STATE_STATIC) {
        estimate.integrator_state = IntegratorStateStatic;
    } else if (e_estimate.integrator_state == ELBRUS_INTEGRATOR_STATE_INERTIAL) {
        estimate.integrator_state = IntegratorStateInertial;
    } else if (e_estimate.integrator_state == ELBRUS_INTEGRATOR_STATE_IMU) {
        estimate.integrator_state = IntegratorStateImu;
    } else {
        throw std::runtime_error("Invalid VoState");
    }
    return estimate;
}

ELBRUS_ImuMeasurement to_elbrus(const ImuMeasurement& imu) {
    ELBRUS_ImuMeasurement e_imu;
    py::buffer_info lin_info = imu.linear_accelerations.request();
    py::buffer_info ang_info = imu.angular_velocities.request();
    std::memcpy(e_imu.linear_accelerations, lin_info.ptr, 3 * sizeof(float));
    std::memcpy(e_imu.angular_velocities, ang_info.ptr, 3 * sizeof(float));
    return e_imu;
}

ELBRUS_Camera to_elbrus(const Camera& cam) {
    ELBRUS_Camera e_cam;
    switch (cam.distortion_model) {
        case Brown5k:
            e_cam.distortion_model = "brown5k";
            // 9 parameters
            if (cam.parameters.size() != 9) {
                throw std::runtime_error("Brown model has to contain 9 parameters");
            }
            e_cam.num_parameters = 9;
            break;
        case Pinhole:
            e_cam.distortion_model = "pinhole";
            // 4 parameters
            if (cam.parameters.size() != 4) {
                throw std::runtime_error("Pinhole model has to contain 4 parameters");
            }
            e_cam.num_parameters = 4;
            break;
        case Fisheye4:
            e_cam.distortion_model = "fisheye4";
            if (cam.parameters.size() != 8) {
                throw std::runtime_error("Fisheye4 model has to contain 4 parameters");
            }
            e_cam.num_parameters = 8;
            break;
        default:
            throw std::runtime_error("Unexpected distortion model");
            break;
    }
    e_cam.parameters = cam.parameters.data();
    e_cam.width = cam.width;
    e_cam.height = cam.height;
    e_cam.pose = to_elbrus(cam.pose); // TODO : do i need it?
    return e_cam;
}

ELBRUS_Configuration to_elbrus(const Configuration& conf) {
    ELBRUS_Configuration e_conf;

    e_conf.use_motion_model = static_cast<int32_t>(conf.use_motion_model);
    e_conf.use_denoising = static_cast<int32_t>(conf.use_denoising);
    e_conf.use_gpu = static_cast<int32_t>(conf.use_gpu);
    switch (conf.sba_mode) {
        case SBA_ON_GPU:
            e_conf.sba_mode = ELBRUS_SBA_ON_GPU;
            break;
        case SBA_ON_CPU:
            e_conf.sba_mode = ELBRUS_SBA_ON_CPU;
            break;
        default :
            e_conf.sba_mode = ELBRUS_SBA_OFF;
            break;
    }
    e_conf.horizontal_stereo_camera = static_cast<int32_t>(conf.horizontal_stereo_camera);
    e_conf.imu_from_left = to_elbrus(conf.imu_from_left);
    e_conf.g[0] = conf.g[0];
    e_conf.g[1] = conf.g[1];
    e_conf.g[2] = conf.g[2];
    e_conf.enable_observations_export = static_cast<int32_t>(conf.enable_observations_export);
    e_conf.enable_landmarks_export = static_cast<int32_t>(conf.enable_landmarks_export);
    e_conf.enable_localization_n_mapping = static_cast<int32_t>(conf.enable_localization_n_mapping);
    e_conf.map_cell_size = conf.map_cell_size;
    e_conf.slam_sync_mode = static_cast<int32_t>(conf.slam_sync_mode);
    e_conf.enable_reading_slam_internals = static_cast<int32_t>(conf.enable_reading_slam_internals);
    e_conf.debug_dump_directory = conf.debug_dump_directory.c_str();
    e_conf.enable_imu_integrator = static_cast<int32_t>(conf.enable_imu_integrator);

    return e_conf;
}

Configuration from_elbrus(const ELBRUS_Configuration& e_conf) {
    Configuration conf;

    conf.use_motion_model = static_cast<bool>(e_conf.use_motion_model);
    conf.use_denoising = static_cast<bool>(e_conf.use_denoising);
    conf.use_gpu = static_cast<bool>(e_conf.use_gpu);

    if (e_conf.sba_mode == ELBRUS_SBA_ON_GPU) {
        conf.sba_mode = SBA_ON_GPU;
    } else if (e_conf.sba_mode == ELBRUS_SBA_ON_CPU) {
        conf.sba_mode = SBA_ON_CPU;
    } else {
        conf.sba_mode = SBA_ON_GPU;
    }

    conf.horizontal_stereo_camera = static_cast<bool>(e_conf.horizontal_stereo_camera);
    conf.imu_from_left = from_elbrus(e_conf.imu_from_left);
    conf.g[0] = e_conf.g[0];
    conf.g[1] = e_conf.g[1];
    conf.g[2] = e_conf.g[2];
    conf.enable_observations_export = static_cast<bool>(e_conf.enable_observations_export);
    conf.enable_landmarks_export = static_cast<bool>(e_conf.enable_landmarks_export);
    conf.enable_localization_n_mapping = static_cast<bool>(e_conf.enable_localization_n_mapping);
    conf.map_cell_size = e_conf.map_cell_size;
    conf.slam_sync_mode = static_cast<bool>(e_conf.slam_sync_mode);
    conf.enable_reading_slam_internals = static_cast<bool>(e_conf.enable_reading_slam_internals);
    conf.debug_dump_directory = e_conf.debug_dump_directory;
    conf.enable_imu_integrator = static_cast<bool>(e_conf.enable_imu_integrator);

    return conf;
}

std::string to_string(ELBRUS_Status status) {
    if (status == ELBRUS_SUCCESS) {
        return "ElbrusSuccess";
    } else if (status == ELBRUS_TRACKING_LOST) {
        return "ElbrusTrackingLost";
    } else if (status == ELBRUS_INVALID_ARG) {
        return "ElbrusInvalidArg";
    } else if (status == ELBRUS_OUT_OF_MEMORY) {
        return "ElbrusOutOfMemory";
    } else if (status == ELBRUS_GENERIC_ERROR) {
        return "ElbrusGenericError";
    } else if (status == ELBRUS_UNSUPPORTED_NUMBER_OF_CAMERAS) {
        return "ElbrusUnsupportedNumberOfCameras";
    } else if (status == ELBRUS_SLAM_IS_NOT_INITIALIZED) {
        return "ElbrusSLAMIsNotInitialized";
    } else if (status == ELBRUS_NOT_IMPLEMENTED) {
        return "ElbrusNotImplemented";
    } else if (status == ELBRUS_READING_SLAM_INTERNALS_DISABLED) {
        return "ElbrusReadingSLAMInternalsDisabled";
    }
    return "Unexpected error";
}

ElbrusStatus from_elbrus(ELBRUS_Status status) {
    if (status == ELBRUS_SUCCESS) {
        return ElbrusSuccess;
    } else if (status == ELBRUS_TRACKING_LOST) {
        return ElbrusTrackingLost;
    } else if (status == ELBRUS_INVALID_ARG) {
        return ElbrusInvalidArg;
    } else if (status == ELBRUS_OUT_OF_MEMORY) {
        return ElbrusOutOfMemory;
    } else if (status == ELBRUS_GENERIC_ERROR) {
        return ElbrusGenericError;
    } else if (status == ELBRUS_UNSUPPORTED_NUMBER_OF_CAMERAS) {
        return ElbrusUnsupportedNumberOfCameras;
    } else if (status == ELBRUS_SLAM_IS_NOT_INITIALIZED) {
        return ElbrusSLAMIsNotInitialized;
    } else if (status == ELBRUS_NOT_IMPLEMENTED) {
        return ElbrusNotImplemented;
    } else if (status == ELBRUS_READING_SLAM_INTERNALS_DISABLED) {
        return ElbrusReadingSLAMInternalsDisabled;
    }
    return ElbrusGenericError;
}

ELBRUS_Image to_elbrus(const Image& image) {
    ELBRUS_Image e_image;
    py::buffer_info image_info = image.pixels.request();
    e_image.pixels = reinterpret_cast<uint8_t*>(image_info.ptr);
    assert(image_info.ndim == 2);
    e_image.width = static_cast<int32_t>(image_info.shape[0]);
    e_image.height = static_cast<int32_t>(image_info.shape[1]);
    e_image.timestamp_ns = image.timestamp_ns;
    e_image.camera_index = image.camera_index;
    return e_image;
}

}

Configuration::Configuration() {
    use_motion_model = true;
    use_denoising = false;
    use_gpu = true;
    sba_mode = SBA_ON_GPU;
    horizontal_stereo_camera = true;

    for (int i = 0; i < 9; ++i) {
       imu_from_left.r.mutable_at(i) = 0.f;
    }
    for (int i = 0; i < 3; ++i) {
        imu_from_left.r.mutable_at(i * 3 + i) = 1.f;  // set 1. to diagonal
        imu_from_left.t.mutable_at(i) = 0.f;
    }

    g = {0.f, -9.81f, 0.f};
    enable_observations_export = false;
    enable_landmarks_export = false;
    enable_localization_n_mapping = false;
    map_cell_size = 0;
    slam_sync_mode = false;
    enable_reading_slam_internals = false;
    debug_dump_directory = "";
    enable_imu_integrator = false;
}

ElbrusTracker::ElbrusTracker(const std::vector<Camera>& cameras,
                             const Configuration& cfg) {

    std::vector<ELBRUS_Camera> elbrus_cams;
    elbrus_cams.reserve(cameras.size());
    for (const auto& cam: cameras) {
        elbrus_cams.emplace_back(to_elbrus(cam));
    }
    ELBRUS_CameraRig rig = {&elbrus_cams[0], static_cast<int32_t>(cameras.size())};
    ELBRUS_Configuration e_cfg = to_elbrus(cfg);
    ELBRUS_Status s = ELBRUS_CreateTracker(&tracker_handle_,&rig,&e_cfg);
    if (s != ElbrusSuccess) {
        throw std::runtime_error(to_string(s));
    }
}

bool ElbrusTracker::register_imu_measurement(int64_t timestamp, const ImuMeasurement& imu) {
    auto e_imu = to_elbrus(imu);
    return ELBRUS_RegisterImuMeasurement(tracker_handle_, timestamp, &e_imu);
}

std::tuple<ElbrusStatus, Pose, PoseEstimate> ElbrusTracker::track(const std::vector<Image>& images) {
    std::vector<ELBRUS_Image> e_images;
    e_images.reserve(images.size());
    for (const auto& i: images) {
        e_images.emplace_back(to_elbrus(i));
    }
    ELBRUS_Pose pose;
    ELBRUS_PoseEstimate estimate;

    ELBRUS_Status s = ELBRUS_Track(tracker_handle_,
                                   e_images.data(),
                                   &pose,
                                   &estimate);
    auto status = from_elbrus(s);
    auto out_pose = from_elbrus(pose);
    auto out_estimate = from_elbrus(estimate);
    return {status, out_pose, out_estimate};
}

std::pair<ElbrusStatus, Pose> ElbrusTracker::get_odometry_pose() {
    ELBRUS_Pose elbrus_pose;
    ELBRUS_Status s = ELBRUS_GetOdometryPose(tracker_handle_,&elbrus_pose);
    return {from_elbrus(s), from_elbrus(elbrus_pose)};
}

std::pair<ElbrusStatus, Pose> ElbrusTracker::get_slam_pose() {
    ELBRUS_PoseSlam elbrus_slam_pose;
    ELBRUS_Status s = ELBRUS_GetSlamPose(tracker_handle_,&elbrus_slam_pose);
    return {from_elbrus(s), from_elbrus(elbrus_slam_pose.pose)};
}

std::vector<PoseStamped> ElbrusTracker::get_all_poses(uint32_t max_poses_count) {
    std::vector<ELBRUS_PoseStamped> poses_stamped;
    poses_stamped.resize(max_poses_count);

    uint32_t len = ELBRUS_GetAllPoses(tracker_handle_,
                                         max_poses_count,
                                         poses_stamped.data());
    std::vector<PoseStamped> output;
    output.reserve(len);
    for (int i = 0; i < len; i++) {
        output.push_back({
            poses_stamped[i].timestamp_ns,
            from_elbrus(poses_stamped[i].pose)
        });
    }
    return output;
}

std::pair<ElbrusStatus, std::vector<Observation>> ElbrusTracker::get_last_left_observations(uint32_t num_observations) {
    std::vector<ELBRUS_Observation> observations;
    observations.resize(num_observations);
    ELBRUS_ObservationVector vect = {
        0, num_observations, observations.data()
    };
    ELBRUS_Status s = ELBRUS_GetLastLeftObservations(tracker_handle_, &vect);

    std::vector<Observation> output;
    output.reserve(vect.num);
    for (int i = 0; i < vect.num; i++) {
        output.push_back({
            observations[i].id,
            observations[i].u,
            observations[i].v
        });
    }
    return {from_elbrus(s), output};

}
std::pair<ElbrusStatus, std::vector<Landmark>> ElbrusTracker::get_last_landmarks(uint32_t num_landmarks) {
    std::vector<ELBRUS_Landmark> landmarks;
    landmarks.resize(num_landmarks);
    ELBRUS_LandmarkVector vect = {
        0, num_landmarks, landmarks.data()
    };
    ELBRUS_Status s = ELBRUS_GetLastLandmarks(tracker_handle_, &vect);

    std::vector<Landmark> output;
    output.reserve(vect.num);
    for (int i = 0; i < vect.num; i++) {
        output.push_back({
            landmarks[i].id,
            landmarks[i].x,
            landmarks[i].y,
            landmarks[i].z
        });
    }
    return {from_elbrus(s), output};
}
std::pair<ElbrusStatus, Gravity> ElbrusTracker::get_last_gravity() {
    ELBRUS_Gravity elbrus_g;
    ELBRUS_Status s = ELBRUS_GetLastGravity(tracker_handle_, &elbrus_g);
    Gravity g = {
        elbrus_g.x,
        elbrus_g.y,
        elbrus_g.z,
    };
    return {from_elbrus(s), g};
}

ElbrusTracker::~ElbrusTracker() {
    ELBRUS_DestroyTracker(tracker_handle_);
}

PYBIND11_MODULE(elbrus_visual_slam, m){
    py::enum_<ElbrusStatus >(m, "ElbrusStatus")
        .value("ElbrusSuccess",ElbrusSuccess)
        .value("ElbrusTrackingLost",ElbrusTrackingLost)
        .value("ElbrusInvalidArg",ElbrusInvalidArg)
        .value("ElbrusOutOfMemory",ElbrusOutOfMemory)
        .value("ElbrusGenericError",ElbrusGenericError)
        .value("ElbrusUnsupportedNumberOfCameras",ElbrusUnsupportedNumberOfCameras)
        .value("ElbrusSLAMIsNotInitialized",ElbrusSLAMIsNotInitialized)
        .value("ElbrusNotImplemented",ElbrusNotImplemented)
        .value("ElbrusReadingSLAMInternalsDisabled",ElbrusReadingSLAMInternalsDisabled)
        .export_values();
    py::enum_<VoState>(m, "VoState")
        .value("VoStateUnknown",VoStateUnknown)
        .value("VoStateSuccess",VoStateSuccess)
        .value("VoStateFailed",VoStateFailed)
        .value("VoStateInvalidated",VoStateInvalidated)
        .export_values();
    py::enum_<IntegratorState>(m, "IntegratorState")
        .value("IntegratorStateUnknown",IntegratorStateUnknown)
        .value("IntegratorStateStatic",IntegratorStateStatic)
        .value("IntegratorStateInertial",IntegratorStateInertial)
        .value("IntegratorStateImu",IntegratorStateImu)
        .export_values();
    py::enum_<DistortionModel>(m, "DistortionModel")
        .value("Brown5k",Brown5k)
        .value("Pinhole",Pinhole)
        .value("Fisheye4",Fisheye4)
        .export_values();
    py::enum_<SBAMode>(m, "SBAMode")
        .value("SBA_ON_GPU",SBA_ON_GPU)
        .value("SBA_ON_CPU",SBA_ON_CPU)
        .value("SBA_OFF",SBA_OFF)
        .export_values();

//    m.def("convertONNX", &convertONNX, "convert ONNX model into engine file",
//          py::arg("modelFile"),
//          py::arg("file_list") = "",
//          py::arg("scale") = std::tuple<float, float, float>(58.395, 57.12 , 57.375),
//          py::arg("shift") = std::tuple<float, float, float>(123.675, 116.28 , 103.53),
//          py::arg("max_batch_size") = 1,
//          py::arg("allowGPUFallback") = true,
//          py::arg("device") = DEVICE_GPU,
//          py::arg("precision") = TYPE_FP32,
//          py::arg("format") = RGB,
//          py::arg("logs_path") = "");

    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("rotation_matrix", &Pose::r)
        .def_readwrite("translation_vector", &Pose::t);

    py::class_<PoseEstimate>(m, "PoseEstimate")
        .def(py::init<>())
        .def_readwrite("pose", &PoseEstimate::pose)
        .def_readwrite("timestamp_ns", &PoseEstimate::timestamp_ns)
        .def_readwrite("covariance_matrix", &PoseEstimate::covariance)
        .def_readwrite("vo_state", &PoseEstimate::vo_state)
        .def_readwrite("integrator_state", &PoseEstimate::integrator_state);

    py::class_<ImuMeasurement>(m, "ImuMeasurement")
        .def(py::init<>())
        .def_readwrite("linear_accelerations", &ImuMeasurement::linear_accelerations)
        .def_readwrite("angular_velocities", &ImuMeasurement::angular_velocities);

    py::class_<Camera>(m, "Camera")
        .def(py::init<>())
        .def_readwrite("distortion_model", &Camera::distortion_model)
        .def_readwrite("parameters", &Camera::parameters)
        .def_readwrite("width", &Camera::width)
        .def_readwrite("height", &Camera::height)
        .def_readwrite("pose", &Camera::pose);

    py::class_<Configuration>(m, "Configuration")
        .def(py::init<>())
        .def_readwrite("use_motion_model", &Configuration::use_motion_model)
        .def_readwrite("use_denoising", &Configuration::use_denoising)
        .def_readwrite("use_gpu", &Configuration::use_gpu)
        .def_readwrite("sba_mode", &Configuration::sba_mode)
        .def_readwrite("horizontal_stereo_camera", &Configuration::horizontal_stereo_camera)
        .def_readwrite("imu_from_left", &Configuration::imu_from_left)
        .def_readwrite("g", &Configuration::g)
        .def_readwrite("enable_observations_export", &Configuration::enable_observations_export)
        .def_readwrite("enable_landmarks_export", &Configuration::enable_landmarks_export)
        .def_readwrite("enable_localization_n_mapping", &Configuration::enable_localization_n_mapping)
        .def_readwrite("map_cell_size", &Configuration::map_cell_size)
        .def_readwrite("slam_sync_mode", &Configuration::slam_sync_mode)
        .def_readwrite("enable_reading_slam_internals", &Configuration::enable_reading_slam_internals)
        .def_readwrite("debug_dump_directory", &Configuration::debug_dump_directory)
        .def_readwrite("enable_imu_integrator", &Configuration::enable_imu_integrator);

    py::class_<Image>(m, "Image")
        .def(py::init<>())
        .def_readwrite("pixels", &Image::pixels)
        .def_readwrite("timestamp_ns", &Image::timestamp_ns)
        .def_readwrite("camera_index", &Image::camera_index);

    py::class_<PoseStamped>(m, "PoseStamped")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &PoseStamped::timestamp_ns)
        .def_readwrite("pose", &PoseStamped::pose);

    py::class_<Observation>(m, "Observation")
        .def(py::init<>())
        .def_readwrite("id", &Observation::id)
        .def_readwrite("u", &Observation::u)
        .def_readwrite("v", &Observation::v);

    py::class_<Landmark>(m, "Landmark")
        .def(py::init<>())
        .def_readwrite("id", &Landmark::id)
        .def_readwrite("x", &Landmark::x)
        .def_readwrite("y", &Landmark::y)
        .def_readwrite("z", &Landmark::z);

    py::class_<Gravity>(m, "Gravity")
        .def(py::init<>())
        .def_readwrite("x", &Gravity::x)
        .def_readwrite("y", &Gravity::y)
        .def_readwrite("z", &Gravity::z);

    py::class_<ElbrusTracker>(m, "ElbrusTracker")
        .def(py::init<const std::vector<Camera>&, const Configuration&>(),
             py::arg("cameras"),
             py::arg("cfg"))
        .def("register_imu_measurement", &ElbrusTracker::register_imu_measurement)
        .def("track", &ElbrusTracker::track)
        .def("get_odometry_pose", &ElbrusTracker::get_odometry_pose)
        .def("get_slam_pose", &ElbrusTracker::get_slam_pose)
        .def("get_all_poses", &ElbrusTracker::get_all_poses)
        .def("get_last_left_observations", &ElbrusTracker::get_last_left_observations)
        .def("get_last_landmarks", &ElbrusTracker::get_last_landmarks)
        .def("get_last_gravity", &ElbrusTracker::get_last_gravity);
}