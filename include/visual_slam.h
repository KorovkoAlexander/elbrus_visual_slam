#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "elbrus.h"
#include "vector"
namespace py = pybind11;

enum ElbrusStatus {
    ElbrusSuccess,
    ElbrusTrackingLost,
    ElbrusInvalidArg,
    ElbrusOutOfMemory,
    ElbrusGenericError,
    ElbrusUnsupportedNumberOfCameras,
    ElbrusSLAMIsNotInitialized,
    ElbrusNotImplemented,
    ElbrusReadingSLAMInternalsDisabled
};

enum VoState {
    VoStateUnknown,
    VoStateSuccess,
    VoStateFailed,
    VoStateInvalidated
};

enum IntegratorState {
    IntegratorStateUnknown,
    IntegratorStateStatic,
    IntegratorStateInertial,
    IntegratorStateImu
};

enum DistortionModel {
    Brown5k,
    Pinhole,
    Fisheye4
};

enum SBAMode {
    SBA_ON_GPU,  /*Run sparse bundle adjustment on GPU. This is a default option.*/
    SBA_ON_CPU,  /*Run sparse bundle adjustment on CPU*/
    SBA_OFF      /*Disable sparse bundle adjustment*/
};

struct Pose
{
    py::array_t<float, py::array::c_style> r; // 3x3
    py::array_t<float, py::array::c_style> t; // 3
};

struct PoseEstimate
{
    Pose pose;

    /* Pose timestamp in nanoseconds. */
    int64_t timestamp_ns;

    py::array_t<float, py::array::c_style> covariance;  // 6*6

    /*
    State of vo tracking
    See #define ELBRUS_VO_TRACKER_STATE_*
    */
    VoState vo_state;
    /*
    State of the integrator (IMU)
    See #define ELBRUS_INTEGRATOR_STATE_*
    */
    IntegratorState integrator_state;
};

struct ImuMeasurement
{
    py::array_t<float, py::array::c_style> linear_accelerations; // 3; in meters per squared second
    py::array_t<float, py::array::c_style> angular_velocities;   // 3; in radians per second
};

struct Camera
{
    DistortionModel distortion_model;
    std::vector<float> parameters;
    int32_t width;
    int32_t height;

    /* Transformation from the coordinate frame of the camera
     * to the coordinate frame of the rig.
     */
    Pose pose; // TODO: why ?
};

struct Configuration
{
    Configuration();
    /* Enable internal pose prediction mechanism based on a kinematic model.
     *
     * If frame rate is high enough it improves tracking performance
     * and stability.
     *
     * Prediction passed into `ELBRUS_TrackStereoSync` overrides prediction
     * from the kinematic model.
     *
     * As a general rule it is better to use a pose prediction mechanism
     * tailored to a specific application. If you have an IMU, consider using
     * it to provide pose predictions to Elbrus.
     *
     */
    bool use_motion_model = true;

    /* Enable image denoising.
     * Disable if the input images have already passed through a denoising
     * filter.
     */
    bool use_denoising = false;

    /* Enable feature tracking using GPU.
     */
    bool use_gpu = true;

    /* Enable sparse bundle adjustment. This option reduces the drift at the cost of increased
     * hardware utilization.
     */
    SBAMode sba_mode = SBA_ON_GPU;

    /* Enable fast and robust left-to-right tracking for rectified
     * cameras with principal points on the horizontal line.
     */
    bool horizontal_stereo_camera = true;

    /* If IMU present this is left camera to imu transformation.
     * vImu = imu_from_left * vLeft;
     *      vImu - vector in imu coordinate system
     *      vLeft - vector in left eye coordinate system
     */
    Pose imu_from_left; //TODO: default identity


    /* Gravitational acceleration that is used for the IMU integration,
     * defined in meters per sec^2.
     * The vector is defined in the world coordinate system:
     *   Cameras are always looking in the negative z direction.
     *   Y is "up", X is from "left" to "right"
     * For example: <0, -9.81, 0> is good enough for the most applications. */
    std::vector<float> g = {0.f, -9.81f, 0.f};

    /*
     * Allow to call ELBRUS_GetLastLeftObservations
     */
    bool enable_observations_export = false;
    /*
     * Allow to call ELBRUS_GetLastLandmarks
     */
    bool enable_landmarks_export = false;

    /*
     * Use localization and mapping
     */
    bool enable_localization_n_mapping = false;

    /*
     * Size of map cell. Default is 0 (the size will be calculated from the camera baseline)
     */
    float map_cell_size = 0;

    /*
     * If localization and mapping is used:
     * sync mode (same thread with visual odometry). Default: slam_sync_mode = 0
     */
    bool slam_sync_mode = false;

    /*
     * Enable reading internal data from SLAM
     * ELBRUS_EnableReadingDataLayer(), ELBRUS_DisableReadingDataLayer()
     */
    bool enable_reading_slam_internals = false;

    /*
     * Set directory where the dump files will be saved:
     *   stereo.edex - cameras and configuration
     *   cam0.00000.png, cam1.00000.png, ... - input images
     * example:
     *    cfg->debug_dump_directory = "/tmp/elbrus"
     */
    std::string debug_dump_directory = "";

    /*
     * Enable IMU integrator
     */
    bool enable_imu_integrator = false;
};

struct Image
{
    py::array_t<uint8_t, py::array::c_style> pixels;

    /* Elbrus preserves timestamps: pose timestamp will match image timestamp
     * Time must be in nanoseconds.
     */
    int64_t timestamp_ns;
    /* index of the camera in the rig */
    int32_t camera_index;
};

struct PoseStamped {
    int64_t timestamp_ns;
    Pose pose;
};

struct Observation
{
    int32_t id;
    float u;    // 0 <= u < image width
    float v;    // 0 <= v < image height
};

struct Landmark
{
    int64_t id;
    // coordinates in the camera space
    float x;
    float y;
    float z;
};

struct Gravity
{
    // coordinates in the left camera space
    float x;
    float y;
    float z;
};

class ElbrusTracker {
 public:
    ElbrusTracker(const std::vector<Camera>& cameras,
                  const Configuration& cfg);

    bool register_imu_measurement(int64_t timestamp, const ImuMeasurement& imu);

    std::tuple<ElbrusStatus, Pose, PoseEstimate> track(const std::vector<Image>& images);

    std::pair<ElbrusStatus, Pose> get_odometry_pose();

    std::pair<ElbrusStatus, Pose> get_slam_pose();

    std::vector<PoseStamped> get_all_poses(uint32_t max_poses_count);

    std::pair<ElbrusStatus, std::vector<Observation>> get_last_left_observations(uint32_t num_observations);
    std::pair<ElbrusStatus, std::vector<Landmark>> get_last_landmarks(uint32_t num_landmarks);
    std::pair<ElbrusStatus, Gravity> get_last_gravity();

    ~ElbrusTracker();
 private:
    ELBRUS_TrackerHandle tracker_handle_;
};