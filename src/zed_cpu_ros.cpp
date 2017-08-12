/*
 * A simple ZED camera driver which only uses the CPU
 * and doesn't need the official StereoLabs drivers
 * only publishes the left and right images and the camera info via ROS
 *
 * author: Michael Grupp
 * additions: C++11 constructs, Kalibr YAML file support, various parameters
 *
 * fork of Di Zeng's original version (https://github.com/willdzeng/zed_cpu_ros)
 */

#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <iterator>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// ROS dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <zed_cpu_ros/zed_cpu_ros_DynConfig.h>  // automatically generated in devel from cfg/...

// frame filter
#include "tearing_filter.hpp"



/**
 * @brief identifiers for supported ZED stereo camera resolutions
 */
enum class ZedResoID {
    TWO_K = 0,
    FHD = 1,
    HD = 2,
    VGA = 3
};

/**
 * @brief map for the (width, height) pairs for each ZedResoID
 */
const std::map<ZedResoID, std::pair<unsigned int, unsigned int>> reso_values = {
        {ZedResoID::TWO_K, {4416, 1242}},
        {ZedResoID::FHD, {3840, 1080}},
        {ZedResoID::HD, {2560, 720}},
        {ZedResoID::VGA, {1344, 376}},
};

/**
 * @brief basic ZED stereo camera driver
 */
class ZedCamera {

public:
    /**
     * @param[in] device_id     the UVC capture device id of the camera
     * @param[in] resolution    ZedResoID value
     */
    ZedCamera(int device_id, ZedResoID resolution, bool flip) {
        camera_ = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(device_id));
        if(!camera_->isOpened())
            throw std::runtime_error("Unable to open stereo camera with device ID " + std::to_string(device_id));
        device_id_ = device_id;
        flip_ = flip;
        cv::Mat raw;
        cv::Mat left_image;
        cv::Mat right_image;
        setResolution(resolution);
        ROS_INFO("Stereo camera resolution set to: %ix%i",
                 int(camera_->get(CV_CAP_PROP_FRAME_WIDTH)), int(camera_->get(CV_CAP_PROP_FRAME_HEIGHT)));
    }

    ZedCamera() { }  // just placeholder

    /**
     * @brief tries to set the resolution of the camera and throws std::runtime_error on failure
     * @param[in]  resoID  ZedResoID value
     */
    void setResolution(ZedResoID resoID) {
        width_ = reso_values.at(resoID).first;
        height_ = reso_values.at(resoID).second;
        camera_->set(CV_CAP_PROP_FRAME_WIDTH, width_);
        camera_->set(CV_CAP_PROP_FRAME_HEIGHT, height_);

        // make sure the hardware uses the correct values
        auto hw_width = camera_->get(CV_CAP_PROP_FRAME_WIDTH);
        auto hw_height = camera_->get(CV_CAP_PROP_FRAME_HEIGHT);
        if (width_ != camera_->get(CV_CAP_PROP_FRAME_WIDTH) || height_ != camera_->get(CV_CAP_PROP_FRAME_HEIGHT)) {
            throw std::runtime_error("Failed to set resolution of stereo camera " + std::to_string(device_id_) + " to "
                                     + std::to_string(width_) + "x" + std::to_string(height_) + " - hardware values: "
                                     + std::to_string(hw_width) + "x" + std::to_string(hw_height));
        }

    }

    /**
     * @brief set a UVC capture parameter of the camera (only a few are supported)
     * @param[in]  propId  identifier of the setting - see CV_CAP_* integer constants
     * @param[in]  value   the desired value to be set
     */
    void setProperty(int propId, double value) {
        camera_->set(propId, value);
    }

    // TODO debug only - show the camera's CV_CAP_* values
    void showSettings() {
        for (int i=-4; i<=37; i++) {
            std::cout << i << " : "<< camera_->get(i) << std::endl;
        }
    }

    /**
     * @brief grabs the current images
     * @param left_image     cv::Mat for the left image
     * @param right_image    cv::Mat for the right image
     * @return true (success) or false (failure)
     */
    bool getImages(cv::Mat &left_image, cv::Mat &right_image) {
        cv::Mat raw;
        if (camera_->grab()) {
            camera_->retrieve(raw);
            if (flip_)
                cv::flip(raw, raw, -1);
            cv::Rect left_rect(0, 0, width_ / 2, height_);
            cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
            left_image = raw(left_rect);
            right_image = raw(right_rect);
            return true;
        } else {
            return false;
        }
    }

private:
    std::unique_ptr<cv::VideoCapture> camera_;
    int device_id_;
    int width_, height_;
    bool flip_;
};


/**
 * @brief ZED stereo camera ROS wrapper class
 */
class ZedCameraROS {

private:
    ZedCamera zed;
    ZedResoID resoID_;
    TearingFilter filter;
    std::mutex frame_mutex;
    int device_;
    double frame_rate_;
    bool show_image_, flip_, filter_tearing_;
    unsigned int width_, height_;
    std::string left_frame_id_, right_frame_id_;
    std::string zed_config_file;

public:
    ZedCameraROS() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        int resolution_param = 1;
        bool config_is_kalibr_yaml = false;
        bool allow_dynamic_reconfig = false;

        // get ROS params
        private_nh.param("device", device_, 0);
        private_nh.param("resolution", resolution_param, 1);
        private_nh.param("frame_rate", frame_rate_, 30.0);
        private_nh.param("flip", flip_, false);
        private_nh.param("config_is_kalibr_yaml", config_is_kalibr_yaml, false);
        private_nh.param("zed_config_file", zed_config_file, std::string(""));
        private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
        private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
        private_nh.param("show_image", show_image_, false);
        private_nh.param("dyn_cfg", allow_dynamic_reconfig, false);
        private_nh.param("filter_tearing", filter_tearing_, false);

        ROS_INFO("Trying to initialize the camera");
        try {
            if (static_cast<int>(ZedResoID::TWO_K) < resolution_param <= static_cast<int>(ZedResoID::VGA)) {
                resoID_ = static_cast<ZedResoID>(resolution_param);
            } else {
                throw std::runtime_error("Invalid resolution parameter");
            }

            zed = ZedCamera(device_, resoID_, flip_);
            width_ = reso_values.at(resoID_).first;
            height_ = reso_values.at(resoID_).second;
            bool publish_cam_info = !zed_config_file.empty();
            ROS_INFO("Initialized the camera");

            // setup dynamic_reconfigure
            if(allow_dynamic_reconfig) {
                ROS_INFO("Starting dynamic_reconfigure server");
                dynamic_reconfigure::Server<zed_cpu_ros::zed_cpu_ros_DynConfig> server;
                dynamic_reconfigure::Server<zed_cpu_ros::zed_cpu_ros_DynConfig>::CallbackType f;
                // WTF boost madness to bin non-static member function
                f = boost::bind(boost::mem_fn(&ZedCameraROS::dynamic_reconfigure_callback), boost::ref(*this), _1, _2);
                server.setCallback(f);
            }
            else
                ROS_INFO("No dynamic_reconfigure server requested");

            // setup publisher stuff
            image_transport::ImageTransport it(nh);
            image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
            image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

            ros::Publisher left_cam_info_pub, right_cam_info_pub;
            sensor_msgs::CameraInfo left_info, right_info;
            if (publish_cam_info) {
                left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
                right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

                ROS_INFO("Loading camera calibration file %s", zed_config_file.c_str());
                try {
                    if (config_is_kalibr_yaml)
                        getKalibrCameraInfo(zed_config_file, left_info, right_info);
                    else
                        getZedCameraInfo(zed_config_file, left_info, right_info);
                }
                catch (std::runtime_error &e) {
                    ROS_WARN("Can't load camera calibration file %s", zed_config_file.c_str());
                    ROS_ERROR("%s", e.what());
                    throw e;
                }
                ROS_INFO("Successfully loaded camera calibration file");
            }
            else
                ROS_INFO("Configuration file not given - won't publish camera_info");

            // thread for high-frequency frame grabbing
            cv::Mat left_image_grab, right_image_grab;
            std::thread grabbing_loop(&ZedCameraROS::grabbingWorkerTask, this,
                                      std::ref(left_image_grab),
                                      std::ref(right_image_grab));
            ROS_INFO("Started frame grabbing worker thread");

            // loop to publish images
            ros::Rate r(frame_rate_);
            cv::Mat left_image, right_image;
            double cum_time_sec = 0.0;
            int count = 0;
            while (nh.ok()) {
                ros::spinOnce();  // for callbacks - http://answers.ros.org/question/11887/significance-of-rosspinonce/
                count++;
                ros::Time now = ros::Time::now();
                frame_mutex.lock();
                left_image_grab.copyTo(left_image);
                right_image_grab.copyTo(right_image);
                frame_mutex.unlock();
                if (left_image.empty() || right_image.empty()) {
                    ROS_WARN_ONCE("Failed to grab an image from the camera");
                } else {
                    ROS_INFO_ONCE("Successfully grabbed an image from the camera");
                    ROS_INFO_ONCE("Publishing frames at %.2f Hz", frame_rate_);
                    if (filter_tearing_) {
                        if (filter.detectTearing(left_image)) {
                            ROS_WARN("ignoring corrupted frame");
                            continue;
                        }
                        else
                            ROS_INFO_ONCE("frame was not corrupted");
                    }
                    if (show_image_) {
                        cv::imshow("left", left_image);
                        cv::imshow("right", right_image);
                        cv::waitKey(10);
                    }
                }
                if (left_image_pub.getNumSubscribers() > 0) {
                    publishImage(left_image, left_image_pub, "left_frame", now);
                }
                if (right_image_pub.getNumSubscribers() > 0) {
                    publishImage(right_image, right_image_pub, "right_frame", now);
                }
                if (publish_cam_info) {
                    if (left_cam_info_pub.getNumSubscribers() > 0) {
                        publishCamInfo(left_cam_info_pub, left_info, now);
                    }
                    if (right_cam_info_pub.getNumSubscribers() > 0) {
                        publishCamInfo(right_cam_info_pub, right_info, now);
                    }
                }
                r.sleep();
                cum_time_sec += r.cycleTime().toSec();
                if (count == frame_rate_) {
                    double avg_actual_rate = 1/(cum_time_sec / count);
                    cum_time_sec = 0; count = 0;
                    if (avg_actual_rate < (frame_rate_ - frame_rate_ / 10)) {
                        ROS_WARN_THROTTLE(10, "low frame rate: %.2f Hz instead of %.2f Hz (warning only every 10s)",
                                          avg_actual_rate, frame_rate_);
                    }
                }
            }
        }
        catch (std::runtime_error &e) {
            ROS_ERROR("%s", e.what());
            throw e;
        }
    }

    void grabbingWorkerTask(cv::Mat &left, cv::Mat &right) {
        bool ok = true;
        while(ok) {
            frame_mutex.lock();
            ok = zed.getImages(left, right);
            frame_mutex.unlock();
            cv::waitKey(1);
        }
    }

    void dynamic_reconfigure_callback(zed_cpu_ros::zed_cpu_ros_DynConfig &config, uint32_t level) {
        ROS_INFO("Received a dynamic_reconfigure request");
        zed.setResolution(static_cast<ZedResoID>(config.resolution_ID));
        if (! config.ignore_values) {
            //zed.setProperty(CV_CAP_PROP_AUTO_EXPOSURE, config.auto_exposure);
            //zed.setProperty(CV_CAP_PROP_EXPOSURE, config.exposure);
            zed.setProperty(CV_CAP_PROP_GAIN, config.defaults ? 0.5 : config.gain);
            zed.setProperty(CV_CAP_PROP_BRIGHTNESS, config.defaults ? 0.375 : config.brightness);
            zed.setProperty(CV_CAP_PROP_CONTRAST, config.defaults ? 0.125 : config.contrast);
            zed.setProperty(CV_CAP_PROP_HUE, config.defaults ? 0 : config.hue);
            zed.setProperty(CV_CAP_PROP_SATURATION, config.defaults ? 0.5 : config.saturation);
        }
        filter_tearing_ = config.filter_tearing;
        filter.setAtol(config.filter_atol);
        filter.setHoughThreshold(config.filter_hough_thr);
    }

    /**
     * @brief parses the "camchain" .yaml file from Kalibr stereo calibration
     * @param[in] config_file    .yaml "camchain" file
     * @param[in] resolution     ZedResoID value
     * @param[in,out] left_cam_info_msg  sensor_msgs::CameraInfo message to populate with left cam's infos
     * @param[in,out] right_cam_info_msg  sensor_msgs::CameraInfo message to populate with right cam's infos
     */
    void getKalibrCameraInfo(std::string config_file,
                             sensor_msgs::CameraInfo &left_info, sensor_msgs::CameraInfo &right_info) {
        // CameraInfo doc: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        // https://github.com/jbeder/yaml-cpp/wiki/Tutorial
        YAML::Node camchain = YAML::LoadFile(config_file);
        if(!(camchain["cam0"] && camchain["cam1"]))
            throw std::runtime_error("yaml file doesn't have cam0 and cam1 entries");
        ROS_ASSERT(camchain["cam0"]["cam_overlaps"].IsDefined());
        ROS_ASSERT(camchain["cam1"]["cam_overlaps"].IsDefined());
        ROS_ASSERT(camchain["cam0"]["camera_model"].IsDefined());
        ROS_ASSERT(camchain["cam1"]["camera_model"].IsDefined());
        ROS_ASSERT(camchain["cam0"]["distortion_model"].IsDefined());
        ROS_ASSERT(camchain["cam1"]["distortion_model"].IsDefined());
        ROS_ASSERT(camchain["cam0"]["distortion_coeffs"].IsSequence());
        ROS_ASSERT(camchain["cam1"]["distortion_coeffs"].IsSequence());
        ROS_ASSERT(camchain["cam1"]["T_cn_cnm1"].IsSequence());
        ROS_ASSERT(camchain["cam0"]["resolution"].IsSequence());
        ROS_ASSERT(camchain["cam1"]["resolution"].IsSequence());
        ROS_ASSERT(camchain["cam0"]["intrinsics"].IsSequence());
        ROS_ASSERT(camchain["cam1"]["intrinsics"].IsSequence());
        ROS_ASSERT(camchain["cam0"]["rostopic"].IsDefined());
        ROS_ASSERT(camchain["cam1"]["rostopic"].IsDefined());
        std::string model_l = camchain["cam0"]["camera_model"].as<std::string>()
                       + "-" + camchain["cam0"]["distortion_model"].as<std::string>();
        std::string model_r = camchain["cam1"]["camera_model"].as<std::string>()
                       + "-" + camchain["cam1"]["distortion_model"].as<std::string>();
        if(!(model_l == model_r && model_l == "pinhole-radtan"))
            throw std::runtime_error("requires Kalibr's pinhole-radtan model - yours: " + model_l + "/" + model_r);

        // Intrinsic camera matrix
        // 	    [fx  0 cx]
        // K =  [ 0 fy cy]
        //	    [ 0  0  1]
        // Kalibr: (fx fy cx cy)
        YAML::Node i_l = camchain["cam0"]["intrinsics"];
        YAML::Node i_r = camchain["cam1"]["intrinsics"];

        // ROS docs: "Intrinsic camera matrix for the raw (distorted) images."
        left_info.K.fill(0.0);
        left_info.K[0] = i_l[0].as<double>();  // fx
        left_info.K[2] = i_l[2].as<double>();  // cx
        left_info.K[4] = i_l[1].as<double>();  // fy
        left_info.K[5] = i_l[3].as<double>();  // cy
        left_info.K[8] = 1.0;

        right_info.K.fill(0.0);
        right_info.K[0] = i_r[0].as<double>();  // fx
        right_info.K[2] = i_r[2].as<double>();  // cx
        right_info.K[4] = i_r[1].as<double>();  // fy
        right_info.K[5] = i_r[3].as<double>();  // cy
        right_info.K[8] = 1.0;

        // distortion parameters
        // for ROS "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3)
        // Kalibr gives (k1, k2, t1, t2), so k3 = 0
        left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        YAML::Node d_l = camchain["cam0"]["distortion_coeffs"];
        left_info.D.resize(5);
        for (std::size_t i=0; i<d_l.size(); i++) {
            left_info.D[i] = d_l[i].as<double>();
        }
        left_info.D[4] = 0.0;
        YAML::Node d_r = camchain["cam1"]["distortion_coeffs"];
        right_info.D.resize(5);
        for (std::size_t i=0; i<d_r.size(); i++) {
            right_info.D[i] = d_r[i].as<double>();
        }
        right_info.D[4] = 0.0;

        // ROS docs: "rotation matrix aligning the camera coordinate system to the ideal stereo image plane"
        // rectification rotation matrix not available from Kalibr! -> calculate with OpenCV
        // convert boost::array to cv::Mat
        cv::Mat mat_K_left(3, 3, CV_64F, &left_info.K[0]);
        cv::Mat mat_K_right(3, 3, CV_64F, &right_info.K[0]);
        cv::Mat mat_D_left(5, 1, CV_64F, &left_info.D[0]);
        cv::Mat mat_D_right(5, 1, CV_64F, &right_info.D[0]);
        YAML::Node T_01 = camchain["cam1"]["T_cn_cnm1"];
        cv::Mat t_01_stereo(3, 1, CV_64F);
        cv::Mat R_01_stereo(3, 3, CV_64F);
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                R_01_stereo.at<double>(i, j) = T_01[i][j].as<double>();
            }
        }
        for (int i=0; i<3; i++) {
            t_01_stereo.at<double>(i) = T_01[i][3].as<double>();
        }
        cv::Mat R0_rect(3, 3, CV_64F);
        cv::Mat R1_rect(3, 3, CV_64F);
        cv::Mat P0_rect(3, 4, CV_64F);
        cv::Mat P1_rect(3, 4, CV_64F);
        cv::Mat Q_rect(4, 4, CV_64F);
        cv::stereoRectify(mat_K_left, mat_D_left, mat_K_right, mat_D_left, cv::Size_<unsigned int>(width_/2, height_),
                          R_01_stereo, t_01_stereo, R0_rect, R1_rect, P0_rect, P1_rect, Q_rect,
                          cv::CALIB_ZERO_DISPARITY, 0.0);
        int id = 0;
        cv::MatIterator_<double> it, end;
        for (it = R0_rect.begin<double>(); it != R0_rect.end<double>(); ++it, id++) {
            left_info.R[id] = *it;
        }
        id = 0;
        for (it = R1_rect.begin<double>(); it != R1_rect.end<double>(); ++it, id++) {
            right_info.R[id] = *it;
        }

        // Projection/camera matrix
        // ROS docs: "of the processed (rectified) image"
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        id = 0;
        for (it = P0_rect.begin<double>(); it != P0_rect.end<double>(); ++it, id++) {
            left_info.P[id] = *it;
        }
        id = 0;
        for (it = P1_rect.begin<double>(); it != P1_rect.end<double>(); ++it, id++) {
            right_info.P[id] = *it;
        }

        // resolution (width/height)
        int exp_width = reso_values.at(resoID_).first;
        int exp_height = reso_values.at(resoID_).second;
        YAML::Node reso_l = camchain["cam0"]["resolution"];
        left_info.width = reso_l[0].as<int>();
        left_info.height = reso_l[1].as<int>();
        YAML::Node reso_r = camchain["cam1"]["resolution"];
        right_info.width = reso_r[0].as<int>();
        right_info.height = reso_r[1].as<int>();
        if ((left_info.width + right_info.width != exp_width)
                || (left_info.height != exp_height || right_info.height != exp_height)) {
            throw std::runtime_error("Resolution values in .yaml file ("
                                     + std::to_string(left_info.width) + "x" + std::to_string(left_info.height)
                                     + ") and ("
                                     + std::to_string(right_info.width) + "x" + std::to_string(right_info.height)
                                     + ") do not yield the camera settings ("
                                     + std::to_string(exp_width) + "x" + std::to_string(exp_height) + ")");
        }

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;
    }

    /**
     * @brief parses the camera information from the ZED manufacturer config file
     * @param[in] config_file    configuration file
     * @param[in] resolution     ZedResoID value
     * @param[in,out] left_cam_info_msg  sensor_msgs::CameraInfo message to populate with left cam's infos
     * @param[in,out] right_cam_info_msg  sensor_msgs::CameraInfo message to populate with right cam's infos
     */
    void getZedCameraInfo(std::string config_file,
                          sensor_msgs::CameraInfo &left_info, sensor_msgs::CameraInfo &right_info) {
        // CameraInfo doc: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        std::string left_str = "LEFT_CAM_";
        std::string right_str = "RIGHT_CAM_";
        std::string reso_str = "";

        switch (resoID_) {
            case ZedResoID::TWO_K:
                reso_str = "2K";
                break;
            case ZedResoID::FHD:
                reso_str = "FHD";
                break;
            case ZedResoID::HD:
                reso_str = "HD";
                break;
            case ZedResoID::VGA:
                reso_str = "VGA";
                break;
        }
        // left value
        double l_cx = pt.get<double>(left_str + reso_str + ".cx");
        double l_cy = pt.get<double>(left_str + reso_str + ".cy");
        double l_fx = pt.get<double>(left_str + reso_str + ".fx");
        double l_fy = pt.get<double>(left_str + reso_str + ".fy");
        double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
        double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
        // right value
        double r_cx = pt.get<double>(right_str + reso_str + ".cx");
        double r_cy = pt.get<double>(right_str + reso_str + ".cy");
        double r_fx = pt.get<double>(right_str + reso_str + ".fx");
        double r_fy = pt.get<double>(right_str + reso_str + ".fy");
        double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
        double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

        // get baseline and convert mm to m
        boost::optional<double> baselineCheck;
        double baseline = 0.0;
        // some config files have "Baseline" instead of "BaseLine", check accordingly...
        if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine")) {
            baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
        } else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline")) {
            baseline = pt.get<double>("STEREO.Baseline") * 0.001;
        } else
            throw std::runtime_error("baseline parameter not found");

        // get Rx and Rz
        double rx = pt.get<double>("STEREO.RX_" + reso_str);
        double rz = pt.get<double>("STEREO.RZ_" + reso_str);
        double ry = pt.get<double>("STEREO.CV_" + reso_str);

        // assume zeros, maybe not right
        double p1 = 0, p2 = 0, k3 = 0;

        left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // distortion parameters
        // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3)
        left_info.D.resize(5);
        left_info.D[0] = l_k1;
        left_info.D[1] = l_k2;
        left_info.D[2] = k3;
        left_info.D[3] = p1;
        left_info.D[4] = p2;

        right_info.D.resize(5);
        right_info.D[0] = r_k1;
        right_info.D[1] = r_k2;
        right_info.D[2] = k3;
        right_info.D[3] = p1;
        right_info.D[4] = p2;

        // Intrinsic camera matrix
        //      [fx  0 cx]
        // K =  [ 0 fy cy]
        //      [ 0  0  1]
        left_info.K.fill(0.0);
        left_info.K[0] = l_fx;
        left_info.K[2] = l_cx;
        left_info.K[4] = l_fy;
        left_info.K[5] = l_cy;
        left_info.K[8] = 1.0;

        right_info.K.fill(0.0);
        right_info.K[0] = r_fx;
        right_info.K[2] = r_cx;
        right_info.K[4] = r_fy;
        right_info.K[5] = r_cy;
        right_info.K[8] = 1.0;

        // rectification matrix
        // Rl = R_rect, R_r = R * R_rect
        // since R is identity, Rl = Rr;
        left_info.R.fill(0.0);
        right_info.R.fill(0.0);
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
        cv::Mat rmat(3, 3, CV_64F);
        cv::Rodrigues(rvec, rmat);
        int id = 0;
        cv::MatIterator_<double> it, end;
        for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++) {
            left_info.R[id] = *it;
            right_info.R[id] = *it;
        }

        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        left_info.P.fill(0.0);
        left_info.P[0] = l_fx;
        left_info.P[2] = l_cx;
        left_info.P[5] = l_fy;
        left_info.P[6] = l_cy;
        left_info.P[10] = 1.0;

        right_info.P.fill(0.0);
        right_info.P[0] = r_fx;
        right_info.P[2] = r_cx;
        right_info.P[3] = (-1 * l_fx * baseline);
        right_info.P[5] = r_fy;
        right_info.P[6] = r_cy;
        right_info.P[10] = 1.0;

        left_info.width = right_info.width = width_;
        left_info.height = right_info.height = height_;

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;
    }

    /**
     * @brief publish camera info
     * @param[in] pub_cam_info   the ROS publisher for the camera info
     * @param[in] cam_info_msg   sensor_msgs::CameraInfo message
     * @param[in] stamp          the message timestamp
     */
    void publishCamInfo(const ros::Publisher &pub_cam_info, sensor_msgs::CameraInfo &cam_info_msg, ros::Time stamp) {
        cam_info_msg.header.stamp = stamp;
        pub_cam_info.publish(cam_info_msg);
    }

    /**
     * @brief publish image
     * @param[in] img           the image
     * @param[in] img_pub       ROS image_transport publisher
     * @param[in] img_frame_id  image frame identifier
     * @param[in] t             timestamp
     */
    void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
        cv_bridge::CvImage cv_image;
        cv_image.image = img;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        img_pub.publish(cv_image.toImageMsg());
    }

};


int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "zed_camera");
        ZedCameraROS zed_ros;
        return EXIT_SUCCESS;
    }
    catch (std::runtime_error &e) {
        ROS_ERROR("%s", e.what());
        ros::shutdown();
        return EXIT_FAILURE;
    }
}
