/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

// Includes
#include "ros2_orb_slam3/common.hpp"

#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// File scope cache for latest mask
static std::mutex g_mask_mutex;
static cv::Mat g_last_mask_mono8;
static bool g_has_mask = false;

// Keep subscription alive even if common.hpp does not declare a member
static rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr g_mask_subscription;

// Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/

    // Find path to home directory
    homeDir = getenv("HOME");
    packagePath = "ros2_ws/src/ros2_orb_slam3/"; // HARDCODED, change it as necessary

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file

    // Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    // Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp node
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive timestep messages

    // Subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName,
        1,
        std::bind(&MonocularMode::experimentSetting_callback, this, _1)
    );

    // Publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    // Subscribe to the image messages coming from the Python driver node
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName,
        1,
        std::bind(&MonocularMode::Img_callback, this, _1)
    );

    // Subscribe to receive the timestep
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName,
        1,
        std::bind(&MonocularMode::Timestep_callback, this, _1)
    );

    // Subscribe to caustics mask
    const std::string subMaskName = "/caustics/mask";
    g_mask_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        subMaskName,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image& mask_msg)
        {
            try
            {
                cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_msg, "mono8");
                if (!mask_ptr || mask_ptr->image.empty())
                {
                    return;
                }

                std::lock_guard<std::mutex> lock(g_mask_mutex);
                g_last_mask_mono8 = mask_ptr->image.clone();
                g_has_mask = true;
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Mask cv_bridge exception, ignoring mask");
                return;
            }
        }
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to mask topic: %s", subMaskName.c_str());
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

// Destructor
MonocularMode::~MonocularMode()
{
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}

// Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();

    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    // Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";

    std::cout << "Sent response: " << message.data.c_str() << std::endl;
    configAck_publisher_->publish(message);

    // Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);
}

// Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString)
{
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    // Build yaml file path
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml");

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = true;

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
}

// Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

// Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr;

    // Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }

    if (!cv_ptr || cv_ptr->image.empty())
    {
        return;
    }

    // Apply mask by zeroing out pixels where mask is 0
    // Convention: mask 255 means keep, mask 0 means remove
    cv::Mat mask_local;
    bool has_mask_local = false;

    {
        std::lock_guard<std::mutex> lock(g_mask_mutex);
        if (g_has_mask && !g_last_mask_mono8.empty())
        {
            mask_local = g_last_mask_mono8.clone();
            has_mask_local = true;
        }
    }

    if (has_mask_local)
    {
        if (mask_local.size() == cv_ptr->image.size())
        {
            cv::Mat filtered;
            cv::bitwise_and(cv_ptr->image, cv_ptr->image, filtered, mask_local);
            cv_ptr->image = filtered;
        }
        else
        {
            // If sizes do not match, resize mask to image size
            cv::Mat resized_mask;
            cv::resize(mask_local, resized_mask, cv_ptr->image.size(), 0.0, 0.0, cv::INTER_NEAREST);
            cv::Mat filtered;
            cv::bitwise_and(cv_ptr->image, cv_ptr->image, filtered, resized_mask);
            cv_ptr->image = filtered;
        }
    }

    // Perform all ORB-SLAM3 operations in Monocular mode
    // Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);

    (void)Tcw;
}
