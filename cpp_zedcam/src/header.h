#include <vector>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg/image.hpp"
#include "vehicle_interfaces/vehicle_interfaces.h"

#include <opencv2/opencv.hpp>

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string topic_ZEDCam_RGB_nodeName = "zed_rgb_0_node";
    std::string topic_ZEDCam_RGB_topicName = "zed_rgb_0";
    float topic_ZEDCam_RGB_pubInterval_s = 0.03;
    int topic_ZEDCam_RGB_width = 640;
    int topic_ZEDCam_RGB_height = 360;

    std::string topic_ZEDCam_Depth_nodeName = "zed_depth_0_node";
    std::string topic_ZEDCam_Depth_topicName = "zed_depth_0";
    float topic_ZEDCam_Depth_pubInterval_s = 0.03;
    int topic_ZEDCam_Depth_width = 640;
    int topic_ZEDCam_Depth_height = 360;

    int camera_cap_id = 0;
    float camera_fps = 30.0;
    int camera_width = 1280;
    int camera_height = 720;
    int camera_sensing_mode = 0;// 0: standard mode, 1: fill mode
    int camera_depth_quality = 0;// 0: performance, 1: quality
    int camera_depth_unit = 1;// index started from 0: micro, milli, centi, meter
    bool camera_use_color = true;
    bool camera_use_depth = true;

private:
    void _getParams()
    {
        this->get_parameter("topic_ZEDCam_RGB_nodeName", this->topic_ZEDCam_RGB_nodeName);
        this->get_parameter("topic_ZEDCam_RGB_topicName", this->topic_ZEDCam_RGB_topicName);
        this->get_parameter("topic_ZEDCam_RGB_pubInterval_s", this->topic_ZEDCam_RGB_pubInterval_s);
        this->get_parameter("topic_ZEDCam_RGB_width", this->topic_ZEDCam_RGB_width);
        this->get_parameter("topic_ZEDCam_RGB_height", this->topic_ZEDCam_RGB_height);

        this->get_parameter("topic_ZEDCam_Depth_nodeName", this->topic_ZEDCam_Depth_nodeName);
        this->get_parameter("topic_ZEDCam_Depth_topicName", this->topic_ZEDCam_Depth_topicName);
        this->get_parameter("topic_ZEDCam_Depth_pubInterval_s", this->topic_ZEDCam_Depth_pubInterval_s);
        this->get_parameter("topic_ZEDCam_Depth_width", this->topic_ZEDCam_Depth_width);
        this->get_parameter("topic_ZEDCam_Depth_height", this->topic_ZEDCam_Depth_height);

        this->get_parameter("camera_cap_id", this->camera_cap_id);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_width", this->camera_width);
        this->get_parameter("camera_height", this->camera_height);
        this->get_parameter("camera_sensing_mode", this->camera_sensing_mode);
        this->get_parameter("camera_depth_quality", this->camera_depth_quality);
        this->get_parameter("camera_depth_unit", this->camera_depth_unit);
        this->get_parameter("camera_use_color", this->camera_use_color);
        this->get_parameter("camera_use_depth", this->camera_use_depth);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("topic_ZEDCam_RGB_nodeName", this->topic_ZEDCam_RGB_nodeName);
        this->declare_parameter<std::string>("topic_ZEDCam_RGB_topicName", this->topic_ZEDCam_RGB_topicName);
        this->declare_parameter<float>("topic_ZEDCam_RGB_pubInterval_s", this->topic_ZEDCam_RGB_pubInterval_s);
        this->declare_parameter<int>("topic_ZEDCam_RGB_width", this->topic_ZEDCam_RGB_width);
        this->declare_parameter<int>("topic_ZEDCam_RGB_height", this->topic_ZEDCam_RGB_height);

        this->declare_parameter<std::string>("topic_ZEDCam_Depth_nodeName", this->topic_ZEDCam_Depth_nodeName);
        this->declare_parameter<std::string>("topic_ZEDCam_Depth_topicName", this->topic_ZEDCam_Depth_topicName);
        this->declare_parameter<float>("topic_ZEDCam_Depth_pubInterval_s", this->topic_ZEDCam_Depth_pubInterval_s);
        this->declare_parameter<int>("topic_ZEDCam_Depth_width", this->topic_ZEDCam_Depth_width);
        this->declare_parameter<int>("topic_ZEDCam_Depth_height", this->topic_ZEDCam_Depth_height);

        this->declare_parameter<int>("camera_cap_id", this->camera_cap_id);
        this->declare_parameter<float>("camera_fps", this->camera_fps);
        this->declare_parameter<int>("camera_width", this->camera_width);
        this->declare_parameter<int>("camera_height", this->camera_height);
        this->declare_parameter<int>("camera_sensing_mode", this->camera_sensing_mode);
        this->declare_parameter<int>("camera_depth_quality", this->camera_depth_quality);
        this->declare_parameter<int>("camera_depth_unit", this->camera_depth_unit);
        this->declare_parameter<bool>("camera_use_color", this->camera_use_color);
        this->declare_parameter<bool>("camera_use_depth", this->camera_use_depth);
        this->_getParams();
    }
};


class ZEDPublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    std::shared_ptr<Params> params;
    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr RGBPub_;
    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr DepthPub_;
    std::string nodeName_;

    bool useColorF_;
    bool useDepthF_;

    vehicle_interfaces::Timer* rgbTimer_;
    vehicle_interfaces::Timer* depthTimer_;

    std::vector<uchar> rgbMatVec_;
    cv::Size rgbMatSize_;
    int rgbMatType_;
    bool rgbMatInitF_;

    std::vector<uchar> depthMatVec_;
    cv::Size depthMatSize_;
    int depthMatType_;
    int depthUnitType_;
    bool depthMatInitF_;

    std::mutex rgbLock_;
    std::mutex depthLock_;

private:
    void _rgbTimerCallback()
    {
        static u_int64_t frame_id = 0;
        if (!this->rgbMatInitF_)
            return;
        std::unique_lock<std::mutex> locker(this->rgbLock_, std::defer_lock);
        locker.lock();
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = frame_id++;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = this->params->topic_ZEDCam_RGB_pubInterval_s * 1000.0;

        msg.format_type = msg.FORMAT_JPEG;
        msg.cvmat_type = this->rgbMatType_;
        msg.width = this->rgbMatSize_.width;
        msg.height = this->rgbMatSize_.height;
        msg.data = this->rgbMatVec_;
        locker.unlock();

        this->RGBPub_->publish(msg);
    }

    void _depthTimerCallback()
    {
        static u_int64_t frame_id = 0;
        if (!this->depthMatInitF_)
            return;
        std::unique_lock<std::mutex> locker(this->depthLock_, std::defer_lock);
        locker.lock();
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = frame_id++;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = this->params->topic_ZEDCam_Depth_pubInterval_s * 1000.0;

        msg.format_type = msg.FORMAT_RAW;
        msg.cvmat_type = this->depthMatType_;
        msg.depth_unit_type = this->depthUnitType_;
        msg.width = this->depthMatSize_.width;
        msg.height = this->depthMatSize_.height;
        msg.data = this->depthMatVec_;
        locker.unlock();

        this->DepthPub_->publish(msg);
    }

public:
    ZEDPublisher(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params(params), 
        rgbMatInitF_(false), 
        depthMatInitF_(false)
    {
        this->nodeName_ = params->nodeName;
        this->useColorF_ = params->camera_use_color;
        this->useDepthF_ = params->camera_use_depth;

        // rclcpp::QoS depth_qos(10);
        // depth_qos.keep_last(1);
        // depth_qos.best_effort();
        // depth_qos.durability_volatile();

        if (params->camera_use_color)
        {
            this->RGBPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(params->topic_ZEDCam_RGB_topicName, 10);
            this->rgbTimer_ = new vehicle_interfaces::Timer(params->topic_ZEDCam_RGB_pubInterval_s * 1000.0, std::bind(&ZEDPublisher::_rgbTimerCallback, this));
        }
        if (params->camera_use_depth)
        {
            this->DepthPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(params->topic_ZEDCam_Depth_topicName, 10);
            this->depthTimer_ = new vehicle_interfaces::Timer(params->topic_ZEDCam_Depth_pubInterval_s * 1000.0, std::bind(&ZEDPublisher::_depthTimerCallback, this));
        }
    }

    ~ZEDPublisher()
    {
        if (this->useColorF_)
            this->rgbTimer_->destroy();
        if (this->useDepthF_)
            this->depthTimer_->destroy();
    }

    void setRGBMat(const std::vector<uchar> vec, cv::Size sz, int type)
    {
        if (!this->useColorF_)
            return;
        std::unique_lock<std::mutex> locker(this->rgbLock_, std::defer_lock);
        locker.lock();
        this->rgbMatVec_ = vec;
        this->rgbMatSize_ = sz;
        this->rgbMatType_ = type;
        locker.unlock();
        if (!this->rgbMatInitF_)
            this->rgbMatInitF_ = true;
    }

    void setDepthMat(const std::vector<uchar> vec, cv::Size sz, int type, int unit)
    {
        if (!this->useDepthF_)
            return;
        std::unique_lock<std::mutex> locker(this->depthLock_, std::defer_lock);
        locker.lock();
        this->depthMatVec_ = vec;
        this->depthMatSize_ = sz;
        this->depthMatType_ = type;
        this->depthUnitType_ = unit;
        locker.unlock();
        if (!this->depthMatInitF_)
            this->depthMatInitF_ = true;
    }

    void startRGBPub() { this->rgbTimer_->start(); }

    void startDepthPub() { this->depthTimer_->start(); }

    void stopRGBPub() { this->rgbTimer_->stop(); }

    void stopDepthPub() { this->depthTimer_->stop(); }
};


void SpinNode(std::shared_ptr<rclcpp::Node> node, std::string threadName)
{
	std::cerr << threadName << " start..." << std::endl;
	rclcpp::spin(node);
	std::cerr << threadName << " exit." << std::endl;
	rclcpp::shutdown();
}