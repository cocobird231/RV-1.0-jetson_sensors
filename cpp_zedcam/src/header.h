#include <vector>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg/image.hpp"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/vehicle_interfaces.h"

#include <opencv2/opencv.hpp>

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string topic_ZEDCam_RGB_nodeName = "zed_rgb_0_node";
    std::string topic_ZEDCam_RGB_topicName = "zed_rgb_0";
    float topic_ZEDCam_RGB_pubInterval_s = 0.033;
    int topic_ZEDCam_RGB_width = 640;
    int topic_ZEDCam_RGB_height = 360;

    std::string topic_ZEDCam_Depth_nodeName = "zed_depth_0_node";
    std::string topic_ZEDCam_Depth_topicName = "zed_depth_0";
    float topic_ZEDCam_Depth_pubInterval_s = 0.033;
    int topic_ZEDCam_Depth_width = 640;
    int topic_ZEDCam_Depth_height = 360;

    std::vector<double> camera_cap_ids = { 0 };
    std::vector<double> camera_cap_sns = {};
    std::string camera_cap_input_type = "id";
    float camera_fps = 30.0;
    int camera_width = 1280;
    int camera_height = 720;
    int camera_sensing_mode = 0;// 0: standard mode, 1: fill mode
    int camera_depth_quality = 0;// 0: performance, 1: quality
    int camera_depth_unit = 1;// index started from 0: micro, milli, centi, meter
    bool camera_use_color = true;
    bool camera_use_depth = true;

    std::vector<double> ids = { 0 };

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

        this->get_parameter("camera_cap_ids", this->camera_cap_ids);
        this->get_parameter("camera_cap_sns", this->camera_cap_sns);
        this->get_parameter("camera_cap_input_type", this->camera_cap_input_type);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_width", this->camera_width);
        this->get_parameter("camera_height", this->camera_height);
        this->get_parameter("camera_sensing_mode", this->camera_sensing_mode);
        this->get_parameter("camera_depth_quality", this->camera_depth_quality);
        this->get_parameter("camera_depth_unit", this->camera_depth_unit);
        this->get_parameter("camera_use_color", this->camera_use_color);
        this->get_parameter("camera_use_depth", this->camera_use_depth);

        this->get_parameter("ids", this->ids);
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

        this->declare_parameter<std::vector<double> >("camera_cap_ids", this->camera_cap_ids);
        this->declare_parameter<std::vector<double> >("camera_cap_sns", this->camera_cap_sns);
        this->declare_parameter<std::string>("camera_cap_input_type", this->camera_cap_input_type);
        this->declare_parameter<float>("camera_fps", this->camera_fps);
        this->declare_parameter<int>("camera_width", this->camera_width);
        this->declare_parameter<int>("camera_height", this->camera_height);
        this->declare_parameter<int>("camera_sensing_mode", this->camera_sensing_mode);
        this->declare_parameter<int>("camera_depth_quality", this->camera_depth_quality);
        this->declare_parameter<int>("camera_depth_unit", this->camera_depth_unit);
        this->declare_parameter<bool>("camera_use_color", this->camera_use_color);
        this->declare_parameter<bool>("camera_use_depth", this->camera_use_depth);

        this->declare_parameter<std::vector<double> >("ids", this->ids);
        this->_getParams();
    }
};


class ZEDPublisher : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    std::string nodeName_;
    int id_;
    std::string rgbTopicName_;
    std::string depthTopicName_;
    float rgbPubPeriod_;
    float depthPubPeriod_;
    std::mutex paramsLock_;
    
    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr rgbPub_;
    std::mutex rgbLock_;
    bool useColorF_;

    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr depthPub_;
    std::mutex depthLock_;
    bool useDepthF_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        std::unique_lock<std::mutex> rgbLocker(this->rgbLock_, std::defer_lock);
        std::unique_lock<std::mutex> depthLocker(this->depthLock_, std::defer_lock);
        std::unique_lock<std::mutex> paramsLocker(this->paramsLock_, std::defer_lock);
        paramsLocker.lock();
        auto rgbTopicName = this->rgbTopicName_;
        auto depthTopicName = this->depthTopicName_;
        paramsLocker.unlock();

        for (const auto& [k, v] : qmap)
        {
            if (k == rgbTopicName || k == (std::string)this->get_namespace() + "/" + rgbTopicName)
            {
                rgbLocker.lock();
                this->rgbPub_.reset();// Call destructor
                this->rgbPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(rgbTopicName, *v);
                rgbLocker.unlock();
            }
            else if (k == depthTopicName || k == (std::string)this->get_namespace() + "/" + depthTopicName)
            {
                depthLocker.lock();
                this->depthPub_.reset();// Call destructor
                this->depthPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(depthTopicName, *v);
                depthLocker.unlock();
            }
        }
    }

public:
    ZEDPublisher(const std::shared_ptr<Params>& params, int id) : 
        vehicle_interfaces::PseudoTimeSyncNode(params->nodeName + "_" + std::to_string(id)), 
        vehicle_interfaces::QoSUpdateNode(params->nodeName + "_" + std::to_string(id), params->qosService, params->qosDirPath), 
        rclcpp::Node(params->nodeName + "_" + std::to_string(id)), 
        id_(id)
    {
        this->useColorF_ = params->camera_use_color;
        this->useDepthF_ = params->camera_use_depth;

        this->nodeName_ = params->nodeName + "_" + std::to_string(id);
        this->rgbTopicName_ = params->topic_ZEDCam_RGB_topicName + "_" + std::to_string(id);
        this->depthTopicName_ = params->topic_ZEDCam_Depth_topicName + "_" + std::to_string(id);

        this->rgbPubPeriod_ = params->topic_ZEDCam_RGB_pubInterval_s;
        this->depthPubPeriod_ = params->topic_ZEDCam_Depth_pubInterval_s;

        this->addQoSCallbackFunc(std::bind(&ZEDPublisher::_qosCallback, this, std::placeholders::_1));

        if (params->camera_use_color)
        {
            vehicle_interfaces::QoSPair qpair = this->addQoSTracking(this->rgbTopicName_);
            if (qpair.first == "")
                RCLCPP_ERROR(this->get_logger(), "[ZEDPublisher] Failed to add topic to track list: %s", this->rgbTopicName_.c_str());
            else
            {
                RCLCPP_INFO(this->get_logger(), "[ZEDPublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                    qpair.first.c_str(), qpair.second->get_rmw_qos_profile().depth, qpair.second->get_rmw_qos_profile().reliability);
            }
            this->rgbPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(this->rgbTopicName_, *qpair.second);
        }
        if (params->camera_use_depth)
        {
            vehicle_interfaces::QoSPair qpair = this->addQoSTracking(this->depthTopicName_);
            if (qpair.first == "")
                RCLCPP_ERROR(this->get_logger(), "[ZEDPublisher] Failed to add topic to track list: %s", this->depthTopicName_.c_str());
            else
            {
                RCLCPP_INFO(this->get_logger(), "[ZEDPublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                    qpair.first.c_str(), qpair.second->get_rmw_qos_profile().depth, qpair.second->get_rmw_qos_profile().reliability);
            }
            this->depthPub_ = this->create_publisher<vehicle_interfaces::msg::Image>(this->depthTopicName_, *qpair.second);
        }
    }

    void pubRGBMat(const std::vector<uchar>& vec, cv::Size sz, int type)
    {
        if (!this->useColorF_)
            return;

        std::lock_guard<std::mutex> locker(this->rgbLock_);
        static u_int64_t rgbFrameID = 0;
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = rgbFrameID++;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = this->rgbPubPeriod_ * 1000.0;

        msg.format_type = msg.FORMAT_JPEG;
        msg.cvmat_type = type;
        msg.width = sz.width;
        msg.height = sz.height;
        msg.data = vec;

        this->rgbPub_->publish(msg);
    }

    void pubDepthMat(const std::vector<uchar>& vec, cv::Size sz, int type, int unit)
    {
        if (!this->useDepthF_)
            return;

        std::lock_guard<std::mutex> locker(this->depthLock_);
        static u_int64_t depthFrameID = 0;
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = depthFrameID++;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = this->depthPubPeriod_ * 1000.0;

        msg.format_type = msg.FORMAT_RAW;
        msg.cvmat_type = type;
        msg.depth_unit_type = unit;
        msg.width = sz.width;
        msg.height = sz.height;
        msg.data = vec;

        this->depthPub_->publish(msg);
    }
};

class ZEDNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    std::shared_ptr<Params> params_;

    std::map<int, std::shared_ptr<ZEDPublisher> > zedPubs;
    std::vector<std::thread> zedThVec_;
    std::mutex zedPubsLock_;

public:
    ZEDNode(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params) {}

    ~ZEDNode()
    {
        for (auto& i : this->zedThVec_)
            i.join();
    }

    void addZEDPublisher(int topicID)
    {
        std::lock_guard<std::mutex> locker(this->zedPubsLock_);
        this->zedPubs[topicID] = std::make_shared<ZEDPublisher>(this->params_, topicID);
        this->zedPubs[topicID]->syncTime(this->getCorrectDuration(), this->getTimestampType());
        this->zedThVec_.emplace_back(vehicle_interfaces::SpinNode, this->zedPubs[topicID], "ZEDPublisher_" + std::to_string(topicID));
    }

    std::shared_ptr<ZEDPublisher>& getZEDPublisher(int topicID)
    {
        std::lock_guard<std::mutex> locker(this->zedPubsLock_);
        return this->zedPubs[topicID];
    }

    void syncZEDPublisher(int topicID)
    {
        std::lock_guard<std::mutex> locker(this->zedPubsLock_);
        this->zedPubs[topicID]->syncTime(this->getCorrectDuration(), this->getTimestampType());
    }

    std::shared_ptr<Params> getParams() const
    {
        return this->params_;
    }
};