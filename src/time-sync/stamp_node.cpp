#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <vector>
#include <queue>
#include <algorithm>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <thread>

// #include <boost/thread/thread.hpp>
// #include <boost/lockfree/queue.hpp>
// #include <boost/atomic.hpp>
// #include <boost/algorithm/string.hpp>
#include <iostream>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <rigtorp/SPSCQueue.h>  

using namespace rigtorp;
using namespace message_filters;


// inline double time_to_seconds_s(rclcpp::Time rcltime)
// {
//     return rcltime.nanoseconds() / 1e9;
// }


struct StereoInertialPacket {
    cv_bridge::CvImageConstPtr image_left_ptr;
    cv_bridge::CvImageConstPtr image_right_ptr;
    std::vector<ORB_SLAM3::IMU::Point> imu_msg_queue;
    double image_left_time_s;

    // Default constructor
    StereoInertialPacket() = default;

    // Constructor that directly takes rvalue references for move semantics
    StereoInertialPacket(cv_bridge::CvImageConstPtr&& left, cv_bridge::CvImageConstPtr&& right, std::vector<ORB_SLAM3::IMU::Point>&& imu_data, double&& left_time_s)
        : image_left_ptr(std::move(left)), image_right_ptr(std::move(right)), imu_msg_queue(std::move(imu_data)), image_left_time_s(left_time_s) {}

    // StereoInertialPacket(cv_bridge::CvImageConstPtr left_ptr, cv_bridge::CvImageConstPtr right_ptr,
    //                      std::vector<ORB_SLAM3::IMU::Point>&& imu_data, double left_time_s) :
    //     image_left_ptr(left_ptr),
    //     image_right_ptr(right_ptr),
    //     imu_msg_queue(std::move(imu_data)),
    //     image_left_time_s(left_time_s) {}

    // // Copy constructor from struct
    // StereoInertialPacket(const StereoInertialPacket& other) :
    //     image_left_ptr(other.image_left_ptr),
    //     image_right_ptr(other.image_right_ptr),
    //     imu_msg_queue(other.imu_msg_queue),
    //     image_left_time_s(other.image_left_time_s) {}

    // // Copy constructor from items
    // StereoInertialPacket(cv_bridge::CvImageConstPtr left_ptr, cv_bridge::CvImageConstPtr right_ptr,
    //                      std::vector<ORB_SLAM3::IMU::Point> imu_data, double left_time_s) :
    //     image_left_ptr(left_ptr),
    //     image_right_ptr(right_ptr),
    //     imu_msg_queue(imu_data),
    //     image_left_time_s(left_time_s) {}
};


class ImageSyncNode : public rclcpp::Node
{
public:
    ImageSyncNode() : Node("image_sync_node"),
                      prev_stereo_time_init_(false),
                      stereo_inertial_packet_queue_(30),
                      SLAM("/root/ORB_SLAM3/Vocabulary/ORBvoc.txt", "/root/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D455.yaml", ORB_SLAM3::System::IMU_STEREO, true)
    {
        // Initialize subscribers
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/infra1/image_rect_raw");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/infra2/image_rect_raw");

        auto sensor_data_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("camera/imu", sensor_data_qos, std::bind(&ImageSyncNode::imu_callback, this, std::placeholders::_1));

        // Set up the synchronization policy
        sync_ = std::make_shared<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
            sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(30), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&ImageSyncNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));

        orbslam_thread_ = std::thread(&ImageSyncNode::orbslam_thread, this);
    }

    ~ImageSyncNode()
    {
        if (orbslam_thread_.joinable()) {
            orbslam_thread_.join();  // Wait for the thread to finish
        }

        SLAM.Shutdown();
    }

private:
    void orbslam_thread()
    {
        while (rclcpp::ok()) {
            // Wait for an incoming stereo inertial packet
            StereoInertialPacket *packet = nullptr;

            // RCLCPP_INFO(this->get_logger(), "2 >> Waiting for new packet in queue");
            while (!(packet = stereo_inertial_packet_queue_.front())) {
                if (!rclcpp::ok()) break;
                std::this_thread::yield();
            }

            if (!packet) throw std::runtime_error("StereoInertialPacket from queue is nullptr!");

            RCLCPP_INFO(this->get_logger(), "2 >> Got a new packet in queue w/ %ld IMU meas, time at %f", packet->imu_msg_queue.size(), packet->image_left_time_s);

            // RCLCPP_INFO(this->get_logger(), "ORBSLAM Thread Recv Packet!");
            // Process packet, feed into ORB_SLAM3 instance

            const cv::Mat image_left = packet->image_left_ptr->image;
            const cv::Mat image_right = packet->image_right_ptr->image;

            // RCLCPP_INFO(this->get_logger(), "2 >> Calling TrackStereo...");
            SLAM.TrackStereo(image_left, image_right, packet->image_left_time_s, packet->imu_msg_queue);
            // RCLCPP_INFO(this->get_logger(), "2 >> Call to TrackStereo complete!");

            // Remove processed packet
            stereo_inertial_packet_queue_.pop();
        }
    }

    void stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        // If this is the first stereo pair, just update the previous pair time and exit
        // TODO: what happens on first msg?
        if (!prev_stereo_time_init_) {
            prev_stereo_time_ = rclcpp::Time(left->header.stamp);
            prev_stereo_time_init_ = true;

            return;
        }

        rclcpp::Time curr_stereo_time = rclcpp::Time(left->header.stamp);
        std::vector<ORB_SLAM3::IMU::Point> imu_msg_queue_in_bounds;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);

            // Check the IMU measumrent queue
            if (imu_msg_queue_.empty()) RCLCPP_ERROR(this->get_logger(), "IMU message queue is empty!");

            while (!imu_msg_queue_.empty())
            {
                auto msg = imu_msg_queue_.front();
                rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);

                if (msg_time <= curr_stereo_time) {
                    imu_msg_queue_in_bounds.push_back(
                        ORB_SLAM3::IMU::Point(
                            cv::Point3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                            cv::Point3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                            msg_time.seconds()
                        )
                    );

                    imu_msg_queue_.pop();
                } else {
                    break;
                }
            }
        }

        cv_bridge::CvImageConstPtr cv_ptr_left;
        cv_bridge::CvImageConstPtr cv_ptr_right;

        try {
            cv_ptr_left = cv_bridge::toCvShare(left);
            cv_ptr_right = cv_bridge::toCvShare(right);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // const StereoInertialPacket packet = StereoInertialPacket(cv_ptr_left, cv_ptr_right, std::move(imu_msg_queue_in_bounds), curr_stereo_time.seconds());
        double curr_stereo_time_s = curr_stereo_time.seconds();

        RCLCPP_INFO(this->get_logger(), "1 >> Had %ld IMU messages fall between consecutive frames, time at %f", imu_msg_queue_in_bounds.size(), curr_stereo_time_s);

        stereo_inertial_packet_queue_.emplace(
            std::move(cv_ptr_left),
            std::move(cv_ptr_right),
            std::move(imu_msg_queue_in_bounds),
            std::move(curr_stereo_time_s)
        );

        prev_stereo_time_ = curr_stereo_time;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        imu_msg_queue_.push(msg);
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_msg_queue_;
    std::mutex queue_mutex_;

    bool prev_stereo_time_init_;
    rclcpp::Time prev_stereo_time_;

    SPSCQueue<StereoInertialPacket> stereo_inertial_packet_queue_;
    std::thread orbslam_thread_;

    ORB_SLAM3::System SLAM;

    // boost::thread consumer_thread_;
    // boost::atomic<bool> done_;
    // boost::lockfree::spsc_queue<int, boost::lockfree::capacity<1024>> spsc_queue_;

    // int produced_count_, consumed_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
