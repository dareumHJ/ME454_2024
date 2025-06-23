#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "midterm_2024_msg/msg/midterm.hpp"
#include "mymat.hpp"
#include "myQuaternion.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using std::placeholders::_1;

class Sun_generator : public rclcpp::Node
{
public:
    Sun_generator(const char* nodeName)
    : Node(nodeName)
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&Sun_generator::topic_callback, this, _1)
                );

        publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/Midterm/ForceInput/TorqueBySun", 10);
        publisher_2 = this->create_publisher<midterm_2024_msg::msg::Midterm>("/Midterm/recordings", 10);
        //////////////// TODO ////////////////
        // TODO: you can set your subscriber and publishers
        // TODO: subscribed topic should be "demo/model_states_demo"
        // TODO: published topic should be "/Midterm/ForceInput/TorqueBySun" with type : geometry_msgs::msg::Wrench
        // TODO: published topic should be "/Midterm/recordings" with type : midterm_2024_msg::msg::Midterm

        //////////////// TODO End ////////////////
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        //////////////// TODO ////////////////
        // TODO: Calculate the torque by sun in world frame.
        // TODO: get information (position, quaternion, angular velocity)publish them by topic (in GCS)
        auto message_pose = msg->pose[1];
        auto message_twist = msg->twist[1];
        geometry_msgs::msg::Point message_position;
        geometry_msgs::msg::Quaternion message_quat;
        geometry_msgs::msg::Vector3 message_ang;
        geometry_msgs::msg::Vector3 torque;

        double wx = message_twist.angular.x;
        double wy = message_twist.angular.y;
        double wz = message_twist.angular.z;

        Vec3 w(wx, wy, wz);
        Vec3 z(0, 0, 1);
        Vec3 T = w.cross(z);
        
        auto message_1 = geometry_msgs::msg::Wrench();
        auto message_2 = midterm_2024_msg::msg::Midterm();

        message_ang.x = wx;
        message_ang.y = wy;
        message_ang.z = wz;
        message_quat = message_twist.oritentation;
        message_position = message_pose;


        message_1.torque = T;
        publisher_->publish(message_1);
        message_2.position = message_position;
        message_2.angular_velocity = message_ang;
        message_2.quaternion = message_quat;
        publisher_2->publish(message_2);
    }
    // TODO: you can set your subscriber and publishers
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_2;
    //////////////// TODO End ////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sun_generator>("sun_torque_generator_20200282"));
    rclcpp::shutdown();

    return 0;
}
