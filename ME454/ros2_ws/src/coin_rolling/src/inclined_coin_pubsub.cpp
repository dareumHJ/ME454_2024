#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "coin_rolling_msg/msg/coin_rolling.hpp"
#include "mymat.hpp"

using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node
{
public:
    MinimalPubSub()
    : Node("inclined_coin_pubsub")
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&MinimalPubSub::topic_callback, this, _1)
                );

        publisher_ = this->create_publisher<coin_rolling_msg::msg::CoinRolling>("coin/inclined_coin",10);
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {

        auto message_pose = msg->pose[1];
        auto message_twist = msg->twist[1];

        geometry_msgs::msg::Vector3 message_momentum;

        //////////////// TODO ////////////////
        // Calculate the angular momentum vector of the rolling coin on the world frame
        // and publiseh the answer.
        
        double x = message_pose.position.x;
        double y = message_pose.position.y;
        double z = message_pose.position.z;

        double wx = message_twist.angular.x;
        double wy = message_twist.angular.y;
        double wz = message_twist.angular.z;

        double dx = message_twist.linear.x;
        double dy = message_twist.linear.y;
        double dz = message_twist.linear.z;
        
        double q_x = message_pose.orientation.x;
        double q_y = message_pose.orientation.y;
        double q_z = message_pose.orientation.z;
        double q_w = message_pose.orientation.w;

        Mat33 ROT;
        ROT.set_elem(0, 0, (1 - 2*q_y*q_y - 2*q_z*q_z));
        ROT.set_elem(0, 1, (2*q_x*q_y - 2*q_w*q_z));
        ROT.set_elem(0, 2, (2*q_x*q_z + 2*q_w*q_y));
        ROT.set_elem(1, 0, (2*q_x*q_y + 2*q_w*q_z));
        ROT.set_elem(1, 1, (1 - 2*q_x*q_x - 2*q_z*q_z));
        ROT.set_elem(1, 2, (2*q_y*q_z - 2*q_w*q_x));
        ROT.set_elem(2, 0, (2*q_x*q_z - 2*q_w*q_y));
        ROT.set_elem(2, 1, (2*q_y*q_z + 2*q_w*q_x));
        ROT.set_elem(2, 2, (1 - 2*q_x*q_x - 2*q_y*q_y));

        Vec3 glob_w(wx, wy, wz);
        Vec3 body_w;
        body_w = ROT.transpose() * glob_w;
        Mat33 Inertia(ixx_, iyy_, izz_);
        Vec3 momentum;
        momentum = Inertia*body_w;
        Vec3 new_momentum;
        Vec3 additional_momentum;
        Vec3 r(x, y, z);
        Vec3 p(mass_*dx, mass_*dy, mass_*dz);
        additional_momentum = r.cross(p);
        new_momentum = ROT*momentum + additional_momentum;

        message_momentum.x = new_momentum.get_elem(0, 0);
        message_momentum.y = new_momentum.get_elem(1, 0);
        message_momentum.z = new_momentum.get_elem(2, 0);

        //////////////// TODO End ////////////////

        auto message = coin_rolling_msg::msg::CoinRolling();
        message.angular_momentum = message_momentum;
        message.pose = message_pose;
        message.twist = message_twist;
        publisher_->publish(message);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
    rclcpp::Publisher<coin_rolling_msg::msg::CoinRolling>::SharedPtr publisher_;

    double mass_ = 10.0;
    double ixx_ = 0.7;
    double iyy_ = 0.7;
    double izz_ = 1.25;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPubSub>());
    rclcpp::shutdown();

    return 0;
}
