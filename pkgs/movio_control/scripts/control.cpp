#include "rclcpp/rclcpp.hpp"
#include "movio_interfaces/msg/sensor.hpp"
#include "movio_interfaces/msg/wheelrpm.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std;
// this nodes subscribes to sensors and publishes to odom
// it subscribes to cmd_vel and publishes the wheel rpm
class MovioControl : public rclcpp::Node{
    public: 
        MovioControl() : Node("movio_control"){
            // odom calc in the callback
            // vel to rpm
            odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
            sensor_sub = this->create_subscription<movio_interfaces::msg::Sensor>("/sensor", 10, 
                std::bind(&MovioControl::odom_cb, this, std::placeholders::_1));

            vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
                std::bind(&MovioControl::vel_cb, this, std::placeholders::_1));

            robot_pose = {0.0, 0.0, 0.0};

            wheel_rad = 0.035;
            wheel_sep = 0.22;
            enc_ppr = 600;
            rad_rot = 0.19;
            met_per_tick = (2 * 3.14 * wheel_rad) / enc_ppr;
        }
    
        private:
            void odom_cb(const movio_interfaces::msg::Sensor::SharedPtr msg){
            
                robot_pose[2] = normalize_angle(msg->yaw);

                float dis_enc_left = met_per_tick * msg->enc_left;
                float dis_enc_right = met_per_tick * msg->enc_right;


                float dis_robot = (dis_enc_right + dis_enc_left) / 2;
                float theta_robot = (dis_enc_right - dis_enc_left) / (2 * rad_rot);

                robot_pose[0] += dis_robot * cos(robot_pose[2] + (theta_robot / 2));
                robot_pose[1] += dis_robot * sin(robot_pose[2] + (theta_robot / 2));
                robot_pose[2] += theta_robot;

                robot_pose[2] = normalize_angle(robot_pose[2]);

            }

            void vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
                float v = 2 * msg->linear.x;
                float w = 2 * msg->angular.z;

                float wheel_rpm_r = ((v + (w * (wheel_sep/2)))/wheel_rad) * 9.55;
                float wheel_rpm_l = ((v - (w * (wheel_sep/2)))/wheel_rad) * 9.55;
            }

            float normalize_angle(float angle) {
                angle = fmod(angle + M_PI, 2 * M_PI);
                if (angle < 0)
                    angle += 2 * M_PI;
                return angle - M_PI;
            }
            
            
        rclcpp::Subscription<movio_interfaces::msg::Sensor>::SharedPtr sensor_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        vector<float> robot_pose;
        float met_per_tick;
        float wheel_rad;
        float wheel_sep;
        float enc_ppr;
        float rad_rot;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovioControl>());
    rclcpp::shutdown();
    return 0;
}