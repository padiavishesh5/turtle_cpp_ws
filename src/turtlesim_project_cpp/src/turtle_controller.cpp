#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using Twist = geometry_msgs::msg::Twist;
using Pose = turtlesim::msg::Pose;
using Turtle = my_robot_interfaces::msg::Turtle;
using TurtleArray = my_robot_interfaces::msg::TurtleArray;
using CatchTurtle = my_robot_interfaces::srv::CatchTurtle;

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("catch_closest_turtle_first", true);
        catch_closest_turtle_first_ =
            this->get_parameter("catch_closest_turtle_first").as_bool();

        cmd_vel_publisher_ = this->create_publisher<Twist>("/turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackPose, this, _1));
        alive_turtles_subscriber_ = this->create_subscription<TurtleArray>(
            "alive_turtles", 10,
            std::bind(&TurtleControllerNode::callbackAliveTurtles, this, _1));
        catch_turtle_client_ = this->create_client<CatchTurtle>("catch_turtle");

        control_loop_timer_ = this->create_wall_timer(
            10ms, std::bind(&TurtleControllerNode::controlLoop, this));
    }

private:
    void callbackPose(const Pose::SharedPtr pose)
    {
        pose_ = *pose;
        pose_received_ = true;
    }

    double distanceFromPose(const Turtle &turtle) const
    {
        const double dx = turtle.x - pose_.x;
        const double dy = turtle.y - pose_.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void callbackAliveTurtles(const TurtleArray::SharedPtr msg)
    {
        if (msg->turtles.empty() || !pose_received_) {
            return;
        }

        if (catch_closest_turtle_first_) {
            const Turtle *closest = &msg->turtles.front();
            double closest_distance = distanceFromPose(*closest);

            for (const auto &turtle : msg->turtles) {
                const double d = distanceFromPose(turtle);
                if (d < closest_distance) {
                    closest = &turtle;
                    closest_distance = d;
                }
            }
            turtle_to_catch_ = *closest;
        } else {
            turtle_to_catch_ = msg->turtles.front();
        }
        has_target_ = true;
    }

    void controlLoop()
    {
        if (!pose_received_ || !has_target_) {
            return;
        }

        const double dist_x = turtle_to_catch_.x - pose_.x;
        const double dist_y = turtle_to_catch_.y - pose_.y;
        const double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        Twist cmd;

        if (distance > 0.5) {
            // position
            cmd.linear.x = 2.0 * distance;

            // orientation
            const double goal_theta = std::atan2(dist_y, dist_x);
            double diff = goal_theta - pose_.theta;
            if (diff > M_PI) {
                diff -= 2.0 * M_PI;
            } else if (diff < -M_PI) {
                diff += 2.0 * M_PI;
            }
            cmd.angular.z = 6.0 * diff;
        } else {
            // target reached
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            callCatchTurtleService(turtle_to_catch_.name);
            has_target_ = false;
        }

        cmd_vel_publisher_->publish(cmd);
    }

    void callCatchTurtleService(const std::string &turtle_name)
    {
        while (!catch_turtle_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for catch_turtle service...");
        }

        auto request = std::make_shared<CatchTurtle::Request>();
        request->name = turtle_name;

        // turtle_name is captured by value, so overlapping calls don't clash
        catch_turtle_client_->async_send_request(
            request,
            [this, turtle_name](rclcpp::Client<CatchTurtle>::SharedFuture future) {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Turtle %s could not be removed", turtle_name.c_str());
                }
            });
    }

    bool catch_closest_turtle_first_{true};
    bool pose_received_{false};
    bool has_target_{false};
    Pose pose_;
    Turtle turtle_to_catch_;

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<TurtleArray>::SharedPtr alive_turtles_subscriber_;
    rclcpp::Client<CatchTurtle>::SharedPtr catch_turtle_client_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
