#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using Turtle = my_robot_interfaces::msg::Turtle;
using TurtleArray = my_robot_interfaces::msg::TurtleArray;
using CatchTurtle = my_robot_interfaces::srv::CatchTurtle;
using Spawn = turtlesim::srv::Spawn;
using Kill = turtlesim::srv::Kill;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"), rng_(std::random_device{}())
    {
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 1.0);
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        alive_turtles_publisher_ = this->create_publisher<TurtleArray>("alive_turtles", 10);
        spawn_client_ = this->create_client<Spawn>("/spawn");
        kill_client_ = this->create_client<Kill>("/kill");
        catch_turtle_service_ = this->create_service<CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));
        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / spawn_frequency_),
            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));
    }

private:
    void callbackCatchTurtle(const CatchTurtle::Request::SharedPtr request,
                             const CatchTurtle::Response::SharedPtr response)
    {
        callKillService(request->name);
        response->success = true;
    }

    void publishAliveTurtles()
    {
        TurtleArray msg;
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

    void spawnNewTurtle()
    {
        turtle_counter_++;
        const std::string name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        std::uniform_real_distribution<double> pos(0.0, 11.0);
        std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
        callSpawnService(name, pos(rng_), pos(rng_), ang(rng_));
    }

    void callSpawnService(const std::string &name, double x, double y, double theta)
    {
        while (!spawn_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for spawn service...");
        }

        auto request = std::make_shared<Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        // The request is captured by the lambda, so each in-flight call keeps its own data
        spawn_client_->async_send_request(
            request,
            [this, request](rclcpp::Client<Spawn>::SharedFuture future) {
                auto response = future.get();
                if (!response->name.empty()) {
                    RCLCPP_INFO(this->get_logger(), "New alive turtle: %s",
                                response->name.c_str());
                    Turtle t;
                    t.name = response->name;
                    t.x = request->x;
                    t.y = request->y;
                    t.theta = request->theta;
                    alive_turtles_.push_back(t);
                    publishAliveTurtles();
                }
            });
    }

    void callKillService(const std::string &name)
    {
        while (!kill_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill service...");
        }

        auto request = std::make_shared<Kill::Request>();
        request->name = name;

        kill_client_->async_send_request(
            request,
            [this, name](rclcpp::Client<Kill>::SharedFuture future) {
                (void)future;
                auto it = std::find_if(
                    alive_turtles_.begin(), alive_turtles_.end(),
                    [&name](const Turtle &t) { return t.name == name; });
                if (it != alive_turtles_.end()) {
                    alive_turtles_.erase(it);
                    publishAliveTurtles();
                }
            });
    }

    std::string turtle_name_prefix_;
    double spawn_frequency_{1.0};
    int turtle_counter_{0};
    std::mt19937 rng_;
    std::vector<Turtle> alive_turtles_;

    rclcpp::Publisher<TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::Client<Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<Kill>::SharedPtr kill_client_;
    rclcpp::Service<CatchTurtle>::SharedPtr catch_turtle_service_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
