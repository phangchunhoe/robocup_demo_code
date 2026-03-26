#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;

int main(int argc, char **argv)
{
    // Starts ROS2
    rclcpp::init(argc, argv);

    // The Behavioral Trees live inside brain class
    // brain -> robot's decision making system
    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    // initialize operations: read parameters, construct BehaviorTree, etc.
    // init() loads:
    // - Behavioral Trees (XML)
    // - Parameters
    // - Sensors / Configs

    brain->init();

    // Start a separate thread to run the tick function of Brain at a fixed frequency (HZ). The main thread will handle ROS2 callbacks.

    // Start a separate thread to execute brain.tick
    // Dedicated to the BT tick loop. Ensures that robots
    // decision making logic stays at 100Hz regardless

    thread t([&brain]() {
        while (rclcpp::ok()) {
            auto start_time = brain->get_clock()->now();
            brain->tick();

            // Starts running Behavioral Trees 
            // Every Tick starts from the root
            auto end_time = brain->get_clock()->now();
            auto duration = (end_time - start_time).nanoseconds() / 1000000.0; // Convert to milliseconds

            // Performance logging
            brain->log->log_scalar("performance", "brain_tick_duration_ms", duration);
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        } 
    });

    /*
        What this listens to:
            - /remote_controller_state (manual control/override)
            - /robocup/game_controller 
              (Referee System -> tells your robot when it is penalty, stop etc)

        These function gets triggered:
            - Brain::joystickCallback(...)
            - Brain::gameControlCallback(...)
        These updates the blackboard / internal state
    */

    // Start a separate thread to handle joystick and gamecontroller callbacks
    thread t1([&brain, &argc, &argv]() {
        // Use a separate context
        auto context = rclcpp::Context::make_shared();
        context->init(argc, argv);
        rclcpp::NodeOptions opt;
        opt.context(context);
        auto node = rclcpp::Node::make_shared("brain_node_ext", opt);
        auto sub1 = node->create_subscription<booster_interface::msg::RemoteControllerState>("/remote_controller_state", 10, bind(&Brain::joystickCallback, brain, std::placeholders::_1));
        auto sub2 = node->create_subscription<game_controller_interface::msg::GameControlData>("/booster_soccer/game_controller", 10, bind(&Brain::gameControlCallback, brain, std::placeholders::_1));
        auto sub3 = node->create_subscription<std_msgs::msg::String>("/booster_agent/soccer_game_control", 1, bind(&Brain::agentCommandCallback, brain, std::placeholders::_1));
    
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin(); 
    });

    // Use a single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // This handles callbacks inside `Brain`, sensors and internal communication
    executor.add_node(brain);
    executor.spin();

    t.join();
    t1.join();
    rclcpp::shutdown();
    return 0;
}
