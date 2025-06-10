#include <moveit_interface.h> 
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("moveit_interface");
    // auto moveitInterfaceNode = std::make_shared<MoveitInterface>(node);
    auto moveitInterfaceNode = std::make_shared<MoveitInterface>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveitInterfaceNode);

    auto spinner = std::thread([&executor]() { 
        executor.spin(); 
    });
    
    moveitInterfaceNode->run();

    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner.join();  // <--- Join the thread before exiting
    return 0;
}