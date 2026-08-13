#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "vicon_receiver/msg/position.hpp"
#include "vicon_receiver/msg/position_list.hpp"

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    double translation[3];
    double rotation[4];
    double rotation_euler[3];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

} typedef PositionStruct;


// Class that allows segment data to be published in a ROS2 topic.
class Publisher
{
private:
    rclcpp::Node* node_;
    rclcpp::Publisher<vicon_receiver::msg::Position>::SharedPtr position_publisher_;
    rclcpp::Publisher<vicon_receiver::msg::PositionList>::SharedPtr position_list_publisher_;

public:
    bool is_ready = false;

    Publisher(std::string topic_name, rclcpp::Node* node, bool default_topic=false);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(PositionStruct p);
    // Publishes the given position list in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(std::vector <PositionStruct> p);
};

#endif