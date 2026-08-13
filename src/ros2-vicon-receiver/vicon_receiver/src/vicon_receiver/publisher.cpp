#include "vicon_receiver/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node, bool default_topic)
{
    node_ = node;
    if (!default_topic)
        position_publisher_ = node->create_publisher<vicon_receiver::msg::Position>(topic_name, 10);
    else
        position_list_publisher_ = node->create_publisher<vicon_receiver::msg::PositionList>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    auto msg = std::make_shared<vicon_receiver::msg::Position>();
    msg->x_trans = p.translation[0];
    msg->y_trans = p.translation[1];
    msg->z_trans = p.translation[2];
    msg->x_rot = p.rotation[0];
    msg->y_rot = p.rotation[1];
    msg->z_rot = p.rotation[2];
    msg->w = p.rotation[3];
    msg->x_rot_euler = p.rotation_euler[0];
    msg->y_rot_euler = p.rotation_euler[1];
    msg->z_rot_euler = p.rotation_euler[2];
    msg->subject_name = p.subject_name;
    msg->segment_name = p.segment_name;
    msg->frame_number = p.frame_number;
    msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}

void Publisher::publish(std::vector <PositionStruct> p)
{   
    auto positionList = std::make_shared<vicon_receiver::msg::PositionList>();
    positionList -> n = p.size();
    positionList->header.stamp = node_->now();        
    positionList->header.frame_id = "vicon_world";    
    for(int i = 0; i < positionList->n; i++){
        auto msg = std::make_shared<vicon_receiver::msg::Position>();
        msg->x_trans = p[i].translation[0];
        msg->y_trans = p[i].translation[1];
        msg->z_trans = p[i].translation[2];
        msg->x_rot = p[i].rotation[0];
        msg->y_rot = p[i].rotation[1];
        msg->z_rot = p[i].rotation[2];
        msg->w = p[i].rotation[3];
        msg->x_rot_euler = p[i].rotation_euler[0];
        msg->y_rot_euler = p[i].rotation_euler[1];
        msg->z_rot_euler = p[i].rotation_euler[2];
        msg->subject_name = p[i].subject_name;
        msg->segment_name = p[i].segment_name;
        msg->frame_number = p[i].frame_number;
        msg->translation_type = p[i].translation_type;
        positionList -> positions.push_back(*msg);
    }
    position_list_publisher_ -> publish(*positionList);
}
