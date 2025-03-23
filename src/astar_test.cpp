
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "astar_test.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

// #define first_move_icp

namespace astar_test
{
    astar_test::astar_test(const rclcpp::NodeOptions& options)
    : Node("astar_test",options)
    {
        RCLCPP_INFO(get_logger(),"a* test starting..");

        //set variable
        start = Eigen::Matrix4d::Identity();
        end = Eigen::Matrix4d::Identity();

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_msg", 10);

        if(view_thread == NULL)
        {
            view_thread = new std::thread(&astar_test::view_loop, this);
        }

        if(astar_thread == NULL)
        {
            astar_thread = new std::thread(&astar_test::astar_loop, this);
        }
        

        
        // timer_ = this->create_wall_timer(50ms,std::bind(&astar_test::timerCallback,this));
    }

    astar_test::~astar_test()
    {
        if(view_thread != NULL)
        {
            view_thread->join();
            delete view_thread;
        }

        if(astar_thread != NULL)
        {
            astar_thread->join();
            delete astar_thread;
        }
    }


    void astar_test::view_loop()
    {
        // RCLCPP_INFO(get_logger(),"view loop start..");
        while(1)
        {
            RCLCPP_INFO(get_logger(), "start matrix: [%lf, %lf, %lf]",
                        start(0, 3), start(1, 3), start(2, 3)); 

            visualization_msgs::msg::MarkerArray text_marker_array;
            visualization_msgs::msg::Marker t_1 = make_text_marker_gn("gn",0,0,0,0);
            visualization_msgs::msg::Marker t_2 = make_text_marker_hn("hn",1,0,0,0);
            visualization_msgs::msg::Marker t_3 = make_text_marker_fn("fn",2,0,0,0);
            visualization_msgs::msg::Marker cube = make_cube_marker(3,0,0,0,COLOR::BLUE);
            text_marker_array.markers.push_back(t_1);
            text_marker_array.markers.push_back(t_2);
            text_marker_array.markers.push_back(t_3);
            text_marker_array.markers.push_back(cube);
            marker_publisher_->publish(text_marker_array);

            std::this_thread::sleep_for(10ms);
        }
    }


    void astar_test::astar_loop()
    {
        // RCLCPP_INFO(get_logger(),"astar loop start..");
        while(astar_run)
        {
            RCLCPP_INFO(get_logger(),"astar loop..");












            std::this_thread::sleep_for(100ms);
        }
    }


    visualization_msgs::msg::Marker astar_test::make_cube_marker(int id, float x, float y, float z, COLOR color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;

        if(color == COLOR::RED)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
        }

        else if (color == COLOR::GREEN)
        {

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        }

        else if (color == COLOR::BLUE)
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        }


        marker.color.a = 0.3;
        return marker;
    }

    visualization_msgs::msg::Marker astar_test::make_text_marker_gn(std::string text, int id, float x, float y, float z)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x-0.25;
        marker.pose.position.y = y+0.25;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.text = text;
        return marker;
    }

    visualization_msgs::msg::Marker astar_test::make_text_marker_hn(std::string text, int id, float x, float y, float z)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x-0.25;
        marker.pose.position.y = y-0.25;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.text = text;
        return marker;
    }
        visualization_msgs::msg::Marker astar_test::make_text_marker_fn(std::string text, int id, float x, float y, float z)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x+0.25;
        marker.pose.position.y = y-0.25;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.text = text;
        return marker;
    }
}  // namespace astar_test