
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

        astar_run = true;
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
            //RCLCPP_INFO(get_logger(), "astar_step_flag : %d", astar_step_flag.load());

            //RCLCPP_INFO(get_logger(), " start : %f %f %f", start(0,3), start(1,3), start(2,3));
            //astar parameter setting..
            int id_idx = 0;

            visualization_msgs::msg::MarkerArray text_marker_array;

            for (auto& p : DEBUG_NODE)
            {
                visualization_msgs::msg::Marker cube = make_cube_marker(id_idx,(float)p.pose(0,3),(float)p.pose(1,3),0,COLOR::GREEN);
                id_idx++;

                float gn_ = p.gn;
                std::string gn_text = std::to_string(gn_);
                visualization_msgs::msg::Marker gn = make_text_marker_gn(gn_text,id_idx,(float)p.pose(0,3),(float)p.pose(1,3),0,0);

                id_idx++;
                float hn_ = p.hn;
                std::string hn_text = std::to_string(hn_);
                visualization_msgs::msg::Marker hn = make_text_marker_hn(hn_text,id_idx,(float)p.pose(0,3),(float)p.pose(1,3),0,0);

                id_idx++;
                float fn_ = p.fn;
                std::string fn_text = std::to_string(fn_);
                visualization_msgs::msg::Marker fn = make_text_marker_fn(fn_text,id_idx,(float)p.pose(0,3),(float)p.pose(1,3),0,0);
                id_idx++;
                
                text_marker_array.markers.push_back(cube);
            }

            marker_publisher_->publish(text_marker_array);
            std::this_thread::sleep_for(10ms);
        }
    }


    void astar_test::astar_loop()
    {

        const double resolution = 0.05;

        const int grid_size = 100;
        const int ns = grid_size * grid_size;

        int grid_center = grid_size / 2;

        Eigen::Vector2i start_grid(grid_center + round(start(0, 3) / resolution),
                                grid_center - round(start(1, 3) / resolution));

        Eigen::Vector2i end_grid(grid_center + round(end(0, 3) / resolution),
                                grid_center - round(end(1, 3) / resolution));

        std::vector<float> potarr(ns, FLT_MAX);
        std::vector<bool> visited(ns, false);
        std::vector<int> came_from(ns, -1);
        std::vector<float> gn(ns, 0.0);
        std::vector<float> fn(ns, 0.0);
        std::vector<float> hn(ns, 0.0);
        std::vector<int> curP, nextP, overP;


        //just array
        int startCell = start_grid.x() * grid_size + start_grid.y();
        int goalCell = end_grid.x() * grid_size + end_grid.y();


        potarr[startCell] = 0.0;
        curP.push_back(startCell);

        std::vector<Eigen::Vector2i> directions =
        {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        float curT = std::hypot(end_grid.x() - start_grid.x(), end_grid.y() - start_grid.y());

        float priInc = 2.0f;
        int cycles = 10000;

        const double w_d = 1.0;
        const double w_theta = 5.0;
        const double k = 0.1;

        bool goal_found = false;

        for (int cycle = 0; cycle < cycles; cycle++)
        {
            if (curP.empty() && nextP.empty())
                break;

            std::vector<int> tempP;

            for (int idx : curP)
            {
                int x = idx / grid_size;
                int y = idx % grid_size;

                if (idx == goalCell) {
                    goal_found = true;
                    break;
                }

                for (auto &dir : directions)
                {
                    int new_x = x + dir.x();
                    int new_y = y + dir.y();
                    int new_idx = new_x * grid_size + new_y;

                    if (new_x < 0 || new_x >= grid_size || new_y < 0 || new_y >= grid_size || visited[new_idx])
                    {

                        continue;
                    }
                    double g = potarr[idx] + 1;//std::hypot(dir.x(), dir.y());

                    double distance = std::hypot(end_grid.x() - new_x, end_grid.y() - new_y);

                    double theta_new = std::atan2(end_grid.y() - new_y, end_grid.x() - new_x);
                    double theta_old = std::atan2(end_grid.y() - y, end_grid.x() - x);
                    double angle_diff = std::abs(std::fmod(theta_new - theta_old + M_PI, 2 * M_PI) - M_PI);

                    double angle_weight = w_theta * angle_diff * std::exp(-k * distance);

                    double h = w_d * distance + angle_weight;

                    float f = g + h;

                    if (f < potarr[new_idx])
                    {
                        potarr[new_idx] = f;
                        fn[new_idx] = f;
                        gn[new_idx] = g;
                        hn[new_idx] = h;

                        came_from[new_idx] = idx;
                        if (f < curT)
                            nextP.push_back(new_idx);
                        else
                            tempP.push_back(new_idx);
                    }
                }
            }

            if (goal_found)
            {
                break;
            }
            curP = nextP;
            nextP = tempP;

            if (curP.empty())
            {
                curT += priInc;
                curP = overP;
                overP.clear();
            }
        }

        if (!goal_found) return {};

        std::vector<Eigen::Matrix4d> path;
        for (int idx = goalCell; idx != startCell && idx != -1; idx = came_from[idx])
        {
            astar_node debug_node;
            debug_node.pose = Eigen::Matrix4d::Identity();
    
            int x = idx / grid_size;
            int y = idx % grid_size;

            debug_node.pose(0, 3) = (x - grid_center) * resolution;
            debug_node.pose(1, 3) = (y - grid_center) * resolution;
            debug_node.pose(2, 3) = 0.0;
            debug_node.pose(3, 3) = 1.0;
            debug_node.fn = fn[idx];
            debug_node.hn = hn[idx];
            debug_node.gn = gn[idx];
            DEBUG_NODE.push_back(debug_node);

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose(0, 3) = (x - grid_center) * resolution;
            pose(1, 3) = (y - grid_center) * resolution;
            path.push_back(pose);
            Eigen::Vector3d debug_pt(pose(0,3), pose(1,3), 0.0);
            debug_frame.push_back(debug_pt);
        }

        std::reverse(path.begin(), path.end());
        // return path;
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