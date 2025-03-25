#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"  
#include "visualization_msgs/msg/marker_array.hpp"
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <random>
#include <fstream>
#include <cfloat> 

// tree typedef


enum class COLOR
{
    RED = 0,
    GREEN = 1,
    BLUE = 2,
};

namespace astar_test
{

    
    class astar_test : public rclcpp::Node{

        public:
            explicit astar_test (const rclcpp::NodeOptions & options);
            ~astar_test();

            struct astar_node{


                Eigen::Matrix4d pose;
                double fn;
                double gn;
                double hn;

            };


            void view_loop();
            void astar_loop();
            void step_astar();
            //for step astar
            std::vector<astar_node> DEBUG_NODE;
            std::vector<float> debug_potarr;
            std::vector<bool> debug_visted;
            std::vector<int> debug_came_from;


            // for viewing ..
            std::atomic<bool> view_run = false;
            std::thread *view_thread = NULL;

            // for astar ..
            std::atomic<bool> astar_run = false;
            std::atomic<bool> astar_step_flag = false;
            std::atomic<bool> used_step = false;
            //std::atomic<int> step_cycle = 0;
            std::thread *astar_thread = NULL;
            Eigen::Matrix4d start;
            Eigen::Matrix4d end;
            
            visualization_msgs::msg::Marker make_text_marker_gn(std::string text, int id, float x, float y, float z);
            visualization_msgs::msg::Marker make_text_marker_hn(std::string text, int id, float x, float y, float z);
            visualization_msgs::msg::Marker make_text_marker_fn(std::string text, int id, float x, float y, float z);
            visualization_msgs::msg::Marker make_cube_marker(int id, float x, float y, float z , COLOR color);
        private:
            void timerCallback();
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_; // Marker 퍼블리셔
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_2; // Marker 퍼블리셔

    
    };




}