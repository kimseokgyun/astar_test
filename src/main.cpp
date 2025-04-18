// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <QApplication>

#include "mainwindow.h"
// #include "qt_test.hpp"

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    MainWindow window;


    window.show();

    const rclcpp::NodeOptions options;
    auto node = std::make_shared<astar_test::astar_test>(options);

    
    std::thread ros_thread([&]() 
    {
        rclcpp::spin(node->get_node_base_interface());
        rclcpp::shutdown();
    });


    window.setQtNode(node);
    int result = app.exec();

    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return result;
}