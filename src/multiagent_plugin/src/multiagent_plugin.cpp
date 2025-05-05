/*
 Copyright (c) 2025 Composiv.ai

 This program and the accompanying materials are made available under the
 terms of the Eclipse Public License 2.0 which is available at
 http://www.eclipse.org/legal/epl-2.0.

 SPDX-License-Identifier: EPL-2.0

 Contributors:
   Composiv.ai - initial API and implementation
*/

#include "multiagent_plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>
namespace multiagent_plugin
{
    MultiagentPanel::MultiagentPanel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        // Initialize elements
        start_button_ = new QPushButton("Start Race", this);
        pause_button_ = new QPushButton("Pause/Resume Race", this);
        agent_dropdown_ = new QComboBox(this);
        agent_dropdown_->addItem("Agent1");
        agent_dropdown_->addItem("Agent2");
        agent_dropdown_->addItem("Agent3");
        agent_dropdown_->addItem("Set Lap Point");
        QTimer *ros_spin_timer_;

        QVBoxLayout *layout = new QVBoxLayout;

        QLabel *label = new QLabel("Choose Pose Estimation Option:", this);
        lap_times_label_ = new QLabel("", this);
        layout->addWidget(lap_times_label_);
        layout->addWidget(label);
        layout->addWidget(agent_dropdown_);
        layout->addWidget(start_button_);
        layout->addWidget(pause_button_);
        setLayout(layout);

        node_ = std::make_shared<rclcpp::Node>("multiagent_plugin_node");
        lap_times_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "lap_times", 10, std::bind(&MultiagentPanel::updateLapTimesLabel, this, std::placeholders::_1));

        start_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_start", 10);
        pause_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_pause", 10);
        agent_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("racecar_to_estimate_pose", 10);
        ros_spin_timer_ = new QTimer(this);
        connect(ros_spin_timer_, SIGNAL(timeout()), this, SLOT(spinSome()));
        ros_spin_timer_->start(100);
        connect(agent_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onAgentSelected(int)));
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
    }

    void MultiagentPanel::spinSome()
    {
        rclcpp::spin_some(node_);
    }

    void MultiagentPanel::updateLapTimesLabel(const std_msgs::msg::String::SharedPtr msg)
    {
        lap_times_label_->setText(QString::fromStdString(msg->data));
    }

    MultiagentPanel::~MultiagentPanel() {}
    void MultiagentPanel::onAgentSelected(int index)
    {
        std_msgs::msg::Int32 msg;
        msg.data = index;
        agent_publisher_->publish(msg);
    }
    void MultiagentPanel::onStartButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        start_publisher_->publish(msg);
    }

    void MultiagentPanel::onResetButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        reset_publisher_->publish(msg);
    }

    void MultiagentPanel::onPauseButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pause_publisher_->publish(msg);
    }

} // namespace multiagent_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multiagent_plugin::MultiagentPanel, rviz_common::Panel)
