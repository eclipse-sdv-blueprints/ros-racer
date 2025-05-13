/*
 Copyright (c) 2025 Composiv.ai

 This program and the accompanying materials are made available under the
 terms of the Eclipse Public License 2.0 which is available at
 http://www.eclipse.org/legal/epl-2.0.

 SPDX-License-Identifier: EPL-2.0

 Contributors:
   Composiv.ai - initial API and implementation
*/

#ifndef MULTIAGENT_PLUGIN_HPP_
#define MULTIAGENT_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>

namespace multiagent_plugin
{
    class MultiagentPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        MultiagentPanel(QWidget *parent = nullptr);
        virtual ~MultiagentPanel() override;

    protected:
        QPushButton *start_button_;
        QPushButton *pause_button_;
        QComboBox *agent_dropdown_;
        QLabel *lap_times_label_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr agent_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lap_times_subscriber_;

    protected Q_SLOTS:
        void onStartButtonClicked();
        void onResetButtonClicked();
        void onPauseButtonClicked();
        void onAgentSelected(int index);
        void spinSome();
    private:
        void updateLapTimesLabel(const std_msgs::msg::String::SharedPtr msg);

    };

} // namespace multiagent_plugin

#endif // MULTIAGENT_PLUGIN_HPP_
