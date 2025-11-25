//
//  Copyright 2024 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef CREEP_GUIDANCE_PANEL_HPP_
#define CREEP_GUIDANCE_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTableWidget>

#ifndef Q_MOC_RUN
// cpp
#include <memory>
#include <string>
// ros
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
// autoware
#include <tier4_creep_guidance_msgs/msg/creep_status.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status_array.hpp>
#endif

namespace rviz_plugins
{
using tier4_creep_guidance_msgs::msg::CreepStatus;
using tier4_creep_guidance_msgs::msg::CreepStatusArray;

class CreepGuidancePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CreepGuidancePanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  void onCreepStatus(const CreepStatusArray::ConstSharedPtr msg);

private:
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<CreepStatusArray>::SharedPtr sub_creep_status_;

  QLabel * status_label_;
  QTableWidget * status_table_;
};

}  // namespace rviz_plugins

#endif  // CREEP_GUIDANCE_PANEL_HPP_
