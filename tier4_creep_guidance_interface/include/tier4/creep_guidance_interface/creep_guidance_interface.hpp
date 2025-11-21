// Copyright 2025 Tier IV, Inc.
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

#ifndef TIER4__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_
#define TIER4__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <tier4_creep_guidance_msgs/msg/command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status_array.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_response.hpp>
#include <tier4_creep_guidance_msgs/msg/module.hpp>
#include <tier4_creep_guidance_msgs/msg/state.hpp>
#include <tier4_creep_guidance_msgs/srv/creep_trigger_command.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace tier4::creep_guidance_interface
{

using tier4_creep_guidance_msgs::msg::Command;
using tier4_creep_guidance_msgs::msg::CreepStatus;
using tier4_creep_guidance_msgs::msg::CreepStatusArray;
using CreepTriggerCommandMsg = tier4_creep_guidance_msgs::msg::CreepTriggerCommand;
using tier4_creep_guidance_msgs::msg::CreepTriggerResponse;
using tier4_creep_guidance_msgs::msg::Module;
using tier4_creep_guidance_msgs::msg::State;
using CreepTriggerCommandSrv = tier4_creep_guidance_msgs::srv::CreepTriggerCommand;
using unique_identifier_msgs::msg::UUID;

class CreepGuidanceInterface
{
public:
  CreepGuidanceInterface(rclcpp::Node * node, const std::string & module_name);

private:
  void on_creep_trigger_command_service(
    const CreepTriggerCommandSrv::Request::SharedPtr request,
    const CreepTriggerCommandSrv::Response::SharedPtr responses);
  void update_status(const std::vector<CreepTriggerCommandMsg> & commands);

  rclcpp::Service<CreepTriggerCommandSrv>::SharedPtr srv_creep_trigger_commands_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Clock::SharedPtr clock_;
  mutable rclcpp::Logger logger_;

  CreepStatusArray creep_status_array_;

  Module module_;
};

}  // namespace tier4::creep_guidance_interface

#endif  // TIER4__CREEP_GUIDANCE_INTERFACE__CREEP_GUIDANCE_INTERFACE_HPP_
