// Copyright 2025 TIER IV, Inc.
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

#ifndef CREEP_GUIDANCE_CONTROLLER_HPP_
#define CREEP_GUIDANCE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <tier4_creep_guidance_msgs/msg/command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status_array.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_response.hpp>
#include <tier4_creep_guidance_msgs/msg/module.hpp>
#include <tier4_creep_guidance_msgs/msg/state.hpp>
#include <tier4_creep_guidance_msgs/srv/creep_trigger_command.hpp>

#include <memory>
#include <string>
#include <vector>

using CreepTriggerCommandSrv = tier4_creep_guidance_msgs::srv::CreepTriggerCommand;
using CreepStatusArray = tier4_creep_guidance_msgs::msg::CreepStatusArray;
using CreepStatus = tier4_creep_guidance_msgs::msg::CreepStatus;
using CreepTriggerCommand = tier4_creep_guidance_msgs::msg::CreepTriggerCommand;
using CreepTriggerResponse = tier4_creep_guidance_msgs::msg::CreepTriggerResponse;
using Module = tier4_creep_guidance_msgs::msg::Module;
using Command = tier4_creep_guidance_msgs::msg::Command;
using State = tier4_creep_guidance_msgs::msg::State;

class CreepGuidanceModule
{
public:
  std::vector<CreepStatus> module_statuses_;
  rclcpp::Subscription<CreepStatusArray>::SharedPtr module_sub_;
  tier4_api_utils::Client<CreepTriggerCommandSrv>::SharedPtr cli_set_module_;

  CreepGuidanceModule(rclcpp::Node * node, const std::string & name);
  void module_callback(const CreepStatusArray::ConstSharedPtr message);
  void insert_message(std::vector<CreepStatus> & creep_statuses);
  void call_service(
    CreepTriggerCommandSrv::Request::SharedPtr request,
    const CreepTriggerCommandSrv::Response::SharedPtr & responses);
};

namespace external_api
{
class CreepGuidanceController : public rclcpp::Node
{
public:
  explicit CreepGuidanceController(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<CreepGuidanceModule> crosswalk_;
  std::unique_ptr<CreepGuidanceModule> intersection_;
  std::unique_ptr<CreepGuidanceModule> intersection_occlusion_;

  /* publishers */
  rclcpp::Publisher<CreepStatusArray>::SharedPtr creep_status_pub_;

  /* service from external */
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<CreepTriggerCommandSrv>::SharedPtr srv_set_creep_trigger_;

  /* Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  void set_creep_trigger(
    const CreepTriggerCommandSrv::Request::SharedPtr requests,
    const CreepTriggerCommandSrv::Response::SharedPtr responses);

  // ros callback
  void on_timer();
};

}  // namespace external_api

#endif  // CREEP_GUIDANCE_CONTROLLER_HPP_
