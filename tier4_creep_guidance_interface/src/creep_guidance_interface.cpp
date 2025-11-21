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

#include "tier4/creep_guidance_interface/creep_guidance_interface.hpp"

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{
/**
 * @brief Convert UUID to string representation
 * @param uuid The UUID to convert
 * @return Hexadecimal string representation of the UUID
 */
[[nodiscard]] std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

/**
 * @brief Convert command type to string representation
 * @param type The command type value
 * @return String representation of the command type
 * @throws std::domain_error if the command type is invalid
 */
[[nodiscard]] std::string command_to_string(const uint8_t type)
{
  switch (type) {
    case tier4_creep_guidance_msgs::msg::Command::ACTIVATE:
      return "ACTIVATE";
    case tier4_creep_guidance_msgs::msg::Command::DEACTIVATE:
      return "DEACTIVATE";
    default:
      throw std::domain_error("Invalid creep guidance command type: " + std::to_string(type));
  }
}

using Module = tier4_creep_guidance_msgs::msg::Module;

Module get_module_type(const std::string & module_name)
{
  Module module;
  if (module_name == "crosswalk") {
    module.type = Module::CROSSWALK;
  } else if (module_name == "intersection") {
    module.type = Module::INTERSECTION;
  } else if (module_name == "intersection_occlusion") {
    module.type = Module::INTERSECTION_OCCLUSION;
  } else {
    module.type = Module::NONE;
  }
  return module;
}

}  // namespace

namespace tier4::creep_guidance_interface
{

CreepGuidanceInterface::CreepGuidanceInterface(rclcpp::Node * node, const std::string & module_name)
: callback_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  clock_(node->get_clock()),
  logger_(node->get_logger())
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Create service for creep trigger commands
  const std::string service_name = "~/" + module_name + "/creep_trigger_commands";
  srv_creep_trigger_commands_ = node->create_service<CreepTriggerCommandSrv>(
    service_name,
    std::bind(&CreepGuidanceInterface::on_creep_trigger_command_service, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_);

  module_ = get_module_type(module_name);
  RCLCPP_INFO(logger_, "CreepGuidanceInterface initialized with service: %s", service_name.c_str());
}

void CreepGuidanceInterface::on_creep_trigger_command_service(
  const CreepTriggerCommandSrv::Request::SharedPtr request,
  const CreepTriggerCommandSrv::Response::SharedPtr response)
{
  {
    (void)response;
    for (const auto & command : request->commands) {
      (void)command;
    }
  }
}
}  // namespace tier4::creep_guidance_interface
