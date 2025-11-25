# tier4_creep_guidance_interface

## Overview

The `tier4_creep_guidance_interface` package provides an interface for managing creep guidance status and commands in autonomous driving behavior planning modules. This interface enables behavior modules (such as crosswalk, intersection, and intersection occlusion modules) to coordinate creep maneuvers with external systems.

## Usage

### Basic Integration Example

```cpp
#include "tier4/creep_guidance_interface/creep_guidance_interface.hpp"

class YourBehaviorModule : public rclcpp::Node
{
public:
  YourBehaviorModule(const int64_t module_id) : Node("your_module")
  {
    // Initialize the creep guidance interface with module name
    creep_interface_ = std::make_shared<tier4::creep_guidance_interface::CreepGuidanceInterface>(
      this, "crosswalk");
    creep_interface_->add(module_id);
  }

  ~YourBehaviorModule()
  {
    creep_interface_->remove(module_id);
  }

  void planPath()
  {
    creep_interface_->update_distance(id, getStartDistance(), getFinishDistance());

    if (creep_interface_->is_activated(id)) {
      // Execute creep maneuver
      executeCreepManeuver();
      creep_interface_->update_state(id, State::ACTIVATED);
    }

    // Publish status
    creep_interface_->publish_creep_status_array();
  }

private:
  std::shared_ptr<tier4::creep_guidance_interface::CreepGuidanceInterface> creep_interface_;
};
```

## API Reference

### Constructor

```cpp
CreepGuidanceInterface(rclcpp::Node * node, const std::string & name)
```

Creates a new interface instance for the specified module.

**Parameters:**

- `node`: Pointer to the ROS2 node
- `name`: Module name (supported values: "crosswalk", "intersection", "intersection_occlusion")

## ROS2 Interface

### Topics

#### Publishers

- `~/{module_name}/creep_status_array` (`tier4_creep_guidance_msgs/msg/CreepStatusArray`)
  - Publishes the array of all registered creep guidance statuses
  - QoS: Default (queue size: 1)

### Services

#### Servers

- `~/{module_name}/creep_trigger_commands` (`tier4_creep_guidance_msgs/srv/CreepTriggerCommand`)
  - Receives commands to trigger creep maneuvers
  - Returns success/failure status for each command

## Module Types

The interface supports the following module types:

- `CROSSWALK`: For pedestrian crosswalk scenarios
- `INTERSECTION`: For intersection scenarios
- `INTERSECTION_OCCLUSION`: For intersection scenarios with occlusion handling
- `NONE`: Default/unspecified module type
