# tier4_creep_guidance_external_api_adaptor

## Overview

This package provides a unified external API interface for controlling multiple creep guidance modules in the Autoware system. It aggregates status information from different creep guidance modules (crosswalk, intersection, intersection_occlusion) and provides a single service interface for sending trigger commands.

## Features

- **Status Aggregation**: Collects and publishes creep status from multiple modules in a unified format
- **Unified Command Interface**: Provides a single service endpoint to control all creep guidance modules
- **Distance-based Sorting**: Automatically sorts creep statuses by start distance for better visualization and prioritization

## Architecture

The package consists of two main classes:

### CreepGuidanceModule

Manages the communication with a single creep guidance module:

- Subscribes to the module's status topic
- Provides a service client for sending commands to the module
- Aggregates status information for publishing

### CreepGuidanceController

Main controller node that:

- Manages multiple `CreepGuidanceModule` instances (one per module type)
- Publishes aggregated status to `/api/external/get/creep_status`
- Provides service interface at `/api/external/set/creep_trigger_commands`
- Performs sorting and validation of status data

## Topics

### Subscribed Topics

- `/planning/creep_guidance/crosswalk/creep_status_array` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
- `/planning/creep_guidance/intersection/creep_status_array` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
- `/planning/creep_guidance/intersection_occlusion/creep_status_array` (tier4_creep_guidance_msgs/msg/CreepStatusArray)

### Published Topics

- `/api/external/get/creep_status` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
  - Aggregated status from all creep guidance modules, sorted by start distance

## Services

### Provided Services

- `/api/external/set/creep_trigger_commands` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
  - Service for sending trigger commands to specific creep guidance modules

### Called Services

- `/planning/creep_trigger_commands/crosswalk/creep_trigger_commands` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
- `/planning/creep_trigger_commands/intersection/creep_trigger_commands` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
- `/planning/creep_trigger_commands/intersection_occlusion/creep_trigger_commands` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)

## Usage

### Launch

```bash
ros2 run tier4_creep_guidance_external_api_adaptor tier4_creep_guidance_external_api_adaptor_node
```

### Send Command Example

```bash
ros2 service call /api/external/set/creep_trigger_commands tier4_creep_guidance_msgs/srv/CreepTriggerCommand \
  "{stamp: {sec: 0, nanosec: 0},
    commands: [{id: 1, module: {type: 1}, command: {type: 1}}]}"
```

## Design Notes

This implementation is based on the `rtc_controller` design pattern from the Autoware external API adaptor, adapted for the creep guidance system.

## Dependencies

- rclcpp
- rclcpp_components
- tier4_api_utils
- tier4_creep_guidance_msgs
