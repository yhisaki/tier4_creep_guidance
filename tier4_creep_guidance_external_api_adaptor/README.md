# tier4_creep_guidance_external_api_adaptor

## Overview

This package provides a unified external API interface for controlling multiple creep guidance modules in the Autoware system. It aggregates status information from different creep guidance modules (crosswalk, intersection, intersection_occlusion) and provides a single service interface for sending trigger commands.

## Features

- **Status Aggregation**: Collects and publishes creep status from multiple modules in a unified format
- **Unified Command Interface**: Provides a single service endpoint to control all creep guidance modules
- **Distance-based Sorting**: Automatically sorts creep statuses by start distance for better visualization and prioritization
- **Multi-threaded Execution**: Requires multi-threaded executor to handle nested service calls without deadlock

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
- Publishes aggregated status to `/api/external/get/creep_status` at 10Hz
- Provides service interface at `/api/external/set/creep_trigger_commands`
- Performs sorting and validation of status data
- Routes commands to appropriate module based on module type

## Topics

### Subscribed Topics

- `/planning/creep_guidance_status/crosswalk` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
- `/planning/creep_guidance_status/intersection` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
- `/planning/creep_guidance_status/intersection_occlusion` (tier4_creep_guidance_msgs/msg/CreepStatusArray)

### Published Topics

- `/api/external/get/creep_status` (tier4_creep_guidance_msgs/msg/CreepStatusArray)
  - Aggregated status from all creep guidance modules, sorted by start distance
  - Published at 10Hz

## Services

### Provided Services

- `/api/external/set/creep_trigger_commands` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
  - Service for sending trigger commands to specific creep guidance modules
  - Routes commands to appropriate modules based on module type field
  - Returns aggregated responses from all called modules

### Called Services

- `/planning/creep_guidance_commands/crosswalk` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
- `/planning/creep_guidance_commands/intersection` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)
- `/planning/creep_guidance_commands/intersection_occlusion` (tier4_creep_guidance_msgs/srv/CreepTriggerCommand)

## Usage

### Launch

```bash
# Recommended: Use the provided launch file
ros2 launch tier4_creep_guidance_external_api_adaptor tier4_creep_guidance_external_api_adaptor.launch.xml
```

### Send Command Example

Activate intersection creep guidance with ID 1:

```bash
ros2 service call /api/external/set/creep_trigger_commands tier4_creep_guidance_msgs/srv/CreepTriggerCommand \
  "{stamp: {sec: 0, nanosec: 0},
    commands: [{id: 1, module: {type: 2}, command: {type: 1}}]}"
```

Module types:

- `0`: NONE
- `1`: CROSSWALK
- `2`: INTERSECTION
- `3`: INTERSECTION_OCCLUSION

Command types:

- `0`: DEACTIVATE
- `1`: ACTIVATE

### Monitor Status

```bash
ros2 topic echo /api/external/get/creep_status
```

## Design Notes

This implementation follows the same pattern as `rtc_controller` in the Autoware external API adaptor.
