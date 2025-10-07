# Phase Update

> [!CAUTION]
> Advises about risks or negative outcomes of certain actions.

## Phase 1
Phase 1: API and Database Integration for Autonomous Forklift
This phase focuses on testing the API integration with the Autonomous Forklift and its database to retrieve and utilize pallet data.

The following four commands have been implemented and are currently being tested:

Pick Pallet to Init: Instructs the forklift to pick up a pallet and move it to the initial position.

Place Pallet from Init: Commands the forklift to place a pallet down from its initial position.

Pick Pallet to Manipulator Station: Directs the forklift to pick up a pallet and transport it to the manipulator station.

Place Pallet from Manipulator Station: Tells the forklift to place a pallet down after arriving at the manipulator station.

# AssignTask
## Assign Task type

**This is Task type id when you call assign task**

| Task Type ID | Description |
|--------------|-------------|
| 1 | PickInit |
| 2 | PlaceInit |
| 3 | PickToManipulator |
| 4 | PickFromManipulator |

# Developer 
* 65340500004 Kraiwich Vichakhot