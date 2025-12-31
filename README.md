# Team 18650 SDK - INTO THE DEEP Season (2024-2025) [ARCHIVED]

> **This repository is archived.** It contains historical code from the INTO THE DEEP competition season for reference only.

This was Team 18650's primary development repo for INTO THE DEEP - the result of significant development effort throughout the season.

## What This Code Does

- **Road Runner integration** for precise autonomous navigation
- **AprilTag vision** for field positioning
- **Multiple autonomous routines** (specimen delivery, sample push paths, net zone)
- **Hardware abstraction layer** for cleaner code organization
- **Trajectory sequences** for complex autonomous movements

## Code Structure

```
TeamCode/
├── AutonomousCore.java          # Base autonomous functionality
├── AutonomousNetDelivery.java   # Net zone delivery routine
├── AutonomousSpecimenDelivery.java
├── Autopilot.java               # Navigation control
├── hardware/                    # Hardware abstraction
├── apriltag/                    # Vision processing
├── trajectorysequence/          # Road Runner paths
└── util/                        # Utility classes
```

## Branches

This repo has many experimental branches from development:
- `master` - Competition-ready code
- `road-runner2`, `roadrunner-retuning` - Drive tuning
- `collector-tuning` - Mechanism adjustments
- `aprilTagTesting` - Vision experiments
- `bulk-reads`, `feed-forward` - Performance optimizations

## Development History

This codebase (originally `CallamJ/Into-the-Deep-18650`) represents what was used at the **2024 Vermont Championship** and **New England Premier** events.

Mid-season, Callam forked the repository to work on a `hardware-wrapper` architecture with the following design goals:
- **"Smart" hardware wrappers** (`SmartMotor`, `SmartServo`, `SmartEncoder`, etc.) wrapping FTC SDK classes
- **Hardware caching system** to avoid repeated hardware map lookups
- **Component abstractions** (`TelescopingArm`, `Grip`, `PitchWrist`, `RollWrist`)
- **Deep inheritance hierarchies** for OpModes

That fork ([18650-sdk-intothedeep-fork](../18650-sdk-intothedeep-fork)) is preserved for reference. During the competition season, the team used the more direct approach in this repo, while Callam's architectural work continued to mature and eventually formed the basis for [18650-sdk-decode](../18650-sdk-decode) in the DECODE season.

---

## About

FTC 18650 is part of [Bennington Area Robotics](https://github.com/bennington-area-robotics), based in Bennington, Vermont.

## Technical Details

- **Season:** INTO THE DEEP (2024-2025)
- **Status:** Archived
- **Key Libraries:** Road Runner, FTC SDK AprilTag
