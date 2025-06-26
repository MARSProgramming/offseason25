# 2025 Offseason Project - Updating Marvin 18

The intention of this project is to update the codebase of our 2025 robot, Marvin 18, to function properly with Advantage Kit and also implement Maplesim.

## Goals

The current goals of this project are listed below.
- Modernize our codebase
- Create a fully simulated model of Marvin 18 that can be driven in simulation. This model should also have scoring capability
- Give ourselves experience with IO-based abstraction
- Experiment with state machines for more streamlined code
- Improve vision implementation utilizing photonvision
- Determine the most optimal pose estimation solution
- Determine the most optimal framework for producing autonomous plays
- Learn how to use Log Replay effectively


## Usage

```
git clone https://github.com/frc2614/marvin18-offseason.git
cd marvin18-offseason
./gradlew simulateJava
```

## Task Tracker
This may be updated accordingly.

- [X] Create project and install dependencies
- [X] Set up AdvantageKit base
- [X] Create "skeleton" code subsystems
- [ ] Add LED implementation
- [ ] Create and complete separate branches for traditional, command-based control and state-based
- [ ] Build simulation model
- [ ] Real-world testing
