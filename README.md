# FRC Team 2408 *Rebuilt* 2026 Competition Code

# Features 
- Command Based
- ? DriveTrain

Extensive Documentation can be found [Here](./repo.md)

# RoadMap
## Test Mode
Test mode will be a seperate mode which will make it easy to test individual components of the robot.
either write a gradle command for it, or a constant in Constants.Settings </br>

## DriveTrain
The DriveTrain manages the components necessary for moving the robot.
Currently it is unclear if we will use SwerveDrive or Differential Drive (Tank Drive)
- [ ] Constants
- [ ] SubSystem
- [ ] Command

## Shooter
The Shooter manages the components required for shooting the "fuel"
- [ ] Constants
- [ ] SubSystem
- [ ] Command

## Shooter Intake
The Shooter intake manages the "fuel" feeding system for the shooter subsystem
- [ ] Constants
- [ ] SubSystem
- [ ] Command

## Climber
The Climber manages the components which will allow the robot to climb (or at least hang)
- [ ] Constants
- [ ] SubSystem
- [ ] Command

# Documentation for 2027
preferably document the code well enough so that who ever is on the programming team next year (2027) will be able to understand the codebase quicker, without having to look at the wpidocs.
because overly documented code via comments is cluttered i would prefer if we used seperate markdown (.md) files at
- [commands/commands.md](src/main/java/frc/robot/commands/commands.md)
- [subsystems/subsystems.md](src/main/java/frc/robot/subsystems/subsystems.md)
- [robot.md](src/main/java/frc/robot/robot.md)
- [repo.md](./repo.md)

which will detail the structure and implementation details of each of its respective code files.
comments can still be used to document code but i want to avoid large multi line comments.

# AI
Please no directly AI generated code in the repo's java code. you may use AI to consult implementation ideas but the code must be written by a real person.

# Library Documentation
- [WPILib](docs.wpilib.org)
<!-- - [YAGSL](docs.yagsl.com) (Yet Another Swerve Library) -->
