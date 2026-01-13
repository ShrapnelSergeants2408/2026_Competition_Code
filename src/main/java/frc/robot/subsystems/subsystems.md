# Subsystems
A Subsystem is a term used in Robotics to denote a major group of components of a Robot, such as the Driving mechanism which allows the robot to move, or the Shooting mechanism which allows the Robot to score points. In the code, these subsystems are classes defined in the [subsystems](.) directory and define the various functions of the subsystem, which can be called in [Commands](../commands/commands.md). 

## Structure
Each Subsystem class is structured in a similar way. all subsystems extend from the SubsystemBase class provided by WPILib,
