# Subsystems
A Subsystem is a term used in Robotics to denote a major group of components of a Robot, such as the Driving mechanism which allows the robot to move, or the Shooting mechanism which allows the Robot to score points. In the code, these subsystems are classes defined in the [subsystems](.) directory and define the various functions of the subsystem, which can be called in [Commands](../commands/commands.md). 

as Subsystems interface directly with hardware through wpilib, this is the least abstract layer of the codebase, and where the most code will be written. 

## Structure
Each Subsystem class is structured in a similar way. all subsystems extend from the SubsystemBase class provided by WPILib,

## Components
WPILib and other external libraries provide many classes which interface with robot hardware

### SparkMax
To use sparkmax... (TODO)

# Commands
old way to do commands is to create a Command class which produces essentially a function using the initialize(), execute(), end(), and isFinished(). However new method is to define a function and then pass it to InstantCommand(), RunCommand, StartEndCommand(), the new method is prefered and easier to work with, as it requires less boilerplate.

Commands are high level functions of the subsystem which are coordinated by the Robot's CommandScheduler. Command functions 

# Shooter
Shooter is the subsystem used for shooting the fuel into the scoring net. it consists of 2 sparkmax motors, the shooter motor and feeding motor, which are configured via the SparkMaxConfig class to add 