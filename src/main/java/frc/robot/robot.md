# What is "Command Based"?
Command based is one of the paradigms (way of structuring code) used in developing a robot, in a Command Based robot, we define Command functions which coordinate multiple subsystems together to do something we want. we can run these command functions every time we press a button on the controller or automated via the test mode. the main robot runtime is just a loop that dispatches commands when they are requested, such as when a button is pressed or every 20 milliseconds if they are registered in the `robotPeriodic` function in `Robot.java`. In this repo, we use a Command Based structure. it is more complicated than Timed but also allows for more complex behavior of our robot.


# Files
## Main.java
this is a default file which defines the entry point class that java requires, we will never touch this file. all it does define the mandatory `public static void main(String... args){...}` function and immediately call the `Robot`'s start function.

## Robot.java
this is a default file which defines the behavior we want our robot runtime to have, mainly it defines when to run what commands. this is the most abstract file we will modify. though we do not modify it often.

## RobotContainer.java
this is a default file which defines the actual structure of our robot, this is where we initialize subsystems and configure the controller, this is wher most modification happens relative to the other folders in this directory.

## Result.java
this is a custom class used for testing each component of the robot programmically, each test returns one Result, which is then reported to in the console (or networktables... unsure.) 
## subsystems/
This is the directory which stores all our subsystem classes,
