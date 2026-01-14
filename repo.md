# Introduction
This is the Spring 2026 FRC Codebase for Team 2408 written in the [Java](https://en.wikipedia.org/wiki/Java_(programming_language)) programming language and using the [WPILib](docs.wpilib.org) Library, it runs on the [RoboRIO](https://www.ni.com/en-us/support/model.roborio.html) Computer, which controls the robot's actions. 

Although WPILib supports Java along with C++ and Python, we only use Java. For an Intro into Java, see [W3Schools](https://w3schools.com/java) or [JavaByExample](https://javabyexample.com/) if you already have programming experience and just need to learn syntax.

# Files
## Hidden Files
Any file/directory starting with a period, such as `.vscode` are hidden files/directories which we will very rarely edit. these are used to store metadata about or repository which our tools such as vscode or git use. in Windows sometimes these are not hidden, but in MacOS/Linux it is convention that these aren't shown by default.

## Gradle
Gradle is the build system used by WPILib Java based robots. Whenever you build the robot code, or deploy it, behind the scenes vscode is running the shell script ([./gradlew](./gradlew) on MacOS/Linux or [./gradlew.bat](./gradlew.bat) on Windows) which uses the [./build.gradle](./build.gradle) and [./settings.gradle](./settings.gradle) files which are preconfigured to correctly build (and deploy) the robot code. these files are written in gradle's configuration language. you will most likely never need to touch any of these.

## vendordeps
the [./vendordeps](./vendordeps) directory stores files which are used by gradle to download our external libraries, usually these are installed interactively by vscode, but manually you would copy the .json manifest from the libraries website into [./vendordeps](./vendordeps)

## .md Files
.md files store the text you are currently reading, they are used for documentation purposes outside of the code.

## WPILib-License.md
this is the Legal License which WPILib distributes their code under. The BSD License is a non-restrictive open-source license. It is present in our code as our repo inherits the WPILib Command Based Robot template which is distributed under the BSD License. it is illegal to remove this file or replace it with a more restrictive license.

## src/main/deploy
The RoboRIO runs an embedded linux distributed by National Instruments, the [./src/main/deploy](./src/main/deploy) directory's contents are copied into `~/deploy` directory of the file system of the RoboRIO. these files are often configuration files loaded at runtime by external libraries such as pathplanner.

## src/main/java/frc/robot
this is the root folder of where all our java code lives. 
go to its [documentation](./src/main/java/frc/robot/robot.md) for more information
