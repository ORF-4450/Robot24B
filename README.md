### Robot23
----------------------------------------------------------------------------
FRC Team 4450 2023 Robot Control program.

This is the 2023 competition robot control program reference implementation created by the Olympia Robotics Federation (FRC Team 4450), updated to 2024 beta.

Operates the robot "Prometheus" for FRC game "Charged Up".

----------------------------------------------------------------------------
## Instructions to setup development environment for VS Code
1) Follow the instructions [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html) to setup the JDK, Visual Studio Code, the FRC plugins and tools. Do not install the C++ portion. You do not need the FRC Update Suite to compile code.
2) Clone this repository to local folder.
3) Open that folder in Visual Studio Code.
4) Build the project using the build project command from the WPILib commands list.

### If RobotLib gets an update:
1) download the RobotLib.json file from the RobotLib Github repo and drop it into the vendordeps folder inside the project folder. Build the project.
****************************************************************************************************************
Version 24.MS2

*   Merge MaxSwerve project into Robot24B (MaxSwerve2 branch).

R. Corn, January 25 2024

Version 24.MS

*   Update to 2024 WPILib 24.1.1 release.
*   Update all vendordeps to 2024 release except PathPlanner.
*   Requires RobotLib 4.8.0 or later.

R. Corn, January 11 2024

Version 24.B4-MS

*   Update 2023 project to 2024 Beta 4.
*   Cut down project to base code level for 2024.
*   Add MaxSwerve branch to modify code to use MaxSwerve support in RobotLib 4.8.0.

R. Corn, December 2023

Version 24.B43

*   Update 2023 project to 2024 Beta 3.

R. Corn, November 2023

Version 23.2

*   Replace Claw with Intake, continue post season development.

R. Corn, May 29 2023

Version 23.1

*   Post season updates. Requires RobotLib v4.5.0 or later.

R. Corn, April 2023

Version 23.0

*   Base code for 2023 robot. Requires RobotLib v4.3.1 or later.

R. Corn, February 2023
