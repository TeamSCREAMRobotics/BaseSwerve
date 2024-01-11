# BaseSwerve </br>

<br>This template includes everything we require for a basic Swerve Drive robot.
<br>
<br><b>Includes:</b>
   * Phoenix Pro Implementation
   * Basic Swerve Code
   * PathPlanner functionality
   * Basic examples for autonomous routines
   * Shuffleboard functionality
   * Various Utilty classes

This is based on Team 364's (dirtbikerxz) [BaseTalonFXSwerve](https://github.com/dirtbikerxz/BaseTalonFXSwerve), though it is heavily modified.

<br><br>**CHANGE TEAM NUMBER**
----
Open the Command Palette (Ctrl+Shift+P) then type ```Set Team Number```.


<br><br>**Setting Constants**
----
The following things must be adjusted to your robot and module's specific constants in the [```Constants.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) file (all distance units must be in meters, and rotation units in radians)</br>
1. Gyro Settings: [```PIGEON_ID```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) and [```GYRO_INVERT```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. [```MODULE_TYPE```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): 
<br>Set the module and drive ratio you are using here.
<br>For our uses, it will typically be the SDS MK4/MK4i Module. Make sure to select the correct gear ratio - for us it will most likely be Level 3.
<br>This will automatically set these constants required for the module to function properly:
    * Wheel Circumference
    * Steer Motor Invert
    * Drive Motor Invert
    * CANcoder Sensor Invert
    * Steer Motor Gear Ratio
    * Drive Motor Gear Ratio
    * Steer Falcon Motor PID Values
    
4. [```TRACK_WIDTH```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Center to Center distance of left and right modules in meters.
5. [```WHEEL_BASE```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Center to Center distance of front and rear module wheels in meters.
6. [```GEAR_RATIO```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Total gear ratio for the drive motor. <br><b>This value will be automatically set by the selected module.</b>
7. [```GEAR_RATIO```(Steer)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Total gear ratio for the steer motor. <br><b>This value will be automatically set by the selected module.</b>
8. [```MOTOR_INVERT```(Steer)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): Must be set such that it is CCW+. <br><b>This value will be automatically set by the selected module, but checking is recommended.</b>
9. [```MOTOR_INVERT```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): This can always remain false, since you set your offsets in step #11 such that a positive input to the drive motor will cause the robot to drive forwards. <br><b>This value will be automatically set by the selected module.</b>

10. [```ModuleConstants```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): set the Can Id's of the motors and CANcoders for the respective modules, see the next step for setting offsets.
11. Setting Offsets
    * Open Phoenix Tuner X
    * Select the CANCoder you want to offset
    * Click the "Zero CANcoder" button
    * Go to the config tab and copy the value in the "Magnet Offset" config. (If it shows 0, refresh using "Refresh/Reload Configs")
    * Paste the value in the ```angleOffset``` parameter of the corresponding module constants.
    <br> <b> Note: The offset value from Phoenix Tuner X is in rotations. You must use ```Rotation2d.fromRotations(value_here)```.

12. Angle Motor PID Values: <br><b>This value will be automatically set through the selected module. If you prefer it to be more or less aggressive, see instructions below</b> 
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
    * If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

14. [```MAX_SPEED```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): In Meters Per Second. [```MAX_ANGULAR_VELOCITY```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): In Radians Per Second. For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.

15. [```KS```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), [```KV```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), and [```KA```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java)
<br>Leave these as the default values. If for some reason they require a change, you can use the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock the modules straight forward, and complete the characterization as if it was a standard tank drive.
17. [```KP```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java): 
<br>After inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot or oscillate around a target velocity.
<br>Leave [```KI```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), [```KD```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java), and [```KF```(Drive)](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/Constants.java) at 0.0.


<br><br>**Controller Mappings**
----
The code is natively setup to use a Xbox controller, though other controllers will work. </br>
<br><b>Note: To add additional button bindings, create methods in [```Controlboard.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/controlboard/Controlboard.java) and reference them in [```RobotContainer.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java).
See [```configButtonBindings```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/RobotContainer.java).</b>
* Left Stick: Translation Control (forwards and sideways movement)
* Right Stick: Rotation Control </br>
* Back Button: Zero Gyro (useful if the gyro drifts mid match, just rotate the robot forwards, and press Back to rezero)
* Start Button: Toggles field-centric mode

<br><br>**Shuffleboard Configuration**
----
<br>The following relates to using tabs and entries with Shuffleboard.
<br>Refer to the [WPILib Wiki](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html) for additional help.

<br><b>Creating tabs and entries</b>
* Create a class extending ShuffleboardTabBase and structure it as follows:
   * GenericEntry objects for each of the values you want to display/get with Shuffleboard.
   * Initialize the tab and use create methods from [```ShuffleboardTabBase.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabBase.java) to create entries with the respective data types in [```createEntries()```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabBase.java)
   * Either set or get the entry's value in [```periodic()```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java)
   * See [```SwerveTab.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/tabs/SwerveTab.java) for an example of this structure.
 
<br><b>Putting tabs on Shuffleboard</b>
* Add tabs to the list in [```ShuffleboardTabManager.java```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java), either as a debug tab or regular tab.
   * Debug tabs will only be shown if [```includeDebug```](https://github.com/TeamSCREAM4522/BaseSwerve/blob/main/src/main/java/frc/robot/shuffleboard/ShuffleboardTabManager.java) is true
      * Displaying to Shuffleboard is resource-intensive, so make sure it is true only when you are debugging/developing code.

<br><b>Using the Shuffleboard application</b>
* Shuffleboard should be automatically installed, but you may have to select it in DriverStation.
* All tabs and corresponding entries should appear with the values/names they are set with, assuming you have deployed the code.
   * If they do not appear, simply restart the robot code and re-open Shuffleboard.
