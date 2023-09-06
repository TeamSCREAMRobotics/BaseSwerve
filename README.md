# BaseSwerve

Base swerve code for both teams.

1. **Change the Team Number of the Project**
   - Use the Command Palette (Ctrl+Shift+P) and select 'Set Team Number.'
   - Type in the number of the team you are creating the project for.

2. **Assign and/or change IDs of devices + configure CANivore**
   - Swerve Module motors
       - Assign motor IDs in Phoenix Tuner
           - Typically motors are assigned in order by module location: FL, FR, BL, BR
       - Change the IDs for each module in the SwerveConstants class of Constants.java
       - For hotswap modules, simply create additional SwerveModuleConstants for each
   - CANivore
       - Assign the CANivore a name in Phoenix Tuner
       - If the robot is not using a CANivore:
           - Remove it in the initialization of the motors in SwerveModule.java
           - Remove it in the initialization of the gyro in Swerve.java
           - Remove the constant in Ports.java
       - If the robot is using a CANivore:
           - Replace NAME_HERE with the assigned device name in the Ports class of Constants.java
           - Remember to add the constant in the initialization of any additional devices using it

