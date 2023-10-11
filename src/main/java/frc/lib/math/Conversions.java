package frc.lib.math;

public class Conversions {

    /**
     * @param positionCounts CANcoder Position Counts
     * @param gearRatio      Gear Ratio between CANcoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANcoder and Mechanism
     * @return CANcoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts    Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
     *                  RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double falconRPM, double circumference, double gearRatio) {
        //double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (falconRPM * circumference);
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
     *                      Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double metersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
    }


    /**
     * Maps a value from one range to another range.
     *
     * @param value The value to be mapped
     * @param oldBottom The bottom value of the old range
     * @param oldTop The top value of the old range
     * @param newBottom The bottom value of the new range
     * @param newTop The top value of the new range
     * @return The mapped value
     */
    public static double mapRange(double value, double oldBottom, double oldTop, double newBottom, double newTop) {
        boolean divByZero = ((oldTop - oldBottom) * (newTop - newBottom)) == 0.0;
        if(divByZero){
            System.out.println("Division by zero... Returning NaN");
            return Double.NaN;
        }
    
        return (value - oldBottom) / ((oldTop - oldBottom) * (newTop - newBottom));
    }
}