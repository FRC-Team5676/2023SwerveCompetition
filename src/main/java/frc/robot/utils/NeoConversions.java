package frc.robot.utils;

public class NeoConversions {

    private final static double DegreesPerRevolution = 360.0;
    private final static double CanCoderStepsPerRev = 4096.0;
    private final static double NeoStepsPerRev = 42.0;

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double canCoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (DegreesPerRevolution / (gearRatio * CanCoderStepsPerRev));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCanCoder(double degrees, double gearRatio) {
        return degrees / (DegreesPerRevolution / (gearRatio * CanCoderStepsPerRev));
    }

    /**
     * @param counts Neo Position Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double neoToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (DegreesPerRevolution / (gearRatio * NeoStepsPerRev));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Neo Position Counts
     */
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / (DegreesPerRevolution / (gearRatio * NeoStepsPerRev));
    }

    /**
     * @param velocityCounts Neo Velocity Counts in steps per sec
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double neoToRpm(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (60 / NeoStepsPerRev);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return Neo Velocity Counts in steps per sec
     */
    public static double rpmToNeo(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double velocityCounts = motorRPM * (NeoStepsPerRev / 60);
        return velocityCounts;
    }

    /**
     * @param velocityCounts Neo Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Neo Velocity Counts
     */
    public static double neoToMps(double velocityCounts, double circumference, double gearRatio){
        double wheelRPM = neoToRpm(velocityCounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity in MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Neo Velocity Counts
     */
    public static double mpsToNeo(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = rpmToNeo(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Neo Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Meters
     */
    public static double neoToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * NeoStepsPerRev));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Neo Position Counts
     */
    public static double metersToNeo(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * NeoStepsPerRev));
    }
}
