package frc.robot.utils;

public class NeoConversions {

    public final static double kDegPerEncRev = 360.0;
    public final static double kCanCoderStepsPerRev = 4096.0;
    public final static double kNeoStepsPerRev = 42.0;
    public final static double kNeoMaxRpm = 5676.0;

    private final static double kMaxMpsFactor = 0.9;
    private final static double kMaxRpsFactor = 0.9;

    /**
     * @param positionCounts CANCoder Step Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double canCoderStepToMechDeg(double positionCounts, double gearRatio) {
        return positionCounts * (kDegPerEncRev / (gearRatio * kCanCoderStepsPerRev));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Step Counts
     */
    public static double mechDegToCanCoderStep(double degrees, double gearRatio) {
        return degrees / (kDegPerEncRev / (gearRatio * kCanCoderStepsPerRev));
    }

    /**
     * @param counts Neo Step Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double neoStepToMechDeg(double positionCounts, double gearRatio) {
        return positionCounts * (kDegPerEncRev / (gearRatio * kNeoStepsPerRev));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Neo Step Counts
     */
    public static double mechDegToNeoStep(double degrees, double gearRatio) {
        return degrees / (kDegPerEncRev / (gearRatio * kNeoStepsPerRev));
    }

    /**
     * @param velocityCounts Neo Velocity in steps per sec
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double neoSpsToMechRpm(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (60 / kNeoStepsPerRev);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return Neo Velocity in steps per sec
     */
    public static double mechRpmToNeoSps(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double velocityCounts = motorRPM * (kNeoStepsPerRev / 60);
        return velocityCounts;
    }

    /**
     * @param velocityCounts Neo Velocity in steps per second
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Velocity in MPS
     */
    public static double neoSpsToMechMps(double velocityCounts, double circumference, double gearRatio){
        double wheelRPM = neoSpsToMechRpm(velocityCounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Mechanism Velocity in MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Neo Velocity in steps per second
     */
    public static double mechMpsToNeoSps(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = mechRpmToNeoSps(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Neo Step Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Meters
     */
    public static double neoStepsToMechMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * kNeoStepsPerRev));
    }

    /**
     * @param meters Mechanism Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Neo Step Counts
     */
    public static double mechMetersToNeoSteps(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * kNeoStepsPerRev));
    }

    /**
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Max Mechanism Velocity in MPS (uses Max MPS Factor)
     */
    public static double maxMechMetersPerSec(double circumference, double gearRatio) {
        double stepsPerSec = kNeoMaxRpm * kNeoStepsPerRev / 60;
        return neoSpsToMechMps(stepsPerSec, circumference, gearRatio) * kMaxMpsFactor;
    }

    /**
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Max Mechanism Velocity in Radians Per Second (uses Max RPS Factor)
     */
    public static double maxMechRadsPerSec(double gearRatio) {
        double stepsPerSec = kNeoMaxRpm * kNeoStepsPerRev / 60;
        double rpm = neoSpsToMechRpm(stepsPerSec, gearRatio) * kMaxRpsFactor;
        return 2 * Math.PI * rpm / 60;
    }
}
