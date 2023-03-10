package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

    NetworkTable table;
    double tx;
    double ty;
    double ta;

    double[] targetPose_CameraSpace;
    double ry;
    double distance;

    // TODO: LED | public LimelightSubsystem(LedSubsystem s_LEDSubsystem){
    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getHorizontalOffset() {
        return tx;
    }

    public double getHorizontalRotation() {
        return ry;
    }

    public double getDistance() {
        return distance;
    }

    private double distanceCalculation(double yAngle) {
        double AprilTagHeight = Constants.LimeLightConstants.AprilTagHeight;
        double LimelightHeight = Constants.LimeLightConstants.LimelightHeight;
        double LimelightAngle = Constants.LimeLightConstants.LimelightAngle;

        return ((AprilTagHeight - LimelightHeight)
                / Math.tan((LimelightAngle + yAngle) * Math.PI / 180));
    }

    public void setMode(int number) {
        table.getEntry("ledMode").setDouble(number);
    }

    @Override
    public void periodic() {
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");

        targetPose_CameraSpace = LimelightHelpers.getTargetPose_CameraSpace("");
        ry = targetPose_CameraSpace[4];
        distance = distanceCalculation(ty);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);

        for (int i = 0; i < targetPose_CameraSpace.length; i++) {
            SmartDashboard.putNumber("targetpose_cameraspace" + i, targetPose_CameraSpace[i]);
        }
        SmartDashboard.putNumber("Limelight Distance", getDistance());
        SmartDashboard.putNumber("LimeLightRY", ry);
    }
}
