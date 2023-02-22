package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final double kMaxPercentOutput;
    private final double kRamp;

    private final WPI_TalonSRX mLeftElevatorMotor;
    private final WPI_TalonSRX mTopArm;
    private final WPI_TalonSRX[] motors;

    private final WPI_TalonSRX elevatorEncoder;

    public double elevatorTargetHeight = Constants.ControlArmConstants.initialHeight;

    public IntakeSubsystem() {
        kMaxPercentOutput = Constants.ControlArmConstants.kMaxPercentOutput;
        kRamp = Constants.ControlArmConstants.kRamp;

        elevatorEncoder = new WPI_TalonSRX(Constants.ControlArmConstants.kControlArmLowerEncoderDioPort);
        configElevatorEncoder();

        mLeftElevatorMotor = new WPI_TalonFX(Constants.ControlArmConstants.kControArmUpperCanId);
        mTopArm = new WPI_TalonFX(Constants.ControlArmConstants.kControlArmLowerCanId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mTopArm};
        configMotors();
    }

    private void resetToAbsolute() {
        double absolutePosition = (getCanCoder());
        mTopArm.setSelectedSensorPosition(absolutePosition);
    }

    public double getCanCoder() {
        return elevatorEncoder.getSelectedSensorPosition() * 5;
    }

    private void configElevatorEncoder() {
        elevatorEncoder.configFactoryDefault();
        elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        // elevatorEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configMotors() {

    }

    public double getDistance() {
        return convertTalonToMeters(getCanCoder());
    }

    public double convertTalonToMeters(double talon) {
        return talon * 1.1 / 198000;
    }

    public void setRotation(int value) {
        resetToAbsolute();
        elevatorTargetHeight = value;
        motors[1].set(ControlMode.Position, elevatorTargetHeight);
    }

    public void drive(double pct) {
        resetToAbsolute();
        elevatorTargetHeight += pct * 1000;
        elevatorTargetHeight =
                MathUtil.clamp(
                        elevatorTargetHeight,
                        Constants.ControlArmConstants.minimumHeight,
                        Constants.ControlArmConstants.maximumHeight);
        motors[1].set(ControlMode.Position, elevatorTargetHeight);
    }

    public void stop() {
        motors[1].set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Absolute Angle", elevatorEncoder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position1 ", motors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position2 ", motors[1].getSelectedSensorPosition());

        if (elevatorTargetHeight - motors[1].getSelectedSensorPosition() < 1000
                && motors[1].getSelectedSensorPosition() < 6000)
            motors[1].set(ControlMode.PercentOutput, 0);

        // for (WPI_TalonFX motor : motors){
        mTopArm.config_kP(0, Constants.ControlArmConstants.kP);
        mTopArm.config_kI(0, Constants.ControlArmConstants.kI);
        mTopArm.config_kD(0, Constants.ControlArmConstants.kD);
        mTopArm.config_kF(0, Constants.ControlArmConstants.kF);
        // }

    }
}
