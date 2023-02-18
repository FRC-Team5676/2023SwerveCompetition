package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControlArmSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;

    private final CANSparkMax upperArmMotor;
    private final DutyCycleEncoder lowerArmEncoder;
    private final WPI_TalonSRX lowerArmMotor;

    public double upperArmTargetPosition = Constants.ControlArmConstants.kUpperArmTargetPosition;
    public double lowerArmTargetPosition = Constants.ControlArmConstants.kLowerArmTargetPosition;

    public ControlArmSubsystem() {
        kMaxPercentOutput = Constants.ControlArmConstants.kMaxPercentOutput;
        kRamp = Constants.ControlArmConstants.kRamp;

        lowerArmEncoder = new DutyCycleEncoder(Constants.ControlArmConstants.kControlArmLowerEncoderDioPort);
        configLowerArmEncoder();

        lowerArmMotor = new WPI_TalonSRX(Constants.ControlArmConstants.kControlArmLowerCanId);
        upperArmMotor = new CANSparkMax(Constants.ControlArmConstants.kControArmUpperCanId, MotorType.kBrushless);
        configMotors();
    }

    private void configLowerArmEncoder() {
        // TODO: Figure out how to config
    }

    private void configMotors() {
        // TODO: Figure out how to config
    }

    public void drive(double pct) {
        upperArmTargetPosition += pct * 1000;
        upperArmTargetPosition =
                MathUtil.clamp(
                        upperArmTargetPosition,
                        Constants.ControlArmConstants.kUpperArmTargetPosition * 0.95,
                        Constants.ControlArmConstants.kUpperArmTargetPosition * 1.05);
                        upperArmMotor.(ControlMode.Position, upperArmTargetPosition);
    }

    public void stop() {
        motors[1].set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Absolute Angle", lowerArmEncoder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position1 ", motors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position2 ", motors[1].getSelectedSensorPosition());

        if (upperArmTargetPosition - motors[1].getSelectedSensorPosition() < 1000
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
