// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CustomConstants {
        public static final double rampRate = 2;
        public static final double stickDeadband = 0.05;

        public static final double lowSpeedMultiplier = 0.2;

        public static final boolean fieldRelative = true;

        public static final int LEDPort = 9;
    }

    public static final class Balance {
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 0;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    }

    public static final class Position {
        public static final class StowPosition {
            public static final int ControlArmUpperPosition = 0;
            public static final int ControlArmLowerPosition = 0;
            public static final int IntakePosition = 0;
        }

        public static final class Cones {
            public static final class HighNodePosition {
                public static final int ControlArmUpperPosition = 0;
                public static final int ControlArmLowerPosition = 0;
                public static final int IntakePosition = 0;
            };
        }

        public static final class Intake {
            public static final class IntakeFromGroundPosition {
                public static final int ControlArmUpperPosition = 0;
                public static final int ControlArmLowerPosition = 0;
                public static final int IntakePosition = 0;
            }
        }
    }

    public static final class ControlArmConstants {
        public static final int kControArmUpperCanId = 20;
        public static final int kControlArmLowerCanId = 21;
        public static final int kControlArmLowerEncoderDioPort = 22;

        // Motor
        public static final double kMaxPercentOutput = 1.0;
        public static final double kRamp = 0.3;

        // Positions
        public static final double kUpperArmTargetPosition = 400;
        public static final double kLowerArmTargetPosition = 200;

        // PID
        public static final double kP = 0.03;
        public static final double kI = 0.00;
        public static final double kD = 0.015;
        public static final double kF = 0.00;
    }

    public static final class IntakeConstants {
        public static final int kIntakeCanId = 7;

        // PID
        public static final double kP = 0.003;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 0.00;

        public static final double startingAngle = 0;

        public static final double rampRate = 0.6;
    }

    public static final class LimeLightConstants {
        public static final double distanceToTarget = 4;

        public static final double AprilTagHeight = 1;
        public static final double LimelightHeight = 0.46;
        public static final double LimelightAngle = 14;
    }

    public static final class DriveConstants {
        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kBackLeftTurningMotorReversed = true;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final boolean kBackRightTurningMotorReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxRotationRadiansPerSecond = Math.PI;
        // public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per
        // second

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(23.25);

        // Distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(19.25);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kRobotLength / 2, kRobotWidth / 2),
                new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
                new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
                new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));

        public static final double kFrontLeftAngularOffset = 44.39;
        public static final double kFrontRightAngularOffset = 112.76;
        public static final double kBackLeftAngularOffset = 38.49;
        public static final double kBackRightAngularOffset = 257.78;

        // Spark MAX Drive Motor CAN IDs
        public static final int kFrontLeftDriveMotorCanId = 23;
        public static final int kFrontRightDriveMotorCanId = 22;
        public static final int kBackLeftDriveMotorCanId = 26;
        public static final int kBackRightDriveMotorCanId = 27;

        // Spark MAX Steer Motor CAN IDs
        public static final int kFrontLeftTurnMotorCanId = 21;
        public static final int kFrontRightTurnMotorCanId = 20;
        public static final int kBackLeftTurnMotorCanId = 24;
        public static final int kBackRightTurnMotorCanId = 25;

        // Spark MAX Steer Encoder CAN IDs
        public static final int kFrontLeftTurnEncoderCanId = 50;
        public static final int kFrontRightTurnEncoderCanId = 51;
        public static final int kBackLeftTurnEncoderCanId = 53;
        public static final int kBackRightTurnEncoderCanId = 52;

        public static final boolean kGyroReversed = true;

        public static double kTranslationSlew = 1.55;
        public static double kRotationSlew = 3.00;
        public static double kVoltCompensation = 12.6;

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
        public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14 .122807
        public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 25 teeth on the first-stage spur gear, 15
        // teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 25) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kDriveMetersPerEncRev = (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;

        // in 1 minute at 1 rpm encoder drive moves kDriveMetersPerEncRev
        // so in 1 second encoder travels 1/60 revs = kDriveMetersPerEncRev/60
        // so MPS
        public static final double kDriveEncRPMperMPS = 60 / kDriveMetersPerEncRev;
        public static double kFreeMetersPerSecond = 5600 / kDriveEncRPMperMPS;

        public static double kPModuleTurnController;

        public static final double kTurningDegreesPerEncRev = 360 / mk4iL1TurnGearRatio;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 1;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.04;
        public static final double kTurningI = 0;
        public static final double kTurningD = 1;
        public static final double kTurningFF = 1;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

        public static final int kDrivingMotorCurrentLimit = 20; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static double ksVolts = .055;
        public static double kvVoltSecondsPerMeter = .2;
        public static double kaVoltSecondsSquaredPerMeter = .02;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
