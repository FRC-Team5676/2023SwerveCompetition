// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Controller {
        public static final double kXDeadband = 0.05;
        public static final double kYDeadband = 0.05;
        public static final double kZDeadband = 0.05;
        public static final double kTriggerDeadband = 0.05;
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

    public static final class DriveConstants {
        public static final boolean kFrontLeftTurningMotorReversed = ModuleConstants.kTurnInverted;
        public static final boolean kBackLeftTurningMotorReversed = ModuleConstants.kTurnInverted;
        public static final boolean kFrontRightTurningMotorReversed = ModuleConstants.kTurnInverted;
        public static final boolean kBackRightTurningMotorReversed = ModuleConstants.kTurnInverted;

        public static final boolean kFrontLeftDriveMotorReversed = ModuleConstants.kDriveInverted;
        public static final boolean kBackLeftDriveMotorReversed = ModuleConstants.kDriveInverted;
        public static final boolean kFrontRightDriveMotorReversed = ModuleConstants.kDriveInverted;
        public static final boolean kBackRightDriveMotorReversed = ModuleConstants.kDriveInverted;

        // Distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(23.25);

        // Distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(19.25);

        // These are not the max speeds, rather the allowed max velocity
        public static final double kMaxVelocityMetersPerSec = 4.14528;
        public static final double kMaxAngularVelocityRadiansPerSec = kMaxVelocityMetersPerSec / 
            Math.hypot(kRobotWidth / 2, kRobotLength / 2);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kRobotWidth / 2, kRobotLength / 2),    // Front Left
                new Translation2d(kRobotWidth / 2, -kRobotLength / 2),   // Front Right
                new Translation2d(-kRobotWidth / 2, kRobotLength / 2),   // Back Left
                new Translation2d(-kRobotWidth / 2, -kRobotLength / 2)); // Back Right

        // Constant Angular Wheel Offset in Degrees
        public static final double kFrontLeftAngularOffset = 41.3;
        public static final double kFrontRightAngularOffset = 113.2;
        public static final double kBackLeftAngularOffset = 125.59;
        public static final double kBackRightAngularOffset = 255.14;

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

        public static final boolean kfieldRelative = true;
        public static final boolean kGyroReversed = true;

        public static final double kTranslationSlew = 1.55;
        public static final double kRotationSlew = 3.00;

    }

    public static final class ModuleConstants {
        public static final boolean kTurningEncoderInverted = true;

        // Values from SDS GitHub repo
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.95);        // 0.10033 meters
        public static double kMk4iDriveGearRatio = ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)); // 0.122807 = 1/8.142858
        public static boolean kDriveInverted = true;
        public static double kMk4iTurnGearRatio = ((14.0 / 50.0) * (10.0 / 60.0));                  // 0.046667 = 1/21.428418
        public static boolean kTurnInverted = false;

        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDriveEncoderRot2Meter = kMk4iDriveGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurnEncoderRot2Rad = kMk4iTurnGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRpm2Mps = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRpm2Rps = kTurnEncoderRot2Rad / 60;

        public static final double kTurnP = 1.0; // SDS Config
        public static final double kTurnI = 0.0; // SDS Config
        public static final double kTurnD = 0.1; // SDS Config

        public static final double kVoltCompensation = 12.0; // SDS Config
        public static final int kDriveMotorCurrentLimit = 80; // amps SDS Config
        public static final int kTurnMotorCurrentLimit = 20; // amps SDS Config
    }
}
