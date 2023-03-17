package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class AngleUtils {

  public static CANCoderConfiguration generateCanCoderConfig() {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return sensorConfig;
  }

  public static SwerveModuleState optimizeTurn(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetSpeed = desiredState.speedMetersPerSecond;

    double newAngleRadians = desiredState.angle.getRadians();
    double currentAngleRadians = currentAngle.getRadians();

    newAngleRadians %= (2.0 * Math.PI);
    if (newAngleRadians < 0.0) {
      newAngleRadians += 2.0 * Math.PI;
    }

    double deltaRadians = newAngleRadians - currentAngleRadians;
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (deltaRadians >= Math.PI) {
      newAngleRadians -= 2.0 * Math.PI;
    } else if (deltaRadians < -Math.PI) {
      newAngleRadians += 2.0 * Math.PI;
    }
    deltaRadians = newAngleRadians - currentAngleRadians; // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (deltaRadians > Math.PI / 2.0 || deltaRadians < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      newAngleRadians += Math.PI;
      targetSpeed *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    newAngleRadians %= (2.0 * Math.PI);
    if (newAngleRadians < 0.0) {
      newAngleRadians += 2.0 * Math.PI;
    }

    return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(newAngleRadians));
  }
}
