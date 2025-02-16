package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConfig {
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
  public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

  public static final double DISTANCE_POSITION_TOLERANCE = 0.0;
  public static final double DISTANCE_VELOCITY_TOLERANCE = 0.0;
  public static final double ROTATION_POSITION_TOLERANCE = 0.0;
  public static final double ROTATION_VELOCITY_TOLERANCE = 0.0;

  public static final double MAX_ACCELERATION = 10; // meters per second per second
  public static final double MAX_DRIVE_ANGULAR_VELOCITY = 10; // radians per second
  public static final double MAX_ROTATIONAL_ACCELERATION = 2; // percent per second

  // Chassis configuration
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(27.5);
  // Distance between centers of right and left wheels on robot
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(27.5);
  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

  // Angular offsets of the modules relative to the chassis in radians
  public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
  public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
  public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
  public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

  public static final boolean GYRO_IS_REVERSED = true;
}
