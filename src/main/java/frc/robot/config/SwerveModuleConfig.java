package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModuleConfig {
  public static final double FREE_SPEED_RPM = 5676;

  /**
   * The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T. This
   * changes the drive speed of the module (a pinion gear with more teeth will result in a robot
   * that drives faster).*
   */
  public static final int DRIVING_MOTOR_PINION_TEETH = 14;

  /**
   * Invert the turning encoder, since the output shaft rotates in the opposite direction of the
   * steering motor in the MAXSwerve Module.*
   */
  public static final boolean IS_TURNING_ENCODER_INVERTED = true;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
  public static final double WHEEL_DIAMETER_METERS = 0.0762;
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
  // bevel pinion
  public static final double DRIVING_MOTOR_REDUCTION =
      (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
      (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

  public static final double DRIVING_ENCODER_POSITION_FACTOR =
      (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters
  public static final double DRIVING_ENCODER_VELOCITY_FACTOR =
      ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  public static final double TURNING_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0; // radians per second

  public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
  public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

  public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
  public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
  //
  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT =
      TURNING_ENCODER_POSITION_FACTOR; // radians
  public static final double DRIVING_P = 0.04;
  public static final double DRIVING_I = 0;
  public static final double DRIVING_D = 0;
  public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
  public static final double DRIVING_MIN_OUTPUT = -1;
  public static final double DRIVING_MAX_OUTPUT = 1;
  public static final double TURNING_P = 1;
  public static final double TURNING_I = 0;
  public static final double TURNING_D = 0;
  public static final double TURNING_FF = 0;
  public static final double TURNING_MIN_OUTPUT = -1;
  public static final double TURNING_MAX_OUTPUT = 1;
}
