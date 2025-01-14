package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeConfig {
  public static final int WHEELS_CURRENT_LIMIT = 35;
  public static final int ARM_CURRENT_LIMIT = 40;
  public static final SparkBaseConfig.IdleMode WHEELS_IDLEMODE = SparkBaseConfig.IdleMode.kBrake;
  public static final SparkBaseConfig.IdleMode ARM_IDLEMODE = SparkBaseConfig.IdleMode.kBrake;
  public static final double ENCODER_POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
  public static final double ENCODER_VELOCITY_CONVERSION_FACTOR =
      ENCODER_POSITION_CONVERSION_FACTOR / 60;
  public static final double ARM_P = 0.0;
  public static final double ARM_I = 0.0;
  public static final double ARM_D = 0.0;
  public static final double ARM_G = 0.0;
  public static final double ARM_S = 0.0;
  public static final double ARM_V = 0.0;
  public static final double ARM_A = 0.0;
  public static final double POSITION_TOLERANCE = Rotation2d.fromDegrees(5).getRadians();
  public static final double VELOCITY_TOLERANCE = Rotation2d.fromDegrees(5).getRadians();
  public static final Rotation2d PICKUP_ANGLE = Rotation2d.fromDegrees(20);
  public static final Rotation2d STORE_ANGLE = Rotation2d.fromDegrees(60);
  public static final double INTAKE_SPEED = 1;
}
