package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig;

public class AlgaeConfig {
  public static final int WHEELS_CURRENT_LIMIT = 35;
  public static final int ARM_CURRENT_LIMIT = 40;
  public static final SparkBaseConfig.IdleMode WHEELS_IDLEMODE = SparkBaseConfig.IdleMode.kBrake;
  public static final SparkBaseConfig.IdleMode ARM_IDLEMODE = SparkBaseConfig.IdleMode.kBrake;
}
