package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralGrabberConfig {
  public static final IdleMode IDLE_MODE = IdleMode.kBrake;
  public static final int CURRENT_LIMIT = 20; // amps

  public static final double RELEASE_SPEED = 1.0;
  public static final double RELEASE_TIME = 0.5; // seconds
  public static final double GRAB_SPEED = 1.0;

  public static final double FILTER_TIME_CONSTANT = 0.3;
  public static final double TRIGGER_CURRENT = 12; // amps
}
