package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

public class ElevatorConfig {
  public static final double EL_P = 0.5;
  public static final double EL_I = 0.0;
  public static final double EL_D = 0.0;
  public static final double EL_S = 0.0;
  public static final double EL_G = 0.0;
  public static final double EL_V = 0.0;
  public static final double EL_A = 0.0;
  public static final int ELEVATOR_CURRENT_LIMIT = 40;
  public static final SparkBaseConfig.IdleMode ELEVATOR_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;
  public static final double ENCODER_POSITION_CONVERSION_FACTOR =
      22.0 * Units.inchesToMeters(.25) * 2.0 / 25.0;
  public static final double ENCODER_VELOCITY_CONVERSION_FACTOR =
      ENCODER_POSITION_CONVERSION_FACTOR / 60.0;
  public static final double POSITION_TOLERANCE = 0.05;
  public static final double VELOCITY_TOLERANCE = 0.05;
  public static final double MAX_VELOCITY = 0.1;
  public static final double MAX_ACCELERATION = 1;
}
