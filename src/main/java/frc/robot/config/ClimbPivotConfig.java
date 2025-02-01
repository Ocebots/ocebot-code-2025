package frc.robot.config;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

public class ClimbPivotConfig {
    public static final double PIVOT_P = 0.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_S = 0.0;
    public static final double PIVOT_G = 0.0;
    public static final double PIVOT_V = 0.0;
    public static final double PIVOT_A = 0.0;
    public static final int PIVOT_CURRENT_LIMIT = 40;
    public static final SparkBaseConfig.IdleMode PIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final double ENCODER_POSITION_CONVERSION_FACTOR =
            Math.PI * 2.0 / 225.0;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR =
            ENCODER_POSITION_CONVERSION_FACTOR
                    / 60.0;
}