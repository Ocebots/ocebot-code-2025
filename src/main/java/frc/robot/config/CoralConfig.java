package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CoralConfig {
  public static final double MOVEMENT_P = 6.0;
  public static final double MOVEMENT_I = 0.0;
  public static final double MOVEMENT_D = 0.0;

  public static final double POSITION_TOLERANCE = 0.5;
  public static final double VELOCITY_TOLERANCE = 1.0;

  public static final double INTAKE_HEIGHT = 0.065;
  public static final Rotation2d INTAKE_ANGLE_UPPER = Rotation2d.fromRadians(5.9);
  public static final Rotation2d INTAKE_ANGLE_LOWER = Rotation2d.fromRadians(5.7);

  // all heights are in meters
  public static final double INTAKE_HEIGHT_SOURCE = 0.490;
  public static final Rotation2d INTAKE_ANGLE_SOURCE = Rotation2d.fromRadians(.497);

  public static final double MOVEMENT_DISTANCE = 1.75;
  public static final double L4_OFFSET = Units.inchesToMeters(-18);
}
