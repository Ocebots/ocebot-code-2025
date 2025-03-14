package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralConfig {
  public static final double MOVEMENT_P = 0.0;
  public static final double MOVEMENT_I = 0.0;
  public static final double MOVEMENT_D = 0.0;

  public static final double POSITION_TOLERANCE = 0.0;
  public static final double VELOCITY_TOLERANCE = 0.0;

  public static final double INTAKE_HEIGHT = 0.051;
  public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromRadians(6.05);

  public static final double INTAKE_HEIGHT_SOURCE = 0.575;
  public static final Rotation2d INTAKE_ANGLE_SOURCE = Rotation2d.fromRadians(.498);

  public static final double MOVEMENT_DISTANCE = 2;
}
