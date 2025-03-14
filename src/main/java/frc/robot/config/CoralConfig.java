package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralConfig {
  public static final double MOVEMENT_P = 6.0;
  public static final double MOVEMENT_I = 0.0;
  public static final double MOVEMENT_D = 0.0;

  public static final double POSITION_TOLERANCE = 0.15;
  public static final double VELOCITY_TOLERANCE = 0.15;

  public static final double INTAKE_HEIGHT = 0.07;
  public static final Rotation2d INTAKE_ANGLE_UPPER = Rotation2d.fromRadians(6.0);
  public static final Rotation2d INTAKE_ANGLE_LOWER = Rotation2d.fromRadians(5.8);

  public static final double INTAKE_HEIGHT_SOURCE = 0.545;
  public static final Rotation2d INTAKE_ANGLE_SOURCE = Rotation2d.fromRadians(.502);

  public static final double MOVEMENT_DISTANCE = 1.75;
}
