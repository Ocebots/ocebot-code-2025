package frc.robot.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Positions {
  public static Pose2d getReef() {
    return new Pose2d(
        Units.inchesToMeters(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 176.75 : 514.13),
        Units.inchesToMeters(158.50),
        new Rotation2d(0));
  }

  public static Pose2d getIndividualReef(int idx) {
    idx = 13 - idx;
    idx %= 12;

    return getReef()
        .transformBy(
            new Transform2d(
                new Translation2d(
                        0.801751 + Units.inchesToMeters(25),
                        Units.inchesToMeters(12.94) / 2.0 * (idx % 2 * 2 - 1))
                    .rotateBy(
                        Rotation2d.fromDegrees(
                            60.0 * (double) (idx / 2)
                                + (DriverStation.getAlliance().orElse(Alliance.Blue)
                                        == Alliance.Blue
                                    ? 0.0
                                    : 180.0))),
                Rotation2d.fromDegrees(
                        60.0 * (double) (idx / 2)
                            + (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? 0.0
                                : 180.0))
                    .plus(Rotation2d.k180deg)));
  }
}
