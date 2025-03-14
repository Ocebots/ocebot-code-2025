package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CoralConfig;
import frc.robot.config.Positions;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

@Logged
public class Coral extends SubsystemBase {
  private CoralGrabber grabber = new CoralGrabber();
  private Elevator elevator = new Elevator();
  private ProfiledPIDController movementController =
      new ProfiledPIDController(
          CoralConfig.MOVEMENT_P,
          CoralConfig.MOVEMENT_I,
          CoralConfig.MOVEMENT_D,
          new TrapezoidProfile.Constraints(1, 10));
  private CoralPivot coralPivot = new CoralPivot();
  private double[] elevatorScoringHeights = {0.0, 0.906, 1.324, 1.310};
  private Rotation2d[] coralScoringRotations = {
    Rotation2d.fromDegrees(60),
    Rotation2d.fromRadians(6.02),
    Rotation2d.fromRadians(6.02),
    Rotation2d.fromDegrees(70.0)
  };
  private Rotation2d[] reefClearRotationsPrimary = {
    Rotation2d.fromDegrees(-35), Rotation2d.fromDegrees(-35)
  };
  private Rotation2d[] reefClearRotationsSecondary = {
    Rotation2d.fromDegrees(30), Rotation2d.fromDegrees(30)
  };
  private double[] reefClearHeights = {0.65, 1.0564};

  public Coral() {
    movementController.setTolerance(CoralConfig.POSITION_TOLERANCE, CoralConfig.VELOCITY_TOLERANCE);
    elevator.setDefaultCommand(elevator.setElevatorHeight(() -> 0.0));
    coralPivot.setDefaultCommand(coralPivot.setPivotAngle(() -> Rotation2d.fromDegrees(75)));
    grabber.setDefaultCommand(
        Commands.run(
            () -> {
              if (!elevator.isAtPosition() || !coralPivot.isPivotReady()) {
                grabber.run();
              } else {
                grabber.stop();
              }
            },
            grabber));
  }

  private Command score(int idx, BooleanSupplier completeScore) {
    return Commands.waitSeconds(0.3)
        .andThen(coralPivot.setPivotAngle(() -> coralScoringRotations[idx]))
        .alongWith(elevator.setElevatorHeight(() -> elevatorScoringHeights[idx]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(0.8))
                .withDeadline(Commands.waitSeconds(1.8))
                .andThen(Commands.waitUntil(completeScore))
                .deadlineFor(grabber.grabCoralRaw())
                .andThen(grabber.releaseCoral()));
  }

  private boolean shouldComplete = false;

  private Command reefClear(int idx, BooleanSupplier completeClear) {
    return Commands.runOnce(() -> shouldComplete = false)
        .andThen(
            coralPivot
                .setPivotAngle(
                    () -> {
                      if (shouldComplete) {
                        return reefClearRotationsSecondary[idx];
                      } else {
                        return reefClearRotationsPrimary[idx];
                      }
                    })
                .alongWith(elevator.setElevatorHeight(() -> reefClearHeights[idx]))
                .withDeadline(
                    Commands.waitUntil(elevator::isAtPosition)
                        .andThen(
                            Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(0.4))
                        .withDeadline(Commands.waitSeconds(1.4))
                        .andThen(
                            Commands.waitUntil(completeClear),
                            Commands.runOnce(() -> shouldComplete = true),
                            grabber
                                .removeAlgae()
                                .withDeadline(
                                    Commands.waitUntil(() -> !completeClear.getAsBoolean())
                                        .andThen(Commands.waitUntil(completeClear))))));
  }

  public Command l4Score(BooleanSupplier completeScore) {
    return Commands.runOnce(() -> shouldComplete = false)
        .andThen(
            coralPivot
                .setPivotAngle(
                    () -> {
                      if (elevator.isAtPosition()) {
                        return coralScoringRotations[3];
                      } else {
                        return coralScoringRotations[3].minus(Rotation2d.fromDegrees(20));
                      }
                    })
                .alongWith(
                    elevator.setElevatorHeight(
                        () -> {
                          if (shouldComplete) {
                            return elevatorScoringHeights[3] - .15;
                          } else {
                            return elevatorScoringHeights[3];
                          }
                        }))
                .withDeadline(
                    Commands.waitUntil(elevator::isAtPosition)
                        .andThen(
                            Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(0.4))
                        .withDeadline(Commands.waitSeconds(1.4))
                        .andThen(Commands.waitUntil(completeScore))
                        .deadlineFor(grabber.grabCoralRaw())
                        .andThen(
                            Commands.runOnce(() -> shouldComplete = true), grabber.releaseCoral())))
        .andThen(
            coralPivot
                .setPivotAngle(() -> coralScoringRotations[3].minus(Rotation2d.fromDegrees(20)))
                .until(coralPivot::isPivotReady))
        .andThen(elevator.setElevatorHeight(() -> 0.0).until(elevator::isAtPosition));
  }

  public Command l1Score(BooleanSupplier completeScore) {
    return score(0, completeScore);
  }

  public Command l2Score(BooleanSupplier completeScore) {
    return score(1, completeScore);
  }

  public Command l3Score(BooleanSupplier completeScore) {
    return score(2, completeScore);
  }

  public Command l1ReefClear(BooleanSupplier completeClear) {
    return reefClear(0, completeClear);
  }

  public Command l2ReefClear(BooleanSupplier completeClear) {
    return reefClear(1, completeClear);
  }

  @Logged public Rotation2d positionAngle;

  public Command goToReef(Drivetrain drivetrain, IntSupplier idx, IntSupplier height) {
    return drivetrain
        .orbit(
            Positions::getReef,
            () -> {
              positionAngle =
                  Positions.getIndividualReef(idx.getAsInt())
                      .getTranslation()
                      .minus(Positions.getReef().getTranslation())
                      .getAngle();
              return -movementController.calculate(
                  drivetrain
                      .getPose()
                      .getTranslation()
                      .minus(Positions.getReef().getTranslation())
                      .getAngle()
                      .getRadians(),
                  Positions.getIndividualReef(idx.getAsInt())
                      .getTranslation()
                      .minus(Positions.getReef().getTranslation())
                      .getAngle()
                      .getRadians());
            },
            () -> CoralConfig.MOVEMENT_DISTANCE)
        .until(
            () ->
                movementController.atGoal()
                    && drivetrain.rotController.atSetpoint()
                    && drivetrain.xController.atSetpoint())
        .andThen(
            Commands.runEnd(
                    () -> {
                      ChassisSpeeds speeds =
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              drivetrain.xController.calculate(
                                  drivetrain.getPose().getX(),
                                  Positions.getIndividualReef(idx.getAsInt()).getX()),
                              drivetrain.yController.calculate(
                                  drivetrain.getPose().getY(),
                                  Positions.getIndividualReef(idx.getAsInt()).getY()),
                              drivetrain.rotController.calculate(
                                  drivetrain.getPose().getRotation().getRadians(),
                                  Positions.getIndividualReef(idx.getAsInt())
                                      .getRotation()
                                      .getRadians()),
                              drivetrain.getPose().getRotation());

                      drivetrain.setChassisSpeeds(speeds);
                    },
                    () -> drivetrain.drive(0, 0, 0, false, false))
                .withDeadline(
                    score(
                        height.getAsInt(),
                        () ->
                            drivetrain.rotController.atSetpoint()
                                && drivetrain.xController.atSetpoint())));
  }

  public Command pickUpCoral() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT)
        .alongWith(
            coralPivot.setPivotAngle(
                () -> {
                  if (Math.round(Timer.getFPGATimestamp() * 2.0) % 2 == 0) {
                    return CoralConfig.INTAKE_ANGLE_LOWER;
                  } else {
                    return CoralConfig.INTAKE_ANGLE_UPPER;
                  }
                }))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.grabCoral()));
  }

  public Command pickUpCoralSource() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT_SOURCE)
        .alongWith(coralPivot.setPivotAngle(() -> CoralConfig.INTAKE_ANGLE_SOURCE))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.grabCoral()));
  }
}
