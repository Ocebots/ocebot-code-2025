package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.config.CoralConfig;
import frc.robot.config.Positions;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

@Logged
public class Coral extends SubsystemBase {
  private CoralGrabber grabber = new CoralGrabber();
  private Elevator elevator = new Elevator();
  private PIDController movementController =
      new PIDController(CoralConfig.MOVEMENT_P, CoralConfig.MOVEMENT_I, CoralConfig.MOVEMENT_D);
  private CoralPivot coralPivot = new CoralPivot();

  // heights for levels
  private double[] elevatorScoringHeights = {0.0, 0.910, 1.31, 1.360};

  // rotations for levels
  private Rotation2d[] coralScoringRotations = {
    Rotation2d.fromRadians(1),
    Rotation2d.fromRadians(1.7),
    Rotation2d.fromRadians(1.7),
    Rotation2d.fromRadians(0.3)
  };
  private Rotation2d[] reefClearRotationsPrimary = {
    Rotation2d.fromRadians(1.9), Rotation2d.fromRadians(1.9)
  };
  private Rotation2d[] reefClearRotationsSecondary = {
    Rotation2d.fromRadians(1.5), Rotation2d.fromRadians(1.5)
  };
  private double[] reefClearHeights = {0.65, 1.0564};

  public Coral() {
    movementController.setTolerance(CoralConfig.POSITION_TOLERANCE, CoralConfig.VELOCITY_TOLERANCE);
    movementController.enableContinuousInput(-Math.PI, Math.PI);
    elevator.setDefaultCommand(elevator.setElevatorHeight(() -> 0.0));
    coralPivot.setDefaultCommand(coralPivot.setPivotAngle(() -> Rotation2d.fromRadians(0.35)));
    grabber.setDefaultCommand(
        Commands.run(
            () -> {
              if (!elevator.isAtPosition()
                  || !coralPivot.isPivotReady()
                  || !RobotContainer.stopGrab) {
                grabber.run();
              } else {
                grabber.stop();
              }
            },
            grabber));
  }

  // scores the current coral on the selected level
  private Command score(int idx, BooleanSupplier completeScore) {
    return Commands.waitSeconds(0.3)
        .andThen(coralPivot.setPivotAngle(() -> coralScoringRotations[idx]))
        .alongWith(elevator.setElevatorHeight(() -> elevatorScoringHeights[idx]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(0.8))
                .withDeadline(Commands.waitSeconds(1.8))
                .andThen(Commands.waitUntil(completeScore))
                .andThen(grabber.releaseCoral()));
  }

  private boolean shouldComplete = false;

  // clears the reef of algae
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

  // scores on the l4 level (button pressed the most often)
  public Command l4Score(BooleanSupplier completeScore, boolean shouldHaveAutoTimeout) {
    return Commands.runOnce(() -> shouldComplete = false)
        .andThen(
            coralPivot
                .setPivotAngle(
                    () -> {
                      if (elevator.getPosition() > 1.0 || shouldComplete) {
                        return coralScoringRotations[3];
                      } else {
                        return coralScoringRotations[3].plus(Rotation2d.fromDegrees(15));
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
                            Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(1.0))
                        .withDeadline(Commands.waitSeconds(2.0))
                        .andThen(
                            Commands.waitUntil(completeScore)
                                .raceWith(
                                    Commands.defer(
                                        () ->
                                            shouldHaveAutoTimeout
                                                ? Commands.waitSeconds(4)
                                                : Commands.run(() -> {}),
                                        Set.of())))
                        .andThen(
                            Commands.runOnce(() -> shouldComplete = true),
                            grabber.releaseCoralL4())))
        .andThen(
            coralPivot
                .setPivotAngle(() -> coralScoringRotations[3].minus(Rotation2d.fromDegrees(15)))
                .until(coralPivot::isPivotReady))
        .andThen(elevator.setElevatorHeight(() -> 0.0).until(elevator::isAtPosition));
  }

  // moves the elevator to score l1, kinda works, not really used
  public Command l1Score(BooleanSupplier completeScore) {
    return score(0, completeScore);
  }

  // moves the elevator to l2
  public Command l2Score(BooleanSupplier completeScore) {
    return score(1, completeScore);
  }

  // moves the elevator to l3
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

  // goes to a specific position around the reef
  public Command goToReef(Drivetrain drivetrain, IntSupplier idx, IntSupplier height) {
    return drivetrain
        .orbit(
            Positions::getReef,
            () -> {
              positionAngle =
                  Positions.getIndividualReef(
                          idx.getAsInt(), height.getAsInt() == 3 ? CoralConfig.L4_OFFSET : 0)
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
                  Positions.getIndividualReef(
                          idx.getAsInt(), height.getAsInt() == 3 ? CoralConfig.L4_OFFSET : 0)
                      .getTranslation()
                      .minus(Positions.getReef().getTranslation())
                      .getAngle()
                      .getRadians());
            },
            () -> CoralConfig.MOVEMENT_DISTANCE)
        .until(
            () ->
                movementController.atSetpoint()
                    && drivetrain.rotController.atSetpoint()
                    && drivetrain.xController.atSetpoint())
        .andThen(
            Commands.waitSeconds(1.0)
                .andThen(
                    Commands.runEnd(
                        () -> {
                          ChassisSpeeds speeds =
                              ChassisSpeeds.fromFieldRelativeSpeeds(
                                  drivetrain.xController.calculate(
                                      drivetrain.getPose().getX(),
                                      Positions.getIndividualReef(
                                              idx.getAsInt(),
                                              height.getAsInt() == 3 ? CoralConfig.L4_OFFSET : 0)
                                          .getX()),
                                  drivetrain.yController.calculate(
                                      drivetrain.getPose().getY(),
                                      Positions.getIndividualReef(
                                              idx.getAsInt(),
                                              height.getAsInt() == 3 ? CoralConfig.L4_OFFSET : 0)
                                          .getY()),
                                  drivetrain.rotController.calculate(
                                      drivetrain.getPose().getRotation().getRadians(),
                                      Positions.getIndividualReef(
                                              idx.getAsInt(),
                                              height.getAsInt() == 3 ? CoralConfig.L4_OFFSET : 0)
                                          .getRotation()
                                          .getRadians()),
                                  drivetrain.getPose().getRotation());

                          drivetrain.setChassisSpeeds(speeds);
                        },
                        () -> drivetrain.drive(0, 0, 0, false, false)))
                .withDeadline(
                    Commands.defer(
                        () ->
                            height.getAsInt() == 3
                                ? l4Score(
                                    () ->
                                        drivetrain.rotController.atSetpoint()
                                            && drivetrain.xController.atSetpoint(),
                                    true)
                                : score(
                                    height.getAsInt(),
                                    () ->
                                        drivetrain.rotController.atSetpoint()
                                            && drivetrain.xController.atSetpoint()),
                        Set.of(elevator, coralPivot, grabber))));
  }

  // picks up coral from ground
  public Command pickUpCoral() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT)
        .alongWith(
            coralPivot.setPivotAngle(
                () -> {
                  // intake wiggle
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

  // picks up coral from source
  public Command pickUpCoralSource() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT_SOURCE)
        .alongWith(coralPivot.setPivotAngle(() -> CoralConfig.INTAKE_ANGLE_SOURCE))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.grabCoral()));
  }

  // safe state if we get a coral in the bot
  public Command safeState() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT_SOURCE)
        .alongWith(coralPivot.setPivotAngle(() -> Rotation2d.fromDegrees(75)));
  }
}
