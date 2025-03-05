package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private PIDController movementController =
      new PIDController(CoralConfig.MOVEMENT_P, CoralConfig.MOVEMENT_I, CoralConfig.MOVEMENT_D);
  private CoralPivot coralPivot = new CoralPivot();
  private double[] elevatorHeights = {0.4, 0.5, 0.6, 0.7};
  private Rotation2d[] coralRotations = {
    Rotation2d.fromDegrees(-30.0),
    Rotation2d.fromDegrees(0.0),
    Rotation2d.fromDegrees(30.0),
    Rotation2d.fromDegrees(60.0)
  };

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
    return coralPivot
        .setPivotAngle(() -> coralRotations[idx])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[idx]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), Commands.waitSeconds(0.4))
                .withDeadline(Commands.waitSeconds(1.4))
                .andThen(Commands.waitUntil(completeScore))
                .deadlineFor(grabber.grabCoralRaw())
                .andThen(grabber.releaseCoral()));
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

  public Command l4Score(BooleanSupplier completeScore) {
    return score(3, completeScore);
  }

  public Command goToReef(Drivetrain drivetrain, IntSupplier idx) {
    return drivetrain
        .orbit(
            Positions::getReef,
            () ->
                movementController.calculate(
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
                        .getRadians()),
            () -> CoralConfig.MOVEMENT_DISTANCE)
        .until(movementController::atSetpoint)
        .andThen(
            Commands.runEnd(
                    () -> {
                      double distance =
                          drivetrain
                              .getPose()
                              .getTranslation()
                              .minus(Positions.getIndividualReef(idx.getAsInt()).getTranslation())
                              .getNorm();
                      Rotation2d targetRotation =
                          Rotation2d.fromDegrees(
                              60.0 * (double) (idx.getAsInt() / 2)
                                  + (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                          == DriverStation.Alliance.Blue
                                      ? 180.0
                                      : 0.0));
                      Translation2d direction =
                          Positions.getReef()
                              .getTranslation()
                              .minus(drivetrain.getPose().getTranslation());
                      direction =
                          direction
                              .div(direction.getNorm())
                              .times(drivetrain.orbitDistanceController.calculate(distance, 0));
                      drivetrain.drive(
                          direction.getX(),
                          direction.getY(),
                          drivetrain.orbitRotationController.calculate(
                              drivetrain.getHeading().getRadians(), targetRotation.getRadians()),
                          true,
                          true);
                    },
                    () -> drivetrain.drive(0, 0, 0, false, false))
                .until(
                    () ->
                        drivetrain.orbitRotationController.atSetpoint()
                            && drivetrain.orbitDistanceController.atSetpoint()));
  }

  public Command pickUpCoral() {
    return elevator
        .setElevatorHeight(() -> CoralConfig.INTAKE_HEIGHT)
        .alongWith(coralPivot.setPivotAngle(() -> CoralConfig.INTAKE_ANGLE))
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
