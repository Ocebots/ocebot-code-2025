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
import java.util.function.IntSupplier;

@Logged
public class Coral extends SubsystemBase {
  private CoralGrabber grabber = new CoralGrabber();
  private Elevator elevator = new Elevator();
  private PIDController movementController =
      new PIDController(CoralConfig.MOVEMENT_P, CoralConfig.MOVEMENT_I, CoralConfig.MOVEMENT_D);
  private CoralPivot coralPivot = new CoralPivot();
  private double[] elevatorHeights = {0.0, 0.46, 0.81, 1.21};
  private Rotation2d[] coralRotations = {
    new Rotation2d(0.0), new Rotation2d(0.0), new Rotation2d(0.0), new Rotation2d(0.0)
  };

  public Coral() {
    movementController.setTolerance(CoralConfig.POSITION_TOLERANCE, CoralConfig.VELOCITY_TOLERANCE);
  }

  public Command l1Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[0])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[0]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l2Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[1])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[1]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l3Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[2])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[2]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l4Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[3])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[3]))
        .withDeadline(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
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
}
