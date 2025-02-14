package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CoralConfig;
import frc.robot.config.Positions;
import java.util.function.IntSupplier;

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

  public Command l1Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[0])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[0]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l2Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[1])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[1]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l3Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[2])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[2]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command l4Score() {
    return coralPivot
        .setPivotAngle(() -> coralRotations[3])
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[3]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command goToReef(Drivetrain drivetrain, IntSupplier idx) {
    return drivetrain.orbit(
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
        () -> CoralConfig.MOVEMENT_DISTANCE);
  }
}
