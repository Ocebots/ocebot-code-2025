package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  private CoralGrabber grabber = new CoralGrabber();
  private Elevator elevator = new Elevator();
  private CoralPivot coralPivot = new CoralPivot();
  private double[] elevatorHeights = {0.0, 0.46, 0.81, 1.21};

  public Command Level1Score() {
    return coralPivot
        .setPivotAngle(0.0)
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[0]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command Level2Score() {
    return coralPivot
        .setPivotAngle(0.0)
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[1]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command Level3Score() {
    return coralPivot
        .setPivotAngle(0.0)
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[2]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }

  public Command Level4Score() {
    return coralPivot
        .setPivotAngle(0.0)
        .alongWith(elevator.setElevatorHeight(() -> elevatorHeights[3]))
        .alongWith(
            Commands.waitUntil(elevator::isAtPosition)
                .andThen(Commands.waitUntil(coralPivot::isPivotReady), grabber.releaseCoral()));
  }
}
