// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ControllerConfig;
import frc.robot.config.OrbitConfig;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private Drivetrain drivetrain = new Drivetrain();
  private Algae algae = new Algae();
  private Elevator elevator = new Elevator();
  private CommandXboxController controller = new CommandXboxController(0);
  private double[] elevatorHeights = {0.0, 0.46, 0.81, 1.21};
  private int elevatorLevel = 0;
  private PIDController OrbitXPIDController =
      new PIDController(OrbitConfig.ORBIT_X_P, OrbitConfig.ORBIT_X_I, OrbitConfig.ORBIT_X_D);
  private PIDController OrbitRotPIDController =
      new PIDController(OrbitConfig.ORBIT_ROT_P, OrbitConfig.ORBIT_ROT_I, OrbitConfig.ORBIT_ROT_D);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.b().whileTrue(algae.pickUpAlgae());
    controller.b().onFalse(algae.storeAlgae());
    controller.y().onTrue(algae.releaseAlgae());
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> elevatorLevel = Math.min(elevatorLevel + 1, elevatorHeights.length - 1)));
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> elevatorLevel = Math.max(elevatorLevel - 1, 0)));
    elevator.setDefaultCommand(elevator.setElevatorHeight(() -> elevatorHeights[elevatorLevel]));

    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.drive(
                    applyDeadband(-controller.getLeftX()),
                    applyDeadband(-controller.getLeftY()),
                    applyDeadband(-controller.getRightX()),
                    true,
                    true),
            drivetrain));

    controller
        .x()
        .whileTrue(
            Commands.run(
                () ->
                    drivetrain.drive(
                        OrbitXPIDController.getSetpoint(),
                        applyDeadband(-controller.getRightX()),
                        OrbitRotPIDController.getSetpoint(),
                        true,
                        true),
                drivetrain));
  }

  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
