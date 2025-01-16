// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Drivetrain;

@Logged
public class RobotContainer {
  private Drivetrain drivetrain = new Drivetrain();
  private Algae algae = new Algae();
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.b().whileTrue(algae.pickUpAlgae());
    controller.b().onFalse(algae.storeAlgae());
    controller.y().onTrue(algae.releaseAlgae());

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
  }

  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, 0.05);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
