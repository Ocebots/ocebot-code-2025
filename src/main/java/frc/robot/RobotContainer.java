// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ControllerConfig;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivetrain;

@Logged
public class RobotContainer {
  private Drivetrain drivetrain = new Drivetrain();
  private Algae algae = new Algae();
  private Coral coral = new Coral();
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID totalController = new CommandGenericHID(0);
  private int lastButtonPressed;
  private int elevatorLevel = 0;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.b().whileTrue(algae.pickUpAlgae());
    controller.b().onFalse(algae.storeAlgae());
    controller.y().onTrue(algae.releaseAlgae());
    controller
        .leftBumper()
        .onTrue(Commands.runOnce(() -> elevatorLevel = Math.max(elevatorLevel - 1, 0)));
    CommandScheduler.getInstance()
        .schedule(
            Commands.run(
                () -> {
                  for (int i = 0; i < 12; i++) {
                    if (totalController.getHID().getRawButtonPressed(i + 1)) {
                      lastButtonPressed = i;
                    }
                  }
                }));

    totalController
        .button(15)
        .onTrue(coral.goToReef(drivetrain, () -> lastButtonPressed).andThen(coral.l4Score()));
    totalController
        .button(16)
        .onTrue(coral.goToReef(drivetrain, () -> lastButtonPressed).andThen(coral.l3Score()));
    totalController
        .button(17)
        .onTrue(coral.goToReef(drivetrain, () -> lastButtonPressed).andThen(coral.l2Score()));
    totalController
        .button(18)
        .onTrue(coral.goToReef(drivetrain, () -> lastButtonPressed).andThen(coral.l1Score()));

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
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
