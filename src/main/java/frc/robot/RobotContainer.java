// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ControllerConfig;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private ClimbPivot climb = new ClimbPivot();
  private Coral coral = new Coral();
  private Algae algae = new Algae();
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID totalController = new CommandGenericHID(1);
  private Drivetrain drivetrain = new Drivetrain();
  private Command pickup =
      coral.pickUpCoral().andThen(Commands.runOnce(() -> speedMultiplier = 0.5));
  private Command pickupSource =
      coral.pickUpCoralSource().andThen(Commands.runOnce(() -> speedMultiplier = 0.5));
  private double speedMultiplier = 0.5;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pickup.isScheduled()) {
                    pickup.cancel();
                    speedMultiplier = 0.5;
                  } else {
                    pickup.schedule();
                    speedMultiplier = 0.2;
                  }
                }));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pickupSource.isScheduled()) {
                    pickupSource.cancel();
                    speedMultiplier = 0.5;
                  } else {
                    pickupSource.schedule();
                    speedMultiplier = 0.2;
                  }
                }));

    controller
        .rightTrigger()
        .onTrue(
            algae
                .pickUpAlgae()
                .until(() -> !controller.rightTrigger().getAsBoolean())
                .andThen(algae.storeAlgae()));

    controller.y().onTrue(algae.releaseAlgae().andThen(algae.returnToUp()));

    totalController.button(15).onTrue(coral.l4Score(controller.leftTrigger()));
    totalController.button(16).onTrue(coral.l3Score(controller.leftTrigger()));
    totalController.button(17).onTrue(coral.l2Score(controller.leftTrigger()));
    totalController.button(18).onTrue(coral.l1Score(controller.leftTrigger()));

    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.drive(
                    applyDeadband(-controller.getLeftY() * speedMultiplier),
                    applyDeadband(-controller.getLeftX() * speedMultiplier),
                    applyDeadband(-controller.getRightX() * speedMultiplier),
                    true,
                    true),
            drivetrain));

    controller.povUp().whileTrue(climb.pivotClimb());
    controller.povDown().whileTrue(climb.pivotRelease());
  }

  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
