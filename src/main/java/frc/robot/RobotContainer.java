// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ControllerConfig;
import frc.robot.config.Positions;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private ClimbPivot climb = new ClimbPivot();
  private Coral coral = new Coral();
  private Algae algae = new Algae();
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID totalController = new CommandGenericHID(1);
  private Drivetrain drivetrain = new Drivetrain();
  private int lastButtonPressed;
  private Command pickup =
          coral
                  .pickUpCoral()
                  .andThen(
                          Commands.runOnce(
                                  () -> {
                                    speedMultiplier = 0.5;
                                    fieldRelative = true;
                                  }));
  private Command pickupSource =
          coral.pickUpCoralSource().andThen(Commands.runOnce(() -> speedMultiplier = 0.5));
  private double speedMultiplier = 0.5;
  private boolean fieldRelative = true;

  public RobotContainer() {
    DataLogManager.start();
    configureBindings();
  }

  private void configureBindings() {
    CommandScheduler.getInstance()
            .schedule(
                    Commands.run(
                            () -> {
                              for (int i = 0; i < 12; i++) {
                                if (totalController.getHID().getRawButtonPressed(i + 1)) {
                                  lastButtonPressed = i;
                                }
                              }
                            }).ignoringDisable(true));
    controller
            .a()
            .onTrue(
                    Commands.runOnce(
                            () -> {
                              if (pickup.isScheduled()) {
                                pickupSource.cancel();
                                pickup.cancel();
                                speedMultiplier = 0.5;
                                fieldRelative = true;
                              } else {
                                pickup.schedule();
                                speedMultiplier = 0.2;
                                fieldRelative = false;
                              }
                            }));
    controller
            .b()
            .onTrue(
                    Commands.runOnce(
                            () -> {
                              if (pickupSource.isScheduled()) {
                                pickupSource.cancel();
                                pickup.cancel();
                                speedMultiplier = 0.5;
                                fieldRelative = true;
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
    controller.x().whileTrue(coral.goToReef(drivetrain, () -> lastButtonPressed));

    totalController.button(15).onTrue(coral.l4Score(controller.leftTrigger()));
    totalController.button(16).onTrue(coral.l3Score(controller.leftTrigger()));
    totalController.button(17).onTrue(coral.l2Score(controller.leftTrigger()));
    totalController.button(18).onTrue(coral.l1Score(controller.leftTrigger()));

    totalController.button(14).onTrue(coral.l1ReefClear(controller.leftTrigger()));
    totalController.button(13).onTrue(coral.l2ReefClear(controller.leftTrigger()));

    drivetrain.setDefaultCommand(
            Commands.run(
                    () ->
                            drivetrain.drive(
                                    applyDeadband(-controller.getLeftY() * speedMultiplier),
                                    applyDeadband(-controller.getLeftX() * speedMultiplier),
                                    applyDeadband(-controller.getRightX() * speedMultiplier),
                                    fieldRelative,
                                    true),
                    drivetrain));

//    drivetrain.setDefaultCommand(drivetrain.orbit(Positions::getReef, () -> applyDeadband(controller.getLeftY()), () -> 1.25));

    controller.povUp().whileTrue(climb.pivotClimb());
    controller.povDown().whileTrue(climb.pivotRelease());
  }

  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Pose2d reef() {
    return Positions.getIndividualReef(lastButtonPressed);
  }
}
