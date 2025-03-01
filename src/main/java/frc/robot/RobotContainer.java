// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID totalController = new CommandGenericHID(1);
  private Drivetrain drivetrain = new Drivetrain();
  private Command pickup = coral.pickUpCoral();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.orbit(
            () -> new Pose2d(new Translation2d(0.5, 0), new Rotation2d()),
            () -> applyDeadband(controller.getLeftX()),
            () -> 0.5));
  }

  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
