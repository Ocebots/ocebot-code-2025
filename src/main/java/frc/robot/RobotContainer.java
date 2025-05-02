// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private AutoFactory autos =
      new AutoFactory(
          drivetrain::getPose,
          (pose) -> drivetrain.poseEstimator.resetPose(pose),
          drivetrain::followTrajectory,
          true,
          drivetrain);
  private int lastButtonPressed;
  // slows down robot durring pickup
  private Command pickup =
      coral
          .pickUpCoral()
          .andThen(
              Commands.runOnce(
                  () -> {
                    speedMultiplier = 1.0;
                    fieldRelative = true;
                  }));
  // changes speed when picking up from source
  private Command pickupSource =
      coral.pickUpCoralSource().andThen(Commands.runOnce(() -> speedMultiplier = 1.0));
  private Command safeState = coral.safeState();
  private double speedMultiplier = 1.0;
  private boolean fieldRelative = true;
  private AutoChooser autoChooser = new AutoChooser();
  @Logged public boolean autoDisabled = true;
  private Field2d selectedSpot = new Field2d();

  public RobotContainer() {
    // how we selected where we were going to score
    DataLogManager.start();
    configureBindings();
    SmartDashboard.putData("scheduler", CommandScheduler.getInstance());
    autoChooser.addCmd(
        "Left",
        () ->
            Commands.sequence(
                Commands.waitSeconds(2), coral.goToReef(drivetrain, () -> 11, () -> 1)));
    autoChooser.addCmd(
        "Right",
        () ->
            Commands.sequence(
                Commands.waitSeconds(2), coral.goToReef(drivetrain, () -> 3, () -> 1)));
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Selected Score Locationn", selectedSpot);
  }

  // all controller bindings
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
                    })
                .ignoringDisable(true));
    // coral pickup
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pickup.isScheduled()) {
                    pickupSource.cancel();
                    pickup.cancel();
                    speedMultiplier = 1.0;
                    fieldRelative = true;
                  } else {
                    pickup.schedule();
                    speedMultiplier = 0.3;
                    fieldRelative = false;
                  }
                }));
    // cancel coral pickup
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pickupSource.isScheduled()) {
                    pickupSource.cancel();
                    pickup.cancel();
                    speedMultiplier = 1.0;
                    fieldRelative = true;
                  } else {
                    pickupSource.schedule();
                    speedMultiplier = 0.4;
                  }
                }));
    // enabling the safe state
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (safeState.isScheduled()) {
                    safeState.cancel();
                  } else {
                    safeState.schedule();
                    speedMultiplier = 1.0;
                  }
                }));

    // picking up algae
    controller
        .rightTrigger()
        .onTrue(
            algae
                .pickUpAlgae()
                .until(() -> !controller.rightTrigger().getAsBoolean())
                .andThen(algae.storeAlgae()));

    // releasing the algae
    controller.y().onTrue(algae.releaseAlgae().andThen(algae.returnToUp()));

    // scoring on all levels
    totalController
        .button(15)
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l4Score(controller.leftTrigger(), false));
                  } else {
                    return coral.goToReef(drivetrain, () -> lastButtonPressed, () -> 3);
                  }
                }));
    totalController
        .button(16)
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l3Score(controller.leftTrigger()));
                  } else {
                    return coral.goToReef(drivetrain, () -> lastButtonPressed, () -> 2);
                  }
                }));
    totalController
        .button(17)
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l2Score(controller.leftTrigger()));
                  } else {
                    return coral.goToReef(drivetrain, () -> lastButtonPressed, () -> 1);
                  }
                }));
    totalController
        .button(18)
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l1Score(controller.leftTrigger()));
                  } else {
                    return coral.goToReef(drivetrain, () -> lastButtonPressed, () -> 0);
                  }
                }));

    // clearing algae off reef
    totalController.button(14).onTrue(coral.l1ReefClear(controller.leftTrigger()));
    totalController.button(13).onTrue(coral.l2ReefClear(controller.leftTrigger()));

    // driving the robot
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

    // deploy climber, releases algae too
    controller.povUp().whileTrue(climb.pivotClimb());
    controller
        .povDown()
        .whileTrue(
            climb
                .pivotRelease()
                .alongWith(algae.deployForClimb().unless(() -> DriverStation.getMatchTime() > 25)));
    controller.povLeft().onTrue(Commands.runOnce(() -> autoDisabled = true));
    controller.povRight().onTrue(Commands.runOnce(() -> autoDisabled = true));
  }

  // deadbands for driving
  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  // sets the pose for auto-align
  public void periodic() {
    selectedSpot.setRobotPose(Positions.getIndividualReef(lastButtonPressed));
  }

  // autonomous command
  public Command getAutonomousCommand() {
    return Commands.sequence(
        Commands.waitSeconds(2), coral.goToReef(drivetrain, () -> lastButtonPressed, () -> 3));
  }
}
