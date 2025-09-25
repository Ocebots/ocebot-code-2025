// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ControllerConfig;
import frc.robot.config.Positions;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private ClimbPivot climb = new ClimbPivot();
  private Coral coral = new Coral();
  private Algae algae = new Algae();
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain drivetrain = new Drivetrain();
  private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private AutoFactory autos =
      new AutoFactory(
          drivetrain::getPose,
          (pose) -> drivetrain.poseEstimator.resetPose(pose),
          drivetrain::followTrajectory,
          true,
          drivetrain);
  // used to determine which section of the reef to score on, defaults at start to pole to front and
  // left of driver
  private int reefSection = 7;
  public static boolean stopGrab = false;
  // slows down robot during pickup
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
    // if left trigger is pressed, shifts selected reef section to score on left one, works while
    // robot is disabled for autonomous
    controller
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (reefSection == 11) {
                        reefSection = 0;
                      } else {
                        reefSection += 1;
                      }
                    })
                .ignoringDisable(true));
    // if right trigger is pressed, shifts selected reef section to score on right one, works while
    // robot is disabled for autonomous
    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (reefSection == 0) {
                        reefSection = 11;
                      } else {
                        reefSection -= 1;
                      }
                    })
                .ignoringDisable(true));

    // when right toggle goes up, cancels ground coral pickup or initiates ground coral pickup
    new Trigger(() -> controller.getRightY() > 0.9)
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
    // when right toggle goes down, cancels station coral pickup or initiates station coral pickup
    new Trigger(() -> controller.getRightY() < -0.9)
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
    // when upper plus button is pressed, cancels safe state or initiates safe state
    controller
        .povUp()
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
    // when x is pressed and if auto is disabled, raises elevator for l4 scoring, when left stick is
    // pressed, completes scoring motion
    // is auto is enabled, automatically goes to previously selected reef section and l4 scores
    controller
        .x()
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l4Score(controller.leftStick(), false));
                  } else {
                    return coral.goToReef(drivetrain, () -> reefSection, () -> 3);
                  }
                }));
    // when y is pressed and if auto is disabled, raises elevator for l3 scoring, when left stick is
    // pressed, completes scoring motion
    // is auto is enabled, automatically goes to previously selected reef section and l3 scores
    controller
        .y()
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l3Score(controller.leftStick()));
                  } else {
                    return coral.goToReef(drivetrain, () -> reefSection, () -> 2);
                  }
                }));
    // when a is pressed and if auto is disabled, raises elevator for l2 scoring, when left stick is
    // pressed, completes scoring motion
    // is auto is enabled, automatically goes to previously selected reef section and l2 scores
    controller
        .a()
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (autoDisabled) {
                    return Commands.startEnd(
                            () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                        .withDeadline(coral.l2Score(controller.leftStick()));
                  } else {
                    return coral.goToReef(drivetrain, () -> reefSection, () -> 1);
                  }
                }));
    // when b is pressed and if auto is disabled, raises elevator for l1 scoring, when left stick is
    // pressed, completes scoring motion
    // is auto is enabled, automatically goes to previously selected reef section and l1 scores
    /* controller
    .b()
    .whileTrue(
        Commands.deferredProxy(
            () -> {
              if (autoDisabled) {
                return Commands.startEnd(
                        () -> speedMultiplier = 0.25, () -> speedMultiplier = 1.0)
                    .withDeadline(coral.l1Score(controller.leftStick()));
              } else {
                return coral.goToReef(drivetrain, () -> reefSection, () -> 0);
              }
            }));*/

    // clearing algae off reef, down on plus for l1 clear, right on plus for l2 clear
    controller.povDown().onTrue(coral.l1ReefClear(controller.leftStick()));
    controller.povRight().onTrue(coral.l2ReefClear(controller.leftStick()));

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

    // deploy climber
    controller.leftBumper().whileTrue(climb.pivotClimb());
    // release climber and deploy algae for climb unless not nearing end of match
    controller
        .rightBumper()
        .whileTrue(
            climb
                .pivotRelease()
                .alongWith(algae.deployForClimb().unless(() -> DriverStation.getMatchTime() > 25)));
    // when left plus pressed, disable or enable auto, defaults at disabled
    controller.povLeft().onTrue(Commands.runOnce(() -> autoDisabled = !autoDisabled));
    controller.back().onTrue(Commands.runOnce(() -> gyro.zeroYaw()));
    controller.leftStick().onTrue(Commands.runOnce(()->stopGrab = true));
    controller.leftStick().onFalse(Commands.runOnce(()->stopGrab = false));
  }

  // deadbands for driving
  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, ControllerConfig.DEADBAND);
  }

  // sets the pose for auto-align
  public void periodic() {
    selectedSpot.setRobotPose(Positions.getIndividualReef(reefSection));
  }

  // autonomous command
  public Command getAutonomousCommand() {
    return Commands.sequence(
        Commands.waitSeconds(2), coral.goToReef(drivetrain, () -> reefSection, () -> 3));
  }
}
