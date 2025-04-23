package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.AlgaeConfig;
import frc.robot.config.CANMappings;

@Logged
public class Algae extends SubsystemBase {
  private SparkMax wheels = new SparkMax(CANMappings.ALGAE_WHEEL_ID, MotorType.kBrushless);
  private SparkMax arm = new SparkMax(CANMappings.ALGAE_ARM_ID, MotorType.kBrushless);
  private RelativeEncoder armEncoder = arm.getEncoder();
  private PIDController armController =
      new PIDController(AlgaeConfig.ARM_P, AlgaeConfig.ARM_I, AlgaeConfig.ARM_D);
  private ArmFeedforward armFF =
      new ArmFeedforward(
          AlgaeConfig.ARM_S, AlgaeConfig.ARM_G, AlgaeConfig.ARM_V, AlgaeConfig.ARM_A);

  public Algae() {
    // configuration of wheel motor in algae mechanism
    wheels.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(AlgaeConfig.WHEELS_CURRENT_LIMIT)
            .idleMode(AlgaeConfig.WHEELS_IDLEMODE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // configuration of motor for arm in algae mechanism
    arm.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(AlgaeConfig.ARM_CURRENT_LIMIT)
            .idleMode(AlgaeConfig.ARM_IDLEMODE)
            .inverted(true)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(AlgaeConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(AlgaeConfig.ENCODER_VELOCITY_CONVERSION_FACTOR)),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // setting position and velocity tolerance of arm motor PID controller
    armController.setTolerance(AlgaeConfig.POSITION_TOLERANCE, AlgaeConfig.VELOCITY_TOLERANCE);
    // setting position of arm relative encoder
    armEncoder.setPosition(0.0);
  }

  // Command for setting the algae arm to specified angle
  private Command armAngChange(Rotation2d angle) {
    return Commands.runEnd(
        () ->
            arm.setVoltage(
                armController.calculate(armEncoder.getPosition(), angle.getRadians())
                    + armFF.calculate(angle.getRadians(), 0)),
        () -> arm.stopMotor());
  }

  // Command that moves algae wheels given a speed and stops wheels when interrupted
  private Command wheelsMove(double speed) {
    return Commands.runEnd(() -> wheels.set(speed), () -> wheels.stopMotor(), this);
  }

  // Command that returns whether the arm is at the set point according to the PID controller
  @Logged
  public boolean isReady() {
    return armController.atSetpoint();
  }

  // Command that has the other commands wait until algae arm is at the setpoint
  private Command waitUntilArmReady() {
    return Commands.waitUntil(() -> armController.atSetpoint());
  }

  // Command that changes arm angle to the pickup angle and move the wheels of the algae mechanism
  // at the given intake speed
  public Command pickUpAlgae() {
    return armAngChange(AlgaeConfig.PICKUP_ANGLE).alongWith(wheelsMove(AlgaeConfig.INTAKE_SPEED));
  }

  // Command that waits until the algae arm is at the setpoint then changes the algae angle to the
  // angle required while the robot climbs
  public Command deployForClimb() {
    return armAngChange(Rotation2d.fromRadians(-1.55)).withDeadline(waitUntilArmReady());
  }

  // Command that waits one second and then until the arm is at its setpoint, then changes the arm
  // angle to the store angle and moves the wheels at the given intake speed
  public Command storeAlgae() {
    return armAngChange(AlgaeConfig.STORE_ANGLE)
        .alongWith(wheelsMove(AlgaeConfig.INTAKE_SPEED))
        .withDeadline(Commands.waitSeconds(1.0).andThen(waitUntilArmReady()));
  }

  // Command that moves the algae wheels for the given release time and then returns the arm to the
  // up position
  public Command releaseAlgae() {
    return wheelsMove(-AlgaeConfig.INTAKE_SPEED)
        .withTimeout(AlgaeConfig.RELEASE_TIME)
        .andThen(returnToUp());
  }

  // Command that sets the arm to a given angle until it is at the setpoint
  public Command returnToUp() {
    return armAngChange(Rotation2d.fromRadians(0.0)).until(armController::atSetpoint);
  }
}
