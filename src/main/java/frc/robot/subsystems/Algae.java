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
    wheels.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(AlgaeConfig.WHEELS_CURRENT_LIMIT)
            .idleMode(AlgaeConfig.WHEELS_IDLEMODE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
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
    armController.setTolerance(AlgaeConfig.POSITION_TOLERANCE, AlgaeConfig.VELOCITY_TOLERANCE);
    armEncoder.setPosition(0.0);
  }

  private Command armAngChange(Rotation2d angle) {
    return Commands.runEnd(
        () ->
            arm.setVoltage(
                armController.calculate(armEncoder.getPosition(), angle.getRadians())
                    + armFF.calculate(angle.getRadians(), 0)),
        () -> arm.stopMotor());
  }

  private Command wheelsMove(double speed) {
    return Commands.runEnd(() -> wheels.set(speed), () -> wheels.stopMotor(), this);
  }

  @Logged
  public boolean isReady() {
    return armController.atSetpoint();
  }

  private Command waitUntilArmReady() {
    return Commands.waitUntil(() -> armController.atSetpoint());
  }

  public Command pickUpAlgae() {
    return armAngChange(AlgaeConfig.PICKUP_ANGLE).alongWith(wheelsMove(AlgaeConfig.INTAKE_SPEED));
  }

  public Command deployForClimb() {
    return armAngChange(Rotation2d.fromRadians(-1.55)).withDeadline(waitUntilArmReady());
  }

  public Command storeAlgae() {
    return armAngChange(AlgaeConfig.STORE_ANGLE)
        .alongWith(wheelsMove(AlgaeConfig.INTAKE_SPEED))
        .withDeadline(Commands.waitSeconds(1.0).andThen(waitUntilArmReady()));
  }

  public Command releaseAlgae() {
    return wheelsMove(-AlgaeConfig.INTAKE_SPEED)
        .withTimeout(AlgaeConfig.RELEASE_TIME)
        .andThen(returnToUp());
  }

  public Command returnToUp() {
    return armAngChange(Rotation2d.fromRadians(-0.65)).until(armController::atSetpoint);
  }
}
