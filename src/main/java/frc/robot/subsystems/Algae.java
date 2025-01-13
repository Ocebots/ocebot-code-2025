package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.AlgaeConfig;
import frc.robot.config.CANMappings;

public class Algae extends SubsystemBase {
  private SparkMax wheels = new SparkMax(CANMappings.ALGAE_WHEEL_ID, MotorType.kBrushless);
  private SparkMax arm = new SparkMax(CANMappings.ALGAE_ARM_ID, MotorType.kBrushless);
  private AbsoluteEncoder armEncoder = arm.getAbsoluteEncoder();
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
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(AlgaeConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(AlgaeConfig.ENCODER_VELOCITY_CONVERSION_FACTOR)),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    armController.setTolerance(AlgaeConfig.POSITION_TOLERANCE, AlgaeConfig.VELOCITY_TOLERANCE);
  }
}
