package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.AlgaeConfig;
import frc.robot.config.CANMappings;

public class Algae extends SubsystemBase {
  private SparkMax wheels = new SparkMax(CANMappings.ALGAE_WHEEL_ID, MotorType.kBrushless);
  private SparkMax arm = new SparkMax(CANMappings.ALGAE_ARM_ID, MotorType.kBrushless);

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
            .idleMode(AlgaeConfig.ARM_IDLEMODE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }
}
