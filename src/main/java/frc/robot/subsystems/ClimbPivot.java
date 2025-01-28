package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.ClimbPivotConfig;
import java.util.function.Supplier;

public class ClimbPivot extends SubsystemBase {
  private SparkMax pivot =
      new SparkMax(CANMappings.CLIMB_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
  private RelativeEncoder pivotEnconder = pivot.getEncoder();
  private PIDController pivotController =
      new PIDController(
          ClimbPivotConfig.PIVOT_P, ClimbPivotConfig.PIVOT_I, ClimbPivotConfig.PIVOT_D);
  private ArmFeedforward pivotFF =
      new ArmFeedforward(
          ClimbPivotConfig.PIVOT_S,
          ClimbPivotConfig.PIVOT_G,
          ClimbPivotConfig.PIVOT_V,
          ClimbPivotConfig.PIVOT_A);

  public ClimbPivot() {
    pivot.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ClimbPivotConfig.PIVOT_CURRENT_LIMIT)
            .idleMode(ClimbPivotConfig.PIVOT_IDLE_MODE)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(ClimbPivotConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(ClimbPivotConfig.ENCODER_VELOCITY_CONVERSION_FACTOR)),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public Command setPivotAngle(Supplier<Rotation2d> angle) {
    return Commands.runEnd(
        () ->
            pivot.setVoltage(
                pivotController.calculate(pivotEnconder.getPosition(), angle.get().getRadians())
                    + pivotFF.calculate(angle.get().getRadians(), 0)),
        () -> pivot.stopMotor(),
        this);
  }

  public Command waitUntilPivotReady() {
    return Commands.waitUntil(() -> pivotController.atSetpoint());
  }

  public boolean isPivotReady() {
    return waitUntilPivotReady().isFinished();
  }
}
