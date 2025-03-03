package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.CoralPivotConfig;
import java.util.function.Supplier;

@Logged
public class CoralPivot extends SubsystemBase {
  private SparkMax pivot =
      new SparkMax(CANMappings.CORAL_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();

  private Rotation2d lastAngle = new Rotation2d();

  public CoralPivot() {
    pivot.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(CoralPivotConfig.PIVOT_CURRENT_LIMIT)
            .idleMode(CoralPivotConfig.PIVOT_IDLE_MODE)
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(CoralPivotConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(CoralPivotConfig.ENCODER_VELOCITY_CONVERSION_FACTOR))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        CoralPivotConfig.PIVOT_P,
                        CoralPivotConfig.PIVOT_I,
                        CoralPivotConfig.PIVOT_D)
                    .positionWrappingEnabled(true)
                    .positionWrappingMinInput(-Math.PI)
                    .positionWrappingMaxInput(Math.PI)
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public Command setPivotAngle(Supplier<Rotation2d> angle) {
    return Commands.runEnd(
        () ->
            pivot
                .getClosedLoopController()
                .setReference((lastAngle = angle.get()).getRadians(), ControlType.kPosition),
        () -> pivot.stopMotor(),
        this);
  }

  public boolean isPivotReady() {
    return Math.abs(pivotEncoder.getPosition() - lastAngle.getRadians())
            < CoralPivotConfig.POSITION_TOLERANCE
        && pivotEncoder.getVelocity() < CoralPivotConfig.VELOCITY_TOLERANCE;
  }
}
