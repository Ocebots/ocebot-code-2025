package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.config.CANMappings;
import frc.robot.config.CoralPivotConfig;
import java.util.function.Supplier;

@Logged
public class CoralPivot extends SubsystemBase {
  private SparkMax pivot =
      new SparkMax(CANMappings.CORAL_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
  private Rotation2d lastAngle = new Rotation2d();
  private PIDController controller = new PIDController(3.8, 0.0, 0.0);
  private ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

  private SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.4).per(Second), Volts.of(0.8), Second.of(8)),
          new SysIdRoutine.Mechanism(
              (voltage) -> pivot.setVoltage(voltage.in(Volts)),
              (log) -> {
                log.motor("pivot")
                    .voltage(Volts.of(pivot.getAppliedOutput() * pivot.getBusVoltage()))
                    .angularPosition(
                        Radians.of(normalize(pivotEncoder.getPosition()) * 2.0 * Math.PI))
                    .angularVelocity(
                        RadiansPerSecond.of(pivotEncoder.getVelocity() * 2.0 * Math.PI));
              },
              this));

  public double normalize(double angle) {
    if (angle > Math.PI || angle < -Math.PI) {
      angle -= Math.round(angle / (2 * Math.PI)) * 2 * Math.PI;
    }

    return angle;
  }

  @Logged
  public double iAccum() {
    return pivot.getClosedLoopController().getIAccum();
  }

  public CoralPivot() {
    pivot.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(CoralPivotConfig.PIVOT_CURRENT_LIMIT)
            .idleMode(CoralPivotConfig.PIVOT_IDLE_MODE)
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(CoralPivotConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(CoralPivotConfig.ENCODER_VELOCITY_CONVERSION_FACTOR))
            .apply(new SignalsConfig().absoluteEncoderPositionPeriodMs(3))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        CoralPivotConfig.PIVOT_P,
                        CoralPivotConfig.PIVOT_I,
                        CoralPivotConfig.PIVOT_D)
                    .positionWrappingEnabled(true)
                    .positionWrappingMinInput(-Math.PI)
                    .positionWrappingMaxInput(Math.PI)
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .iMaxAccum(0.02)
                    .iZone(Rotation2d.fromDegrees(15).getRadians())),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(
        CoralPivotConfig.POSITION_TOLERANCE, CoralPivotConfig.VELOCITY_TOLERANCE);
  }

  public Command setPivotAngle(Supplier<Rotation2d> angle) {
    return Commands.runOnce(() -> pivot.getClosedLoopController().setIAccum(0.0))
        .andThen(
            Commands.runEnd(
                () ->
                    pivot.setVoltage(
                        controller.calculate(pivotEncoder.getPosition(), angle.get().getRadians())
                            + ff.calculate(angle.get().getRadians(), 0)),
                () -> pivot.stopMotor(),
                this));
  }

  public boolean isPivotReady() {
    return controller.atSetpoint();
  }

  public Command quasi(Direction dir) {
    return routine.quasistatic(dir);
  }

  public Command dynamic(Direction dir) {
    return routine.dynamic(dir);
  }
}
