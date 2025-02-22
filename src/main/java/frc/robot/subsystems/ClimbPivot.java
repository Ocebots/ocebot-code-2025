package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.ClimbPivotConfig;

public class ClimbPivot extends SubsystemBase {
  private SparkMax pivot =
      new SparkMax(CANMappings.CLIMB_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
  private PIDController pivotController =
      new PIDController(
          ClimbPivotConfig.PIVOT_P, ClimbPivotConfig.PIVOT_I, ClimbPivotConfig.PIVOT_D);

  public ClimbPivot() {
    pivot.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ClimbPivotConfig.PIVOT_CURRENT_LIMIT)
            .idleMode(ClimbPivotConfig.PIVOT_IDLE_MODE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public Command pivotClimb() {
    return Commands.runEnd(() -> pivot.set(1), () -> pivot.set(0), this);
  }

  public Command pivotRelease() {
    return Commands.runEnd(() -> pivot.set(-1), () -> pivot.set(0), this);
  }

  public boolean isPivotReady() {
    return pivotController.atSetpoint();
  }
}
