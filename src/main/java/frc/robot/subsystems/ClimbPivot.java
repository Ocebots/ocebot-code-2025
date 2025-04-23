package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.ClimbPivotConfig;

@Logged
public class ClimbPivot extends SubsystemBase {
  private SparkMax pivot =
      new SparkMax(CANMappings.CLIMB_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);

  public ClimbPivot() {
    // Configures climb pivot motor
    pivot.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ClimbPivotConfig.PIVOT_CURRENT_LIMIT)
            .idleMode(ClimbPivotConfig.PIVOT_IDLE_MODE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  // Runs the climb motor and stops it when interrupted
  public Command pivotClimb() {
    return Commands.runEnd(() -> pivot.set(0.5), () -> pivot.set(0), this);
  }

  // Runs the climb motor in the opposite direction until the robot is interrupted
  public Command pivotRelease() {
    return Commands.runEnd(() -> pivot.set(-0.5), () -> pivot.set(0), this);
  }
}
