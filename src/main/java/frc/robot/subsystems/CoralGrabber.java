package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.CoralGrabberConfig;

public class CoralGrabber extends SubsystemBase {
  private SparkMax motor = new SparkMax(CANMappings.END_EFFECTOR_ID, MotorType.kBrushless);
  private LinearFilter filter =
      LinearFilter.singlePoleIIR(
          CoralGrabberConfig.FILTER_TIME_CONSTANT, TimedRobot.kDefaultPeriod);

  public CoralGrabber() {
    // Running positive should grab the coral, negative should release it
    motor.configure(
        new SparkMaxConfig()
            .idleMode(CoralGrabberConfig.IDLE_MODE)
            .smartCurrentLimit(CoralGrabberConfig.CURRENT_LIMIT),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("coralGrabber/current", filter.lastValue());
  }

  public Command grabCoral() {
    return Commands.runEnd(
            () -> motor.set(CoralGrabberConfig.GRAB_SPEED), () -> motor.stopMotor(), this)
        .until(
            () -> filter.calculate(motor.getOutputCurrent()) > CoralGrabberConfig.TRIGGER_CURRENT);
  }

  public Command releaseCoral() {
    return Commands.runEnd(
            () -> motor.set(-CoralGrabberConfig.RELEASE_SPEED), () -> motor.stopMotor(), this)
        .withTimeout(CoralGrabberConfig.RELEASE_TIME);
  }
}
