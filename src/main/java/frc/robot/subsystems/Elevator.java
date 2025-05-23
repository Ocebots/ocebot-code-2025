package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.ElevatorConfig;
import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {
  private SparkFlex elevator =
      new SparkFlex(CANMappings.ELEVATOR_L_ID, SparkLowLevel.MotorType.kBrushless);
  private SparkFlex elevatorFollower =
      new SparkFlex(CANMappings.ELEVATOR_R_ID, SparkLowLevel.MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevator.getEncoder();
  private ElevatorFeedforward elevatorFF =
      new ElevatorFeedforward(
          ElevatorConfig.EL_S, ElevatorConfig.EL_G, ElevatorConfig.EL_V, ElevatorConfig.EL_A);
  private ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          ElevatorConfig.EL_P,
          ElevatorConfig.EL_I,
          ElevatorConfig.EL_D,
          new TrapezoidProfile.Constraints(
              ElevatorConfig.MAX_VELOCITY, ElevatorConfig.MAX_ACCELERATION),
          TimedRobot.kDefaultPeriod);

  public Elevator() {
    elevator.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(ElevatorConfig.ELEVATOR_CURRENT_LIMIT)
            .idleMode(ElevatorConfig.ELEVATOR_IDLE_MODE)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(ElevatorConfig.ENCODER_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(ElevatorConfig.ENCODER_VELOCITY_CONVERSION_FACTOR)),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    elevatorFollower.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(ElevatorConfig.ELEVATOR_CURRENT_LIMIT)
            .idleMode(ElevatorConfig.ELEVATOR_IDLE_MODE)
            .follow(elevator, true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    elevatorController.setTolerance(
        ElevatorConfig.POSITION_TOLERANCE, ElevatorConfig.VELOCITY_TOLERANCE);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command setElevatorHeight(DoubleSupplier height) {
    return Commands.runEnd(
        () -> {
          elevator.setVoltage(
              elevatorController.calculate(elevatorEncoder.getPosition(), height.getAsDouble())
                  + elevatorFF.calculate(elevatorController.getSetpoint().velocity));
        },
        () -> elevator.stopMotor(),
        this);
  }

  public boolean isAtPosition() {
    return elevatorController.atGoal();
  }
}
