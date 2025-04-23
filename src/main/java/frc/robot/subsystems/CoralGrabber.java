package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.CoralGrabberConfig;

@Logged
public class CoralGrabber extends SubsystemBase { //CoralGrabber is a wheel that spins to suck in coral
  private SparkMax motor = new SparkMax(CANMappings.END_EFFECTOR_ID, MotorType.kBrushless);//Creates the SparkMax with its End Effector ID number and brushless motor type.
  private LinearFilter filter =
      LinearFilter.singlePoleIIR(
          CoralGrabberConfig.FILTER_TIME_CONSTANT, TimedRobot.kDefaultPeriod);//Creates a filter for motor input that smooths down unexpected changes in current.

  public CoralGrabber() {//creating motor and determining settings
    // Running positive should grab the coral, negative should release it
    motor.configure(
        new SparkMaxConfig()
            .idleMode(CoralGrabberConfig.IDLE_MODE)//idle mode is brake (stops movement when not in use, doesn't coast)
            .smartCurrentLimit(CoralGrabberConfig.CURRENT_LIMIT)// Sets current limit to config number
            .inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double filteredCurrent() {
    return filter.lastValue();
  }//Returns the most recent current value from the filter.

  private Command grabCoralRaw() {//Grabs coral while ignoring the current limit
    return Commands.runEnd(
        () -> motor.set(CoralGrabberConfig.GRAB_SPEED), () -> motor.stopMotor(), this);
  }

  public Command grabCoral() {//Grabs coral as long as the current is not over the limit.
    return Commands.runOnce(() -> filter.reset())
        .andThen(
            grabCoralRaw()
                .until(
                    () ->
                        filter.calculate(motor.getOutputCurrent())
                            > CoralGrabberConfig.TRIGGER_CURRENT));
  }

  public Command releaseCoral() {//Releases coral for L1,L2,and L3 by spinning the wheel away from the robot.
    return Commands.runEnd(
            () -> motor.set(-CoralGrabberConfig.RELEASE_SPEED), () -> motor.stopMotor(), this)
        .withTimeout(CoralGrabberConfig.RELEASE_TIME);
  }

  public Command releaseCoralL4() {//Spins the wheel away from the robot in order to place the coral on the reef. This instance uses setting constants particularly for lever 4 on the reef.
    return Commands.runEnd(
            () -> motor.set(-CoralGrabberConfig.L4_RELEASE_SPEED), () -> motor.stopMotor(), this)
        .withTimeout(CoralGrabberConfig.RELEASE_TIME);
  }

  public Command removeAlgae() {//Spins the wheel towards the robot in order to remove algae.
    return Commands.runEnd(
        () -> motor.set(-CoralGrabberConfig.ALGAE_REMOVAL_SPEED), () -> motor.stopMotor(), this);
  }

  public void stop() {
    motor.stopMotor();
  } //Stops motor

  public void run() {
    motor.set(0.1);
  }// Sets motor speed to (roughly) 10% of max
}
