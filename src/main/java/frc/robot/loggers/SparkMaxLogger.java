package frc.robot.loggers;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkBase.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkBase> {
  public SparkMaxLogger() {
    super(SparkBase.class);
  }

  @Override
  protected void update(EpilogueBackend backend, SparkBase motor) {
    backend.log("Motor Current (A)", motor.getOutputCurrent());
    backend.log("Motor Applied Output", motor.getAppliedOutput());
    backend.log("Motor Requested Output", motor.get());
    backend.log("Bus Voltage", motor.getBusVoltage());
  }
}
