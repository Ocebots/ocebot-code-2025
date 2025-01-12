package frc.robot.loggers;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
  public RelativeEncoderLogger() {
    super(RelativeEncoder.class);
  }

  @Override
  protected void update(EpilogueBackend backend, RelativeEncoder encoder) {
    backend.log("Relative Position", encoder.getPosition());
    backend.log("Velocity", encoder.getVelocity());
  }
}
