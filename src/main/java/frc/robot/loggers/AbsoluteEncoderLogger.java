package frc.robot.loggers;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AbsoluteEncoder.class)
public class AbsoluteEncoderLogger extends ClassSpecificLogger<AbsoluteEncoder> {
  public AbsoluteEncoderLogger() {
    super(AbsoluteEncoder.class);
  }

  @Override
  protected void update(EpilogueBackend backend, AbsoluteEncoder encoder) {
    backend.log("Absolute Position", encoder.getPosition());
    backend.log("Velocity", encoder.getVelocity());
  }
}
