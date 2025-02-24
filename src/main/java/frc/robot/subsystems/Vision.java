package frc.robot.subsystems;

import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;

public class Vision {
  public static final PhotonCamera camera = new PhotonCamera(VisionConfig.CAMERA_NAME);
}
