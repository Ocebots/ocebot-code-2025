package frc.robot.subsystems;

import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;

public class Vision {
  public static final PhotonCamera camera = new PhotonCamera(VisionConfig.CAMERA_NAME);
  // Creates a new camera with the designated name from the config
  public static final PhotonCamera secondCamera = new PhotonCamera(VisionConfig.SECOND_CAMERA_NAME);
  // Creates a new camera with the designated name from the config
}
