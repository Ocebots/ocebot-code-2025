package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator;

public class VisionConfig {
  public static final String CAMERA_NAME = "apriltag";

  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final Transform3d CAMERA_POSTION =
      new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

  public static double CAMERA_HEIGHT_METERS = 0.0;
  public static double CAMERA_PITCH_RADIANS = 0.0;
}
