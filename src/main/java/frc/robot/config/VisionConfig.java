package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator;

public class VisionConfig {
  public static final String CAMERA_NAME = "apriltag"; // left
  public static final String SECOND_CAMERA_NAME = "apriltag2"; // right

  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final Transform3d LEFT_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(12.25),
          Units.inchesToMeters(10.75),
          0.0,
          new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d RIGHT_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(12.25),
          Units.inchesToMeters(-8.75),
          0.0,
          new Rotation3d(0.0, 0.0, 0.0));
}
