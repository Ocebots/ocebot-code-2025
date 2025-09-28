// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.*;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule frontLeft =
      new SwerveModule(
          CANMappings.FRONT_LEFT_DRIVING,
          CANMappings.FRONT_LEFT_TURNING,
          DrivetrainConfig.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRight =
      new SwerveModule(
          CANMappings.FRONT_RIGHT_DRIVING,
          CANMappings.FRONT_RIGHT_TURNING,
          DrivetrainConfig.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule rearLeft =
      new SwerveModule(
          CANMappings.REAR_LEFT_DRIVING,
          CANMappings.REAR_LEFT_TURNING,
          DrivetrainConfig.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule rearRight =
      new SwerveModule(
          CANMappings.REAR_RIGHT_DRIVING,
          CANMappings.REAR_RIGHT_TURNING,
          DrivetrainConfig.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // creates PID controllers
  public final PIDController xController =
      new PIDController(
          OrbitConfig.ORBIT_DISTANCE_P, OrbitConfig.ORBIT_DISTANCE_I, OrbitConfig.ORBIT_DISTANCE_D);
  public final PIDController yController =
      new PIDController(
          OrbitConfig.ORBIT_DISTANCE_P, OrbitConfig.ORBIT_DISTANCE_I, OrbitConfig.ORBIT_DISTANCE_D);
  public final PIDController rotController =
      new PIDController(
          OrbitConfig.ORBIT_ROTATION_P, OrbitConfig.ORBIT_ROTATION_I, OrbitConfig.ORBIT_ROTATION_D);

  // Creates PhotonPoseEstimator that estimates the robot position from one camera
  private PhotonPoseEstimator vision =
      new PhotonPoseEstimator(
          VisionConfig.LAYOUT, VisionConfig.STRATEGY, VisionConfig.LEFT_CAMERA_POSITION);

  // Creates PhotonPoseEstimator that estimates the robot position from another camera
  private PhotonPoseEstimator driverVision =
      new PhotonPoseEstimator(
          VisionConfig.LAYOUT, VisionConfig.STRATEGY, VisionConfig.RIGHT_CAMERA_POSITION);

  // creates Ultrasonic rangeFinder that measures distance (currently not in use)
  Ultrasonic rangeFinder = new Ultrasonic(1, 2);

  // The gyro sensor
  @NotLogged private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // creates new field 2d to be shared with smart dashboard
  @NotLogged private final Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  @NotLogged
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DrivetrainConfig.MAX_ACCELERATION);

  // slew rate filter variables for controlling rotation acceleration
  @NotLogged
  private SlewRateLimiter rotLimiter =
      new SlewRateLimiter(DrivetrainConfig.MAX_ROTATIONAL_ACCELERATION);

  // creates new robot chassis speeds
  @Logged private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  // creates new swerve drive pose estimator
  @NotLogged
  public SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DrivetrainConfig.DRIVE_KINEMATICS,
          getHeading(),
          new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.rearLeft.getPosition(),
            this.rearRight.getPosition()
          },
          new Pose2d(0, 0, getHeading()));

  public Drivetrain() {
    // sends field data to smart dashboard
    SmartDashboard.putData(field);
    // makes sure that rotController automatically maps shortest route to setpoint
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    // sets xController, yController, and rotController position and velocity tolerance
    xController.setTolerance(
        DrivetrainConfig.DISTANCE_POSITION_TOLERANCE, DrivetrainConfig.DISTANCE_VELOCITY_TOLERANCE);
    yController.setTolerance(
        DrivetrainConfig.DISTANCE_POSITION_TOLERANCE, DrivetrainConfig.DISTANCE_VELOCITY_TOLERANCE);
    rotController.setTolerance(
        DrivetrainConfig.ROTATION_POSITION_TOLERANCE, DrivetrainConfig.ROTATION_VELOCITY_TOLERANCE);
  }

  // Logs desired swerve module states
  @Logged(name = "desiredStates")
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getDesiredState(),
      this.frontRight.getDesiredState(),
      this.rearLeft.getDesiredState(),
      this.rearRight.getDesiredState()
    };
  }

  // Logs actual swerve module states
  @Logged(name = "actualStates")
  public SwerveModuleState[] getActualStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.rearLeft.getState(),
      this.rearRight.getState()
    };
  }

  // logs actual chassis speeds
  @Logged(name = "actualChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConfig.DRIVE_KINEMATICS.toChassisSpeeds(
        new SwerveModuleState[] {
          this.frontLeft.getState(),
          this.frontRight.getState(),
          this.rearLeft.getState(),
          this.rearRight.getState()
        });
  }

  // sets desired chassis speeds given a speed and gives swerveModuleStates new module states
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    desiredChassisSpeeds = speeds;
    var swerveModuleStates = DrivetrainConfig.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    // renormalizes wheel speeds if any individual speed is above the specified maximum given the
    // current swerve module states and the absolute maximum speed the swerve modules can reach
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

    // sets the desired states of the swerve modules
    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // the range measured from the rangefinder in millimeters
  public double getDistance() {
    return rangeFinder.getRangeMM();
  }

  @Override
  public void periodic() {
    // has ultrasonic sensor send ping and get distance measurement periodically
    rangeFinder.ping();
    // Update the odometry in the periodic block

    field.setRobotPose(
        this.poseEstimator.update(
            getHeading(),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.rearLeft.getPosition(),
              this.rearRight.getPosition()
            }));

    // Updates the stored reference pose for use when using the CLOSEST_TO_REFERENCE_POSE_STRATEGY
    // (not in use)
    vision.setReferencePose(this.poseEstimator.getEstimatedPosition());
    driverVision.setReferencePose(this.poseEstimator.getEstimatedPosition());

    // Puts the pose data from one camera into a list
    List<PhotonPipelineResult> results = Vision.camera.getAllUnreadResults();

    // If there is pose data from the cameras, get the latest estimated pose and update the
    // 'vision'
    // photon pose estimator
    // If there is no multi tag result and the distance from the camera to the target is greater
    // than
    // 4 meters, return
    // Otherwise, add the latest vision pose estimate to a filter with the odometry pose estimate
    // and set
    // the guessed pose from that to the current pose
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      vision
          .update(result)
          .ifPresent(
              (pose) -> {
                if (result.multitagResult.isEmpty()
                    && result.targets.get(0).bestCameraToTarget.getTranslation().getNorm() > 4) {
                  return;
                }
                this.poseEstimator.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds);
              });
    }

    results = Vision.secondCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      driverVision
          .update(result)
          .ifPresent(
              (pose) -> {
                if (result.multitagResult.isEmpty()
                    && result.targets.get(0).bestCameraToTarget.getTranslation().getNorm() > 4) {
                  return;
                }
                this.poseEstimator.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds);
              });
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Logged
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public Command followLine(
      DoubleSupplier distance, DoubleSupplier speed, Supplier<Rotation2d> rotation) {
    return Commands.runEnd(
        () -> {
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  -xController.calculate(getDistance(), distance.getAsDouble()),
                  speed.getAsDouble(),
                  rotController.calculate(
                      getPose().getRotation().getRadians(), rotation.get().getRadians()),
                  rotation.get());

          setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getPose().getRotation()));
        },
        () -> drive(0, 0, 0, false, false),
        this);
  }

  // Initializes and logs targetAngle and lastDistance
  @Logged public Rotation2d targetAngle;
  @Logged public double lastDistance;

  public Command orbit(Supplier<Pose2d> center, DoubleSupplier speed, DoubleSupplier distance) {
    return Commands.runEnd(
        () -> {
          Transform2d relativeTransform = getPose().minus(center.get());
          targetAngle = center.get().getTranslation().minus(getPose().getTranslation()).getAngle();
          lastDistance = relativeTransform.getTranslation().getNorm();
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  -xController.calculate(
                      relativeTransform.getTranslation().getNorm(), distance.getAsDouble()),
                  speed.getAsDouble(),
                  rotController.calculate(
                      getPose().getRotation().getRadians(), targetAngle.getRadians()),
                  targetAngle);

          setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getPose().getRotation()));
        },
        () -> drive(0, 0, 0, false, false),
        this);
  }

  /**
   * Method to drive the robot using joystick info. This method may not work as intended if the
   * joystick does not move in a circle
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotationCommanded;

    if (rateLimit) {
      // Convert XY to polar(theta and magnitude) for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag =
          Math.min(
              Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2))
                  * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND,
              DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

      inputTranslationMag = magLimiter.calculate(inputTranslationMag);

      xSpeedCommanded = inputTranslationMag * Math.cos(inputTranslationDir);
      ySpeedCommanded = inputTranslationMag * Math.sin(inputTranslationDir);
      rotationCommanded = rotLimiter.calculate(rot) * DrivetrainConfig.MAX_ANGULAR_SPEED;
    } else {
      xSpeedCommanded = xSpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      ySpeedCommanded = ySpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      rotationCommanded = rot * DrivetrainConfig.MAX_ANGULAR_SPEED;
    }

    this.setChassisSpeeds(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedCommanded,
                ySpeedCommanded,
                rotationCommanded,
                getPose()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? Rotation2d.fromDegrees(0)
                            : Rotation2d.fromDegrees(180)))
            : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotationCommanded));
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    this.frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    this.frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        this.gyro.getAngle() * (DrivetrainConfig.GYRO_IS_REVERSED ? -1.0 : 1.0));
  }

  public void followTrajectory(SwerveSample sample) {
    System.out.println("Follow trajectory");
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + rotController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
  }
}
