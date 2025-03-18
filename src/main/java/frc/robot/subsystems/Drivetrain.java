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

  public final PIDController xController =
      new PIDController(
          OrbitConfig.ORBIT_DISTANCE_P, OrbitConfig.ORBIT_DISTANCE_I, OrbitConfig.ORBIT_DISTANCE_D);
  public final PIDController yController =
      new PIDController(
          OrbitConfig.ORBIT_DISTANCE_P, OrbitConfig.ORBIT_DISTANCE_I, OrbitConfig.ORBIT_DISTANCE_D);
  public final PIDController rotController =
      new PIDController(
          OrbitConfig.ORBIT_ROTATION_P, OrbitConfig.ORBIT_ROTATION_I, OrbitConfig.ORBIT_ROTATION_D);

  private PhotonPoseEstimator vision =
      new PhotonPoseEstimator(
          VisionConfig.LAYOUT, VisionConfig.STRATEGY, VisionConfig.CAMERA_POSITION);

  Ultrasonic rangeFinder = new Ultrasonic(1, 2);

  // The gyro sensor
  @NotLogged private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  @NotLogged private final Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  @NotLogged
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DrivetrainConfig.MAX_ACCELERATION);

  @NotLogged
  private SlewRateLimiter rotLimiter =
      new SlewRateLimiter(DrivetrainConfig.MAX_ROTATIONAL_ACCELERATION);

  @Logged private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  @NotLogged
  private SwerveDrivePoseEstimator poseEstimator =
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
    SmartDashboard.putData(field);
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(
        DrivetrainConfig.DISTANCE_POSITION_TOLERANCE, DrivetrainConfig.DISTANCE_VELOCITY_TOLERANCE);
    yController.setTolerance(
        DrivetrainConfig.DISTANCE_POSITION_TOLERANCE, DrivetrainConfig.DISTANCE_VELOCITY_TOLERANCE);
    rotController.setTolerance(
        DrivetrainConfig.ROTATION_POSITION_TOLERANCE, DrivetrainConfig.ROTATION_VELOCITY_TOLERANCE);
  }

  @Logged(name = "desiredStates")
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getDesiredState(),
      this.frontRight.getDesiredState(),
      this.rearLeft.getDesiredState(),
      this.rearRight.getDesiredState()
    };
  }

  @Logged(name = "actualStates")
  public SwerveModuleState[] getActualStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.rearLeft.getState(),
      this.rearRight.getState()
    };
  }

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

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    desiredChassisSpeeds = speeds;
    var swerveModuleStates = DrivetrainConfig.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public double getDistance() {
    return rangeFinder.getRangeMM();
  }

  @Override
  public void periodic() {
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

    vision.setReferencePose(this.poseEstimator.getEstimatedPosition());

    List<PhotonPipelineResult> results = Vision.camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      vision
          .update(result)
          .ifPresent(
              (pose) -> {
                if (result.multitagResult.isEmpty()
                    && result.targets.get(0).bestCameraToTarget.getTranslation().getNorm() > 2) {
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
