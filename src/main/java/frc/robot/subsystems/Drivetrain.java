// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.DrivetrainConfig;

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

  // The gyro sensor
  private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final Field2d field = new Field2d();

  private double currentRotation = 0.0;

  // Slew rate filter variables for controlling lateral acceleration
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DrivetrainConfig.MAX_ACCELERATION);
  private SlewRateLimiter rotLimiter =
      new SlewRateLimiter(DrivetrainConfig.MAX_ROTATIONAL_ACCELERATION);

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

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    SmartDashboard.putData(field);
  }

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
    var swerveModuleStates = DrivetrainConfig.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void logData() {
    frontLeft.sendData("drive/frontLeft");
    frontRight.sendData("drive/frontRight");
    rearLeft.sendData("drive/rearLeft");
    rearRight.sendData("drive/rearRight");

    SmartDashboard.putNumber("drive/gyro", getHeading().getRadians());

    field.setRobotPose(getPose());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.poseEstimator.update(
        getHeading(),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.rearLeft.getPosition(),
          this.rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
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
      this.currentRotation = rotLimiter.calculate(rot) * DrivetrainConfig.MAX_ANGULAR_SPEED;
    } else {
      xSpeedCommanded = xSpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      ySpeedCommanded = ySpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      this.currentRotation = rot * DrivetrainConfig.MAX_ANGULAR_SPEED;
    }

    this.setChassisSpeeds(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedCommanded,
                ySpeedCommanded,
                this.currentRotation,
                getPose()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? Rotation2d.fromDegrees(0)
                            : Rotation2d.fromDegrees(180)))
            : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, this.currentRotation));
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    this.frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    this.frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Resets the pose estimate to the given pose */
  public void zeroPoseEstimate(Pose2d newPose) {
    this.poseEstimator.resetPose(newPose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
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
}
