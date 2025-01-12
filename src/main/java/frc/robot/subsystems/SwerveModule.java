// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.SwerveModuleConfig;

@Logged
public class SwerveModule {
  private final SparkMax drivingSparkMax;

  private final SparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;

  private final AbsoluteEncoder turningEncoder;

  @NotLogged private final SparkClosedLoopController drivingPIDController;
  @NotLogged private final SparkClosedLoopController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    this.turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    this.drivingSparkMax.configure(
        new SparkMaxConfig()
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(SwerveModuleConfig.DRIVING_ENCODER_POSITION_FACTOR)
                    .velocityConversionFactor(SwerveModuleConfig.DRIVING_ENCODER_VELOCITY_FACTOR))
            .apply(
                new ClosedLoopConfig()
                    .pidf(
                        SwerveModuleConfig.DRIVING_P,
                        SwerveModuleConfig.DRIVING_I,
                        SwerveModuleConfig.DRIVING_D,
                        SwerveModuleConfig.DRIVING_FF)
                    .outputRange(
                        SwerveModuleConfig.DRIVING_MIN_OUTPUT,
                        SwerveModuleConfig.DRIVING_MAX_OUTPUT)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder))
            .idleMode(SwerveModuleConfig.DRIVING_MOTOR_IDLE_MODE)
            .smartCurrentLimit(SwerveModuleConfig.DRIVING_MOTOR_CURRENT_LIMIT),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.turningSparkMax.configure(
        new SparkMaxConfig()
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(SwerveModuleConfig.TURNING_ENCODER_POSITION_FACTOR)
                    .velocityConversionFactor(SwerveModuleConfig.TURNING_ENCODER_VELOCITY_FACTOR)
                    .inverted(SwerveModuleConfig.IS_TURNING_ENCODER_INVERTED))
            .apply(
                new ClosedLoopConfig()
                    .pidf(
                        SwerveModuleConfig.TURNING_P,
                        SwerveModuleConfig.TURNING_I,
                        SwerveModuleConfig.TURNING_D,
                        SwerveModuleConfig.TURNING_FF)
                    .outputRange(
                        SwerveModuleConfig.TURNING_MIN_OUTPUT,
                        SwerveModuleConfig.TURNING_MAX_OUTPUT)
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(
                        SwerveModuleConfig.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        SwerveModuleConfig.TURNING_ENCODER_POSITION_PID_MAX_INPUT))
            .idleMode(SwerveModuleConfig.TURNING_MOTOR_IDLE_MODE)
            .smartCurrentLimit(SwerveModuleConfig.TURNING_MOTOR_CURRENT_LIMIT),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    this.drivingEncoder = drivingSparkMax.getEncoder();
    this.turningEncoder = turningSparkMax.getAbsoluteEncoder();
    this.drivingPIDController = drivingSparkMax.getClosedLoopController();
    this.turningPIDController = turningSparkMax.getClosedLoopController();

    this.chassisAngularOffset = chassisAngularOffset;
    this.desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    this.drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Logged
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        this.drivingEncoder.getVelocity(),
        new Rotation2d(this.turningEncoder.getPosition() - this.chassisAngularOffset));
  }

  public SwerveModuleState getDesiredState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        this.desiredState.speedMetersPerSecond,
        new Rotation2d(this.desiredState.angle.getRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        this.drivingEncoder.getPosition(),
        new Rotation2d(this.turningEncoder.getPosition() - this.chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(this.chassisAngularOffset));

    correctedDesiredState.optimize(new Rotation2d(this.turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.drivingPIDController.setReference(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    this.turningPIDController.setReference(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.drivingEncoder.setPosition(0);
  }
}
