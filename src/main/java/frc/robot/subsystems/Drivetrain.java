// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  
  
  final double TrackWidth = 0.1317625; // This is the distance between the left wheel and right wheel
  final double MaxXSpeed = 1; // Maximum speed in the forward/backward direction that you want it to go
  final double MaxAngularSpeed = 3*Math.PI; // Maximum rotational speed (2*Math.PI is one rotation per second)
  DifferentialDriveOdometry m_odometry; // Odometry - not used in this code really
  DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TrackWidth); // Kinematics - does math to determine speeds
  PIDController m_leftPIDController = new PIDController(0.7, 0.01, 0.01); // PID for left wheel
  PIDController m_rightPIDController = new PIDController(0.7, 0.01, 0.01); // PID for right wheel
  PIDController m_gyroPidController = new PIDController(0.05, 0.001, 0.002); // PID for gyro
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(Units.inchesToMeters(Math.PI * kWheelDiameterInch / kCountsPerRevolution));
    m_rightEncoder.setDistancePerPulse(Units.inchesToMeters(Math.PI * kWheelDiameterInch / kCountsPerRevolution));
    
    // Reset encoders and gyros
    resetEncoders();
    m_gyro.reset();

    // Start position for odometry
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(Units.degreesToRadians(m_gyro.getAngleZ())));
    
    // Sets tolerance for the PIDs
    m_leftPIDController.setTolerance(0.01);
    m_rightPIDController.setTolerance(0.01);
  }
  boolean straight = false; // True is the romi is going straight
  double start = 0.0; // Start angle when the romi starts going straight
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    double xSpeed = xaxisSpeed * MaxXSpeed; // Forward/Backward speed capped at MaxXSpeed
    double rotSpeed = -zaxisRotate * MaxAngularSpeed; // Rotational speed capped at MaxAngularSpeed
    
    // If you are not trying to turn
    if (rotSpeed == 0) {
      // If the romi is not already moving straight
      if (!straight) {
        // Set the start angle and set the straight variable to true because it is going straight now
        start = m_gyro.getAngleZ();
        straight = true;
      }
      // Gets the offset of the angle from the starting angle
      double offset = start - m_gyro.getAngleZ();
      // Sets the rotational speed based on the gyro offset and trying to make it 0
      rotSpeed = m_gyroPidController.calculate(offset, 0);
    }
    else {
      // If trying to turn then you are not going straight - set it to false
      straight = false;
    }
    // Update odometry to the current position
    m_odometry.update(new Rotation2d(Units.degreesToRadians(m_gyro.getAngleZ())), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // Determines left and right speeds based on kinematics (physics and math)
    // ChassisSpeeds determines the direction the robot is moving based on forward/backwards speed and rotational speed
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rotSpeed));
    
    // Determines the PID output based on the current velocities and what they should be at
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    // Sets the motor speeds capped at -1 and 1 based on the PID outputs
    m_leftMotor.set(MathUtil.clamp(leftOutput, -1, 1));
    m_rightMotor.set(MathUtil.clamp(rightOutput, -1, 1));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
