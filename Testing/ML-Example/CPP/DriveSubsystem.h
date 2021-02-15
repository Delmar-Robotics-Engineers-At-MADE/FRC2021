#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

namespace edu::wpi::first::wpilibj::examples::machinelearning
{

	using AnalogGyro = edu::wpi::first::wpilibj::AnalogGyro;
	using Encoder = edu::wpi::first::wpilibj::Encoder;
	using PWMVictorSPX = edu::wpi::first::wpilibj::PWMVictorSPX;
	using SpeedController = edu::wpi::first::wpilibj::SpeedController;
	using SpeedControllerGroup = edu::wpi::first::wpilibj::SpeedControllerGroup;
	using PIDController = edu::wpi::first::wpilibj::controller::PIDController;
	using Rotation2d = edu::wpi::first::wpilibj::geometry::Rotation2d;
	using DifferentialDriveKinematics = edu::wpi::first::wpilibj::kinematics::DifferentialDriveKinematics;
	using DifferentialDriveOdometry = edu::wpi::first::wpilibj::kinematics::DifferentialDriveOdometry;
	using DifferentialDriveWheelSpeeds = edu::wpi::first::wpilibj::kinematics::DifferentialDriveWheelSpeeds;
	using SubsystemBase = edu::wpi::first::wpilibj2::command::SubsystemBase;

	/**
	 * Represents a differential drive style drivetrain.
	 */
	class DriveSubsystem : public SubsystemBase
	{
  public:
	  static constexpr double kMaxSpeed = 3.0; // meters per second
	  static const double kMaxAngularSpeed; // one rotation per second

  private:
	  static constexpr double kTrackWidth = 0.381 * 2; // meters
	  static constexpr double kWheelRadius = 0.0508; // meters
	  static constexpr int kEncoderResolution = 4096;

	  SpeedController *const m_leftMaster = new PWMVictorSPX(1);
	  SpeedController *const m_leftFollower = new PWMVictorSPX(2);
	  SpeedController *const m_rightMaster = new PWMVictorSPX(3);
	  SpeedController *const m_rightFollower = new PWMVictorSPX(4);

	  Encoder *const m_leftEncoder = new Encoder(0, 1);
	  Encoder *const m_rightEncoder = new Encoder(2, 3);

	  SpeedControllerGroup *const m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
	  SpeedControllerGroup *const m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

	  AnalogGyro *const m_gyro = new AnalogGyro(0);

	  PIDController *const m_leftPIDController = new PIDController(1, 0, 0);
	  PIDController *const m_rightPIDController = new PIDController(1, 0, 0);

	  DifferentialDriveKinematics *const m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

	  DifferentialDriveOdometry *const m_odometry;

	  /**
	   * Constructs a differential drive object.
	   * Sets the encoder distance per pulse and resets the gyro.
	   */
  public:
	  virtual ~DriveSubsystem()
	  {
		  delete m_leftMaster;
		  delete m_leftFollower;
		  delete m_rightMaster;
		  delete m_rightFollower;
		  delete m_leftEncoder;
		  delete m_rightEncoder;
		  delete m_leftGroup;
		  delete m_rightGroup;
		  delete m_gyro;
		  delete m_leftPIDController;
		  delete m_rightPIDController;
		  delete m_kinematics;
		  delete m_odometry;
	  }

	  DriveSubsystem();

	  /**
	   * Returns the angle of the robot as a Rotation2d.
	   *
	   * @return The angle of the robot.
	   */
	  virtual Rotation2d *getAngle();

	  /**
	   * Sets the desired wheel speeds.
	   *
	   * @param speeds The desired wheel speeds.
	   */
	  virtual void setSpeeds(DifferentialDriveWheelSpeeds *speeds);

	  /**
	   * Drives the robot with the given linear velocity and angular velocity.
	   *
	   * @param xSpeed Linear velocity in m/s.
	   * @param rot    Angular velocity in rad/s.
	   */
//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public void drive(double xSpeed, double rot)
	  virtual void drive(double xSpeed, double rot);

	  /**
	   * Updates the field-relative position.
	   */
	  virtual void updateOdometry();
	};

}
