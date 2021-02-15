#include "DriveSubsystem.h"

namespace edu::wpi::first::wpilibj::examples::machinelearning
{
	using AnalogGyro = edu::wpi::first::wpilibj::AnalogGyro;
	using Encoder = edu::wpi::first::wpilibj::Encoder;
	using PWMVictorSPX = edu::wpi::first::wpilibj::PWMVictorSPX;
	using SpeedController = edu::wpi::first::wpilibj::SpeedController;
	using SpeedControllerGroup = edu::wpi::first::wpilibj::SpeedControllerGroup;
	using PIDController = edu::wpi::first::wpilibj::controller::PIDController;
	using Rotation2d = edu::wpi::first::wpilibj::geometry::Rotation2d;
	using ChassisSpeeds = edu::wpi::first::wpilibj::kinematics::ChassisSpeeds;
	using DifferentialDriveKinematics = edu::wpi::first::wpilibj::kinematics::DifferentialDriveKinematics;
	using DifferentialDriveOdometry = edu::wpi::first::wpilibj::kinematics::DifferentialDriveOdometry;
	using DifferentialDriveWheelSpeeds = edu::wpi::first::wpilibj::kinematics::DifferentialDriveWheelSpeeds;
	using SubsystemBase = edu::wpi::first::wpilibj2::command::SubsystemBase;
const double DriveSubsystem::kMaxAngularSpeed = 2 * M_PI;

	DriveSubsystem::DriveSubsystem() : m_odometry(new DifferentialDriveOdometry(getAngle()))
	{
	  m_gyro->reset();

	  // Set the distance per pulse for the drive encoders. We can simply use the
	  // distance traveled for one rotation of the wheel divided by the encoder
	  // resolution.
	  m_leftEncoder->setDistancePerPulse(2 * M_PI * kWheelRadius / kEncoderResolution);
	  m_rightEncoder->setDistancePerPulse(2 * M_PI * kWheelRadius / kEncoderResolution);

	  m_leftEncoder->reset();
	  m_rightEncoder->reset();

	}

	Rotation2d *DriveSubsystem::getAngle()
	{
	  // Negating the angle because WPILib gyros are CW positive.
	  return Rotation2d::fromDegrees(-m_gyro->getAngle());
	}

	void DriveSubsystem::setSpeeds(DifferentialDriveWheelSpeeds *speeds)
	{
	  double leftOutput = m_leftPIDController->calculate(m_leftEncoder->getRate(), speeds->leftMetersPerSecond);
	  double rightOutput = m_rightPIDController->calculate(m_rightEncoder->getRate(), speeds->rightMetersPerSecond);
	  m_leftGroup->set(leftOutput);
	  m_rightGroup->set(rightOutput);
	}

//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public void drive(double xSpeed, double rot)
	void DriveSubsystem::drive(double xSpeed, double rot)
	{
	  ChassisSpeeds tempVar(xSpeed, 0.0, rot);
	  auto wheelSpeeds = m_kinematics->toWheelSpeeds(&tempVar);
	  setSpeeds(wheelSpeeds);
	}

	void DriveSubsystem::updateOdometry()
	{
	  m_odometry->update(getAngle(), m_leftEncoder->getDistance(), m_rightEncoder->getDistance());
	}
}
