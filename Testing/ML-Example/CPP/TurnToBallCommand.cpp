#include "TurnToBallCommand.h"

namespace edu::wpi::first::wpilibj::examples::machinelearning
{
	using PIDController = edu::wpi::first::wpilibj::controller::PIDController;
	using PIDCommand = edu::wpi::first::wpilibj2::command::PIDCommand;
	using DriveSubsystem = edu::wpi::first::wpilibj::examples::machinelearning::DriveSubsystem;
	using VisionSubsystem = edu::wpi::first::wpilibj::examples::machinelearning::VisionSubsystem;

	TurnToBallCommand::TurnToBallCommand(DriveSubsystem *driveSubsystem, VisionSubsystem *visionSubsystem) : 
PIDController tempVar(1, 0, 0);
edu::wpi::first::wpilibj2::command::PIDCommand(&tempVar, [&] ()
{
	  driveSubsystem->getAngle()->getRadians();
}, 0.0, [&] (double value)
{
	  driveSubsystem->drive(0, value);
  }, driveSubsystem, visionSubsystem)
  {
	  this->driveSubsystem = driveSubsystem;
	  this->visionSubsystem = visionSubsystem;
	  // Set tolerance for PID
	  getController().setTolerance(10, 5);
	  // Using gyro heading in Radians, so continuous input
	  getController().enableContinuousInput(-M_PI, M_PI);
	}

	void TurnToBallCommand::initialize()
	{
	  edu::wpi::first::wpilibj2::command::PIDCommand::initialize();
	  getController().setSetpoint(getBallAngle());
	}

	bool TurnToBallCommand::isFinished()
	{
	  return getController().atSetpoint();
	}

	double TurnToBallCommand::getBallAngle()
	{
	  if (visionSubsystem->getTotalBalls() > 0)
	  {
		return visionSubsystem->getBalls()[0]->getAngle();
	  }
	  // return current drivetrain angle if no hatch detected, so no turning
	  return driveSubsystem->getAngle()->getRadians();
	}
}
