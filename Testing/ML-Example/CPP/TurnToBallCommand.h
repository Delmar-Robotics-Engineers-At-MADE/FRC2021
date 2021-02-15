#pragma once

#define _USE_MATH_DEFINES
#include "DriveSubsystem.h"
#include "VisionSubsystem.h"
#include <cmath>

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

namespace edu::wpi::first::wpilibj::examples::machinelearning
{

	using PIDCommand = edu::wpi::first::wpilibj2::command::PIDCommand;
	using DriveSubsystem = edu::wpi::first::wpilibj::examples::machinelearning::DriveSubsystem;
	using VisionSubsystem = edu::wpi::first::wpilibj::examples::machinelearning::VisionSubsystem;

	/**
	 * Command that turns the robot to face the first detected hatch
	 */
	class TurnToBallCommand : public PIDCommand
	{
  private:
	  VisionSubsystem *visionSubsystem;
	  DriveSubsystem *driveSubsystem;

	  /**
	   * PIDCommand uses relative hatch angle at time of initialization, not construction.
	   * Turn is based on gyro.
	   * @param driveSubsystem the drive subsystem
	   * @param visionSubsystem the vision subsystem
	   */
  public:
	  virtual ~TurnToBallCommand()
	  {
		  delete visionSubsystem;
		  delete driveSubsystem;
	  }

	  TurnToBallCommand(DriveSubsystem *driveSubsystem, VisionSubsystem *visionSubsystem);

	  /**
	   * Sets setpoint at init
	   */
	  void initialize() override;

	  bool isFinished() override;

	  /**
	   * Gets the angle of the hatch if there is one
	   * @return angle of hatch
	   */
	  virtual double getBallAngle();
	};

}
