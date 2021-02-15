#pragma once

#include "VisionSubsystem.h"
#include "DriveSubsystem.h"

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

namespace edu::wpi::first::wpilibj::examples::machinelearning
{

	using TimedRobot = edu::wpi::first::wpilibj::TimedRobot;
	using XboxController = edu::wpi::first::wpilibj::XboxController;

	class Robot : public TimedRobot
	{
  private:
	  VisionSubsystem *visionSubsystem = new VisionSubsystem();
	  DriveSubsystem *driveSubsystem = new DriveSubsystem();
	  XboxController *const m_controller = new XboxController(0);


  public:
	  virtual ~Robot()
	  {
		  delete visionSubsystem;
		  delete driveSubsystem;
		  delete m_controller;
	  }

	  void robotInit() override;
	};

}
