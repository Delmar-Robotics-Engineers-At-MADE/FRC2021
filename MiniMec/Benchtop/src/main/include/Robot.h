/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Joystick.h>
#include <frc/Talon.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/DriverStation.h>  // for reporting errors from navx

#include "AHRS.h" // was "navx/AHRS.h"

using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  Robot() {    
      m_robotDrive.SetExpiration(0.1);  
      // m_leftRear.SetInverted(true);
      // m_rightRear.SetInverted(true);
      // m_leftFront.SetInverted(true);
      // m_rightFront.SetInverted(true);  
      m_robotDrive.SetRightSideInverted(true);      
      m_timer.Start();  
      try {
          ahrs = new AHRS(SPI::Port::kMXP);
      } catch (std::exception& ex ) {
          std::string err_string = "Error instantiating navX MXP:  ";
          err_string += ex.what();
          DriverStation::ReportError(err_string.c_str());
      }
    }

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Robot drive system  
  frc::Talon m_leftRear{0};  // was {0, 1}
  frc::Talon m_leftFront{1};  // was {1, 1}
  frc::Talon m_rightRear{2};  
  frc::Talon m_rightFront{3};  
  frc::MecanumDrive m_robotDrive{m_leftFront, m_leftRear, m_rightFront, m_rightRear};
  frc::Joystick m_stick{0};  
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();  
  frc::Timer m_timer;

  AHRS *ahrs;  // navX MXP (update for Magic House April 2019)





};
