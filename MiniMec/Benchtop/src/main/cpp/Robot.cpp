/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h> // for logging to riolog window

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    m_timer.Reset();
    m_timer.Start();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    double timer = m_timer.Get();

    // test each motor individually
    if (timer < 2.0) {
      m_leftRear.Set(0.7);
    } else if (timer < 4.0) {
      m_leftRear.Set(0);
      m_leftFront.Set(0.7);
    } else if (timer < 6.0) {
      m_leftFront.Set(0);
      m_rightFront.Set(0.7);
    } else if (timer < 8.0) {
      m_rightFront.Set(0);
      m_rightRear.Set(0.7);
    } else if (timer < 10.0) {
      m_rightRear.Set(0);
    }

    // Drive for some seconds
    if (timer > 10 && timer < 20.0) {
      // Drive forwards half speed
      m_robotDrive.DriveCartesian(0, 0.5, 0, 0);
    } else if (timer > 20.0) {
      // Stop robot
      m_robotDrive.DriveCartesian(0, 0, 0, 0);
    }
  }
}

#define MOTOR_SCALE 1.7
#define DEADZONE 0.1
void Robot::TeleopPeriodic() {
  wpi::outs() << "teleop loop";
  bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
  if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
  }
  
  double xspeed = abs(m_stick.GetRawAxis(2)) > DEADZONE ? -m_stick.GetRawAxis(2) : 0;
  double yspeed = abs(m_stick.GetRawAxis(3)) > DEADZONE ? m_stick.GetRawAxis(3) : 0;
  double zspeed = abs(m_stick.GetRawAxis(0)) > DEADZONE ? -m_stick.GetRawAxis(0) : 0;

  m_robotDrive.DriveCartesian(xspeed, yspeed, zspeed, -ahrs->GetAngle()); // MJS: navx heading
}

void Robot::TestPeriodic() {}
void Robot::TeleopInit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
