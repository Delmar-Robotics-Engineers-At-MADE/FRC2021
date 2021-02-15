#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

const static double kToleranceDegrees = 2.0f;
const static double kMaxRotateRate = 0.5;
const static double kGamepadDeadZone = 0.1;
const static double kSlowSpeedFactor = 0.8;
const static double kFastSpeedFactor = 1.0;

const static double kPtuned = 0.006;
const static double kItuned = 0.0015;
const static double kDtuned = 0.001;

const static double kFieldRelDriveSmoothTime = 0.4;
const static double kHeadingDiscontinuityZone = 3; // degrees either side of 0
const static double kTargetDiscontinuityZone = 15; // degrees either side of 0
//const static double kHeadingDiscontinuityZone2 = 360 - kHeadingDiscontinuityZone1;
//const static double kTargetDiscontinuityZone2 = 360 - kTargetDiscontinuityZone1;

void Robot::RobotInit() {

  m_stick = new Joystick(joystickChannel);
	rotateToAngleRate = 0.0f;

  m_robotDrive.SetExpiration(0.1);

  try
  {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi:
     * - Communication via USB.
     * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    ahrs = new AHRS(SPI::Port::kMXP);
    m_visionSubsystem = new VisionSubsystem();
  }
  catch (std::exception &ex)
  {
    std::string what_string = ex.what();
    std::string err_msg("Error instantiating navX MXP:  " + what_string);
    const char *p_err_msg = err_msg.c_str();
    DriverStation::ReportError(p_err_msg);
  }

  // set all motors inverted, using drive groups
  m_left.SetInverted(true);
  m_right.SetInverted(true);

  /* this is used to tune the PID numbers
  frc::SmartDashboard::PutNumber("kP", kP);
  frc::SmartDashboard::PutNumber("kI", kI);
  frc::SmartDashboard::PutNumber("kD", kD);
  frc::SmartDashboard::PutNumber("MaxRotateRate", MaxRotateRate);
  */

}

void Robot::RobotPeriodic() {
  m_visionSubsystem->periodic();
}

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
}

void Robot::AutonomousPeriodic() {
    // simple motion to validate motor configuration
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      m_robotDrive.TankDrive(0.5, 0); // left motor only
    } else if (m_timer.Get() < 4.0) {
      m_robotDrive.TankDrive(0, 0.5); // right motor only
    } else if (m_timer.Get() < 6.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }

}

void Robot::TeleopInit() {
  ahrs->ZeroYaw();
  kP = kPtuned;
  kI = kItuned;
  kD = kDtuned;
  /* used to tune PID numbers
  kP = frc::SmartDashboard::GetNumber("kP", kP);
  kI = frc::SmartDashboard::GetNumber("kI", kI);
  kD = frc::SmartDashboard::GetNumber("kD", kD);
  MaxRotateRate = frc::SmartDashboard::GetNumber("MaxRotateRate", MaxRotateRate);
  */
  m_pidController = new frc2::PIDController (kP, kI, kD);
  m_pidController->SetTolerance(8, 8);  // within 8 degrees of target is considered on set point
  
  // Talon SR has a jumper for brake/coast... also this should be in robot init!
  // m_leftfront.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  // m_leftrear.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  // m_rightfront.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  // m_rightfront.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

  // needed to smooth out field-relative driving
  m_timer.Reset();
  m_timer.Start();
}

double Robot::TrimSpeed (double s, double max) {
  double result = s > max ? max : s;
  result = result < -max ? -max : result;
  return result;
}

double Robot::ScaleSpeed (double s, double scale) {
  return s * scale;
}

double Robot::ConvertRadsToDegrees (double rads) {
  const static double conversion_factor = 180.0/3.141592653589793238463;
  return rads * conversion_factor;
}

int quadrant (double a) { // quadrant of angle, I, II, III, or IV
  assert (a >= 0 && a <= 360);
  int result = 0;
  if (a <= 90) result = 1;
  else if (a <= 180) result = 2;
  else if (a <= 270) result = 3;
  else result = 4;
  return result;
}

bool isInDiscontinuityZone (double curr, double target) {
  // heading is close to 0 and target is close to heading
  return ( (curr < kHeadingDiscontinuityZone || curr > 360 - kHeadingDiscontinuityZone)
     && abs(curr - target) < kHeadingDiscontinuityZone );
        // && (target < kTargetDiscontinuityZone || target > 360 - kTargetDiscontinuityZone) ) 
}

void Robot::TeleopPeriodic() {
  
  bool reset_yaw_button_pressed = m_stick->GetRawButton(1);
  if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
  }

  double currAngle = (int)ahrs->GetAngle() % 360;  // angle accumulates past 360, so modulus
  if (currAngle < 0) {currAngle = 360 + currAngle;} // shift from -360>360 to 0>360
  frc::SmartDashboard::PutNumber("Angle", currAngle);
  frc::SmartDashboard::PutNumber("Heading", ahrs->GetCompassHeading());
  frc::SmartDashboard::PutNumber("Yaw", ahrs->GetYaw());

  // exercise vision system
  frc::SmartDashboard::PutNumber("Power Cells", m_visionSubsystem->getTotalBalls());
  if (m_visionSubsystem->getTotalBalls() > 0) {
    std::vector<VisionSubsystem::Ball*> balls = m_visionSubsystem->getBalls();
    if (balls[0] != NULL) {
		  frc::SmartDashboard::PutNumber("Cell0 angle", balls[0]->getAngle());
    }
    m_visionSubsystem->disposeBalls(balls);
  }

  bool rotateToAngle = false;
  //bool stepOver = false;
  if ( m_stick->GetPOV() == 0) {
      m_pidController->SetSetpoint(0.0f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 90) {
      m_pidController->SetSetpoint(90.0f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 180) {
      m_pidController->SetSetpoint(179.9f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 270) {
      m_pidController->SetSetpoint(-90.0f);
      rotateToAngle = true;
  // }  else if ( stick->GetRawButton(1)) {
  //     m_pidController->SetSetpoint(-90.0f);
  //     stepOver = true;
  // }  else if ( stick->GetRawButton(3)) {
  //     m_pidController->SetSetpoint(90.0f);
  //     stepOver = true;
  }

  double field_rel_X = m_stick->GetRawAxis(2);
  double field_rel_Y = -m_stick->GetRawAxis(3);
  double field_rel_R = sqrt(field_rel_X*field_rel_X + field_rel_Y*field_rel_Y);
  if (field_rel_R > kGamepadDeadZone) {
    rotateToAngle = true;
    // was angle = copysign(angle, field_rel_X); // make angle negative if X is negative
  }
  frc::SmartDashboard::PutNumber ("X", field_rel_X);
  frc::SmartDashboard::PutNumber ("Y", field_rel_Y);
  frc::SmartDashboard::PutNumber ("R", field_rel_R);

  // bool slow_gear_button_pressed = m_stick->GetRawButton(5);
  bool high_gear_button_presssed = m_stick->GetRawButton(7);
  // if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
  // else if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
  if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
  else {speed_factor = kSlowSpeedFactor;}
  frc::SmartDashboard::PutNumber ("Drive Speed Factor", speed_factor);

  try {

    if (rotateToAngle) {
      // MJS: since it's diff drive instead of mecanum drive, use tank method for rotation

      // a little trig to convert joystick to angle
      double targetAngle =  90 - ConvertRadsToDegrees(atan(field_rel_Y/abs(field_rel_X)));
      if (field_rel_X < 0) {targetAngle = 360 - targetAngle;}  // shift from -180>180 to 0>360
      m_pidController->SetSetpoint(targetAngle);
      frc::SmartDashboard::PutNumber ("Angle set point", targetAngle);

      // use pid for motor speed, unless in "discontinuity zone" near 0
      if (isInDiscontinuityZone(currAngle, targetAngle)) {
        rotateToAngleRate = 0.0;   // avoid bouncing back and forth as heading flips between 1 and 355
      } else {
        rotateToAngleRate = m_pidController->Calculate(currAngle);
      }
      // trim the speed so it's not too fast
      rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);
      // if heading is quadrant I and target is IV, or vice versa, flip motor direction
      if ( (quadrant(currAngle) == 1 && quadrant(targetAngle) == 4) 
        || (quadrant(currAngle) == 4 && quadrant(targetAngle) == 1)) {
        rotateToAngleRate = -rotateToAngleRate;
      } else if (abs(currAngle - targetAngle) > 180) {
        // if delta angle > 180, flip motor direction so we take shorter route
        rotateToAngleRate = -rotateToAngleRate;
      }

      frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
      double left_power = rotateToAngleRate;
      double right_power = -rotateToAngleRate;
      if (abs(kMaxRotateRate - abs(rotateToAngleRate)) / kMaxRotateRate > 0.4) { 
        m_field_rel_timer = m_timer.Get(); // once decide it's ok to move forward, don't stutter
      }
      if (m_timer.Get() < m_field_rel_timer + kFieldRelDriveSmoothTime) { 
        // add forard driving to rotation, to get field relative driving
        double addition = field_rel_R * speed_factor;
        left_power += addition;
        right_power += addition;
      }
      m_robotDrive.TankDrive(left_power, right_power, false);
    // } else if (stepOver) {
    //   frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
    //   if (m_pidController->AtSetpoint()) {
    //     m_robotDrive.TankDrive(MaxRotateRate, MaxRotateRate, false); // drive forward
    //   } else {
    //     m_robotDrive.TankDrive(rotateToAngleRate, -rotateToAngleRate, false);
    //   }
    } else {
      // not rotating; drive by stick
      m_robotDrive.ArcadeDrive(ScaleSpeed(-m_stick->GetY(), speed_factor), ScaleSpeed(m_stick->GetX(), speed_factor));
      // was... m_pidController->Reset(); // clears out integral state, etc
    }
  } catch (std::exception& ex ) {
    std::string err_string = "Error communicating with Drive System:  ";
    err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
