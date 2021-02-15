#include "Robot.h"
#include "TurnToBallCommand.h"

namespace edu::wpi::first::wpilibj::examples::machinelearning
{
	using TimedRobot = edu::wpi::first::wpilibj::TimedRobot;
	using XboxController = edu::wpi::first::wpilibj::XboxController;
	using Button = edu::wpi::first::wpilibj2::command::button::Button;
	using JoystickButton = edu::wpi::first::wpilibj2::command::button::JoystickButton;

	void Robot::robotInit()
	{
	  edu::wpi::first::wpilibj::TimedRobot::robotInit();
	  Button * const a = new JoystickButton(m_controller, XboxController::Button::kA::value);
	  TurnToBallCommand tempVar(driveSubsystem, visionSubsystem);
	  a->whenPressed(&tempVar);
	}
}
