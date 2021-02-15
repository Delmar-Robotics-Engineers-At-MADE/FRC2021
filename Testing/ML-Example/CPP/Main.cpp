#include "Main.h"
#include "Robot.h"

namespace edu::wpi::first::wpilibj::examples::machinelearning
{
	using RobotBase = edu::wpi::first::wpilibj::RobotBase;

	Main::Main()
	{
	}

	void Main::main(std::vector<std::wstring> &args)
	{
//JAVA TO C++ CONVERTER TODO TASK: Method reference constructor syntax is not converted by Java to C++ Converter:
	  RobotBase::startRobot(Robot::new);
	}
}
