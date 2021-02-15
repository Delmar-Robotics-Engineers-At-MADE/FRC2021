#pragma once

#include <string>
#include <vector>
#include <cmath>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace edu::wpi::first::wpilibj::examples::machinelearning { class Ball; }

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

namespace edu::wpi::first::wpilibj::examples::machinelearning
{

	using NetworkTable = edu::wpi::first::networktables::NetworkTable;
	using NetworkTableEntry = edu::wpi::first::networktables::NetworkTableEntry;
	using SubsystemBase = edu::wpi::first::wpilibj2::command::SubsystemBase;

	class VisionSubsystem : public SubsystemBase
	{
  private:
	  NetworkTable *table;
	  int totalObjects = 0, totalBalls = 0;
	  std::vector<Ball*> balls = std::vector<Ball*>(0);
	  std::vector<std::wstring> classes;
	  std::vector<double> boxes, box;
	  NetworkTableEntry *totalObjectsEntry, *classesEntry, *boxesEntry;

  private:
	  class Gamepiece
	  {
	public:
		double distance = 0, xOffset = 0;

		/**
		 * Gets the relative angle of the game piece in radians
		 * 
		 * @return the angle
		 */
		virtual double getAngle();
	  };

	  /**
	   * Represents a detected cargo from the Coral
	   */
  public:
	  class Ball : public Gamepiece
	  {

		// You will need to change this value depending on the year's game piece
	private:
		static constexpr double kGamepieceHeightInch = 7.0;

		/**
		 * Holds the data determined by Coral
		 *
		 * @param box the array of points
		 */
	public:
		Ball(std::vector<double> &box);
	  };

  public:
	  virtual ~VisionSubsystem()
	  {
		  delete table;
		  delete totalObjectsEntry;
		  delete classesEntry;
		  delete boxesEntry;
	  }

	  VisionSubsystem();

	  /**
	   * Periodically updates the list of detected objects with the data found on
	   * NetworkTables Also creates array of cargo and their relative position.
	   */
	  void periodic() override;

	  virtual int getTotalBalls();

	  virtual std::vector<Ball*> getBalls();
	};

}
