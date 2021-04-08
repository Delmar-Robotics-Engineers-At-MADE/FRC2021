#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/Timer.h>

const static double kGalacticSearchAngleTolerance = 3.5;

enum GalacticSearchPath {
		kARed = 0,
		kABlue,
		kBRed,
        kBBlue
	};

extern double ConvertRadsToDegrees (double rads);
extern double ConvertDegreesToRads (double degs);

class AutoPath{
public:
    std::string m_searchAngles;
    std::string m_searchPositions;
    std::string m_searchPathName;
    AutoPath(std::string name, std::string angles, std::string positions);
};

class VisionSubsystem {
private:
    frc::Timer m_timer;
    double timeBallsLastSeen = 0.0;
    std::string allBallsSorted = "none";
    const static int kMaxTriesToSeeThreeBalls = 50;

    AutoPath *m_autoPaths[sizeof(GalacticSearchPath)];

public:
    double distanceClosestBall = 0.0;
    double angleClosestBall = 0.0;
    double distanceSecondClosestBall = 0.0;
    double angleSecondClosestBall = 0.0;
    double turnToNextBallAngle = 0.0;

    class Gamepiece{
    public:
        double distance = 0, xOffset = 0;
        virtual double getAngle();
        // virtual ~Gamepiece();
    };

    class Ball : public Gamepiece {
    private:
        static constexpr double kGamepieceHeightInch = 7.0;
    public:
        Ball(std::vector<double> &box);
        virtual ~Ball(); // destructor
        virtual bool operator < (const Ball& b) const; // comparison for sorting
    };

    static bool ballPtrComparator (Ball *a, Ball *b);

    std::shared_ptr<NetworkTable> table;
    int totalObjects = 0, totalBalls = 0;
    // std::vector<Ball*> balls = std::vector<Ball*>(0);  // this is an unnecessary memory management complexity
    std::vector<std::string> classes;
    std::vector<double> boxes /*, box*/;
    nt::NetworkTableEntry totalObjectsEntry, classesEntry, boxesEntry; // were pointers

    virtual ~VisionSubsystem(); // destructor
    
    VisionSubsystem(); // constructor

    /**
     * Periodically updates the list of detected objects with the data found on
     * NetworkTables Also creates array of cargo and their relative position.
     */
    void periodic(); // was override

    virtual int getTotalBalls();
    virtual std::vector<Ball*> getBalls();
    virtual std::vector<Ball*> getFakeBalls();
    virtual void disposeBalls(std::vector<Ball*> &balls);
    virtual void updateClosestBall();
    virtual void clearSecondClosestBall();
    virtual std::string sortBallAngles();
    virtual std::string sortFakeBallAngles();
    virtual AutoPath *selectAutoPath();
    virtual AutoPath *defaultAutoPath();
};
