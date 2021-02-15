#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

class VisionSubsystem {
public:

    class Gamepiece{
    public:
        double distance = 0, xOffset = 0;
        virtual double getAngle();
    };

    class Ball : public Gamepiece {
    private:
        static constexpr double kGamepieceHeightInch = 7.0;
    public:
        Ball(std::vector<double> &box);
    };

    std::shared_ptr<NetworkTable> table;
    int totalObjects = 0, totalBalls = 0;
    std::vector<Ball*> balls = std::vector<Ball*>(0);
    std::vector<std::string> classes;
    std::vector<double> boxes, box;
    nt::NetworkTableEntry totalObjectsEntry, classesEntry, boxesEntry; // were pointers

    virtual ~VisionSubsystem() {
        //delete table;
        //delete totalObjectsEntry;
        //delete classesEntry;
        //delete boxesEntry;
    }
    VisionSubsystem();

    /**
     * Periodically updates the list of detected objects with the data found on
     * NetworkTables Also creates array of cargo and their relative position.
     */
    void periodic(); // was override

    virtual int getTotalBalls();
    virtual std::vector<Ball*> getBalls();
};
