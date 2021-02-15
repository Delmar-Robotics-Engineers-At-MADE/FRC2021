#include "Vision.h"

double VisionSubsystem::Gamepiece::getAngle()
{
    return std::atan(xOffset / distance);
}

VisionSubsystem::Ball::Ball(std::vector<double> &box)
{

    // You will need to tune these constants for this year's game piece
    this->distance = 231.13 * std::pow(box[3] - box[1], -1.303);

    // This equation is constant between years. You only need to change the value of the constant.
    this->xOffset = (160 - ((box[0] + box[2]) / 2)) / (((box[3] - box[1]) / kGamepieceHeightInch) * 39.37);
}

VisionSubsystem::VisionSubsystem()
{
    table = nt::NetworkTableInstance::GetDefault().GetTable("ML");
    totalObjectsEntry = table->GetEntry ("nb_objects");
    classesEntry = table->GetEntry("object_classes");
    boxesEntry = table->GetEntry("boxes");
}

void VisionSubsystem::periodic()
{
    totalBalls = 0;
    totalObjects = static_cast<int>(totalObjectsEntry.GetDouble(0));
    classes = classesEntry.GetStringArray(std::vector<std::string>(totalObjects));
    boxes = boxesEntry.GetDoubleArray(std::vector<double>(4 * totalObjects));

    // Count up number of balls
    for (auto s : classes)
    {
    if (s == "Ball")
    {
        totalBalls++;
    }
    }

    balls = std::vector<Ball*>(totalBalls);
    int index = 0;

    // Generate array of Ball objects
    for (int i = 0; i < totalObjects; i += 4)
    {
    box = std::vector<double>(4);
    for (int j = 0; j < 4; j++)
    {
        box[j] = boxes[i + j];
    }
    if (classes[i] == "Ball")
    {
        balls[index] = new Ball(box);
        index++;
    }
    }
}

int VisionSubsystem::getTotalBalls()
{
    return totalBalls;
}

std::vector<VisionSubsystem::Ball*> VisionSubsystem::getBalls()
{
    return balls;
}
