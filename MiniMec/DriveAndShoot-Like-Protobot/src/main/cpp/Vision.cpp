// Original Java: https://github.com/wpilibsuite/allwpilib/tree/69cc85db8347f6519c65e360072e75852adc5851/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/machinelearning
// Converted to C++ using converter from Tangible: https://www.tangiblesoftwaresolutions.com/product_details/java_to_cplusplus_converter_details.html

#include "Vision.h"
#include <iostream> // for std::cout

const static double kBallTrackSmoothingTime = 2.0;
const static double kBallAngleTolerance = 0.8; // ignore up to this % fluctuation

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

VisionSubsystem::VisionSubsystem()  // constructor
{
    table = nt::NetworkTableInstance::GetDefault().GetTable("ML");
    totalObjectsEntry = table->GetEntry ("nb_objects");
    classesEntry = table->GetEntry("object_classes");
    boxesEntry = table->GetEntry("boxes");
    m_timer.Reset();
    m_timer.Start();
}

void VisionSubsystem::periodic()
{
    totalBalls = 0;
    totalObjects = static_cast<int>(totalObjectsEntry.GetDouble(0));
    classes = classesEntry.GetStringArray(std::vector<std::string>(totalObjects));
    boxes = boxesEntry.GetDoubleArray(std::vector<double>(4 * totalObjects));

    // Count up number of balls
    for (auto s : classes) {
        if (s == "Power_Cell") {
            totalBalls++;
        }
    }

    // was... balls = std::vector<Ball*>(totalBalls);

    // Generate array of Ball objects... moved to getBalls()
}

int VisionSubsystem::getTotalBalls()
{
    return totalBalls;
}

std::vector<VisionSubsystem::Ball*> VisionSubsystem::getBalls()
{
    // was... return balls;
    // Generate array of Ball objects... moved here from periodic
    // std::cout << "Start of getBalls" << std::endl;
    int index = 0;
    std::vector<Ball*> balls = std::vector<Ball*>(totalBalls); 
    for (int i = 0; i < totalObjects; i += 4) {
        box = std::vector<double>(4);
        for (int j = 0; j < 4; j++) {
            box[j] = boxes[i + j];
        }
        if (classes[i] == "Power_Cell") {
            balls[index] = new Ball(box);
            index++;
        }
    }
    // std::cout << "End of getBalls" << std::endl;
    return balls;
}

void VisionSubsystem::disposeBalls(std::vector<VisionSubsystem::Ball*> balls)
{
    // std::cout << "disposing " << balls.size() << " balls" << std::endl;
    for (int i = 0; i < balls.size(); i += 4) {
        if (balls[i] != NULL) {
            delete(balls[i]);
        }
    }
}

void VisionSubsystem::updateClosestBall() {
    // frc::SmartDashboard::PutNumber("Power Cells", ballcount);
    int ballcount = getTotalBalls();
    bool noBallsForAWhile = m_timer.Get() > timeBallsLastSeen + kBallTrackSmoothingTime;
    if (ballcount == 0 && noBallsForAWhile) { // no balls for a while, so zero things out
        distanceClosestBall = 0.0;
        angleClosestBall = 0.0; 
    } else for (int i = 0; i < ballcount; i++) {
        std::vector<VisionSubsystem::Ball*> balls = getBalls();
        if (balls[i] != NULL) {
            if (   (distanceClosestBall == 0) /* seeing a ball for first time in a while */
                || (balls[i]->distance < distanceClosestBall) ) { /* or new ball is closer than old ball */
                double candidateAngle = balls[i]->getAngle();
                // if (angleClosestBall == 0.0 /* seeing a ball for the first time in a while */
                //  || abs(candidateAngle - angleClosestBall) / abs(angleClosestBall) > kBallAngleTolerance) {
                //     // change of more than (tolerance)% angle, so update closest ball
                distanceClosestBall = balls[i]->distance;
                angleClosestBall = candidateAngle;
                timeBallsLastSeen = m_timer.Get();
            }
        }
        disposeBalls(balls);
    }
}