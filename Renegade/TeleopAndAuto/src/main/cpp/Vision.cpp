// Original Java: https://github.com/wpilibsuite/allwpilib/tree/69cc85db8347f6519c65e360072e75852adc5851/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/machinelearning
// Converted to C++ using converter from Tangible: https://www.tangiblesoftwaresolutions.com/product_details/java_to_cplusplus_converter_details.html

#include "Vision.h"
#include <iostream> // for std::cout
#include <sstream> // for stringstream

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

VisionSubsystem::Ball::~Ball() { // destructor; nothing to do
}

bool VisionSubsystem::Ball::operator < (const Ball& b) const {  // comparator for sorting
        return (distance < b.distance);
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

VisionSubsystem::~VisionSubsystem() { // destructor
    // was...
    //delete table;
    //delete totalObjectsEntry;
    //delete classesEntry;
    //delete boxesEntry;
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
    for (uint i = 0; i < balls.size(); i += 4) {
        if (balls[i] != NULL) {
            delete(balls[i]);
        }
    }
}

double ComputeTurnAngle (double a, double b, double theta) {
    /*    c
       O------O  balls      \ angle to turn
        \bta /               \---------------------0
       a \th/ b               \ beta
          \/                   \ 
         robot                robot
    
    law of cosines says: c = sqrt(a^2 + b^2 - 2ab Cos(theta))
    law of sines says: beta = Sin^-1(b Sin(theta) / c)
    angle to turn is 180 - beta
    */
   double c = sqrt(a*a + b*b - 2*a*b * cos(theta));
   double beta = asin(b * sin(theta) / c);
   return M_PI - beta;
}

void VisionSubsystem::updateClosestBall() {
    // frc::SmartDashboard::PutNumber("Power Cells", ballcount);
    int ballcount = getTotalBalls();
    std::vector<VisionSubsystem::Ball*> balls = getBalls();
    bool noBallsForAWhile = m_timer.Get() > timeBallsLastSeen + kBallTrackSmoothingTime;
    if (ballcount == 0 && noBallsForAWhile) { // no balls for a while, so zero things out
        distanceClosestBall = 0.0;
        angleClosestBall = 0.0; 
        // don't zero out second closest info, because now is the time to turn toward it
    } else for (uint i = 0; i < balls.size(); i++) {
        if (balls[i] != NULL) {
            if (   (distanceClosestBall == 0) /* seeing a ball for first time in a while */
                || (balls[i]->distance < distanceClosestBall) ) { /* or new ball is closer than old ball */
                double candidateAngle = balls[i]->getAngle();
                //if (abs((candidateAngle - angleClosestBall) / angleClosestBall) > kBallAngleTolerance) {
                    // this jump in angle implies previous closest ball lost and new one acquired
                    // ... what to do with this?
                //} else {
                    distanceClosestBall = balls[i]->distance;
                    angleClosestBall = candidateAngle;
                    timeBallsLastSeen = m_timer.Get();
                //}
            } else if ( (distanceSecondClosestBall == 0 && balls[i]->distance > distanceClosestBall)
                     || (balls[i]->distance < distanceSecondClosestBall) ) { 
                // this is not the closest ball, but is the second closest ball
                double candidateAngle = balls[i]->getAngle();
                if (abs((candidateAngle - angleSecondClosestBall) / angleSecondClosestBall) > kBallAngleTolerance) {
                    // too great a jump; ignore
                } else {
                    distanceSecondClosestBall = balls[i]->distance;
                    angleSecondClosestBall = candidateAngle;
                    turnToNextBallAngle = ComputeTurnAngle(distanceClosestBall, distanceSecondClosestBall,
                                                           angleSecondClosestBall - angleClosestBall);
                }
            }
        }
        disposeBalls(balls);
    }
}

std::string VisionSubsystem::sortedBallAngles() {
    std::vector<VisionSubsystem::Ball*> balls = getBalls();
    std::sort(balls.begin(), balls.end());  // uses custom operator < for comparison
    std::stringstream ss;
    for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
      it != balls.end(); ++it) { 
        ss << (*it)->distance << " "; 
    }
    std::string result = ss.str();
    disposeBalls(balls);
    return result;
}

// call this after turn to next ball is complete
void VisionSubsystem::clearSecondClosestBall() {
    distanceSecondClosestBall = 0.0;
    angleSecondClosestBall = 0.0;
    turnToNextBallAngle = 0.0;
}