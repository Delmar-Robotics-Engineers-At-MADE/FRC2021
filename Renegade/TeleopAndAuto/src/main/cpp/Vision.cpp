// Original Java: https://github.com/wpilibsuite/allwpilib/tree/69cc85db8347f6519c65e360072e75852adc5851/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/machinelearning
// Converted to C++ using converter from Tangible: https://www.tangiblesoftwaresolutions.com/product_details/java_to_cplusplus_converter_details.html

#include "Vision.h"
#include <iostream> // for std::cout
#include <sstream> // for stringstream
#include <frc/DriverStation.h>

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
    // std::cout << "totalBalls= " << totalBalls << std::endl;
    // std::cout << "totalObjects= " << totalObjects << std::endl;
    // std::cout << "boxes size= " << boxes.size() << std::endl;
    int index = 0;
    std::vector<Ball*> balls = std::vector<Ball*>(totalBalls); 
    for (int cornerIndex = 0, objIndex=0; objIndex < totalObjects; cornerIndex += 4, objIndex++) {
        // std::cout << "getBalls obj index= " << objIndex << std::endl;
        // std::cout << "getBalls corner index= " << cornerIndex << std::endl;
        // std::cout << "getBalls class[i]= " << classes[objIndex] << std::endl;
        if (classes[objIndex] == "Power_Cell") {
            std::vector<double> corners = std::vector<double>(4);
            for (int j = 0; j < 4; j++) {
                //std::cout << "setting corner " << j << " to " << boxes[j + cornerIndex] << std::endl;
                corners[j] = boxes[j + cornerIndex];
                //std::cout << "confirming corner " << j << " is " << corners[j] << std::endl;
            }
            //std::cout << "getBalls setting result, index=" << index << std::endl;
            balls[index] = new Ball(corners);
            index++;
        }
    }
    // std::cout << "End of getBalls, returning " << balls.size() << std::endl;
    return balls;
}

// return 3 fake balls
// Note: getBalls above was rewritting to remove bug around referencing class names
std::vector<VisionSubsystem::Ball*> VisionSubsystem::getFakeBalls()
{
    int index = 0;
    int fakeBallCount = 3;
    std::vector<Ball*> balls = std::vector<Ball*>(fakeBallCount); 
    for (int i = 0; i < fakeBallCount*4; i += 4) {
        std::vector<double> corners  = std::vector<double>(4);
        for (int j = 0; j < 4; j++) {
            corners[j] = index*2 + j*2;
        }
        balls[index] = new Ball(corners);
        index++;
    }
    // std::cout << "End of getBalls" << std::endl;
    return balls;
}

void VisionSubsystem::disposeBalls(std::vector<VisionSubsystem::Ball*> &balls)
{
    // std::cout << "disposing " << balls.size() << " balls" << std::endl;
    for (uint i = 0; i < balls.size(); i += 4) {
        if (balls[i] != NULL) {
            delete(balls[i]);
        }
    }
    while (!balls.empty()) {
        balls.pop_back();
        // std::cout << "in dispose, size= " << balls.size() << std::endl;
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

std::string VisionSubsystem::sortBallAngles() {
    
    std::cout << "Beginning of sort ball angles" << std::endl;
    allBallsSorted = "unknown";
    // try {
        std::vector<VisionSubsystem::Ball*> balls;
        // afraid of a memory leak somewhere, so don't do this forever
        for (int i = 0; i < kMaxTriesToSeeThreeBalls; i++) {
            balls = getBalls();
            if (balls.size() == 3) {
                std::cout << "found 3 balls" << std::endl;
                break;  // exit for loop
            }
            disposeBalls(balls);
            std::cout << "size after dispose: " << balls.size() << std::endl;
        }
        std::cout << "sorting balls= " << balls.size() << std::endl;
        if (balls.size() > 0) {
            std::sort(balls.begin(), balls.end());  // uses custom operator < for comparison
            std::cout << "balls have been sorted, size=" << balls.size() << std::endl;
            std::stringstream ss;
            for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
              it != balls.end(); ++it) { 
                std::cout << "appending ball to string" << std::endl;
                // std::cout << "appending ball to string: " << (*it)->getAngle() << std::endl;
                ss << (*it)->getAngle() << " "; 
            }
            allBallsSorted = ss.str();
            std::cout << "sort disposing balls" << std::endl;
            disposeBalls(balls);
        }

    // } catch (std::exception& ex ) {
	// 		std::string err_string = "Error sorting balls";
	// 		err_string += ex.what();
	// 		//DriverStation::ReportError(err_string.c_str());
    //         std::cout <<err_string << std::endl;
	// }
    std::cout << "sort returning string= " << allBallsSorted << std::endl;
    return allBallsSorted;
}

std::string VisionSubsystem::sortFakeBallAngles() {
    std::vector<VisionSubsystem::Ball*> balls = getFakeBalls();
    std::sort(balls.begin(), balls.end());  // uses custom operator < for comparison
    std::stringstream ss;
    for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
      it != balls.end(); ++it) { 
        ss << (*it)->getAngle() << " "; 
    }
    allBallsSorted = ss.str();

    disposeBalls(balls);
    return allBallsSorted;
}

// call this after turn to next ball is complete
void VisionSubsystem::clearSecondClosestBall() {
    distanceSecondClosestBall = 0.0;
    angleSecondClosestBall = 0.0;
    turnToNextBallAngle = 0.0;
}