// Original Java: https://github.com/wpilibsuite/allwpilib/tree/69cc85db8347f6519c65e360072e75852adc5851/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/machinelearning
// Converted to C++ using converter from Tangible: https://www.tangiblesoftwaresolutions.com/product_details/java_to_cplusplus_converter_details.html

#include "Vision.h"
#include <iostream> // for std::cout
#include <sstream> // for stringstream
#include <frc/DriverStation.h>
#include <algorithm> // for vector sorting

const static double kBallTrackSmoothingTime = 2.0;
const static double kBallAngleTolerance = 0.8; // ignore up to this % fluctuation


double ConvertRadsToDegrees (double rads) {
    const static double conversion_factor = 180.0/3.141592653589793238463;
    return rads * conversion_factor;
}

double ConvertDegreesToRads (double degs) {
    const static double conversion_factor = 3.141592653589793238463/180.0;
    return degs * conversion_factor;
}

AutoPath::AutoPath(std::string name, std::string angles, std::string positions) {
    m_searchAngles = angles;
    m_searchPositions = positions;
    m_searchPathName = name;
}

double VisionSubsystem::Gamepiece::getAngle()
{
    // std::cout << "getAngle xOffset: " << xOffset << std::endl;
    // std::cout << "getAngle distance: " << distance << std::endl;
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

    m_autoPaths[kARed] = new AutoPath("A Red", "19.0 -75 10", "-24056 -37076 -70886");
	m_autoPaths[kABlue] = new AutoPath("A Blue", "25.5 -72.0 20.5 0", "-31621 -44390 -58237 -73616");
	m_autoPaths[kBRed] = new AutoPath("B Red", "-59.0 43 -45 0", "-13269 -29660 -41168 -66349");
	m_autoPaths[kBBlue] = new AutoPath("B Blue", "35 -30.0 15.0 0", "-28776 -40636 -57063 -67203");
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
    // if (getBallsInProgress) {
    //     // don't do anything, because getBalls is building a structure that depends on these objects
    // } else {

    std::lock_guard<std::mutex> guard(ballMutex);  // will lock mutex until we leave scope, preventing getBalls from running
    // std::cout << "Beginning of periodic" << std::endl;

    //periodicInProgress = true;
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
    //periodicInProgress = false;
    // std::cout << "End of periodic" << std::endl;

}

int VisionSubsystem::getTotalBallsX()
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

    if (totalObjects != totalBalls) {
        // if we have objects that are not balls, why?  And if we have more balls than objects, why?
        std::cout << "***************** CONFUSION in getBalls" << std::endl;
        return std::vector<Ball*>(0);
    } else { try {

    int index = 0;
    std::vector<Ball*> balls = std::vector<Ball*>(totalBalls); 
    for (int cornerIndex = 0, objIndex=0; objIndex < totalObjects; cornerIndex += 4, objIndex++) {
        // std::cout << "getBalls obj index= " << objIndex << std::endl;
        // std::cout << "getBalls corner index= " << cornerIndex << std::endl;
        // std::cout << "getBalls class[i]= " << classes[objIndex] << std::endl;
        if (classes[objIndex] == "Power_Cell") {
            if (index >= totalBalls) {std::cout << "getBalls: missmatch in counts!" << std::endl;}
            else {
                std::vector<double> corners = std::vector<double>(4);
                for (int j = 0; j < 4; j++) {
                    //std::cout << "setting corner " << j << " to " << boxes[j + cornerIndex] << std::endl;
                    corners[j] = boxes[j + cornerIndex];
                    //std::cout << "confirming corner " << j << " is " << corners[j] << std::endl;
                }
                // std::cout << "getBalls setting result, index=" << index << std::endl;
                balls[index] = new Ball(corners);
                index++;
            }
        }
    }
    // std::cout << "End of getBalls, returning " << balls.size() << std::endl;
    // getBallsInProgress = false;
    return balls;

    } catch (std::exception& ex ) {
        std::cout << "***************** EXCEPTION in getBalls" << std::endl;
        return std::vector<Ball*>(0);
    }
    }
    
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

void VisionSubsystem::OLDupdateClosestBall() {
    // frc::SmartDashboard::PutNumber("Power Cells", ballcount);
    // std::cout << "closest ball: begin" << std::endl;
    int ballcount = getTotalBallsX();
    // std::cout << "closest ball: ball count: " << ballcount << std::endl;
    std::vector<VisionSubsystem::Ball*> balls = getBalls();
    bool noBallsForAWhile = m_timer.Get() > timeBallsLastSeen + kBallTrackSmoothingTime;
    if (ballcount == 0 && noBallsForAWhile) { // no balls for a while, so zero things out
        // std::cout << "closest ball: zeroing out" << std::endl;
        distanceClosestBall = 0.0;
        angleClosestBall = 0.0; 
        // don't zero out second closest info, because now is the time to turn toward it
    } else for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
      it != balls.end(); ++it) {
    // was (erroneously): for (uint i = 0; i < balls.size(); i++) {
        // std::cout << "closest ball: iterating" << std::endl;
        if ((*it) != NULL) {
            if (   (distanceClosestBall == 0) /* seeing a ball for first time in a while */
                || ((*it)->distance < distanceClosestBall) ) { /* or new ball is closer than old ball */
                // std::cout << "closest ball: first or closest ball seen" << std::endl;
                double candidateAngle = (*it)->getAngle();
                //if (abs((candidateAngle - angleClosestBall) / angleClosestBall) > kBallAngleTolerance) {
                    // this jump in angle implies previous closest ball lost and new one acquired
                    // ... what to do with this?
                //} else {
                    distanceClosestBall = (*it)->distance;
                    angleClosestBall = candidateAngle;
                    timeBallsLastSeen = m_timer.Get();
                //}

            // } else if ( (distanceSecondClosestBall == 0 && (*it)->distance > distanceClosestBall)
            //          || ((*it)->distance < distanceSecondClosestBall) ) { 
            //     // this is not the closest ball, but is the second closest ball
            //     // std::cout << "closest ball: not closest" << std::endl;
            //     double candidateAngle = (*it)->getAngle();
            //     if (abs((candidateAngle - angleSecondClosestBall) / angleSecondClosestBall) > kBallAngleTolerance) {
            //         // too great a jump; ignore
            //     } else {
            //         // std::cout << "closest ball: recording 2nd ball" << std::endl;
            //         distanceSecondClosestBall = (*it)->distance;
            //         angleSecondClosestBall = candidateAngle;
            //         turnToNextBallAngle = ComputeTurnAngle(distanceClosestBall, distanceSecondClosestBall,
            //                                                angleSecondClosestBall - angleClosestBall);
            //     }

            }
        }
    }
    // std::cout << "closest ball: disposing balls" << std::endl;
    disposeBalls(balls);
}

bool VisionSubsystem::ballPtrComparator (VisionSubsystem::Ball *a, VisionSubsystem::Ball *b) {
    // std::cout << "compare: " << (a->distance < b->distance) << std::endl;
    return (a->distance < b->distance);
}

std::string VisionSubsystem::sortBallAngles() {
    
    // std::cout << "Beginning of sort ball angles" << std::endl;
    allBallsSorted = "unknown";
    // try {
        std::vector<VisionSubsystem::Ball*> balls;
        // afraid of a memory leak somewhere, so don't do this forever
        for (int i = 0; i < kMaxTriesToSeeThreeBalls; i++) {
            balls = getBalls();
            if (balls.size() == 3) {
                // std::cout << "found 3 balls" << std::endl;
                break;  // exit for loop
            }
            disposeBalls(balls);
            //std::cout << "size after dispose: " << balls.size() << std::endl;
        }
        // std::cout << "sorting balls= " << balls.size() << std::endl;
        if (balls.size() > 0) {
            std::sort(balls.begin(), balls.end(), ballPtrComparator);  // because vector elements are Ball pointers, not Balls, cannot use < operator
            // std::cout << "balls have been sorted, size=" << balls.size() << std::endl;
            std::stringstream ss;
            for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
              it != balls.end(); ++it) { 
                // std::cout << "appending ball to string" << std::endl;
                // std::cout << "appending ball to string: " << (*it)->getAngle() << std::endl;
                ss << ConvertRadsToDegrees((*it)->getAngle()) << " "; 
            }
            allBallsSorted = ss.str();
            // std::cout << "sort disposing balls" << std::endl;
            disposeBalls(balls);
        }

    // } catch (std::exception& ex ) {
	// 		std::string err_string = "Error sorting balls";
	// 		err_string += ex.what();
	// 		//DriverStation::ReportError(err_string.c_str());
    //         std::cout <<err_string << std::endl;
	// }
    // std::cout << "sort returning string= " << allBallsSorted << std::endl;
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
// void VisionSubsystem::clearSecondClosestBall() {
//     distanceSecondClosestBall = 0.0;
//     angleSecondClosestBall = 0.0;
//     turnToNextBallAngle = 0.0;
// }

void VisionSubsystem::updateClosestBall() {

    std::lock_guard<std::mutex> guard(ballMutex);  // will lock mutex until we leave scope, preventing periodic from running

    // std::cout << "Beginning of updateClosest" << std::endl;

    try {
        std::vector<VisionSubsystem::Ball*> balls = getBalls();
        if (balls.size() == 0) {
            angleClosestBall = 0.0;
        } else {
            // std::cout << "sorting balls= " << balls.size() << std::endl;
            std::sort(balls.begin(), balls.end(), ballPtrComparator);  // because vector elements are Ball pointers, not Balls, cannot use < operator
            // std::cout << "balls have been sorted, size=" << balls.size() << std::endl;
            // std::stringstream ss;
            // for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
            //   it != balls.end(); ++it) { 
            //     // std::cout << "appending ball to string" << std::endl;
            //     // std::cout << "appending ball to string: " << (*it)->getAngle() << std::endl;
            //     ss << ConvertRadsToDegrees((*it)->getAngle()) << " "; 
            // }
            // allBallsSorted = ss.str();
            // std::cout << "sorted angles: " << allBallsSorted << std::endl;

            Ball *closestBall = *(balls.begin());
            angleClosestBall = ConvertRadsToDegrees(closestBall->getAngle());
            // std::cout << "sort: disposing balls" << std::endl;
            disposeBalls(balls);
            // std::cout << "End of updateClosest" << std::endl;
        }
    } catch (std::exception& ex ) {
        std::cout << "******************* EXCEPTION in updateClosest" << std::endl;
        angleClosestBall = 0;
    }

}

AutoPath * VisionSubsystem::selectAutoPath(){
    // std::cout << "Beginning of selectAutoPath" << std::endl;
    std::vector<VisionSubsystem::Ball*> balls;
    GalacticSearchPath determination = kBBlue; 
    // afraid of a memory leak somewhere, so don't do this forever
    for (int i = 0; i <= kMaxTriesToSeeThreeBalls; i++) {
        ballMutex.lock();
        balls = getBalls();
        if (balls.size() == 3) {
            // std::cout << "found 3 balls" << std::endl;
            break;  // exit for loop
        }
        if (i < kMaxTriesToSeeThreeBalls) {// don't dispose or unlock mutex on last iteration of loop
            disposeBalls(balls);
            ballMutex.unlock();
            frc::Wait(0.1);
        }
    }

    if (balls.size() > 0) {
        // std::cout << "sorting balls= " << balls.size() << std::endl;
        std::sort(balls.begin(), balls.end(), ballPtrComparator);  // because vector elements are Ball pointers, not Balls, cannot use < operator
        // std::cout << "balls have been sorted, size=" << balls.size() << std::endl;
        std::stringstream ss;
        for(std::vector<VisionSubsystem::Ball*>::iterator it = balls.begin(); 
          it != balls.end(); ++it) { 
            // std::cout << "appending ball to string" << std::endl;
            // std::cout << "appending ball to string: " << (*it)->getAngle() << std::endl;
            ss << ConvertRadsToDegrees((*it)->getAngle()) << " "; 
        }
        allBallsSorted = ss.str();
        std::cout << "sorted angles: " << allBallsSorted << std::endl;

        Ball *closestBall = *(balls.begin());
        Ball *farthestBall = balls.back(); // or should it be *(--(balls.end())) ?
        if (balls.size() == 2) {
            // for B Blue, one ball is too far away to identify, so we usually just get 2 balls
            determination = kBBlue;        
        } else if (abs(ConvertRadsToDegrees(closestBall->getAngle())) < kGalacticSearchAngleTolerance) { // only A Red has close center ball
            determination = kARed;
        } else if (ConvertRadsToDegrees(closestBall->getAngle()) > 0) { // only B Red has closest ball to the left
            determination = kBRed;
        } else if (abs(ConvertRadsToDegrees(farthestBall->getAngle())) < kGalacticSearchAngleTolerance) { // only A Blue has farthest center ball
            determination = kABlue;
        } else { // by process of elimination, must be B Blue
            determination = kBBlue;
        }

        std::cout << "sort disposing balls" << std::endl;
        ballMutex.unlock();
        disposeBalls(balls);

    }

    return m_autoPaths[determination];
}

AutoPath * VisionSubsystem::defaultAutoPath(){
    return m_autoPaths[kARed];
}