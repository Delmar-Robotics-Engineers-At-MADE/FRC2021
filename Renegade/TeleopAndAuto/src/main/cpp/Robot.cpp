
#include <iostream>
#include <string>
#include <Thread>

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/util/color.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#include "Vision.h"

using namespace frc;

    // constants

    const static int k_joystick_pilot = 0;
    const static int k_joystick_copilot = 1;
	
	const static double kToleranceDegrees = 2.0f;
	const static double kMaxRotateRate = 0.4;
	const static double kGamepadDeadZone = 0.15;
	const static double kSlowSpeedFactor = 0.7;
	const static double kFastSpeedFactor = 0.9;
	const static double kArcadeTurningBoost = 1.5;
	const static double kMinTargetAreaPercent = 0.1;
	const static double kDriveAgainstCPSpeed = 0.2;
	const static double kDriveAgainstCPDiff = 0.4;
	const static double kManualShooterBoost = 1.05;
	const static double kManualShooterDeboost = 0.95;

	const static double kConveyerSpeed = 0.95;
	//const static double kFirstConveyerSpeed = 0.95;
	const static double kIntakeSpeed = 0.7;
	const static double kIntakeDelayArrival = 0.2; // in 2020 was 1;
	const static double kIntakeDelayGap = 0.0; // in 2020 was 0.05;

	const static double kIdleShooterPower = 0.25; 
	const static double kIdleShooterVelocity = 6000; 
	const static double kCloseRangeShooterSpeed = 15000;
	const static double kCloseRangeTargetAngle = 1.8;
	const static double kMaxShooterSpeedError = 3500;  // move conveyer automatically when speed is good
	// const static double kInitialShooterSlope = 250; // was 300 in 2020
	// const static double kInitialShooterIntercept = 13000; // was 12696.1 in 2020
	// from Wolfram: 4.03411 x^3 - 171.395 x^2 + 2526.17 x + 499.393
	// new: 0.829278 x^3 - 37.7529 x^2 + 553.419 x + 14534.8
	const static double kInitialShooterC1 = 0.829278; // cubic coefficients
	const static double kInitialShooterC2 = -37.7529; // cubic coefficients
	const static double kInitialShooterC3 = 553.419; // cubic coefficients
	const static double kInitialShooterC4 = 14534.8; // cubic coefficients

	const static double kMinColorConfidence = 0.85;
	const static double kControlPanelSpeed = 0.8;

	const static double kTurretSpeedInitial = 0.4;
	const static double kTurretSpeedMax = 0.6;
	const static long kMaxTurretInitialSeek = -4000; // in encoder counts
	const static long kTurretTolerance = 500; // in encoder counts
	const static long kTurretLimitPort = 15000;
	const static long kTurretLimitStarboard = -15000;
	const static long kTurretUP = 1;  // basicaly zero, but needs to be non-zero
	const static long kTurretRIGHT = 7000;
	const static long kTurretDOWN = 10000;
	const static long kTurretLEFT = -7000;
	const static long kLimelightTolerance = 2; // degrees
	const static long kLimelightCenter = 1.5; // 0 isn't producing a centered shot anymore (2021)

	const static long kPixyTolerance = 5; // X values
	const static double kChaseBallSpeed = 0.3;

	const static double kAutoDriveSpeed = 0.5;
	const static long kAutoDriveDistance = 6000;


	/* stock color set */
	static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
	static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
	static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
	static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
	static constexpr frc::Color kNoColor = frc::Color(0,0,0);
	

	/* our mockup control panel *
	static constexpr frc::Color kBlueTarget = frc::Color(0.175, 0.436, 0.388);
	static constexpr frc::Color kGreenTarget = frc::Color(0.206, 0.545, 0.248);
	static constexpr frc::Color kRedTarget = frc::Color(0.424, 0.386, 0.190);
	static constexpr frc::Color kYellowTarget = frc::Color(0.330, 0.525, 0.145);
	static constexpr frc::Color kNoColor = frc::Color(0,0,0);
	*/
	
	
	const static double kPtunedGyro = 0.05;
	const static double kItunedGyro = 0.0;
	const static double kDtunedGyro = 0.0;
	const static double kPtunedSearch = 0.05;
	const static double kItunedSearch = 0.0;
	const static double kDtunedSearch = 0.0;
	const static double kPtunedPixy = 0.01;
	const static double kItunedPixy = 0.0;
	const static double kDtunedPixy = 0.0;
	const static double kPturret = 0.01;
	const static double kIturret = 0.003;
	const static double kDturret = 0.0;
	const static double kPtunedShooter = 0.009;
	const static double kItunedShooter = 0.00005;
	const static double kDtunedShooter = 0.0;
	const static double kFtunedShooter = 0.045;

	// for field relative and vision
	const static double kFieldRelDriveSmoothTime = 0.4;
	const static double kHeadingDiscontinuityZone = 6; // degrees either side of 0
	const static double kTargetDiscontinuityZone = 15; // degrees either side of 0


/********************************************** main class  ********************************************/

class Robot: public TimedRobot {

	/*
	TalonFX 0	Left Shooter Motor
	TalonSRX 1	Left Froward Drive Motor
	TalonSRX 2	Left Aft Drive Motor
	TalonSRX 3	Intake Motor
	TalonSRX 5	Shooter Turret Rotator
	TalonSRX 7	Control Panel Rotator
	TalonSRX 10	Vertical Conveyor
	TalonSRX 11	Horizontal Conveyor
	TalonSRX 13	Right Forward Drive Motor
	TalonSRX 14	Right Aft Drive Motor
	TalonFX 15	Right Shooter Motor
	*/

	// shooter 
	TalonFX * m_shooter_star = new TalonFX(15); // 15 is starboard, 0 is port
	TalonFX * m_shooter_port = new TalonFX(0); // 15 is starboard, 0 is port
	double m_shooter_C1 = 0.0;
	double m_shooter_C2 = 0.0;
	double m_shooter_C3 = 0.0;
	double m_shooter_C4 = 0.0;
	//Joystick * _joy = new Joystick(0);
	// std::string _sb;
	// int _loops = 0;
	double m_IdleShooterPower = kIdleShooterPower;
	double m_IdleShooterVelocity = kIdleShooterVelocity;

	// drive motors
    WPI_TalonSRX m_leftfront{1};
    WPI_TalonSRX m_leftrear{2};
    WPI_TalonSRX m_rightfront{13}; 
    WPI_TalonSRX m_rightrear{14};
    frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
    frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
    frc::DifferentialDrive m_robotDrive{m_left, m_right};

    // conveyers and intake
    WPI_TalonSRX m_vert_conveyer{10}; 
    WPI_TalonSRX m_hor_conveyer{11};
    WPI_TalonSRX m_intake{3};

	// turret and control panel gizmo
	TalonSRX * m_turret = new TalonSRX(5); 
	WPI_TalonSRX m_control_spinner{7}; 

	// solenoids
	frc::DoubleSolenoid m_ponytail_solenoid{0, 1};

	// joysticks
	Joystick *m_stick, *m_stick_copilot;

	// sensors
	AHRS *ahrs;
	frc::DigitalInput eye_intake{0}; // photo eye at collector
	frc::DigitalInput eye_turret{1}; // photo eye at turret
	frc::DigitalInput hall_effect{2}; // hall effect sensor on turret
	frc::DigitalInput hall_effect_port{8}; // hall effect sensor on turret
	frc::DigitalInput hall_effect_star{9}; // hall effect sensor on turret
	static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
	rev::ColorSensorV3 m_colorSensor{i2cPort};
	rev::ColorMatch m_colorMatcher;

	// state machine for counting rotations of control panel
	enum RotateStates {
		kUnknownState = 0,
		kOffStartingColor,
		kOnStartingColor,
		kCompletedRotations
	};
	RotateStates m_wheel_state = kUnknownState;
	frc::Color m_starting_color = kNoColor;
	int m_half_rotation_count = 0;
	// state machine for spinning to color
	enum RotateToColorStates {
		kOffTargetColor = 0,
		kOnTargetColor,
		kToColorComplete
	};
	RotateToColorStates m_wheel_state_to_color = kOffTargetColor;
	bool m_need_to_reset_spinner = false;

	// for moving turret
	bool m_need_to_reset_manual_turret_move = false;
	bool m_need_to_reset_tracking_turret_move = false;

	// state machine for intake
	enum IntakeStates {
		kBallJustArrived = 0, 
		kBallInBreach,
		kBallJustLeft,
		kBreachEmpty
	};
	IntakeStates m_intake_state = kBreachEmpty;

	// state machine for galactic search
	enum SearchStates {
		kSearchStart = 0, 
		// kSearchBallCollected1,
		kSearchTurningToBall1,
		kSearchDrivingToBall1,
		kSearchTurningToBall2,
		kSearchDrivingToBall2,
		// kSearchBallCollected2,
		kSearchTurningToBall3,
		kSearchDrivingToBall3,
		// kSearchBallCollected3,
		kSearchTurningToEnd,
		kSearchDrivingToEnd,
		kSearchComplete
	};
	SearchStates m_search_state = kSearchStart;

	// pid and timer
    frc2::PIDController *m_pidController_gyro; // for orienting robot with gyro
    frc2::PIDController *m_pidController_search; // for driving to power cells
	frc2::PIDController *m_pidController_limelight_robot;
	frc2::PIDController *m_pidController_limelight_turret;
	frc2::PIDController *m_pidController_pixycam;
    frc::Timer m_timer, m_timer2;

	// limelight
	std::shared_ptr<NetworkTable> m_limetable;  // for LimeLight
	std::shared_ptr<NetworkTable> m_pixytable;  // for Pixy Cam

	// misc members
    double m_rotate_to_angle_rate;           // Current rotation rate
    // double speed_factor = 0.5;
	frc::SendableChooser<std::string> m_chooser;
	frc::SendableChooser<std::string> m_chooser_options_dir;
	frc::SendableChooser<std::string> m_chooser_options_speed;
	frc::SendableChooser<std::string> m_chooser_options_wait;
	const std::string kAutoNameTestWheels = "Move Wheels";
	const std::string kAutoNameMoveOnly = "Move Only";
	const std::string kAutoNameJustInit = "Just Initialize";
	const std::string kAutoNameShootAndMove = "Shoot/Move";
	const std::string kAutoNameSearchMan = "Galactic Search Manual";
	const std::string kAutoNameSearchAuto = "Galactic Search Auto";
	// const std::string kAutoNameSearchBRed = "Search B Red";
	// const std::string kAutoNameSearchABlue = "Search A Blue";
	// const std::string kAutoNameSearchBBlue = "Search B Blue";
	const std::string kAutoOptionForward = "Forward";
	const std::string kAutoOptionBackward = "Backard";
	const std::string kAutoOptionNoMove = "No Move";
	const std::string kAutoOptionFast = "Fast";
	const std::string kAutoOptionSlow = "Slow";
	const std::string kAutoOptionWait = "Wait";
	const std::string kAutoOptionNoWait = "No wait";
	std::string m_autoSelected;
	std::string m_autoSelected_options_dir;
	std::string m_autoSelected_options_speed;
	std::string m_autoSelected_options_wait;
	bool m_do_once_inited = false;
	bool m_need_to_reset_coast = false;

	// Galactic Search
	std::vector <double> m_search_angles;
	std::vector <double> m_search_positions;

	// vision and field relative
	double m_field_rel_timer;
	VisionSubsystem *m_visionSubsystem;

	double TrimSpeed (double s, double max) {
		double result = s > max ? max : s;
		result = result < -max ? -max : result;
		return result;
	}

	double ScaleSpeed (double s, double scale) {
		return s * scale;
	}

public:

/********************************************** initialization  ********************************************/

	void RobotInit() {

		/*************** drive motor setup ***********************/

		m_leftfront.ConfigFactoryDefault();
		m_leftrear.ConfigFactoryDefault();
		m_rightfront.ConfigFactoryDefault();
		m_rightrear.ConfigFactoryDefault();

        /* feedback sensor */
		m_leftfront.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		m_rightfront.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		m_leftfront.SetSensorPhase(false);
		m_rightfront.SetSensorPhase(false);

		/* set the peak and nominal outputs */
		m_leftfront.ConfigNominalOutputForward(0, kTimeoutMs);
		m_leftfront.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_leftfront.ConfigPeakOutputForward(1, kTimeoutMs);
		m_leftfront.ConfigPeakOutputReverse(-1, kTimeoutMs);
		m_rightfront.ConfigNominalOutputForward(0, kTimeoutMs);
		m_rightfront.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_rightfront.ConfigPeakOutputForward(1, kTimeoutMs);
		m_rightfront.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_leftfront.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_leftfront.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		m_leftfront.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_leftfront.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		m_rightfront.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

		/*************** shooter setup ***********************/

		m_shooter_star->ConfigFactoryDefault();
		m_shooter_port->ConfigFactoryDefault();

		// one follower and one reversed
		m_shooter_port->Follow(*m_shooter_star);
    	m_shooter_star->SetInverted(false); // flipped from 2020, to clear up minus signs
    	m_shooter_port->SetInverted(true);
		
		// braking mode
		m_shooter_star->SetNeutralMode(NeutralMode::Coast);
		m_shooter_port->SetNeutralMode(NeutralMode::Coast);

		// follower's dead zone should be zero, so it doesn't apply dead zone logic to signal from other motor
		// per Matt's 2021 question, no parameters need to be set on follower anymore
		// https://www.chiefdelphi.com/t/two-falcon-500-follower-not-working/392133/9
		// m_shooter_port->ConfigNeutralDeadband(0.0);

        /* feedback sensor */
		m_shooter_star->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);

		// phase for TalonFX integrated sensor is automatically correct

		/* set the peak and nominal outputs */
		m_shooter_star->ConfigNominalOutputForward(0, kTimeoutMs);
		m_shooter_star->ConfigNominalOutputReverse(0, kTimeoutMs);
		m_shooter_star->ConfigPeakOutputForward(1, kTimeoutMs);
		m_shooter_star->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		// m_shooter_star->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs); // was .1097
		// m_shooter_star->Config_kP(kPIDLoopIdx, 0.2, kTimeoutMs); // was 0.22
		// m_shooter_star->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs); // was 0
		// m_shooter_star->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs); // was 0

		// set follow again, in case one of the other settings undid following
		m_shooter_port->Follow(*m_shooter_star);

		/************** turret setup **********************/

		m_turret->ConfigFactoryDefault();
		// int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		// m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		m_turret->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, kPIDLoopIdx,kTimeoutMs);
		m_turret->SetSensorPhase(false);

		/* set the peak and nominal outputs, 12V means full */
		m_turret->ConfigNominalOutputForward(0, kTimeoutMs);
		m_turret->ConfigNominalOutputReverse(0, kTimeoutMs);
		m_turret->ConfigPeakOutputForward(kTurretSpeedMax, kTimeoutMs);
		m_turret->ConfigPeakOutputReverse(-kTurretSpeedMax, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_turret->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kP(kPIDLoopIdx, 0.2, kTimeoutMs);
		m_turret->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kD(kPIDLoopIdx, 0.5, kTimeoutMs);

		/************** other motor setup *****************/

		// Conveyers
		m_vert_conveyer.ConfigFactoryDefault();
		m_hor_conveyer.ConfigFactoryDefault();
		m_hor_conveyer.Follow(m_vert_conveyer);
		m_vert_conveyer.ConfigNominalOutputForward(0, kTimeoutMs);
		m_vert_conveyer.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_vert_conveyer.ConfigPeakOutputForward(0.5, kTimeoutMs);
		m_vert_conveyer.ConfigPeakOutputReverse(-0.5, kTimeoutMs);

		/***************** limelight *************************/

		m_limetable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
		//m_limetable->PutNumber("camMode",0.0); // camera in normal CV mode
		//m_limetable->PutNumber("ledMode",1.0); // LED off
		//m_limetable->PutNumber("stream",0.0);  // secondary camera side-by-side

		/***************** pixycam *************************/

		m_pixytable = nt::NetworkTableInstance::GetDefault().GetTable("PixyBlocks");

		/***************** joysticks *************************/

		m_stick = new Joystick(k_joystick_pilot);
		m_stick_copilot = new Joystick(k_joystick_copilot);

		/********************* Gyro ****************************/

		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * VMX-pi:
			 * - Communication via USB.
			 * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception &ex){
			std::string what_string = ex.what();
			std::string err_msg("Error instantiating navX MXP:  " + what_string);
			const char *p_err_msg = err_msg.c_str();
			DriverStation::ReportError(p_err_msg);
		}

		// for vision
		m_visionSubsystem = new VisionSubsystem();

		/* this is used to tune the PID numbers with shuffleboard */
		frc::SmartDashboard::PutNumber("T kP", kPtunedShooter);
		frc::SmartDashboard::PutNumber("T kI", kItunedShooter);
		frc::SmartDashboard::PutNumber("T kD", kDtunedShooter); 
		frc::SmartDashboard::PutNumber("T kF", kFtunedShooter); 
		
		frc::SmartDashboard::PutNumber("T C1", kInitialShooterC1);
		frc::SmartDashboard::PutNumber("T C2", kInitialShooterC2);
		frc::SmartDashboard::PutNumber("T C3", kInitialShooterC3);
		frc::SmartDashboard::PutNumber("T C4", kInitialShooterC4);

		// for Galactic Search
		frc::SmartDashboard::PutString("S Angles", m_visionSubsystem->defaultAutoPath()->m_searchAngles);
		frc::SmartDashboard::PutString("S Positions", m_visionSubsystem->defaultAutoPath()->m_searchPositions);

		// control panel colors
		m_colorMatcher.AddColorMatch(kBlueTarget);
		m_colorMatcher.AddColorMatch(kGreenTarget);
		m_colorMatcher.AddColorMatch(kRedTarget);
		m_colorMatcher.AddColorMatch(kYellowTarget);

		/********************** stuff ************************/

		m_rotate_to_angle_rate = 0.0f;
		m_robotDrive.SetExpiration(0.1);

		m_chooser.SetDefaultOption(kAutoNameShootAndMove, kAutoNameShootAndMove);
		m_chooser.AddOption(kAutoNameMoveOnly, kAutoNameMoveOnly);
		m_chooser.AddOption(kAutoNameJustInit, kAutoNameJustInit);
		m_chooser.AddOption(kAutoNameTestWheels, kAutoNameTestWheels);
		m_chooser.AddOption(kAutoNameSearchMan, kAutoNameSearchMan);
		m_chooser.AddOption(kAutoNameSearchAuto, kAutoNameSearchAuto);
		// m_chooser.AddOption(kAutoNameSearchBRed, kAutoNameSearchBRed);
		// m_chooser.AddOption(kAutoNameSearchABlue, kAutoNameSearchABlue);
		// m_chooser.AddOption(kAutoNameSearchBBlue, kAutoNameSearchBBlue);
		m_chooser_options_dir.SetDefaultOption(kAutoOptionForward,kAutoOptionForward);
		m_chooser_options_dir.AddOption(kAutoOptionBackward, kAutoOptionBackward);
		m_chooser_options_dir.AddOption(kAutoOptionNoMove, kAutoOptionNoMove);
		m_chooser_options_speed.SetDefaultOption(kAutoOptionFast,kAutoOptionFast);
		m_chooser_options_speed.AddOption(kAutoOptionSlow, kAutoOptionSlow);
		m_chooser_options_wait.AddOption(kAutoOptionWait, kAutoOptionWait);
		m_chooser_options_wait.AddOption(kAutoOptionNoWait, kAutoOptionNoWait);
		frc::SmartDashboard::PutData("A Modes", &m_chooser);
		frc::SmartDashboard::PutData("A Move?", &m_chooser_options_dir);
		frc::SmartDashboard::PutData("A Fast?", &m_chooser_options_speed);
		frc::SmartDashboard::PutData("A Wait?", &m_chooser_options_wait);
	}


	void TeleopInit() {
		DoOnceInit();
		RepeatableInit();
		m_wheel_state = kUnknownState;
		m_starting_color = kNoColor;
		m_half_rotation_count = 0;
		m_intake_state = kBreachEmpty;
		
		// braking mode
		m_leftfront.SetNeutralMode(NeutralMode::Coast);  // was brake in 2020
		m_leftrear.SetNeutralMode(NeutralMode::Coast);
		m_rightfront.SetNeutralMode(NeutralMode::Coast);
		m_rightrear.SetNeutralMode(NeutralMode::Coast);
	}

	void DoOnceInit() {
		if (!m_do_once_inited) {

			ahrs->ZeroYaw();				
			
			// PIDs
			m_pidController_gyro = new frc2::PIDController (kPtunedGyro, kItunedGyro, kDtunedGyro);
			m_pidController_gyro->SetTolerance(8, 8);  // within 8 degrees of direction is considered on set point
			m_pidController_search = new frc2::PIDController (kPtunedSearch, kItunedSearch, kDtunedSearch);
			m_pidController_search->SetTolerance(1000);  // within this  many units is considered on set point for posiiton
			m_pidController_limelight_robot = new frc2::PIDController (kPtunedGyro, kItunedGyro, kDtunedGyro);
			m_pidController_limelight_robot->SetTolerance(kLimelightTolerance, kLimelightTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_limelight_turret = new frc2::PIDController (kPturret, kIturret, kDturret);
			m_pidController_limelight_turret->SetTolerance(kLimelightTolerance, kLimelightTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_limelight_turret->SetSetpoint(kLimelightCenter);
			m_pidController_pixycam = new frc2::PIDController (kPtunedPixy, kItunedPixy, kDtunedPixy);
			m_pidController_pixycam->SetTolerance(kPixyTolerance, kPixyTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_pixycam->SetSetpoint(0);  // always use same setpoint

			// position turret
			MoveTurretToStartingPosition();

		} else { // do-once already done, don't do again
			frc::SmartDashboard::PutString("I turr state", "init skipped");
		}

		m_do_once_inited = true;
	}

	std::vector <double> UnpackNumbers (std::string packedNumbers) {
		std::stringstream numberStream(packedNumbers);
		std::vector <double> result;
		std::string s;
		while (getline(numberStream, s, ' ')) {
			result.push_back(std::stod(s));
		}
		return result;
	}

	void RepeatableInit() {

		double P_shooter = kPtunedShooter;
		double I_shooter = kItunedShooter;
		double D_shooter = kDtunedShooter;
		double F_shooter = kFtunedShooter;
	
		/* used to tune PID numbers */
		double P = frc::SmartDashboard::GetNumber("T kP", P_shooter);
		double I = frc::SmartDashboard::GetNumber("T kI", I_shooter);
		double D = frc::SmartDashboard::GetNumber("T kD", D_shooter);
		double F = frc::SmartDashboard::GetNumber("T kF", F_shooter);

		// bring up shooter
		// m_shooter_star->Set(ControlMode::Velocity, m_IdleShooterVelocity);
		m_shooter_star->Set(ControlMode::PercentOutput, m_IdleShooterPower);
		m_shooter_C1 = frc::SmartDashboard::GetNumber("T C1", kInitialShooterC1);
		m_shooter_C2 = frc::SmartDashboard::GetNumber("T C2", kInitialShooterC2);
		m_shooter_C3 = frc::SmartDashboard::GetNumber("T C3", kInitialShooterC3);
		m_shooter_C4 = frc::SmartDashboard::GetNumber("T C4", kInitialShooterC4);

		m_shooter_star->Config_kF(kPIDLoopIdx, F, kTimeoutMs); // was .1097  /* set closed loop gains in slot0 */
		m_shooter_star->Config_kP(kPIDLoopIdx, P, kTimeoutMs); // was 0.22
		m_shooter_star->Config_kI(kPIDLoopIdx, I, kTimeoutMs); // was 0
		m_shooter_star->Config_kD(kPIDLoopIdx, D, kTimeoutMs); // was 0
		// frc::SmartDashboard::PutNumber("t pid check F", F);

		// position ponytail up
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward);

		// for smoothing field rel driving
		m_timer.Reset();
		m_timer.Start();

		// added in 2021 for galactic search
		m_autoSelected = m_chooser.GetSelected();
		m_autoSelected_options_dir = m_chooser_options_dir.GetSelected();
		m_autoSelected_options_speed = m_chooser_options_speed.GetSelected();
		m_autoSelected_options_wait = m_chooser_options_wait.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//     kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		// important stuff for auto
		if (m_autoSelected == kAutoNameShootAndMove) {
			m_limetable->PutNumber("ledMode",3.0);
		}
		ahrs->ZeroYaw();
		m_leftfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		m_rightfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		m_timer.Reset();
		m_timer.Start();
		m_timer2.Reset();
		m_timer2.Start();

		// braking mode
		m_leftfront.SetNeutralMode(NeutralMode::Brake);  // need brake when moving
		m_leftrear.SetNeutralMode(NeutralMode::Brake);
		m_rightfront.SetNeutralMode(NeutralMode::Brake);
		m_rightrear.SetNeutralMode(NeutralMode::Brake);

	}

	void GalacticSearchInit() {
		// for Galactic Search
		m_search_state = kSearchStart;
		AutoPath * selectedPath = m_visionSubsystem->selectAutoPath();
		frc::SmartDashboard::PutString("S Path name", selectedPath->m_searchPathName);
		// frc::SmartDashboard::PutString("S Ball angles", "looking for 3...");
		// frc::SmartDashboard::PutNumber("S Ball count", m_visionSubsystem->totalBalls);
		// frc::SmartDashboard::PutString("S Ball angles", m_visionSubsystem->sortBallAngles());
		
		// none of these catches will catch a memory access error it seems
		// try { frc::SmartDashboard::PutString("S Ball angles", m_visionSubsystem->sortBallAngles());
		// } catch (std::exception& ex ) {
		// 	std::string err_string = "Error observing balls: ";
		// 	err_string += ex.what();
		// 	DriverStation::ReportError(err_string.c_str());
		// }catch (std::string& ex ) {
		// 	std::string err_string = "(str) Error observing balls: ";
		// 	err_string += ex;
		// 	DriverStation::ReportError(err_string.c_str());
		// }catch (...) {
		// 	std::string err_string = "(bad) Error observing balls: ";
		// 	DriverStation::ReportError(err_string.c_str());
		// }

		// at this point we should determine path heuristically, not read dashboard
		std::string searchAngles = frc::SmartDashboard::GetString("S Angles", "");
		std::string searchPositions = frc::SmartDashboard::GetString("S Positions", "");
		if (m_autoSelected == kAutoNameSearchAuto) {
			searchAngles = selectedPath->m_searchAngles;
			searchPositions = selectedPath->m_searchPositions;
			frc::SmartDashboard::PutString("S Angles", searchAngles);
			frc::SmartDashboard::PutString("S Positions", searchPositions);
		}
		m_search_angles = UnpackNumbers (searchAngles);
		m_search_positions = UnpackNumbers (searchPositions);
		std::cout << "search angles: ";
		for(std::vector<double>::iterator it = m_search_angles.begin(); 
		  it != m_search_angles.end(); ++it) { std::cout << *it << " "; }
		std::cout << std::endl;
		std::cout << "search positions: ";
		for(std::vector<double>::iterator it = m_search_positions.begin(); 
		  it != m_search_positions.end(); ++it) { std::cout << *it << " "; }
		std::cout << std::endl;
	}

	void AutonomousInit() {
		DoOnceInit();
		RepeatableInit();

		/* Todo: investigate these errors:
		in driver station log window: ERROR -6 A timeout has been exceeded: DifferentialDrive... Output not updated often enough.  Check [MotorSafety.cpp:100]
		in RioLog window: 
			Auto selected: Shoot/Move
			Beginning of selectAutoPath
			A timeout has been exceeded: DifferentialDrive... Output not updated often enough.
			Error at Check [MotorSafety.cpp:100]: A timeout has been exceeded: DifferentialDrive... Output not updated often enough.
			at frc::MotorSafety::Check()
			at frc::MotorSafety::CheckMotors()
			at frc::DriverStation::Run()
			at
			search angles: 19 -75 10
			search positions: -24056 -37076 -70886
			navX-Sensor Software Yaw Reset completed.
		*/
	}

/********************************************** telo-op  ********************************************/

	void RobotPeriodic() {
		m_visionSubsystem->periodic();
	}

	// for field relative
	int quadrant (double a) { // quadrant of angle, I, II, III, or IV
		assert (a >= 0 && a <= 360);
		int result = 0;
		if (a <= 90) result = 1;
		else if (a <= 180) result = 2;
		else if (a <= 270) result = 3;
		else result = 4;
		return result;
	}
	bool isInDiscontinuityZone (double curr, double target) {
		// heading is close to 0 and target is close to heading
		return ( (curr < kHeadingDiscontinuityZone || curr > 360 - kHeadingDiscontinuityZone)
			&& abs(curr - target) < kHeadingDiscontinuityZone );
			// && (target < kTargetDiscontinuityZone || target > 360 - kTargetDiscontinuityZone) ) 
	}

	void MoveTurretToManualPosition (long target_position) {
		m_turret->Set(ControlMode::Position, target_position);
		if (abs(target_position - m_turret->GetSelectedSensorPosition(0)) < kTurretTolerance) {
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
		}
	}
	
	void MoveTurretToStartingPosition() {

		bool turret_on_hall = false;
		bool turret_on_port_or_star_hall = false;
		frc::SmartDashboard::PutString("I turr state", "initial move");

		// code from WPI to set starting position
		// int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		// m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		m_turret->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		
		// move turret to starting position using Hall sensor

		//int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		//m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		

		m_turret->Set(ControlMode::Position, kMaxTurretInitialSeek);  // positive moves turret clockwise

		m_timer.Reset(); m_timer.Start(); while (m_timer.Get() < 5) {
			turret_on_hall = !hall_effect.Get();
			turret_on_port_or_star_hall = !hall_effect_port.Get() || !hall_effect_star.Get();
			if (turret_on_port_or_star_hall) {
				m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
				frc::SmartDashboard::PutString("I turr state", "found wrong hall");
				// throw "found port or starboard hall sensor"; // should terminate program
				// terminating with an exception might not shut down robot, so instead, just wait here forever
				while (true) {Wait(0.1);}
				break; // exit loop
			} else if (turret_on_hall) { 
				m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
				frc::SmartDashboard::PutString("I turr state", "found hall 1");
				break; // exit loop
			}
			Wait(0.1);
		} // while
		m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
		if (m_timer.Get() >= 5) {
			frc::SmartDashboard::PutString("I turr state", "time");
		}

		turret_on_hall = !hall_effect.Get();
		if (!turret_on_hall) {
			// we didn't land on Hall sensor, go back the other way looking for it
			m_turret->Set(ControlMode::Position, -2 * kMaxTurretInitialSeek);
			m_timer.Reset(); m_timer.Start(); while (m_timer.Get() < 5) {
				turret_on_hall = !hall_effect.Get();
				turret_on_port_or_star_hall = !hall_effect_port.Get() || !hall_effect_star.Get();
				if (turret_on_port_or_star_hall) {
					m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
					frc::SmartDashboard::PutString("I turr state", "found wrong 2");
					// throw "found port or starboard hall sensor"; // should terminate program
					// terminating with an exception might not shut down robot, so instead, just wait here forever
					while (true) {Wait(0.1);}
					break; // exit loop
				} else if (turret_on_hall) { 
					m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
					frc::SmartDashboard::PutString("I turr state", "found hall 2");
					break; // exit loop
				}
				Wait(0.1);
			} // while		
			if (m_timer.Get() >= 5) {
				frc::SmartDashboard::PutString("I turr state", "time 2 !!");
				// this condition is bad, we should have found sensor, so stop here
				while (true) {Wait(0.1);}

			}
		}
		m_turret->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);  // set zero position now
	}

	void TrackTargetWithRobot(double targetOffsetAngle) {
		m_pidController_limelight_robot->SetSetpoint(0);
		double diff_speed = m_pidController_limelight_robot->Calculate(targetOffsetAngle);

		// move robot
		m_robotDrive.TankDrive(-diff_speed, diff_speed, false);

		// check to see if we're within tolerance, and if so, reset
		if (m_pidController_limelight_robot->AtSetpoint()) {
			m_pidController_limelight_robot->Reset(); // clears out integral state, etc
		}
	}

	void ChasePowerCellsByPixy() {
		bool ball_seen = m_pixytable->GetNumber("STATUS", -1);
		if (ball_seen == 1) {
			double pixy_X = 167 - m_pixytable->GetNumber("X",0.0);
			frc::SmartDashboard::PutString("BC Balls", "yes");
			frc::SmartDashboard::PutNumber("BC Ball X", pixy_X);

			double diff_speed = m_pidController_pixycam->Calculate(pixy_X);

			// move robot
			frc::SmartDashboard::PutNumber("BC speed", diff_speed);
			m_robotDrive.TankDrive(-diff_speed+kChaseBallSpeed, diff_speed+kChaseBallSpeed, false);

			// don't reset pid controller
		} else if (ball_seen == 0) { // no balls seen
			frc::SmartDashboard::PutString("BC Balls", "no");
		} else { // error of some kind
			frc::SmartDashboard::PutString("BC Balls", "error");
		}
	}

	// returns angle to unseen second ball, in degrees
	void ChasePowerCellsByCoral() {

		frc::SmartDashboard::PutString("V ball count", "N/A"); // was m_visionSubsystem->getTotalBalls()
		m_visionSubsystem->updateClosestBall();
		double ballAngle = m_visionSubsystem->angleClosestBall; // std::cout << "ball angle " << ballAngle << std::endl;
		// double ball2Angle = ConvertRadsToDegrees(m_visionSubsystem->angleSecondClosestBall); // std::cout << "ball angle 2 " << ball2Angle << std::endl;
		frc::SmartDashboard::PutString("V ball dist", "N/A");
		frc::SmartDashboard::PutNumber("V ball angle", ballAngle);
		frc::SmartDashboard::PutString("V ball 2 angle", "N/A");

		bool ball_seen = m_visionSubsystem->angleClosestBall != 0.0; // std::cout << "ball seen? " << ball_seen << std::endl;
		// bool second_ball_seen = m_visionSubsystem->distanceSecondClosestBall > 0.0; // std::cout << "2nd ball seen? " << ball_seen << std::endl;
		if (ball_seen) {
			// note: pid set point is always 0
			double diff_speed = m_pidController_pixycam->Calculate(ballAngle);

			// move robot
			frc::SmartDashboard::PutNumber("V Ball chase", diff_speed);
			m_robotDrive.TankDrive(diff_speed+kChaseBallSpeed, -diff_speed+kChaseBallSpeed, false);

		// } else if (second_ball_seen) {
		// 	// std::cout << "returning 2nd ball" << std::endl;
		// 	result = ball2Angle;
		// 	// caller will initiate rotation to 2nd ball

		} else { // no balls seen, but may have just closed on a ball, so cruise forward
			m_robotDrive.TankDrive(kChaseBallSpeed, kChaseBallSpeed);
		}
	}

	bool TrackTargetWithTurret(double targetOffsetAngle) {
		bool result = false;  // return true when on set point
		targetOffsetAngle = -targetOffsetAngle; 
		double current_pos = m_turret->GetSelectedSensorPosition(0);
		if (current_pos > kTurretLimitStarboard && current_pos < kTurretLimitPort) {
			frc::SmartDashboard::PutString("I turr state", "tracking");
			m_need_to_reset_tracking_turret_move = true;
			// m_pidController_limelight_turret->SetSetpoint(0);  // try moving this to RobotInit() and doing it only once
			double new_speed = m_pidController_limelight_turret->Calculate(targetOffsetAngle);
			// frc::SmartDashboard::PutNumber("turr speed", new_speed);
			// frc::SmartDashboard::PutNumber("off ang", targetOffsetAngle);

			// move turret
			m_turret->Set(ControlMode::PercentOutput, new_speed);

			result = m_pidController_limelight_turret->AtSetpoint();

			// hold position; don't reset when on target
		} else {
			frc::SmartDashboard::PutString("I turr state", "unsafe");
			m_turret->Set(ControlMode::PercentOutput, 0);
		}
		return result;
	}

	std::string ColorToString (frc::Color color) {
		std::string result;
		if (color == kBlueTarget) {
			result = "Blue";
		} else if (color == kRedTarget) {
			result = "Red";
		} else if (color == kGreenTarget) {
			result = "Green";
		} else if (color == kYellowTarget) {
			result = "Yellow";
		} else {
			result = "Unknown";
		}	
		return result;
	}

	void ResetSpinner() {
		m_robotDrive.TankDrive(0, 0); // stop pressing into CP
		m_wheel_state = kUnknownState;
		m_half_rotation_count = 0;
		m_wheel_state_to_color = kOffTargetColor;
		m_control_spinner.Set(0.0);
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner
	}

	void SpinThreeTimes() {
		std::string colorString;
		double confidence = 0.0;
		frc::SmartDashboard::PutNumber("CP Complete", false); // CP = Control Panel

		m_robotDrive.TankDrive(-(kDriveAgainstCPSpeed-kDriveAgainstCPDiff), -(kDriveAgainstCPSpeed+kDriveAgainstCPDiff) ); // press against control panel
		MoveTurretToManualPosition(kTurretUP); // hold turret in position
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kReverse); // lower spinner here
		
		frc::Color detectedColor = m_colorSensor.GetColor();
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		switch (m_wheel_state) {
		case kUnknownState: 
			m_control_spinner.Set(kControlPanelSpeed); // spinning
			if (confidence > kMinColorConfidence) {
				m_wheel_state = kOnStartingColor;
				m_starting_color = matchedColor;
			}
			break;
		case kOnStartingColor:
			if (confidence > kMinColorConfidence && !(matchedColor == m_starting_color)) {
				m_wheel_state = kOffStartingColor;
			}
			if (m_half_rotation_count >= 7) { // 3.5 times around
				m_wheel_state = kCompletedRotations;
			}
			break;
		case kOffStartingColor:
			if (confidence > kMinColorConfidence && matchedColor == m_starting_color) {
				m_wheel_state = kOnStartingColor;
				m_half_rotation_count++;
			}
			break;
		case kCompletedRotations:
			m_control_spinner.Set(0.0); // stop
			//m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner
			frc::SmartDashboard::PutNumber("CP Complete", true);
			break;
		}
		frc::SmartDashboard::PutString("CP color", ColorToString(matchedColor));
		frc::SmartDashboard::PutNumber("CP conf", confidence);
		
	}

	void SpinToColor(frc::Color spin_to_color) {
		std::string colorString;
		double confidence = 0.0;
		frc::SmartDashboard::PutNumber("CP Complete", false);

		m_robotDrive.TankDrive(-(kDriveAgainstCPSpeed-kDriveAgainstCPDiff), -(kDriveAgainstCPSpeed+kDriveAgainstCPDiff) ); // press against control panel
		MoveTurretToManualPosition(kTurretUP); // hold turret in position
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kReverse); // lower spinner here

		frc::Color detectedColor = m_colorSensor.GetColor();
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		// spin to color
		switch (m_wheel_state_to_color) {
			case kOnTargetColor:
				// stop; we're there
				m_wheel_state_to_color = kToColorComplete;
				break;
			case kOffTargetColor:
				if (confidence > kMinColorConfidence && matchedColor == spin_to_color) {
					m_wheel_state_to_color = kOnTargetColor;
				} else {
					m_control_spinner.Set(kControlPanelSpeed);
				}
				break;
			case kToColorComplete:
				m_control_spinner.Set(0.0);
				//m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner here
				frc::SmartDashboard::PutNumber("CP Complete", true);
				break;
		}
		frc::SmartDashboard::PutString("CP color", ColorToString(matchedColor));
		frc::SmartDashboard::PutNumber("CP conf", confidence);
	}

	void AutoIntakeBalls() {
		if (eye_turret.Get()) { // there's a ball under the turret; no more collecting until that's gone
			m_intake.Set(0.0);
			m_vert_conveyer.Set(0.0);
		} else { // turret on-deck empty; ok to intake
			// was... m_vert_conveyer.Set(-kConveyerSpeed);
			// frc::SmartDashboard::PutNumber("intake state", m_intake_state);
			switch (m_intake_state) {
				case kBallJustArrived:
					// frc::SmartDashboard::PutString("ab state", "just arrived");
					/* both on */ m_intake.Set(-kIntakeSpeed); m_vert_conveyer.Set(-kConveyerSpeed);
					m_timer.Reset();
					m_timer.Start();
					m_intake_state = kBallInBreach; 
					break;
				case kBallInBreach: // run long enough to get ball into conveyer, then stop intake
					// frc::SmartDashboard::PutNumber("ab in breach", true);
					/* both on */ m_intake.Set(-kIntakeSpeed); m_vert_conveyer.Set(-kConveyerSpeed);
					if (m_timer.Get() >= kIntakeDelayArrival) {
						m_intake_state = kBallJustLeft;
					}
					break;
				case kBallJustLeft:  //  keep intake stopped for a period to create space
					// frc::SmartDashboard::PutNumber("ab just left", true);
					// experiment not making this gap anymore
					/* both on */ m_intake.Set(-kIntakeSpeed); m_vert_conveyer.Set(-kConveyerSpeed);
					if (m_timer.Get() >= kIntakeDelayArrival + kIntakeDelayGap) {
						m_intake_state = kBreachEmpty;
					}
					break;
				case kBreachEmpty:
					// frc::SmartDashboard::PutNumber("ab empty", true);
					/* conveyer stopped */ m_intake.Set(-kIntakeSpeed); m_vert_conveyer.Set(0);
					if (eye_intake.Get()) { // a ball just arrived at breach
						m_intake_state = kBallJustArrived;
					} else { // no balls yet
						// don't run converer until another ball arrives
						// was... m_vert_conveyer.Set(-kConveyerSpeed);
					}
			} // switch
		} // if turret is not on-deck
	}

	void OperateConveyer (bool conveyer_in_button, bool conveyer_out_button, double &conveyer_speed) {
			// frc::SmartDashboard::PutNumber("conv man ok", true);
			if (conveyer_in_button) {
				conveyer_speed = -kConveyerSpeed;
			} else if (conveyer_out_button) {
				conveyer_speed = kConveyerSpeed;
			}
			m_vert_conveyer.Set(conveyer_speed);
	}

	void OperateShooter(bool &manual_conveyer_ok, double &conveyer_speed,
		bool manual_boost, bool manual_deboost) {

		double targetSeen = m_limetable->GetNumber("tv",0.0);
		double targetArea = m_limetable->GetNumber("ta",0.0);

		double targetOffsetAngle_Vertical = 0.0;
		double targetOffsetAngle_Horizontal = 0.0;
		if (targetSeen != 0.0) {
		  if (targetArea > kMinTargetAreaPercent) {  // tv is true if there is a target detected
			targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
			targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
		  }
		}
		conveyer_speed = 0.0; 
		manual_conveyer_ok = false;

		double shooter_speed_in_units = m_IdleShooterVelocity;
		if (targetSeen != 0.0) {
			frc::SmartDashboard::PutNumber("L targ angle", targetOffsetAngle_Vertical);
			bool limelight_on_target = TrackTargetWithTurret(targetOffsetAngle_Horizontal);

			if (targetOffsetAngle_Vertical < -18) {
				shooter_speed_in_units = 25000;  // max out shooter if far away
			} else if (targetOffsetAngle_Vertical >= kCloseRangeTargetAngle) { // boost if really close to target
				shooter_speed_in_units = kCloseRangeShooterSpeed;
			} else {
				// 2020 Y intercept was 12696.1 
				// shooter_speed_in_units = m_shooter_y_intercept - m_shooter_slope * targetOffsetAngle_Vertical; // originally 317.502
				// distance in inches =(TargetHeight_h2-CameraHeight_h1)/TAN(RADIANS(CameraAngle_a1+CameraAngle_a2))
				// speed = -4.11574 d_in_feet^3 + 283.464 d_in_feet^2 - 5519.65 d_in_feet + 45450.9
				double dist_in_feet = (52.55 / tan(ConvertDegreesToRads(targetOffsetAngle_Vertical + 26.5))) / 12;
				shooter_speed_in_units = m_shooter_C1 * pow(dist_in_feet,3) 
				                       + m_shooter_C2 * pow(dist_in_feet,2) 
									   + m_shooter_C3 * dist_in_feet + m_shooter_C4; 
				frc::SmartDashboard::PutNumber("L targ dist", dist_in_feet);
			}
			if (manual_boost) {shooter_speed_in_units *= kManualShooterBoost;}
			else if (manual_deboost) {shooter_speed_in_units *= kManualShooterDeboost;}
			double shooter_speed_error = m_shooter_star->GetClosedLoopError(kPIDLoopIdx);
			frc::SmartDashboard::PutNumber("L fly err", shooter_speed_error);
			if (shooter_speed_error < kMaxShooterSpeedError && limelight_on_target) {
				// auto feed balls into shooter
				conveyer_speed = -kConveyerSpeed;
			} else {
				manual_conveyer_ok = true;
			}
		} else { // no target; permit manual control of conveyer
			manual_conveyer_ok = true;
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret; needs to hold position
			frc::SmartDashboard::PutString("I turr state", "stopped");
		}
		frc::SmartDashboard::PutNumber("L targ speed", shooter_speed_in_units);
		m_shooter_star->Set(ControlMode::Velocity, shooter_speed_in_units);

	}

	double BoostSpeedIfTurning (double s, double y, double x) {
		if (x > 0 && y > 0) {
			double angle = ConvertRadsToDegrees(atan(abs(y/x)));
			// frc::SmartDashboard::PutNumber("D joy angle", angle);
			if (angle > 30 && angle < 60) { // turning, so boost
				s *= kArcadeTurningBoost;
			}
		}
		return s;
	}

	void TeleopPeriodic() {

		//double targetPositionRotations =  -2000 * shooter_Y; // positive moves turret clockwise
		//m_turret->Set(ControlMode::Position, targetPositionRotations); 

		// frc::SmartDashboard::PutNumber("turret pos", m_turret->GetSelectedSensorPosition(0));

		/****************************************** buttons and joysticks **************************************/

		/************************************** pilot 
		Left Stick	Robot relative arcade	
		Right Stick	Field relative arcade	
		D Pad	Rotate to compass pts	
		Button 1	Reset Yaw / (color)	
		Button 2	Brake / (color)
		Button 3	Spin control panel / (color)	
		Button 4	Auto chase balls / (color)
		Button 5	Spin to color	
		Button 7	Turbo speed		
		*/
		double robot_rel_X = m_stick->GetRawAxis(0);  // robot relative
		double robot_rel_Y = -m_stick->GetRawAxis(1);
		double field_rel_X = m_stick->GetRawAxis(2);  // field relative
		double field_rel_Y = -m_stick->GetRawAxis(3);
		double field_rel_R = sqrt(field_rel_X*field_rel_X + field_rel_Y*field_rel_Y);

		bool rotateToAngle = false;
		double targetAngle = 0.0;
		if ( m_stick->GetPOV() == 0) {
			targetAngle= 0.0f;
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 90) {
			targetAngle = 90.0f;
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 180) {
			targetAngle = 179.9f;
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 270) {
			targetAngle = 270.0f;
			rotateToAngle = true;
		}
		// no field rel driving for GRC 2021
		// if (field_rel_R > kGamepadDeadZone) {
		// 	rotateToAngle = true;
		// 	// was angle = copysign(angle, field_rel_X); // make angle negative if X is negative
		// 	// a little trig to convert joystick to angle
		// 	targetAngle =  90 - ConvertRadsToDegrees(atan(field_rel_Y/abs(field_rel_X)));
		// 	if (field_rel_X < 0) {targetAngle = 360 - targetAngle;}  // shift from -180>180 to 0>360
		// }

		bool reset_yaw_button_pressed = false;  // reset gyro angle
		bool chase_cells_button = false;
		bool spin_control_panel_button = false;
		bool spin_to_color_pressed = false;
		bool brake_button_pressed = false;
		frc::Color spin_to_color = kNoColor;
		if (m_stick->GetRawButton(5)) { // spin to color
			if (m_stick->GetRawButton(1)) { // blue
				spin_to_color = kRedTarget; // blue
				spin_to_color_pressed = true;
			} else if (m_stick->GetRawButton(2)) { // green
				spin_to_color = kYellowTarget; 
				spin_to_color_pressed = true;
			}else if (m_stick->GetRawButton(3)) { // red
				spin_to_color = kBlueTarget; 
				spin_to_color_pressed = true;
			}else if (m_stick->GetRawButton(4)) { // yellow
				spin_to_color = kGreenTarget; 
				spin_to_color_pressed = true;
			}
			// frc::SmartDashboard::PutString ("Spin to", ColorToString(spin_to_color));
		} else { // color buttons have normal functions
			reset_yaw_button_pressed = m_stick->GetRawButton(1);  // reset gyro angle
			chase_cells_button = m_stick->GetRawButton(4);
			spin_control_panel_button = m_stick->GetRawButton(3);
			brake_button_pressed = m_stick->GetRawButton(2);
		}
		
		bool high_gear_button_presssed = m_stick->GetRawButton(7);

		/****************************************** co-pilot
		Left Stick	deploy hanger and hang	
		Right Stick	Manual intake in/out	
		D Pad	Turret positioning	
		Button 1	+ left Y to deploy hanger
		Button 2	Automatic shoot	
		Button 3	+ left Y to hang
		Button 4	Automatic intake	
		Button 5	Boost shooter up	
		Button 6	Conveyer in/up	
		Button 7	De-boost shooter down	
		Button 8	Conveyer down/out	
		*/ 
		double hanger_Y = m_stick_copilot->GetRawAxis(1);
		// double shooter_R = sqrt(shooter_X*shooter_X + shooter_Y*shooter_Y); // could be used for manual shooter speed
		double intake_Y = m_stick_copilot->GetRawAxis(3);

		long turret_manual_position = 0;
		if ( m_stick_copilot->GetPOV() == 0) {
			turret_manual_position = kTurretUP;
		} else if ( m_stick_copilot->GetPOV() == 90) {
			turret_manual_position = kTurretRIGHT;
		} else if ( m_stick_copilot->GetPOV() == 180) {
			turret_manual_position = kTurretDOWN;
		} else if ( m_stick_copilot->GetPOV() == 270) {
			turret_manual_position = kTurretLEFT;
		}

		bool hanger_deploy_button = m_stick_copilot->GetRawButton(1);
		bool auto_shoot_button =  m_stick_copilot->GetRawButton(2);
		bool hang_button = m_stick_copilot->GetRawButton(3);
		bool auto_intake_button =  m_stick_copilot->GetRawButton(4);
		bool boost_shooter_up_button =  m_stick_copilot->GetRawButton(5);
		bool boost_shooter_down_button =  m_stick_copilot->GetRawButton(7);
		bool conveyer_in_button =  m_stick_copilot->GetRawButton(6);
		bool conveyer_out_button =  m_stick_copilot->GetRawButton(8);

		/******************* move stuff **********************/

		//m_turret->Set(ControlMode::PercentOutput, shooter_Y);  // temporary test

		// just in case turret has a mind of its own, try to stop it... shouldn't happen
		double turret_pos = m_turret->GetSelectedSensorPosition(0);
		if (turret_pos < kTurretLimitStarboard || turret_pos > kTurretLimitPort) {
			frc::SmartDashboard::PutString("I turr state", "out of range");
			m_turret->Set(ControlMode::PercentOutput, 0);
		}
		
		// reset gyro angle
		if ( reset_yaw_button_pressed ) {
			ahrs->ZeroYaw();
			m_leftfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
			m_rightfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		}

		double currAngle = (int)ahrs->GetAngle() % 360;  // angle accumulates past 360, so modulus

		// for planning autonomous
		frc::SmartDashboard::PutNumber ("D Heading", currAngle); 
		frc::SmartDashboard::PutNumber ("D Pos", m_leftfront.GetSelectedSensorPosition(0)); 

		if (m_need_to_reset_coast && !brake_button_pressed) { // no longer braking; go back to coasting
			m_leftfront.SetNeutralMode(NeutralMode::Coast);
			m_leftrear.SetNeutralMode(NeutralMode::Coast);
			m_rightfront.SetNeutralMode(NeutralMode::Coast);
			m_rightrear.SetNeutralMode(NeutralMode::Coast);			
			m_need_to_reset_coast = false;
		}

		// if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
		double speed_factor = kSlowSpeedFactor;
		if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
		// frc::SmartDashboard::PutNumber ("Rotate ratio", abs(kMaxRotateRate - abs(rotateToAngleRate)) / kMaxRotateRate);

		// auto-chase balls
		// double angleToSecondBall = 0.0;
		if (chase_cells_button) {
			m_IdleShooterVelocity = 0.0; // kill shooter because vibration makes camera image blurry
			m_IdleShooterPower = 0.0;
			ChasePowerCellsByCoral();  // was angleToSecondBall=
		} else { // not shooting, so idle normally
			m_IdleShooterVelocity = kIdleShooterVelocity; // idle normally
			m_IdleShooterPower = kIdleShooterPower; // idle normally
		}
		// if (angleToSecondBall > 0.0) {
		// 	rotateToAngle = true;
		// 	targetAngle = angleToSecondBall;
		// }

		// rotation stuff
		try {
			if (brake_button_pressed) {
				m_leftfront.SetNeutralMode(NeutralMode::Brake);
				m_leftrear.SetNeutralMode(NeutralMode::Brake);
				m_rightfront.SetNeutralMode(NeutralMode::Brake);
				m_rightrear.SetNeutralMode(NeutralMode::Brake);
				m_robotDrive.TankDrive(0.0, 0.0, false);
				m_need_to_reset_coast = true;
			} else if (rotateToAngle) {
				// MJS: since it's diff drive instead of mecanum drive, use tank method for rotation
				m_pidController_gyro->SetSetpoint(targetAngle);

				if (currAngle < 0) {currAngle = 360 + currAngle;} // shift from -360>360 to 0>360
				// frc::SmartDashboard::PutNumber("FR Heading", currAngle); // FR = Field Relative
				// frc::SmartDashboard::PutNumber("FR Target", targetAngle);

				// use pid for motor speed, unless in "discontinuity zone" near 0
				if (isInDiscontinuityZone(currAngle, targetAngle)) {
					m_rotate_to_angle_rate = 0.0;   // avoid bouncing back and forth as heading flips between 1 and 355
				} else {
					m_rotate_to_angle_rate = m_pidController_gyro->Calculate(ahrs->GetAngle());
				}
				// trim the speed so it's not too fast
				m_rotate_to_angle_rate = TrimSpeed(m_rotate_to_angle_rate, kMaxRotateRate);
				// if heading is quadrant I and target is IV, or vice versa, flip motor direction
				if ( (quadrant(currAngle) == 1 && quadrant(targetAngle) == 4) 
					|| (quadrant(currAngle) == 4 && quadrant(targetAngle) == 1)) {
					m_rotate_to_angle_rate = -m_rotate_to_angle_rate;
				} else if (abs(currAngle - targetAngle) > 180) {
					// if delta angle > 180, flip motor direction so we take shorter route
					m_rotate_to_angle_rate = -m_rotate_to_angle_rate;
				}

				double left_power = m_rotate_to_angle_rate;
				double right_power = -m_rotate_to_angle_rate;
				if (abs(kMaxRotateRate - abs(m_rotate_to_angle_rate)) / kMaxRotateRate > 0.4) { 
					m_field_rel_timer = m_timer.Get(); // once decide it's ok to move forward, don't stutter
				}
				// frc::SmartDashboard::PutNumber("FR Timer", m_field_rel_timer);
				// frc::SmartDashboard::PutNumber("FR Time", m_timer.Get());
				if (m_timer.Get() < m_field_rel_timer + kFieldRelDriveSmoothTime) { 
					// add forard driving to rotation, to get field relative driving
					double addition = field_rel_R * speed_factor / 2;
					left_power += addition;
					right_power += addition;
				}
				m_robotDrive.TankDrive(left_power, right_power, false);

			} else if (spin_control_panel_button || spin_to_color_pressed) {
				// let cp program control wheels
			} else if (chase_cells_button) {
				// we called ChaseBalls above
			} else {
				// not rotating or chasing balls; drive by stick
				speed_factor = BoostSpeedIfTurning(speed_factor, robot_rel_Y, robot_rel_X);
				frc::SmartDashboard::PutNumber("D arcade boost", speed_factor);
				// change to tank drive for GRC 2021, was m_robotDrive.ArcadeDrive(ScaleSpeed(robot_rel_Y, speed_factor), ScaleSpeed(robot_rel_X, speed_factor));
				m_robotDrive.TankDrive(ScaleSpeed(robot_rel_Y, speed_factor), ScaleSpeed(field_rel_Y, speed_factor));
				m_pidController_gyro->Reset(); // clears out integral state, etc
			}
		} catch (std::exception& ex ) {
			std::string err_string = "Error communicating with Drive System:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		/* get gamepad axis */
		//double leftYstick = _joy->GetY();
		//double motorOutput = m_shooter_star->GetMotorOutputPercent();

		// ++++++++++++++++++++ shooter +++++++++++++++++++++++++++
		bool manual_conveyer_ok = false;
		double conveyer_speed = 0.0;
		
		if (auto_shoot_button) {
			m_limetable->PutNumber("ledMode",3.0); // LED on
			OperateShooter(manual_conveyer_ok, conveyer_speed, 
			    boost_shooter_up_button, boost_shooter_down_button);
		} else { // not shooting
			m_limetable->PutNumber("ledMode",1.0); // LED off
			// m_shooter_star->Set(ControlMode::Velocity, m_IdleShooterVelocity);
			m_shooter_star->Set(ControlMode::PercentOutput, m_IdleShooterPower);
			if (m_need_to_reset_tracking_turret_move) {
				m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
				m_need_to_reset_tracking_turret_move = false;
				frc::SmartDashboard::PutString("I turr state", "tracking stopped");
			}
		}
		OperateConveyer(conveyer_in_button, conveyer_out_button, conveyer_speed);

		if (auto_intake_button /*|| chase_cells_button*/) { // for now, don't auto-intake when chasing; let co-pilot do it
			// bring balls in and index using photo eye
			AutoIntakeBalls();
		} else { // enable manual control of intake
			m_intake.Set(-intake_Y * kIntakeSpeed);
		}
		// operate control panel
		if (spin_control_panel_button) {
			m_need_to_reset_spinner = true;
			SpinThreeTimes();
		} else if (spin_to_color_pressed) {
			m_need_to_reset_spinner = true;
			SpinToColor(spin_to_color);
		} else if (m_need_to_reset_spinner) {
			ResetSpinner();
			m_need_to_reset_spinner = false;
		}
		// frc::SmartDashboard::PutNumber("wheel state", m_wheel_state);
		
		if (turret_manual_position != 0) {
			m_need_to_reset_manual_turret_move = true;
			MoveTurretToManualPosition(turret_manual_position);
		} else if (m_need_to_reset_manual_turret_move) {
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
			m_need_to_reset_manual_turret_move = false;
			frc::SmartDashboard::PutString("I turr state", "man stopped");
		}
		// frc::SmartDashboard::PutNumber("turret pos3", m_turret->GetSelectedSensorPosition(0));
	}


/********************************************** autonomous ********************************************/

	bool RotateToAngle (double targetAngle) {
		// std::cout << "Rotating to: " << targetAngle << std::endl;
		bool result = false; // return true if we are close to target angle

		m_pidController_gyro->SetSetpoint(targetAngle);
		if (m_pidController_gyro->AtSetpoint()) {
			m_robotDrive.TankDrive(0, 0, false);
			m_pidController_gyro->Reset(); // clears out integral state, etc
			result = true;

		} else { // not close to target; keep turning
			double rotateToAngleRate = m_pidController_gyro->Calculate(ahrs->GetAngle());

			// trim the speed so it's not too fast
			rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);

			m_robotDrive.TankDrive(rotateToAngleRate, -rotateToAngleRate, false);
		}

		return result;
	}

	bool DriveToPosition (double pos) {
		// std::cout << "Driving to: " << pos << std::endl;
		bool result = false; // return true if we are close to target position

		m_pidController_search->SetSetpoint(pos);
		if (m_pidController_search->AtSetpoint()) {
			m_robotDrive.TankDrive(0, 0, false);
			m_pidController_search->Reset(); // clears out integral state, etc
			result = true;

		} else { // not close to target; keep moving
			// std::cout << "Still driving, pos= " << m_leftfront.GetSelectedSensorPosition(0) << std::endl;
			//encoder values are negative for the left motor, so invert this
			double speed = -m_pidController_search->Calculate(m_leftfront.GetSelectedSensorPosition(0));

			// trim the speed so it's not too fast
			speed = TrimSpeed(speed, kMaxRotateRate);

			m_robotDrive.TankDrive(speed, speed, false);
		}

		return result;
	}

	void AutoMoveOffLine (bool moveForward, bool moveFast) {
		double motor_speed = 0.0;
		if (moveForward) {
			motor_speed = kAutoDriveSpeed;
		} else {
			motor_speed = -kAutoDriveSpeed;
		}				
		if (moveFast) {
			motor_speed *= 2;
		}
		if (abs(m_leftfront.GetSelectedSensorPosition(0)) < kAutoDriveDistance) {
			m_robotDrive.TankDrive(motor_speed, motor_speed);
			// frc::SmartDashboard::PutString("Auto Mode", "moving");
		} else {
			m_robotDrive.TankDrive(0, 0);
			// frc::SmartDashboard::PutString("Auto Mode", "done moving");
		}
	}

	void AutonomousPeriodic() {

		if (m_autoSelected == kAutoNameSearchMan || m_autoSelected == kAutoNameSearchAuto) {
			if (m_timer2.Get() < 3.0) {
				AutoMoveOffLine(true, true);
			} else { // done dropping intake; start searching
				AutoIntakeBalls();
				// pause to pick up powercell 1
				if (m_timer2.Get() > 4.0) {
					bool doneDriving = false; bool doneRotating = false;
					switch (m_search_state) {
						case kSearchStart:
							m_search_state = kSearchTurningToBall1;
							break;
						// case kSearchBallCollected1:
						// 	m_search_state = kSearchTurningToBall2;
						// 	break;
						case kSearchTurningToBall1:
							//std::cout << "state= turning to: 1: " << m_search_angles[0] << std::endl;
							doneRotating = RotateToAngle(m_search_angles[0]);
							if (doneRotating) {m_search_state = kSearchDrivingToBall1;}
							break;
						case kSearchDrivingToBall1:
							//std::cout << "state= driving to: 1" << m_search_positions[0] << std::endl;
							doneDriving = DriveToPosition(m_search_positions[0]);
							if (doneDriving) {m_search_state = kSearchTurningToBall2;}
							break;
						case kSearchTurningToBall2:
							//std::cout << "state= turning to: 2" << m_search_angles[1] <<std::endl;
							doneRotating = RotateToAngle(m_search_angles[1]);
							if (doneRotating) {m_search_state = kSearchDrivingToBall2;}
							break;
						case kSearchDrivingToBall2:
							//std::cout << "state= driving to: 2" << m_search_positions[1] << std::endl;
							doneDriving = DriveToPosition(m_search_positions[1]);
							if (doneDriving) {m_search_state = kSearchTurningToBall3;}
							break;
						case kSearchTurningToBall3:
							//std::cout << "state= turning to: 3" << m_search_angles[2] << std::endl;
							doneRotating = RotateToAngle(m_search_angles[2]);
							if (doneRotating) {m_search_state = kSearchDrivingToBall3;}
							break;
						case kSearchDrivingToBall3:
							//std::cout << "state= driving to: 3" << m_search_positions[2] << std::endl;
							doneDriving = DriveToPosition(m_search_positions[2]);
							if (doneDriving) {
								if (m_search_angles.size() == 3) { // A Red has only 2 turns plus end run
									m_search_state = kSearchComplete;
								} else {m_search_state = kSearchTurningToEnd;}
							}							
							break;
						case kSearchTurningToEnd:
							//std::cout << "state= turning to: 3" << m_search_angles[3] << std::endl;
							doneRotating = RotateToAngle(m_search_angles[3]);
							if (doneRotating) {m_search_state = kSearchDrivingToEnd;}
							break;
						case kSearchDrivingToEnd:
							//std::cout << "state= driving to: 3" << m_search_positions[3] << std::endl;
							doneDriving = DriveToPosition(m_search_positions[3]);
							if (doneDriving) {m_search_state = kSearchComplete;}
							break;
						case kSearchComplete:
							// might need to keep collector running for a second
							//std::cout << "state= done" << std::endl;
							m_robotDrive.TankDrive(0, 0);
							break;
						default: // done
							std::cout << "state= something wrong" << std::endl;
							m_robotDrive.TankDrive(0, 0);
							break;
					}
				}

			}

		} else if (m_autoSelected == kAutoNameTestWheels) {
			// simple motion to validate motor configuration
			// Drive for 2 seconds
			if (m_timer2.Get() < 2.0) {
			m_robotDrive.TankDrive(0.5, 0); // left motor only
			} else if (m_timer2.Get() < 4.0) {
			m_robotDrive.TankDrive(0, 0.5); // right motor only
			} else if (m_timer2.Get() < 6.0) {
			// Drive forwards half speed
			m_robotDrive.ArcadeDrive(0.5, 0.0);
			} else {
			// Stop robot
			m_robotDrive.ArcadeDrive(0.0, 0.0);
			}
		} else if (m_autoSelected == kAutoNameMoveOnly) {
			AutoMoveOffLine(m_autoSelected_options_dir == kAutoOptionForward, 
							m_autoSelected_options_speed == kAutoOptionFast);
		} else if (m_autoSelected == kAutoNameShootAndMove) {
			// frc::SmartDashboard::PutString("Auto Mode", "kAutoNameShootAndMove");
			if (m_autoSelected_options_wait == kAutoOptionWait && m_timer2.Get() < 3) {
				// frc::SmartDashboard::PutString("Auto Mode", "waiting");
				// do nothing for x secs		
			} else if (m_timer2.Get() < 6) {
				// frc::SmartDashboard::PutString("Auto Mode", "shooting");
				bool manual_conveyer_ok; double conveyer_speed;
				OperateShooter(manual_conveyer_ok, conveyer_speed, false, false);
				OperateConveyer (false, false, conveyer_speed);
			} else { // after 6 seconds
				// m_shooter_star->Set(ControlMode::Velocity, m_IdleShooterVelocity);
				m_shooter_star->Set(ControlMode::PercentOutput, m_IdleShooterPower);
				m_vert_conveyer.Set(0);
				double motor_speed = 0.0;
				if (m_autoSelected_options_dir == kAutoOptionForward) {
					motor_speed = kAutoDriveSpeed;
				} else if (m_autoSelected_options_dir == kAutoOptionBackward) {
					motor_speed = -kAutoDriveSpeed;
				}				
				if (m_autoSelected_options_speed == kAutoOptionFast) {
					motor_speed *= 2;
				}
				// frc::SmartDashboard::PutString("Auto Mode", "should move");
				if (abs(m_leftfront.GetSelectedSensorPosition(0)) < kAutoDriveDistance) {
					m_robotDrive.TankDrive(motor_speed, motor_speed);
					// frc::SmartDashboard::PutString("Auto Mode", "moving");
				} else {
					m_robotDrive.TankDrive(0, 0);
					// frc::SmartDashboard::PutString("Auto Mode", "done moving");
				}
				// frc::SmartDashboard::PutNumber("encoder", m_leftfront.GetSelectedSensorPosition(0));
			}
		} else {
			// only doing init
			// frc::SmartDashboard::PutString("Auto Mode", "nothing");
		}
	}

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
