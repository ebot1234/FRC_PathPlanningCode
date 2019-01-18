#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/MotorControl/ControlMode.h"
#include "ctre/phoenix/MotorControl/NeutralMode.h"
#include "ctre/phoenix/MotorControl/FeedbackDevice.h"
#include "AHRS.h"
#include "pathfinder.h"
#include <WPILib.h>

Robot::Robot() {
	
}
//Hype's Constants
const double THRESHOLD = 0.1;
const int LIFT_TOP = 600;
const int LIFT_BOTTOM = 0;
const bool TOP = true;
const bool BOTTOM = false;
const bool OPEN = true;
const bool CLOSED = false;

//Pathfinding Varibles
const double TIMESTEP = 0.02;
const double MAX_VEL = 18;
const double MAX_ACCEL = 12;
const double MAX_JERK = 60;
const double WHEEL_CIRCUMFERENCE = 13.8;
double Wheel_Base_Width = 30;
TrajectoryCandidate candidate;
Segment* trajectory;
Segment* leftTrajectory;
Segment* rightTrajectory;
int length;
EncoderFollower* leftFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower)); 
EncoderFollower* rightFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
EncoderConfig leftConfig;
EncoderConfig rightConfig;
//Encoder Config Varibles
const int TICKS_PER_REV = 26214;
const double K_P = 1.0;
const double K_I = 0.0;
const double K_D = 0.15;
const double K_V = 0.06;
const double K_A = 0.0856;
const double K_T = 0.35;


//Motors and CS
frc::Joystick stick0{0};
frc::Joystick stick1{1};
TalonSRX driveLF = {12};
TalonSRX driveLR = {13};
TalonSRX driveRF = {4};
TalonSRX driveRR = {6};
frc::Solenoid clamp{0};
frc::Solenoid pentTilt{1};
TalonSRX PTO0 = {5};
TalonSRX PTO1 = {9};
TalonSRX pentacept0 = {7};
TalonSRX pentacept1 = {8};
frc::DigitalInput limTop{6};
frc::DigitalInput limBottom{7};
frc::Encoder PTO_Enc = {4, 5, frc::Encoder::EncodingType::k4X};
frc::PowerDistributionPanel m_pdp;
frc::Relay redLED{0};
frc::Relay greenLED{1};
frc::Relay blueLED{2};

AHRS ahrs = {SerialPort::kMXP};

double fabs(double input){
	if(input >= 0){
		return input;
	}
	else if(input < 0){
		return -input;
	}
	else{
		std::cout << "Math is broken" << std::endl;
	}
}

void Robot::RobotInit() {
	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	driveLF.SetNeutralMode(NeutralMode::Brake);
	driveLR.SetNeutralMode(NeutralMode::Brake);
	driveRF.SetNeutralMode(NeutralMode::Brake);
	driveRR.SetNeutralMode(NeutralMode::Brake);

	driveLF.SetInverted(true);
	driveLR.SetInverted(true);

}

void TestPath(){
	//Path Generation
	trajectory = NULL;
	leftTrajectory = NULL;
	rightTrajectory = NULL;
	length = 0;

	const int POINT_LENGTH = 2;

	Waypoint points[POINT_LENGTH];
	Waypoint p1 = {0, 0, d2r(0)}; //X, Y, d2r(i) i = angle. Ex: 0, 9, d2r(90)
	Waypoint p2 = {1, 0, d2r(0)}; 
	points[0] = p1;
	points[1] = p2;

	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VEL, MAX_ACCEL, MAX_JERK, &candidate);
	length = candidate.length;
	trajectory = (Segment*)malloc(length * sizeof(Segment));
	pathfinder_generate(&candidate, trajectory);

	//Modify for tank drive base
	leftTrajectory = (Segment*)malloc(length * sizeof(Segment));
	rightTrajectory = (Segment*)malloc(length * sizeof(Segment));

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, Wheel_Base_Width);

	leftConfig = {driveLF.GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};
	rightConfig = {driveRF.GetSelectedSensorPosition(0), TICKS_PER_REV, WHEEL_CIRCUMFERENCE, K_P, K_I, K_D, K_V, K_A};

	leftFollower->last_error = 0;
	leftFollower->segment = 0;
	leftFollower->finished = 0;

	rightFollower->last_error = 0;
	rightFollower->segment = 0;
	rightFollower->finished = 0;

	//Current encoder positions
	int currentLeftPos = driveLF.GetSelectedSensorPosition(0);
	int currentRightPos = driveRF.GetSelectedSensorPosition(0);

	//path followers
	double leftSide = pathfinder_follow_encoder(leftConfig, leftFollower, leftTrajectory, length, currentLeftPos);
	double rightSide = pathfinder_follow_encoder(rightConfig, rightFollower, rightTrajectory, length, currentRightPos);

	//gyro calculations
	double currentYaw = ahrs.GetYaw();
	double desired_heading = r2d(leftFollower->heading);
	double angleDifference = r2d(leftFollower->heading) - currentYaw;
	double turn = K_T * angleDifference;

	//Set the motors to the path
	driveLF.Set(ControlMode::PercentOutput, leftSide + turn);
	driveLR.Set(ControlMode::PercentOutput, leftSide + turn);

	driveRF.Set(ControlMode::PercentOutput, rightSide - turn);
	driveRR.Set(ControlMode::PercentOutput, rightSide - turn);
}


void Robot::Autonomous() {
	driveLF.SetNeutralMode(NeutralMode::Brake);
	driveLR.SetNeutralMode(NeutralMode::Brake);
	driveRF.SetNeutralMode(NeutralMode::Brake);
	driveRR.SetNeutralMode(NeutralMode::Brake);
	
	frc::Wait(0.1); //Waits 1 second before running the path

	//Runs a path that is 1ft in lenght straight
	TestPath();
}


void Robot::OperatorControl() {
	double left;
	double right;
	PTO_Enc.Reset();

	while (IsOperatorControl() && IsEnabled()) {
		 left = stick0.GetY() - stick0.GetX();
		 right = stick0.GetY() + stick0.GetX();

		//Driver 1
		if(fabs(left) >= THRESHOLD){
			driveLF.Set(ControlMode::PercentOutput, left);
			driveLR.Set(ControlMode::PercentOutput, left);
		}
		else{
			driveLR.Set(ControlMode::PercentOutput, 0);
			driveLF.Set(ControlMode::PercentOutput, 0);
		}
		
		if(fabs(right) >= THRESHOLD){
			driveRF.Set(ControlMode::PercentOutput, right);
			driveRR.Set(ControlMode::PercentOutput, right);
		}
		else{
			driveRR.Set(ControlMode::PercentOutput, 0);
			driveRF.Set(ControlMode::PercentOutput, 0);
		}

		if(stick0.GetRawButton(5)){
			redLED.Set(Relay::Value::kForward);
		}
		else{
			redLED.Set(Relay::Value::kOff);
		}

		if(stick0.GetRawButton(2)){
			greenLED.Set(Relay::Value::kForward);
		}
		else{
			greenLED.Set(Relay::Value::kOff);
		}

		if(stick0.GetRawButton(6)){
			blueLED.Set(Relay::Value::kForward);
		}
		else{
			blueLED.Set(Relay::Value::kOff);
		}

		//Driver 2
		if(stick1.GetRawButton(2)){
			pentacept0.Set(ControlMode::PercentOutput, -1);
			pentacept1.Set(ControlMode::PercentOutput, -1);
		}
		else if(stick1.GetRawButton(1)){
			pentacept0.Set(ControlMode::PercentOutput, 1);
			pentacept1.Set(ControlMode::PercentOutput, 1);
		}
		else if(stick1.GetRawButton(5)){
			pentacept0.Set(ControlMode::PercentOutput, -0.4);
			pentacept1.Set(ControlMode::PercentOutput, -0.4);
		}
		else{
			pentacept0.Set(ControlMode::PercentOutput, 0.0);
			pentacept1.Set(ControlMode::PercentOutput, 0.0);
		}

		if(stick1.GetRawButton(7)){
			clamp.Set(true);
			printf("Clamp Open\n");
		}
		else{
			clamp.Set(false);
		}

		if(stick1.GetRawButton(7)){
			pentTilt.Set(true);
			printf("Tilt Open\n");
		}
		else{
			clamp.Set(false);
		}

		printf("Left Encoder: %d\n", driveLF.GetSelectedSensorPosition(0));
		printf("Right Encoder: %d\n", driveRF.GetSelectedSensorPosition(0));
		printf("PTO Encoder: %d\n", PTO_Enc.Get());
		printf("Top %i, Bottom %i\n", limTop.Get(), limBottom.Get());

		frc::Wait(0.04);// The motors will be updated every 4ms
	}

	driveLF.SetNeutralMode(NeutralMode::Brake);
	driveLR.SetNeutralMode(NeutralMode::Brake);
	driveRF.SetNeutralMode(NeutralMode::Brake);
	driveRR.SetNeutralMode(NeutralMode::Brake);
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif