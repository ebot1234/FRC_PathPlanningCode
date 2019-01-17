#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix/MotorControl/ControlMode.h>
#include <ctre/phoenix/MotorControl/NeutralMode.h>
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>
#include <AHRS.h>
#include <pathfinder.h>

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

void setLeftMotors(double speed){
	driveLF.Set(ControlMode::PercentOutput, speed);
	driveLR.Set(ControlMode::PercentOutput, speed);
}

void setRightMotors(double speed){
	driveRF.Set(ControlMode::PercentOutput, speed);
	driveRR.Set(ControlMode::PercentOutput, speed);
}

void TestPath(){
	
	//Path Generation
	trajectory = NULL;
	leftTrajectory = NULL;
	rightTrajectory = NULL;
	length = 0;

	const int POINT_LENGTH = 2;

	Waypoint points[POINT_LENGTH];
	Waypoint p1 = {0, 0, d2r(0)};
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
	printf("left encoder position %i, right encoder position %i", currentLeftPos, currentRightPos);

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
	
	//Runs a path that is 1ft in lenght straight
	TestPath();
}


void Robot::OperatorControl() {
	while (IsOperatorControl() && IsEnabled()) {
		double leftStick = -stick0.GetY();
		double rightStick = stick1.GetY();

		if(fabs(leftStick) > THRESHOLD){
			driveLF.Set(ControlMode::PercentOutput, leftStick);
			driveLR.Set(ControlMode::PercentOutput, leftStick);
		}
		else{
			driveLR.Set(ControlMode::PercentOutput, 0);
			driveLF.Set(ControlMode::PercentOutput, 0);
		}
		
		if(fabs(rightStick) > THRESHOLD){
			driveRF.Set(ControlMode::PercentOutput, rightStick);
			driveRR.Set(ControlMode::PercentOutput, rightStick);
		}
		else{
			driveRR.Set(ControlMode::PercentOutput, 0);
			driveRF.Set(ControlMode::PercentOutput, 0);
		}
		
		frc::Wait(0.005);// The motors will be updated every 5ms
	}
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif