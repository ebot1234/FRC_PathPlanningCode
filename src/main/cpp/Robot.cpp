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
#include <pathfinder.h>
#include <frc/WPILib.h>
#include <string>

using namespace std;

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

static const int k_ticks_per_rev = 1024;
static const int k_wheel_diameter = 6;
static const double k_max_velocity = 0.2;


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

	driveLF.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	driveLF.SetSensorPhase(true);

	driveRF.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	driveRF.SetSensorPhase(true);

	driveLF.SetSelectedSensorPosition(0, 0, 0);
	driveRF.SetSelectedSensorPosition(0, 0, 0);

	frc::CameraServer::GetInstance()->StartAutomaticCapture();

	std::cout << "Robot is ready for the stuff!\n" << endl;

}
double Map(double x, double in_min, double in_max, double out_min, double out_max){//This function scales one value to a set range
		return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}

void SetSpeed(double left, double right){
	driveLR.Follow(driveLF);
	driveRR.Follow(driveRF);

	driveLF.Set(ControlMode::PercentOutput, left);

	driveRF.Set(ControlMode::PercentOutput, right);
}

void PIDTurn(int angle, double timeOut){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs.Reset();
		Timer t1;
		t1.Get();
		t1.Reset();
		t1.Start();

		frc::Wait(0.25);


		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 4;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.25;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs.GetYaw();
		double output;

		while(timeBuffer < 5 && t1.Get() < timeOut){//Need to find a stop condition
			error = angle - ahrs.GetYaw();//Error = Final - Current

			printf("Timer %f\n", t1.Get());

			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error
			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together
			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				SetSpeed(output, -output);
			}
			else if(angle > 0){
				SetSpeed(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			if(fabs(error) < 3){
				timeBuffer++;
			}
			else{
				timeBuffer = 0;
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs.GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		SetSpeed(0.0, 0.0);

		printf("PID Complete\n");
	}

void TestPath(int timeOut){
	Timer t1;
	t1.Get();
	t1.Reset();
	t1.Start();


	driveLF.SetSelectedSensorPosition(0, 0, 0);
	driveRF.SetSelectedSensorPosition(0, 0, 0);

	ahrs.Reset();
	std::cout << "Started the thing\n" << endl;
int POINT_LENGTH = 2;

Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

Waypoint p1 = { 0, 0, d2r(0)};      
Waypoint p2 = { 1, 0, d2r(0)};             
points[0] = p1;
points[1] = p2;

TrajectoryCandidate candidate;

pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);
free(points);
int length = candidate.length;

// Array of Segments (the trajectory points) to store the trajectory in
Segment* trajectory = (Segment*)malloc(length * sizeof(Segment));

// Generate the trajectory
int result = pathfinder_generate(&candidate, trajectory);
if (result < 0) {
    // An error occured
    std::cout << "Uh-Oh! Trajectory could not be generated!\n" << endl;
} else{
	std::cout << "Trajectory Generated\n" << endl;
}
Segment* lTrajectory = (Segment*)malloc(length * sizeof(Segment));
Segment* rTrajectory = (Segment*)malloc(length * sizeof(Segment));
// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.6;
std::cout << "about to run the modifier\n" << endl;
// Generate the Left and Right trajectories of the wheelbase using the 
// originally generated trajectory
pathfinder_modify_tank(trajectory, candidate.length, lTrajectory, rTrajectory, wheelbase_width);
std::cout << "Ran modifier\n" << endl;

std::cout << "About to start encoder follower\n" << endl;

EncoderFollower* leftFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
EncoderFollower* rightFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));

leftFollower->last_error = 0;
leftFollower->segment = 0;
leftFollower->finished = 0;
std::cout << "Finished the left encoder follower\n" << endl;

rightFollower->last_error = 0;
rightFollower->segment = 0;
rightFollower->finished = 0;
std::cout << "Finished the right encoder follower\n" << endl;


double wheel_cir = 18.84956; 

std::cout << "About to start the encoder configs\n" << endl;

EncoderConfig leftConfig;
leftConfig = {driveLF.GetSelectedSensorPosition(0), k_ticks_per_rev, wheel_cir, 1.0, 0.0, 0.0, 1.0 / k_max_velocity, 0.0};
std::cout << "Finished left config\n" << endl;

EncoderConfig rightConfig;
rightConfig = {driveRF.GetSelectedSensorPosition(0), k_max_velocity, wheel_cir, 1.0, 0.0, 0.0, 1.0 / k_max_velocity, 0.0};
std::cout << "Finished right config\n" << endl;



while(t1.Get() < timeOut){

		double l = pathfinder_follow_encoder(leftConfig, leftFollower, lTrajectory, candidate.length, driveLF.GetSelectedSensorPosition(0));
		double r = pathfinder_follow_encoder(rightConfig, rightFollower, rTrajectory, candidate.length, driveRF.GetSelectedSensorPosition(0));

		printf("Left Percent Encoder %f", l);
		printf("Right Percent Encoder %f", r);

		std::cout << "Started to set up gyro\n" << endl;

		//double currentYaw = GetAdjustedYaw();
		double currentYaw = ahrs.GetYaw();
		double desired_heading = r2d(leftFollower->heading);
		double angle_diffrence = r2d(leftFollower->heading) - currentYaw;
		const double K_T = 0.35;
		double turn = K_T * angle_diffrence;

		std::cout << "Gyro Setup Complete\n" << endl;

		std::cout << "Setting Motors to path\n" << endl;

		driveLR.Set(ControlMode::PercentOutput, l + turn);
		driveLF.Set(ControlMode::PercentOutput, l + turn);

		driveRR.Set(ControlMode::PercentOutput, r - turn);
		driveRF.Set(ControlMode::PercentOutput, r - turn);

		printf("Left Motors %f\n", l + turn);
		printf("Right Motors %f\n", r - turn);

	}

	std::cout << "Path End\n" << endl;

	frc::Wait(0.1);

	free(trajectory);
	free(lTrajectory);
	free(rTrajectory);
	free(leftFollower);
	free(rightFollower);

	std::cout << "Free'd the trajectories and follower\n" << endl;
}

double GetAdjustedYaw(){
	double yawOffset = 0;
	ahrs.Reset();
	double ahrsYaw = ahrs.GetYaw();
	double calculatedOffset = ahrsYaw + yawOffset;

	if(calculatedOffset >= 180)
	{
		calculatedOffset = calculatedOffset - 180;
	}

	return calculatedOffset;
}


void Robot::Autonomous() {
	driveLF.SetNeutralMode(NeutralMode::Brake);
	driveLR.SetNeutralMode(NeutralMode::Brake);
	driveRF.SetNeutralMode(NeutralMode::Brake);
	driveRR.SetNeutralMode(NeutralMode::Brake);
	
	TestPath(10);
}


void Robot::OperatorControl() {


	double left;
	double right;
	PTO_Enc.Reset();

	while (IsOperatorControl() && IsEnabled()) {
		 left = stick0.GetY() - stick0.GetX();
		 right = stick0.GetY() + stick0.GetX();


		if(stick1.GetY() >= THRESHOLD && limTop.Get()){//Upper limits
				printf("Stick1 Y = %f\n", stick1.GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1.GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1.GetY());
			}
			else if(stick1.GetY() <= -THRESHOLD && limBottom.Get()){//Lower limits
				printf("Stick1 Y = %f\n", stick1.GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1.GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1.GetY());
			}
			else{
				PTO0.Set(ControlMode::PercentOutput, 0.1);
				PTO1.Set(ControlMode::PercentOutput, 0.1);
			}


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

		/*if(stick1.GetRawButton(5)){
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
		}*/

		printf("Left Encoder: %d\n", driveLF.GetSelectedSensorPosition(0));
		printf("Right Encoder: %d\n", driveRF.GetSelectedSensorPosition(0));
		printf("PTO Encoder: %d\n", PTO_Enc.Get());
		printf("Top %i, Bottom %i\n", limTop.Get(), limBottom.Get());

		frc::Wait(0.004);// The motors will be updated every 4ms
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