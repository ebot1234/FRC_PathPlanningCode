#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <AHRS.h>
#include <pathfinder.h>

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;

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
}

void setLeftMotors(double speed){
	driveLF.Set(ControlMode::PercentOutput, speed);
	driveLR.Set(ControlMode::PercentOutput, speed);
}

void setRightMotors(double speed){
	driveRF.Set(ControlMode::PercentOutput, speed);
	driveRR.Set(ControlMode::PercentOutput, speed);
}

void testPathCSV(){
	//Deserializes the pre-generated trajectories
	FILE *lf = fopen("Test_left_detailed.csv", "r");
	Segment leftTrajectory[1024];
	int leftLength = pathfinder_deserialize_csv(lf, leftTrajectory);

	FILE *rf = fopen("Test_right_detailed.csv", "r");
	Segment rightTrajectory[1024];
	int rightLength = pathfinder_deserialize_csv(rf, rightTrajectory);

	//Left Encoder Follower for left side of robot
	EncoderFollower *leftFollower = (EncoderFollower*)malloc(sizeof(leftFollower));
	leftFollower->last_error = 0; leftFollower->segment = 0; leftFollower->finished = 0;

	//Right Encoder Follower for Right side of robot
	EncoderFollower *rightFollower = (EncoderFollower*)malloc(sizeof(rightFollower));
	rightFollower->last_error = 0; rightFollower->segment = 0; rightFollower->finished = 0;


	int left_encoder_position = 0.0;
	int right_encoder_position = 0.0;
	double wheelCircumpfrence = 0.35052;
	double max_velocity = 1;
	//Left Encoder Config ****!!!!Edit if needed!!!!****
	EncoderConfig leftConfig = {left_encoder_position, 1000, wheelCircumpfrence,
		        1.0, 0.0, 0.0, 1.0 / max_velocity, 0.0};
	//Right Encoder Config ****!!!!Edit if needed!!!!****
	EncoderConfig rightConfig = {right_encoder_position, 1000, wheelCircumpfrence,
				1.0, 0.0, 0.0, 1.0 / max_velocity, 0.0};



	double l_encoder_value = driveLF.GetSelectedSensorPosition(0);
	double r_encoder_value = driveLR.GetSelectedSensorPosition(0);

	double l = pathfinder_follow_encoder(leftConfig, leftFollower, leftTrajectory, leftLength, l_encoder_value);
	double r = pathfinder_follow_encoder(rightConfig, rightFollower, rightTrajectory, rightLength, r_encoder_value);

	//Gyro Code
	double gyro_heading = ahrs.GetYaw();
	double desired_heading = r2d(leftFollower->heading);
	double angle_diffrence = desired_heading - gyro_heading;
	double turn = 0.8 * (-1.0/80.0) * angle_diffrence;

	setLeftMotors(l + turn);
	setRightMotors(r - turn);

	//Free's the malloc pointers
	free(leftFollower);
	free(rightFollower);

}

void testPath(){
	//Calculate the points
	int point_length = 3;

	Waypoint *points  = (Waypoint*)malloc(sizeof(Waypoint) * point_length);

	Waypoint p1 = {0, 0, 0}; //X,Y,Angle
	Waypoint p2 = {4, 0, 0};
	Waypoint p3 = {6, 0, 0};
	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, point_length, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);
	free(points);

	int length = candidate.length;
	Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));
	
	//Generates the path into trajectory
	pathfinder_generate(&candidate, trajectory);

	//Finds the left and right trajectories by using the length of the trajectory candidate
	Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	//Wheel Base Width
	double wheel_base_width = 0.762;
	//Modifies the trajectories to work with our type of drivebase
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheel_base_width);

	//Left Encoder Follower for left side of robot
	EncoderFollower *leftFollower = (EncoderFollower*)malloc(sizeof(leftFollower));
	leftFollower->last_error = 0; leftFollower->segment = 0; leftFollower->finished = 0;

	//Right Encoder Follower for Right side of robot
	EncoderFollower *rightFollower = (EncoderFollower*)malloc(sizeof(rightFollower));
	rightFollower->last_error = 0; rightFollower->segment = 0; rightFollower->finished = 0;


	int left_encoder_position = 0.0;
	int right_encoder_position = 0.0;
	double wheelCircumpfrence = 0.35052;
	double max_velocity = 1;
	//Left Encoder Config ****!!!!Edit if needed!!!!****
	EncoderConfig leftConfig = {left_encoder_position, 1000, wheelCircumpfrence,
		        1.0, 0.0, 0.0, 1.0 / max_velocity, 0.0};
	//Right Encoder Config ****!!!!Edit if needed!!!!****
	EncoderConfig rightConfig = {right_encoder_position, 1000, wheelCircumpfrence,
				1.0, 0.0, 0.0, 1.0 / max_velocity, 0.0};



	double l_encoder_value = driveLF.GetSelectedSensorPosition(0);
	double r_encoder_value = driveLR.GetSelectedSensorPosition(0);

	double l = pathfinder_follow_encoder(leftConfig, leftFollower, leftTrajectory, length, l_encoder_value);
	double r = pathfinder_follow_encoder(rightConfig, rightFollower, rightTrajectory, length, r_encoder_value);

	//Gyro Code
	double gyro_heading = ahrs.GetYaw();
	double desired_heading = r2d(leftFollower->heading);
	double angle_diffrence = desired_heading - gyro_heading;
	double turn = 0.8 * (-1.0/80.0) * angle_diffrence;

	setLeftMotors(l + turn);
	setRightMotors(r - turn);

	//Free's the malloc pointers
	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);
	free(leftFollower);
	free(rightFollower);

}

void Robot::Autonomous() {
	testPath();
	//testPathCSV();
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