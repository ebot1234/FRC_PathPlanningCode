#include <iostream>
#include <string>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <AHRS.h>
#include <Timer.h>
#include <wpilib.h>
#include <Math.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/MotorControl/ControlMode.h>
#include <ctre/phoenix/MotorControl/NeutralMode.h>
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>
#include <pathfinder.h>





class Robot : public frc::SampleRobot {


	//System Constants
	const double THRESHOLD = 0.1;
	const int LIFT_TOP = 600;
	const int LIFT_BOTTOM = 0;
	const bool TOP = true;
	const bool BOTTOM = false;
	const bool OPEN = true;
	const bool CLOSED = false;
	const double wheelbase_width = 0.6;//in meters


	//Control System
	Joystick *stick0;
	Joystick *stick1;
	PowerDistributionPanel* m_pdp;
	DigitalInput posRight;
	DigitalInput posCenter;
	DigitalInput posLeft;
	AHRS *ahrs;
	//AnalogInput LV_MAX_Sonar;

	//Drivetrain
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;
	Encoder *leftEn;
	Encoder *rightEn;

	//Lift
	TalonSRX PTO0;
	TalonSRX PTO1;
	Encoder *PTO_Enc;
	DigitalInput limTop;
	DigitalInput limBottom;


	//Intake
	TalonSRX pentacept0;
	TalonSRX pentacept1;
	Solenoid clamp;
	Solenoid pentaTilt;
	DigitalInput boxSensor;

	//LED
	Relay *redLED;
	Relay *greenLED;
	Relay *blueLED;



public:
	Robot():
		//Control System
		//LV_MAX_Sonar(3),
		posRight(1),
		posCenter(2),
		posLeft(3),

		//Drivetrain
		left0(12),
		left1(13),
		right0(4),
		right1(6),

		//Lift
		PTO0(5),
		PTO1(9),
		limTop(6),
		limBottom(7),

		//Intake
		pentacept0(7),
		pentacept1(8),
		clamp(0),
		pentaTilt(1),
		boxSensor(0)


	{
		ahrs = new AHRS(SerialPort::kMXP);
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);
		PTO_Enc = new Encoder(4, 5, true, Encoder::EncodingType::k4X);
		m_pdp = new PowerDistributionPanel();
		redLED = new Relay(0);
		greenLED = new Relay(1);
		blueLED = new Relay(2);
		leftEn = new Encoder(5,6,true, Encoder::EncodingType::k4X);
		rightEn = new Encoder(7, 8, true, Encoder::EncodingType::k4X);
	}

	void RobotInit() {

		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);
		PTO0.SetNeutralMode(NeutralMode::Brake);
		PTO1.SetNeutralMode(NeutralMode::Brake);
		pentacept0.SetNeutralMode(NeutralMode::Coast);
		pentacept1.SetNeutralMode(NeutralMode::Coast);

		left0.SetInverted(true);
		left1.SetInverted(true);
		PTO0.SetInverted(true);
		PTO1.SetInverted(true);
		pentacept1.SetInverted(true);

		clamp.Set(false);
		pentaTilt.Set(false);


		left0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		left0.SetSensorPhase(true);

		right0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		right0.SetSensorPhase(true);

		CameraServer::GetInstance()->StartAutomaticCapture();

		printf("Valentina Tereshkova, Reporting for duty.");
	}

	void SetSpeed(double left, double right){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
		printf("SetSpeed: %f, %f\n", left, right);
		SmartDashboard::PutNumber("SetSpeed Left", left);
		SmartDashboard::PutNumber("SetSpeed Right", right);
	}

	void SetIntakeSpeed(double speed){
		pentacept0.Set(ControlMode::PercentOutput, speed);
		pentacept1.Set(ControlMode::PercentOutput, speed);
		printf("SetSpeed: %f\n", speed);
		SmartDashboard::PutNumber("SetIntakeSpeed", speed);
	}

	void SetLiftSpeed(double speed){
		PTO0.Set(ControlMode::PercentOutput, speed);
		PTO1.Set(ControlMode::PercentOutput, speed);
		printf("SetSpeed: %f\n", speed);
		SmartDashboard::PutNumber("SetLiftSpeed", speed);
	}

	void RunIntakeTime(double speed, double time){
		pentacept0.Set(ControlMode::PercentOutput, speed);
		pentacept1.Set(ControlMode::PercentOutput, speed);
		SmartDashboard::PutNumber("RunIntakeTime", speed);
		Wait(time);
		pentacept0.Set(ControlMode::PercentOutput, 0.0);
		pentacept1.Set(ControlMode::PercentOutput, 0.0);
		SmartDashboard::PutNumber("RunIntakeTime", 0.0);
	}

	void RunLiftTime(double speed, double time){
		PTO0.Set(ControlMode::PercentOutput, speed);
		PTO1.Set(ControlMode::PercentOutput, speed);
		SmartDashboard::PutNumber("RunLiftTime", speed);
		Wait(time);
		PTO0.Set(ControlMode::PercentOutput, 0.0);
		PTO1.Set(ControlMode::PercentOutput, 0.0);
		SmartDashboard::PutNumber("RunLiftTime", 0.0);
	}

	void RunLiftLimits(bool side){
		SmartDashboard::PutBoolean("RunLiftLimits", false);
		if(side){
			while(limTop.Get() && (IsEnabled() && IsAutonomous())){
				PTO0.Set(ControlMode::PercentOutput, 0.7);
				PTO1.Set(ControlMode::PercentOutput, 0.7);
			}
		}
		else{
			while(limBottom.Get() && (IsEnabled() && IsAutonomous())){
				PTO0.Set(ControlMode::PercentOutput, -0.7);
				PTO1.Set(ControlMode::PercentOutput, -0.7);
			}
		}
		PTO0.Set(ControlMode::PercentOutput, 0.0);
		PTO1.Set(ControlMode::PercentOutput, 0.0);
		printf("Lift complete ENC: %i\n", PTO_Enc->Get());
		SmartDashboard::PutBoolean("RunLiftLimits", true);
	}

	void RunLiftPosition(double position){
		PTO_Enc->Reset();

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = position * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		if(position > 0){
			while(PTO_Enc->Get() < EncTarget && (IsEnabled() && IsAutonomous())){
				SetLiftSpeed(0.7);
				printf("While Enc: %i\n", PTO_Enc->Get());
			}
		}
		else if(position < 0){
			while(PTO_Enc->Get() > EncTarget && (IsEnabled() && IsAutonomous())){
				SetLiftSpeed(-0.7);
				printf("While Enc: %i\n", PTO_Enc->Get());
			}
		}
		SetLiftSpeed(0.0);
	}

	void ClampToggle(bool state){
		clamp.Set(state);
	}

	double Map(double x, double in_min, double in_max, double out_min, double out_max){//This function scales one value to a set range
		return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}
	void setLeftMotors(double left){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
	}

	void setRightMotors(double right){
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
	}

void GenerateProfile(){

}

	//For Testing purposes only
void DriveStraight(){
	ahrs->Reset();
		int POINT_LENGTH = 3;
		Waypoint points[POINT_LENGTH];
		Waypoint p1 = {0,4,0}; //{X,Y,Angle}
		Waypoint p2 = {10, 4,0};
		Waypoint p3 = {12, 4, 0};
		points[0] = p1;
		points[1] = p2;
		points[2] = p3;

		TrajectoryCandidate candidate;

		pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

		int length = candidate.length;

		// Array of Segments (the trajectory points) to store the trajectory in
		Segment *trajectory = (Segment*)malloc(length * sizeof(trajectory));

		// Generate the trajectory
		pathfinder_generate(&candidate, trajectory);

		Segment leftTrajectory[length];
		Segment rightTrajectory[length];

		pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

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



		double l_encoder_value = leftEn->Get();
		double r_encoder_value = rightEn->Get();

		double l = pathfinder_follow_encoder(leftConfig, leftFollower, leftTrajectory, length, l_encoder_value);
		double r = pathfinder_follow_encoder(rightConfig, rightFollower, rightTrajectory, length, r_encoder_value);

		//Gyro Code
		double gyro_heading = ahrs->GetYaw();
		double desired_heading = r2d(leftFollower->heading);
		double angle_diffrence = desired_heading - gyro_heading;
		double turn = 0.8 * (-1.0/80.0) * angle_diffrence;

		setLeftMotors(l + turn);
		setRightMotors(r - turn);

}


void Left_LeftScale(){
	ahrs->Reset();
 int POINT_LENGTH = 4;
 Waypoint points[POINT_LENGTH];
 Waypoint p1 = {0, 24, 0};
 Waypoint p2 = {18, 24, 0};
 Waypoint p3 = {21, 24.50, 0};
 Waypoint p4 = {26, 21, d2r(90)};
 points[0] = p1;
 points[1] = p2;
 points[2] = p3;
 points[3] = p4;

 TrajectoryCandidate candidate;

 pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

 int length = candidate.length;

 		// Array of Segments (the trajectory points) to store the trajectory in
 Segment *trajectory = (Segment*)malloc(length * sizeof(trajectory));

 		// Generate the trajectory
 pathfinder_generate(&candidate, trajectory);

 Segment leftTrajectory[length];
 Segment rightTrajectory[length];

 pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

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



 		double l_encoder_value = leftEn->Get();
 		double r_encoder_value = rightEn->Get();

 		double l = pathfinder_follow_encoder(leftConfig, leftFollower, leftTrajectory, length, l_encoder_value);
 		double r = pathfinder_follow_encoder(rightConfig, rightFollower, rightTrajectory, length, r_encoder_value);

 		//Gyro Code
 		double gyro_heading = ahrs->GetYaw();
 		double desired_heading = r2d(leftFollower->heading);
 		double angle_diffrence = desired_heading - gyro_heading;
 		double turn = 0.8 * (-1.0/80.0) * angle_diffrence;

 		setLeftMotors(l + turn);
 		setRightMotors(r - turn);
}

void Right_RightScale(){

}

void Left_CrossLine(){

}

void Right_CrossLine(){}

void Left_LeftSwitch(){}

void Right_RightSwitch(){}

void Center_LeftSwitch(){}

void Center_RightSwitch(){}





	void Autonomous(){
		//http://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		left0.SetNeutralMode(NeutralMode::Brake);
		left1.SetNeutralMode(NeutralMode::Brake);
		right0.SetNeutralMode(NeutralMode::Brake);
		right1.SetNeutralMode(NeutralMode::Brake);
		PTO_Enc->Reset();


		int startPos = 0;//0 = Default, 1 = Right, 2 = Center, 3 = Left
		char allianceSwitch = gameData[0];
		char scale = gameData[1];

		if(!posRight.Get()){
			startPos = 1;
		}
		else if(!posCenter.Get()){
			startPos = 2;
		}
		else if(!posLeft.Get()){
			startPos = 3;
		}
		else{
			startPos = 0;
		}

		if(startPos == 1){//Starting: Right
			printf("Right\n");
			if(allianceSwitch == 'L' && scale == 'L'){//If both scale and switch are on the wrong side
				DriveStraight();//!!!!TO TEST WITH ONLY

			}
			else if(allianceSwitch == 'R' && scale == 'L'){//If switch is right and scale is wrong

			}
			else if(allianceSwitch == 'L' && scale == 'R'){//If scale is right and switch is wrong

			}
			else if(allianceSwitch == 'R' && scale == 'R'){//If both scale and switch are on the right side

			}
		}
		else if(startPos == 2){//Starting: Center
			printf("Center\n");
			if(allianceSwitch == 'L'){//If switch is left

			}
			else if(allianceSwitch == 'R'){//If switch is right

			}
		}
		else if(startPos == 3){//Starting: Left
			printf("Left\n");
			if(allianceSwitch == 'R' && scale == 'R'){//If both scale and switch are on the wrong side

			}
			else if(allianceSwitch == 'L' && scale == 'R'){//If switch is right and scale is wrong

			}
			else if(allianceSwitch == 'R' && scale == 'L'){//If scale is right and switch is wrong

			}
			else if(allianceSwitch == 'L' && scale == 'L'){//If both scale and switch are on the right side

			}
		}
		else{
			printf("Default\n");
		}
	}


	void OperatorControl() override{
		double left;
		double right;
		bool twoStick = true;

		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);
		PTO_Enc->Reset();

		while(IsEnabled() && IsOperatorControl()){
			//Arcade
			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();

			//Tank
			//left = stick0->GetY();
			//right = stick1->GetY();

			//Drive Train (driver 1)
			if(fabs(left) >= THRESHOLD && twoStick == true){
				left0.Set(ControlMode::PercentOutput, left);
				left1.Set(ControlMode::PercentOutput, left);
			}
			else if(twoStick == true){
				left0.Set(ControlMode::PercentOutput, 0);
				left1.Set(ControlMode::PercentOutput, 0);
			}

			if(fabs(right) >= THRESHOLD && twoStick == true){
				right0.Set(ControlMode::PercentOutput, right);
				right1.Set(ControlMode::PercentOutput, right);
			}
			else if(twoStick == true){
				right0.Set(ControlMode::PercentOutput, 0);
				right1.Set(ControlMode::PercentOutput, 0);
			}

			//Lift (driver 2)
			if(stick1->GetY() >= THRESHOLD && limTop.Get()){//Should add limits based on mag switch or encoder
				printf("Stick1 Y = %f\n", stick1->GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1->GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1->GetY());
			}
			else if(stick1->GetY() <= -THRESHOLD && limBottom.Get()){//Lower limits
				printf("Stick1 Y = %f\n", stick1->GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1->GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1->GetY());
			}
			else{
				PTO0.Set(ControlMode::PercentOutput, 0.1);
				PTO1.Set(ControlMode::PercentOutput, 0.1);
			}

			if(stick1->GetPOV() == 0 && twoStick == false){
				SetSpeed(-stick1->GetThrottle(), -stick1->GetThrottle());
			}
			else if(stick1->GetPOV() == 90 && twoStick == false){
				SetSpeed(-stick1->GetThrottle(), stick1->GetThrottle());
			}
			else if(stick1->GetPOV() == 180 && twoStick == false){
				SetSpeed(stick1->GetThrottle(), stick1->GetThrottle());
			}
			else if(stick1->GetPOV() == 270 && twoStick == false){
				SetSpeed(stick1->GetThrottle(), -stick1->GetThrottle());
			}
			else if(twoStick == false){
				SetSpeed(0.0, 0.0);
			}

			//Intake (driver 2)
			if(stick1->GetRawButton(2)){//Button 2
				pentacept0.Set(ControlMode::PercentOutput, -1);
				pentacept1.Set(ControlMode::PercentOutput, -1);
			}
			else if(stick1->GetRawButton(1)){//Trigger button
				pentacept0.Set(ControlMode::PercentOutput, 1);
				pentacept1.Set(ControlMode::PercentOutput, 1);
			}
			else if(stick1->GetRawButton(5)){//Button 2
				pentacept0.Set(ControlMode::PercentOutput, -0.4);
				pentacept1.Set(ControlMode::PercentOutput, -0.4);
			}
			else{
				pentacept0.Set(ControlMode::PercentOutput, 0.0);
				pentacept1.Set(ControlMode::PercentOutput, 0.0);
			}

			if(stick1->GetRawButton(3)){//Button 3
				clamp.Set(true);
				printf("Clamp Open\n");
			}
			else{
				clamp.Set(false);
			}

			if(stick1->GetRawButton(7)){
				pentaTilt.Set(true);
				printf("Tilt Down\n");
			}
			else{
				pentaTilt.Set(false);
			}


			if(stick0->GetRawButton(5)){
				redLED->Set(Relay::Value::kForward);
			}
			else{
				redLED->Set(Relay::Value::kOff);
			}

			if(stick0->GetRawButton(2)){
				greenLED->Set(Relay::Value::kForward);
			}
			else{
				greenLED->Set(Relay::Value::kOff);
			}

			if(stick0->GetRawButton(6)){
				blueLED->Set(Relay::Value::kForward);
			}
			else{
				blueLED->Set(Relay::Value::kOff);
			}


			printf("Left Encoder: %d\n", left0.GetSelectedSensorPosition(0));
			printf("Right Encoder: %d\n", right0.GetSelectedSensorPosition(0));
			printf("PTO Encoder: %d\n", PTO_Enc->Get());
			printf("Top %i, Bottom %i\n", limTop.Get(), limBottom.Get());

			Wait(0.04);
		}
	}


	void Test() override {//OMG 1080!!!1!11! Lacey made me do this

	}

private:

};

START_ROBOT_CLASS(Robot)
