#pragma

#include <string>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "pathfinder.h"

class Robot : public frc::SampleRobot {
  public:
      Robot();

      void RobotInit() override;
      void Autonomous() override;
      void OperatorControl() override;
      void Test() override;

  private:

      frc::SendableChooser<std::string> m_chooser;
      const std::string kAutoNameDefault = "Default";
      const std::string kAutoNameCustom = "My Auto";

    Segment trajStraight[1024];
    int lenghtStraight = 0;

};