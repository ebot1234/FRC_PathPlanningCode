#ifdef SRC_INCLUDE_PATH_H
#define SRC_INCLUDE_PATH_H

#include "ctre/Phoenix"
#include <WPILib.h>
#include <pathfinder.h>

class Path {
    public:
        Path();
        vitrual ~Path();
        void Generate();
        bool FollowPath();
        void ConfigureEncoders();
        void StopFollowing();

    private:
        int POINT_LENGTH = 2;
        const double TIME_STEP = 0.02;
        const double MAX_VEL = 18;
        const double MAX_ACCEL = 12;
        const double MAX_JERK = 60;
        const int TICKS_PER_REV = 26214;
        const double Wheel_Cir = 0.5;
        const double K_P = 1;
        const double K_I = 0.0;
        const double K_D = .15;
        const double K_V = .06;
        const double K_A = 0.0856;
        const double K_T = .35;
        double WHEEL_WIDTH;
        double WHEEL_LENGTH;
        TrajectoryCandidate candidate;
        Segment* trajectory;
        Segment* leftTraj;
        Segment* rightTraj;
        int length;
        EncoderFollower* leftFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
        EncoderFollower* rightFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
        EncoderConfig leftConfig;
        EncoderConfig rightConfig;

};
#endif