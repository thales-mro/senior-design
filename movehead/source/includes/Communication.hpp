#include <iostream>
#include <defines.hpp>
#include "Simulator.hpp"
#include "Exceptions/VRepException.hpp"
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <thread>


class Communication {

	public:
		Communication(simxInt clientID);
		void testConnectionVREP();
		void testConnectionChoregraphe();
		void updateJointsPositions();
		void startThread();

		std::vector<simxFloat> getBallCoords();
		std::vector<simxFloat> getRobot0Coords();
		std::vector<simxFloat> getRobot1Coords();
		std::vector<simxFloat> getRobot2Coords();
		std::vector<simxFloat> getOpp0Coords();
		std::vector<simxFloat> getOpp1Coords();
		std::vector<simxFloat> getOpp2Coords();
		std::vector<simxFloat> getRobot0Orientation();
		std::vector<simxFloat> getRobot1Orientation();
		std::vector<simxFloat> getRobot2Orientation();
		std::vector<simxFloat> getRobot0HeadOrientation();
		std::vector<simxFloat> getRobot1HeadOrientation();
		std::vector<simxFloat> getRobot2HeadOrientation();
		std::vector<simxFloat> getOurGoalCoords();
		std::vector<simxFloat> getOppGoalCoords();

	private:
		void updateVariables();
		simxInt clientID;
		AL::ALMotionProxy motion;
		simxInt naoJointsIds0[41], naoJointsIds1[41], naoJointsIds2[41], oppRobotsIds[3], headsIds[3], robotsIds[3], ourGoalId, oppGoalId;
		simxInt ballId;
		simxFloat aux[3];
		std::vector<simxFloat> ballCoords;
		std::vector<simxFloat> robot0Coords;
		std::vector<simxFloat> robot1Coords;
		std::vector<simxFloat> robot2Coords;
		std::vector<simxFloat> opp0Coords;
		std::vector<simxFloat> opp1Coords;
		std::vector<simxFloat> opp2Coords;
		std::vector<simxFloat> robot0Orientation;
		std::vector<simxFloat> robot1Orientation;
		std::vector<simxFloat> robot2Orientation;
		std::vector<simxFloat> robot0HeadOrientation;
		std::vector<simxFloat> robot1HeadOrientation;
		std::vector<simxFloat> robot2HeadOrientation;
		std::vector<simxFloat> ourGoalPosition;
		std::vector<simxFloat> oppGoalPosition;

		std::vector<std::string> headsNames{"HeadPitch_link_respondable#0", "HeadPitch_link_respondable#1", "HeadPitch_link_respondable#2"};

		std::vector<std::string> teamNames{"NAO#0", "NAO#1", "NAO#2"};

		std::vector<std::string> oppNames{"NAO#3", "NAO#4", "NAO#5"};

		std::vector<std::string> robot{"HeadYaw", "HeadPitch", "LHipYawPitch3", "LHipRoll3", "LHipPitch3", "LKneePitch3", "LAnklePitch3", "LAnkleRoll3", "RHipYawPitch3", "RHipRoll3", "RHipPitch3", "RKneePitch3", "RAnklePitch3", "RAnkleRoll3", "LShoulderPitch3", "LShoulderRoll3", "LElbowYaw3", "LElbowRoll3", "LWristYaw3", "RShoulderPitch3", "RShoulderRoll3", "RElbowYaw3",  "RElbowRoll3", "RWristYaw3", "NAO_LThumbBase","Revolute_joint8","NAO_LLFingerBase","Revolute_joint12", "Revolute_joint14", "NAO_LRFingerBase", "Revolute_joint11", "Revolute_joint13", "NAO_RThumbBase", "Revolute_joint0", "NAO_RLFingerBase", "Revolute_joint5", "Revolute_joint6", "NAO_RRFingerBase", "Revolute_joint2", "Revolute_joint3"};

		std::vector<std::string> robot0{"HeadYaw#0", "HeadPitch#0", "LHipYawPitch3#0", "LHipRoll3#0", "LHipPitch3#0", "LKneePitch3#0", "LAnklePitch3#0", "LAnkleRoll3#0", "RHipYawPitch3#0", "RHipRoll3#0", "RHipPitch3#0", "RKneePitch3#0", "RAnklePitch3#0", "RAnkleRoll3#0", "LShoulderPitch3#0", "LShoulderRoll3#0", "LElbowYaw3#0", "LElbowRoll3#0", "LWristYaw3#0", "RShoulderPitch3#0", "RShoulderRoll3#0", "RElbowYaw3#0",  "RElbowRoll3#0", "RWristYaw3#0", "NAO_LThumbBase#0","Revolute_joint8#0","NAO_LLFingerBase#0","Revolute_joint12#0", "Revolute_joint14#0", "NAO_LRFingerBase#0", "Revolute_joint11#0", "Revolute_joint13#0", "NAO_RThumbBase#0", "Revolute_joint0#0", "NAO_RLFingerBase#0", "Revolute_joint5#0", "Revolute_joint6#0", "NAO_RRFingerBase#0", "Revolute_joint2#0", "Revolute_joint3#0"};

		std::string oppGoalName = "Goal_right";
		std::string ourGoalName = "Goal_left";
};
