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

		simxFloat getBallX() {return ballCoords[0];}
		simxFloat getBallY() {return ballCoords[1];}
		simxFloat getRobot0X() {return robot0Coords[0];}
		simxFloat getRobot1X() {return robot1Coords[0];}
		simxFloat getRobot2X() {return robot2Coords[0];}
		simxFloat getRobot0Y() {return robot0Coords[1];}
		simxFloat getRobot1Y() {return robot1Coords[1];}
		simxFloat getRobot2Y() {return robot2Coords[1];}
		simxFloat getRobot0Z() {return robot0Orient[2];}
		simxFloat getRobot1Z() {return robot1Orient[2];}
		simxFloat getRobot2Z() {return robot2Orient[2];}
		simxFloat getRobot0HeadZ() {return robot0HeadOrient[2];}
		simxFloat getRobot1HeadZ() {return robot1HeadOrient[2];}
		simxFloat getRobot2HeadZ() {return robot2HeadOrient[2];}
		simxFloat getOpp0X() {return opp0Coords[0];}
		simxFloat getOpp1X() {return opp1Coords[0];}
		simxFloat getOpp2X() {return opp2Coords[0];}
		simxFloat getOpp0Y() {return opp0Coords[1];}
		simxFloat getOpp1Y() {return opp1Coords[1];}
		simxFloat getOpp2Y() {return opp2Coords[1];}
		simxFloat getOpp0Z() {return opp0Orient[2];}
		simxFloat getOpp1Z() {return opp1Orient[2];}
		simxFloat getOpp2Z() {return opp2Orient[2];}
	private:
		void updateVariables();
		simxInt clientID;
		AL::ALMotionProxy motion;
		simxInt naoJointsIds0[41], naoJointsIds1[41], naoJointsIds2[41], oppRobotsIds[3], headsIds[3], robotsIds[3];
		simxInt ballId;
		simxFloat ballCoords[3], robot0Coords[3], robot1Coords[3], robot2Coords[3], robot0Orient[3], robot1Orient[3], robot2Orient[3], opp0Coords[3], opp1Coords[3], opp2Coords[3], opp0Orient[3], opp1Orient[3], opp2Orient[3], robot0HeadOrient[3], robot1HeadOrient[3], robot2HeadOrient[3];
		std::vector<std::string> robot0{"HeadYaw#0", "HeadPitch#0", "LHipYawPitch3#0", "LHipRoll3#0", "LHipPitch3#0", "LKneePitch3#0", "LAnklePitch3#0", "LAnkleRoll3#0", "RHipYawPitch3#0", "RHipRoll3#0", "RHipPitch3#0", "RKneePitch3#0", "RAnklePitch3#0", "RAnkleRoll3#0", "LShoulderPitch3#0", "LShoulderRoll3#0", "LElbowYaw3#0", "LElbowRoll3#0", "LWristYaw3#0", "RShoulderPitch3#0", "RShoulderRoll3#0", "RElbowYaw3#0",  "RElbowRoll3#0", "RWristYaw3#0", "NAO_LThumbBase#0","Revolute_joint8#0","NAO_LLFingerBase#0","Revolute_joint12#0", "Revolute_joint14#0", "NAO_LRFinger_Base#0", "Revolute_joint11#0", "Revolute_joint13#0", "NAO_RThumbBase#0", "Revolute_joint0#0", "NAO_RLFingerBase#0", "Revolute_joint5#0", "Revolute_joint6#0", "NAO_RRFinger_Base#0", "Revolute_joint2#0", "Revolute_joint3#0"};
		std::vector<std::string> robot1{"HeadYaw#", "HeadPitch#1", "LHipYawPitch3#1", "LHipRoll3#1", "LHipPitch3#1", "LKneePitch3#1", "LAnklePitch3#1", "LAnkleRoll3#1", "RHipYawPitch3#1", "RHipRoll3#1", "RHipPitch3#1", "RKneePitch3#1", "RAnklePitch3#1", "RAnkleRoll3#1", "LShoulderPitch3#1", "LShoulderRoll3#1", "LElbowYaw3#1", "LElbowRoll3#1", "LWristYaw3#1", "RShoulderPitch3#1", "RShoulderRoll3#1", "RElbowYaw3#1",  "RElbowRoll3#1", "RWristYaw3#1", "NAO_LThumbBase#1","Revolute_joint8#1","NAO_LLFingerBase#1","Revolute_joint12#1", "Revolute_joint14#1", "NAO_LRFinger_Base#1", "Revolute_joint11#1", "Revolute_joint13#1", "NAO_RThumbBase#1", "Revolute_joint0#1", "NAO_RLFingerBase#1", "Revolute_joint5#1", "Revolute_joint6#1", "NAO_RRFinger_Base#1", "Revolute_joint2#1", "Revolute_joint3#1"};
		std::vector<std::string> robot2{"HeadYaw#2", "HeadPitch#2", "LHipYawPitch3#2", "LHipRoll3#2", "LHipPitch3#2", "LKneePitch3#2", "LAnklePitch3#2", "LAnkleRoll3#2", "RHipYawPitch3#2", "RHipRoll3#2", "RHipPitch3#2", "RKneePitch3#2", "RAnklePitch3#2", "RAnkleRoll3#2", "LShoulderPitch3#2", "LShoulderRoll3#2", "LElbowYaw3#2", "LElbowRoll3#2", "LWristYaw3#2", "RShoulderPitch3#2", "RShoulderRoll3#2", "RElbowYaw3#2",  "RElbowRoll3#2", "RWristYaw3#2", "NAO_LThumbBase#2","Revolute_joint8#2","NAO_LLFingerBase#2","Revolute_joint12#2", "Revolute_joint14#2", "NAO_LRFinger_Base#2", "Revolute_joint11#2", "Revolute_joint13#2", "NAO_RThumbBase#2", "Revolute_joint0#2", "NAO_RLFingerBase#2", "Revolute_joint5#2", "Revolute_joint6#2", "NAO_RRFinger_Base#2", "Revolute_joint2#2", "Revolute_joint3#2"};
		std::vector<std::string> oppRobotsNames{"NAO#3", "NAO#4", "NAO#5"};
		std::vector<std::string> heads{"HeadPitch_link_respondable#0", "HeadPitch_link_respondable#1", "HeadPitch_link_respondable#2"};
		std::vector<std::string> robots{"NAO#0", "NAO#1", "NAO#2"};


};
