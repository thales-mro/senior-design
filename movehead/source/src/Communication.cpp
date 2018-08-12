#include <Communication.hpp>
#include <stdlib.h>
#include <time.h>

using namespace std;

Communication::Communication(simxInt clientID) {
	this->clientID = clientID;

	//init joints ids for controlled robot
	for(int i = 0; i < 40; i++) {
	 	cout << robot[i];
	 	simxGetObjectHandle(clientID, robot[i].c_str(), &naoJointsIds0[i], simx_opmode_oneshot_wait);
	 	cout << " " << naoJointsIds0[i] << endl;
	}
	std::string ball = "Ball";
  simxGetObjectHandle(clientID, ball.c_str(), &ballId, simx_opmode_oneshot_wait);

	//for isolated robot
	std::string name = "NAO";
	simxGetObjectHandle(clientID, name.c_str(), &robotsIds[0], simx_opmode_oneshot_wait);

	/*
	//for entire team
	for(int i = 0; i < teamNames.size(); i++) {
		simxGetObjectHandle(clientID, teamNames.at(i).c_str(), &robotsIds[0], simx_opmode_oneshot_wait);
	}

	//for opp robots
	for(int i = 0; i < oppNames.size(); i++) {
		simxGetObjectHandle(clientID, oppNames.at(i).c_str(), &oppRobotsIds[0], simx_opmode_oneshot_wait);
	}*/

	//for isolated robot
	std::string head = "HeadPitch_link_respondable";
	simxGetObjectHandle(clientID, head.c_str(), &headsIds[0], simx_opmode_oneshot_wait);

	//for entire team
	/*for(int i = 0; i < headsNames.size(); i++) {
		simxGetObjectHandle(clientID, headsNames.at(i).c_str(), &headsIds[i], simx_opmode_oneshot_wait);
	}*/

	/*for(int i = 0; i < oppRobotsNames.size(); i++) {
		simxGetObjectHandle(clientID, oppRobotsNames.at(i).c_str(), &oppRobotsIds[i], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, heads.at(i).c_str(), &headsIds[i], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, robots.at(i).c_str(), &robotsIds[i], simx_opmode_oneshot_wait);
	}*/
	simxGetObjectHandle(clientID, ourGoalName.c_str(), &ourGoalId, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, oppGoalName.c_str(), &oppGoalId, simx_opmode_oneshot_wait);

}

//////////////////////////////////////////////////////////////////////////////////////////////////

void Communication::testConnectionVREP() {
	if(clientID != -1) {
		cout << "ClientID: " << clientID << endl;

		simxInt handle = -1;
		std::string name = "HeadYaw";
		simxGetObjectHandle(clientID, name.c_str(), &handle, simx_opmode_oneshot_wait);
		cout << "Handle: " << handle << endl;
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////

void Communication::testConnectionChoregraphe() {
	AL::ALMotionProxy m("127.0.0.1:39501", 9559);
	cout << "Entered Choregraphe routine" << endl;
	std::vector<float> listAngles;

	listAngles = motion.getAngles("Body", false);

	for(int i = 0; i < listAngles.size(); i++) {
		cout << "Angle " << i << ": " << listAngles.at(i) << endl;
	}

	AL::ALValue x = 0.2f;
	AL::ALValue y = 0.2f;
	AL::ALValue theta = 0.0f;

}

//////////////////////////////////////////////////////////////////////////////////////////////////

void Communication::updateJointsPositions() {
	AL::ALMotionProxy m("127.0.0.1:39501", 9559);
	cout << "Entered Choregraphe routine" << endl;
	std::vector<float> listAngles;
	motion.moveInit();
	sleep(5);
	m.moveToward(0.0f, 0.0f, 0.0f);
	bool first = true;
	while(true) {
		int result = 0;
		listAngles = m.getAngles("Body", false);
		simxSetJointTargetPosition(clientID, naoJointsIds0[2], listAngles.at(8), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[3], listAngles.at(9), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[4], listAngles.at(10), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[5], listAngles.at(11), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[6], listAngles.at(12), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[7], listAngles.at(13), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[8], listAngles.at(14), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[9], listAngles.at(15), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[10], listAngles.at(16), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[11], listAngles.at(17), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[12], listAngles.at(18), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[13], listAngles.at(19), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[14], listAngles.at(2), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[15], listAngles.at(3), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[16], listAngles.at(4), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[17], listAngles.at(5), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[18], listAngles.at(6), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[19], listAngles.at(20), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[20], listAngles.at(21), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[21], listAngles.at(22), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[22], listAngles.at(23), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[23], listAngles.at(24), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[24], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[25], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[26], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[27], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[28], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[29], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[30], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[31], 1.0f - listAngles.at(7), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[32], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[33], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[34], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[35], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[36], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[37], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[38], 1.0f - listAngles.at(25), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[39], 1.0f - listAngles.at(25), simx_opmode_streaming);
		if(first) {
			sleep(10);
			first = false;
			m.moveToward(0.5f, 0.0f, 0.5f);
		}

	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void Communication::startThread() {
	std::thread t1(&Communication::updateJointsPositions, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getBallCoords() {
	simxGetObjectPosition(clientID, ballId, -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		ballCoords.clear();
		for(simxFloat coords : aux)
			ballCoords.push_back(coords);
	}
	return ballCoords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot0Coords() {
	simxGetObjectPosition(clientID, robotsIds[0], -1, aux, simx_opmode_oneshot_wait);
	if(robot0Coords.size() == 0) {
		for(simxFloat coords : aux)
			robot0Coords.push_back(coords);
	}
	else if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		if(!(std::abs(robot0Coords.at(0) - aux[0]) >= 0.15 || std::abs(robot0Coords.at(1) - aux[1]) >= 0.15)) {
			robot0Coords.clear();
			for(simxFloat coords : aux)
				robot0Coords.push_back(coords);
		}
	}
	return robot0Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot1Coords() {
	simxGetObjectPosition(clientID, robotsIds[1], -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		if(!(std::abs(robot1Coords.at(0) - aux[0]) >= 0.15 || std::abs(robot1Coords.at(1) - aux[1]) >= 0.15)) {
			robot1Coords.clear();
			for(simxFloat coords : aux)
				robot1Coords.push_back(coords);
		}
	}
	return robot1Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot2Coords() {
	simxGetObjectPosition(clientID, robotsIds[2], -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot2Coords.clear();
		for(simxFloat coords : aux)
			robot2Coords.push_back(coords);
	}
	return robot2Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getOpp0Coords() {
	simxGetObjectPosition(clientID, oppRobotsIds[0], -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		opp0Coords.clear();
		for(simxFloat coords : aux)
			opp0Coords.push_back(coords);
	}
	return opp0Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getOpp1Coords() {
	simxGetObjectPosition(clientID, oppRobotsIds[1], -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		opp1Coords.clear();
		for(simxFloat coords : aux)
			opp1Coords.push_back(coords);
	}
	return opp1Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getOpp2Coords() {
	simxGetObjectPosition(clientID, oppRobotsIds[2], -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		opp2Coords.clear();
		for(simxFloat coords : aux)
			opp2Coords.push_back(coords);
	}
	return opp2Coords;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot0Orientation() {
	simxGetObjectOrientation(clientID, robotsIds[0], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot0Orientation.clear();
		for(simxFloat coords : aux)
			robot0Orientation.push_back(coords);
	}
	return robot0Orientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot1Orientation() {
	simxGetObjectOrientation(clientID, robotsIds[1], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot1Orientation.clear();
		for(simxFloat coords : aux)
			robot1Orientation.push_back(coords);
	}
	return robot1Orientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot2Orientation() {
	simxGetObjectOrientation(clientID, robotsIds[2], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot2Orientation.clear();
		for(simxFloat coords : aux)
			robot2Orientation.push_back(coords);
	}
	return robot2Orientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot0HeadOrientation() {
	simxGetObjectOrientation(clientID, headsIds[0], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot0HeadOrientation.clear();
		for(simxFloat coords : aux)
			robot0HeadOrientation.push_back(coords);
	}
	return robot0HeadOrientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot1HeadOrientation() {
	simxGetObjectOrientation(clientID, headsIds[1], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot1HeadOrientation.clear();
		for(simxFloat coords : aux)
			robot1HeadOrientation.push_back(coords);
	}
	return robot1HeadOrientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getRobot2HeadOrientation() {
	simxGetObjectOrientation(clientID, headsIds[2], -1, aux, simx_opmode_streaming);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		robot2HeadOrientation.clear();
		for(simxFloat coords : aux)
			robot2HeadOrientation.push_back(coords);
	}
	return robot2HeadOrientation;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getOurGoalCoords() {
	simxGetObjectPosition(clientID, ourGoalId, -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		ourGoalPosition.clear();
		for(simxFloat coords : aux)
			ourGoalPosition.push_back(coords);
	}
	return ourGoalPosition;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<simxFloat> Communication::getOppGoalCoords() {
	simxGetObjectPosition(clientID, oppGoalId, -1, aux, simx_opmode_oneshot_wait);
	if(!(aux[0] != aux[0] || aux[1] != aux[1] || aux[2] != aux[2])) {
		oppGoalPosition.clear();
		for(simxFloat coords : aux)
			oppGoalPosition.push_back(coords);
	}
	return oppGoalPosition;
}
