#include <Communication.hpp>

using namespace std;

Communication::Communication(simxInt clientID) {
	this->clientID = clientID;
	cout << "I`m alive" << endl;

	for(int i = 0; i < 24; i++) {
		simxGetObjectHandle(clientID, robot0[i].c_str(), &naoJointsIds0[i], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, robot1[i].c_str(), &naoJointsIds1[i], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, robot2[i].c_str(), &naoJointsIds2[i], simx_opmode_oneshot_wait);
	}
	std::string ball = "Ball";
	simxGetObjectHandle(clientID, ball.c_str(), &ballId, simx_opmode_oneshot_wait);
	for(int i = 0; i < oppRobotsNames.size(); i++) {
		simxGetObjectHandle(clientID, oppRobotsNames.at(i).c_str(), &oppRobotsIds[i], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, heads.at(i).c_str(), &headsIds[i], simx_opmode_oneshot_wait);
	}

	updateVariables();

}


void Communication::testConnectionVREP() {
	if(clientID != -1) {
		cout << "ClientID: " << clientID << endl;

		simxInt handle = -1;
		std::string name = "HeadYaw";
		simxGetObjectHandle(clientID, name.c_str(), &handle, simx_opmode_oneshot_wait);
		cout << "Handle: " << handle << endl;
	}

}

void Communication::testConnectionChoregraphe() {
	AL::ALMotionProxy m("127.0.0.1:33375", 9559);
	cout << "Entered Choregraphe routine" << endl;
	std::vector<float> listAngles;

	listAngles = motion.getAngles("Body", false);

	for(int i = 0; i < listAngles.size(); i++) {
		cout << "Angle " << i << ": " << listAngles.at(i) << endl;
	}

	AL::ALValue x = 0.0f;
	AL::ALValue y = 1.0f;
	AL::ALValue theta = 0.0f;
	motion.moveToward(x, y, theta);

}

void Communication::updateVariables() {
	simxGetObjectPosition(clientID, ballId, -1, ballCoords, simx_opmode_streaming);
	simxGetObjectPosition(clientID, headsIds[0], -1, robot0Coords, simx_opmode_streaming);
	simxGetObjectPosition(clientID, headsIds[1], -1, robot1Coords, simx_opmode_streaming);
	simxGetObjectPosition(clientID, headsIds[2], -1, robot2Coords, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, headsIds[0], -1, robot0Orient, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, headsIds[1], -1, robot1Orient, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, headsIds[2], -1, robot2Orient, simx_opmode_streaming);
	simxGetObjectPosition(clientID, oppRobotsIds[0], -1, opp0Coords, simx_opmode_streaming);
	simxGetObjectPosition(clientID, oppRobotsIds[1], -1, opp1Coords, simx_opmode_streaming);
	simxGetObjectPosition(clientID, oppRobotsIds[2], -1, opp2Coords, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, oppRobotsIds[0], -1, opp0Orient, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, oppRobotsIds[1], -1, opp1Orient, simx_opmode_streaming);
	simxGetObjectOrientation(clientID, oppRobotsIds[2], -1, opp2Orient, simx_opmode_streaming);
}


void Communication::updateJointsPositions() {
	AL::ALMotionProxy m("127.0.0.1:33375", 9559);
	cout << "Entered Choregraphe routine" << endl;
	std::vector<float> listAngles;
	while(true) {
		updateVariables();
		//cout << "Hello, deepest fears" << endl;
		listAngles = m.getAngles("Body", false);
		//simxSetJointTargetPosition(clientID, naoJointsIds0[0], listAngles.at(0), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds0[1], listAngles.at(1), simx_opmode_streaming);
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
	}
	cout << "I`m about to dieee" << endl;
}

void Communication::startThread() {
	std::thread t1(&Communication::updateJointsPositions, this);
}
