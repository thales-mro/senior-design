#include <Communication.hpp>

using namespace std;

Communication::Communication(simxInt clientID) {
	this->clientID = clientID;
	cout << "I`m alive" << endl;
	std::string name = "HeadYaw#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[0], simx_opmode_oneshot_wait);
	name = "HeadPitch#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[1], simx_opmode_oneshot_wait);
	name = "LHipYawPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[2], simx_opmode_oneshot_wait);
	name = "LHipRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[3], simx_opmode_oneshot_wait);
	name = "LHipPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[4], simx_opmode_oneshot_wait);
	name = "LKneePitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[5], simx_opmode_oneshot_wait);
	name = "LAnklePitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[6], simx_opmode_oneshot_wait);
	name = "LAnkleRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[7], simx_opmode_oneshot_wait);
	name = "RHipYawPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[8], simx_opmode_oneshot_wait);
	name = "RHipRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[9], simx_opmode_oneshot_wait);
	name = "RHipPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[10], simx_opmode_oneshot_wait);
	name = "RKneePitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[11], simx_opmode_oneshot_wait);
	name = "RAnklePitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[12], simx_opmode_oneshot_wait);
	name = "RAnkleRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[13], simx_opmode_oneshot_wait);
	name = "LShoulderPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[14], simx_opmode_oneshot_wait);
	name = "LShoulderRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[15], simx_opmode_oneshot_wait);
	name = "LElbowYaw3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[16], simx_opmode_oneshot_wait);
	name = "LElbowRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[17], simx_opmode_oneshot_wait);
	name = "LWristYaw3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[18], simx_opmode_oneshot_wait);
	name = "RShoulderPitch3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[19], simx_opmode_oneshot_wait);
	name = "RShoulderRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[20], simx_opmode_oneshot_wait);
	name = "RElbowYaw3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[21], simx_opmode_oneshot_wait);
	name = "RElbowRoll3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[22], simx_opmode_oneshot_wait);
	name = "RWristYaw3#0";
	simxGetObjectHandle(clientID, name.c_str(), &naoJointsIds[23], simx_opmode_oneshot_wait);
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

void Communication::updateJointsPositions() {
	AL::ALMotionProxy m("127.0.0.1:33375", 9559);
	cout << "Entered Choregraphe routine" << endl;
	std::vector<float> listAngles;
	while(true) {
		//cout << "Hello, deepest fears" << endl;
		listAngles = m.getAngles("Body", false);
		simxSetJointTargetPosition(clientID, naoJointsIds[0], listAngles.at(0), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[1], listAngles.at(1), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[2], listAngles.at(8), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[3], listAngles.at(9), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[4], listAngles.at(10), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[5], listAngles.at(11), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[6], listAngles.at(12), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[7], listAngles.at(13), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[8], listAngles.at(14), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[9], listAngles.at(15), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[10], listAngles.at(16), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[11], listAngles.at(17), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[12], listAngles.at(18), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[13], listAngles.at(19), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[14], listAngles.at(2), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[15], listAngles.at(3), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[16], listAngles.at(4), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[17], listAngles.at(5), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[18], listAngles.at(6), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[19], listAngles.at(20), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[20], listAngles.at(21), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[21], listAngles.at(22), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[22], listAngles.at(23), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID, naoJointsIds[23], listAngles.at(24), simx_opmode_streaming);
	}
	cout << "I`m about to dieee" << endl;
}

void Communication::startThread() {
	std::thread t1(&Communication::updateJointsPositions, this);
}
