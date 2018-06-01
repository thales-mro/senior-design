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
	private:
		simxInt clientID;
		AL::ALMotionProxy motion;
		simxInt naoJointsIds[41];
		
};
