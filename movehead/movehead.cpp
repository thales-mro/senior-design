/**
* Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
*
* \file movehead.cpp
* \brief Move NAO's head.
*
* A simple example showing how to move NAO's head by using ALMotionProxy.
* This example will make NAO turn its head left and right slowly.
* We use here a specialized proxy to ALMotion.
*/
#define _GLIBCXX_USE_CXX11_ABI 0
#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <stdlib.h>
#include <time.h>

#include <defines.hpp>
#include "Simulator.hpp"
#include "teste.hpp"
#include "Exceptions/VRepException.hpp"
#include <iomanip>
#include <unistd.h>
#include <random>
#include <boost/tuple/tuple.hpp>
#include "/home/thales/gnuplot-iostream/gnuplot-iostream.h"
#include <math.h>
#include <cmath>
#include <string>
#include "fl/Headers.h"

using namespace std;
using namespace fl;

int main(int argc, char* argv[]) {

  Gnuplot gp;
  
  fl::Engine *engine = new fl::Engine;
	// InputVariable *ballAngle = new InputVariable;
	// InputVariable *ballDistance = new InputVariable;
	// InputVariable *ballConfidence = new InputVariable;

  simxInt clientID = simxStart("127.0.0.1", PORT_NUMBER, TRUE, TRUE, 2000, 5);
  if(clientID != -1) {
		cout << "Deu certo, porra: " << clientID << endl;
		simxInt handle = -1;
		std::string name = "HeadYaw";
		simxGetObjectHandle(clientID, name.c_str(), &handle, simx_opmode_oneshot_wait);
		cout << "Sera?: " << handle << endl;
	}

  if(argc != 2)
  {
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: movehead NAO_IP" << std::endl;
    exit(2);
  }

  /** The name of the joint to be moved. */
  //const AL::ALValue jointName = "HeadYaw";


  try {
    /** Create a ALMotionProxy to call the methods to move NAO's head.
    * Arguments for the constructor are:
    * - IP adress of the robot
    * - port on which NAOqi is listening, by default 9559
    */
    AL::ALMotionProxy motion("127.0.0.1:33375", 9559);

    std::vector<float> listangles;
    std::vector<float> parallel;

    AL::ALValue x = 1.0f;
    AL::ALValue y = 0.0f;
    AL::ALValue theta = 0.5f;

    //motion.moveToward(0.0f, 0.0f, 0.0f);
    listangles = motion.getAngles("Body", false);

    for(int i = 0; i < listangles.size(); i++) {
      cout << "Angle " << i << ": " << listangles.at(i) << endl;
    }

    motion.moveInit();
    motion.moveToward(x, y, theta);

    //sleep(10);
    while(true) {
      parallel = motion.getAngles("Body", false);
      for(int i = 0; i < listangles.size(); i++) {
        //if(parallel.at(i) != listangles.at(i))
          //cout << "Angle " << i << ": " << parallel.at(i) << endl;
      }
      listangles = parallel;
    }

    //motion.moveToward(0.0f, 0.0f, 0.0f);

    /** Make sure the head is stiff to be able to move it.
    * To do so, make the stiffness go to the maximum in one second.
    */

    /** Target stiffness. */
    //AL::ALValue stiffness = 1.0f;

    /** Time (in seconds) to reach the target. */
    //AL::ALValue time = 1.0f;

    /** Call the stiffness interpolation method. */
    //motion.stiffnessInterpolation(jointName, stiffness, time);

    /** Set the target angle list, in radians. */
    //AL::ALValue targetAngles = AL::ALValue::array(-1.5f, 1.5f, 0.0f);

    /** Set the corresponding time lists, in seconds. */
    //AL::ALValue targetTimes = AL::ALValue::array(3.0f, 6.0f, 9.0f);

    /** Specify that the desired angles are absolute. */
    //bool isAbsolute = true;

    /** Call the angle interpolation method. The joint will reach the
    * desired angles at the desired times.
    */
    //motion.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);

    /** Remove the stiffness on the head. */
    //stiffness = 0.0f;
    //time = 1.0f;
    //motion.stiffnessInterpolation(jointName, stiffness, time);

  }
  catch (const AL::ALError& e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(1);
  }
  exit(0);
}
