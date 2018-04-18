//
// Created by previato on 02/04/17.
//

#include "Simulator.hpp"
#include <defines.hpp>
#include "Exceptions/VRepException.hpp"

using std::cout;
using std::endl;

Simulator::Simulator(const simxChar* ip, int port) {
    this->clientID = -1;
    this->ip = ip;
    this->portNumber = port;
}

void Simulator::connectServer() {
    if (clientID == -1) {
        clientID = simxStart(ip, portNumber, TRUE, TRUE, 2000, 5);
        if (clientID != -1) {
            cout << "Connected to server..." << endl;
            return;
        }
    }

    throw VRepException("Connect attempt with server failed...");
}

void Simulator::disconnectServer() const {
    if (clientID != -1) {
        cout << "Disconnected from server..." << endl;
        simxFinish(clientID);
        return;
    }
    throw VRepException("Disconnect attempt with server failed...");
}

simxUChar Simulator::readProximitySensor(const simxInt sensorHandle, std::vector<simxFloat> &read, const int initial) const {
    simxUChar state;
    simxFloat reads[3];
    if (clientID != -1) {
        if (simxReadProximitySensor(clientID, sensorHandle, &state, reads, nullptr, nullptr, simx_opmode_oneshot_wait)
            == simx_return_ok) {

            if (state) {
                for (int i = 0; i < 3; i++) {
                    read[initial + i] = reads[i];
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    read[initial + i] = -1;
                }
            }

            return state;
        }

        throw VRepException("Unable to connect with sensor...");

    }

    throw VRepException("Unable to get server connection...");
}

simxInt Simulator::getHandle(const std::string name) const {
    simxInt handle = -1;
    if (clientID != -1) {
        if (simxGetObjectHandle(clientID, name.c_str(), &handle, simx_opmode_oneshot_wait) != simx_return_ok) {
            throw VRepException("Attempt to get Handle failed...");
        }

        return handle;
    }

    throw VRepException("Unable to get server connection...");
}

void Simulator::setJointVelocity(const simxInt jointHandle, simxFloat velocity) const {
    if (clientID != -1) {
        if (simxSetJointTargetVelocity(clientID, jointHandle, velocity, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to connect with joint...");
    }

    throw VRepException("Unable to get server connection...");
}


void Simulator::getObjectOrientation(simxInt handle, simxFloat* coord) const {
    if (clientID != -1) {
        if (simxGetObjectOrientation(clientID, handle, -1, coord, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to connect with object...");

    }
}

void Simulator::getJointPosition(simxInt jointHandle, simxFloat* coord) const {
    if (clientID != -1) {
        if (simxGetJointPosition(clientID, jointHandle, coord, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to connect with joint...");
    }
}

void Simulator::getObjectPosition(const simxInt handle, simxFloat* coord) const {
    if (clientID != -1) {
        if (simxGetObjectPosition(clientID, handle, -1, coord, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to connect with object...");
    }

    throw VRepException("Unable to get server connection...");
}

simxInt Simulator::getVisionSensorImage(const simxInt visionSensorHandle, simxInt *resolution, simxUChar **buffer) const {
    if (clientID != -1) {
        static int firstCall = 0;

        simxInt ret;

        if (firstCall){
            ret = simxGetVisionSensorImage(clientID, visionSensorHandle, resolution, buffer, 0, simx_opmode_streaming);
            if (ret == simx_return_ok || ret == simx_return_novalue_flag) {
            return ret;
            }

            throw VRepException("Unable to connect with vision sensor...");
        }
        else{
            ret = simxGetVisionSensorImage(clientID, visionSensorHandle, resolution, buffer, 0, simx_opmode_buffer);
            if (ret == simx_return_ok || ret == simx_return_novalue_flag) {
                firstCall = 1;
                return ret;
            }

            throw VRepException("Unable to connect with vision sensor...");
        }
    }

    throw VRepException("Unable to get server connection...");
}

simxInt Simulator::getClientID() const {
    return clientID;
}

simxInt Simulator::getDistanceHandle(const std::string name) const {
    simxInt handle;

    if (clientID != -1) {
        if (simxGetDistanceHandle(clientID, name.c_str(), &handle, simx_opmode_blocking) == simx_return_ok) {
            return handle;
        }

        throw VRepException("Unable to connect with distance object...");
    }

    throw VRepException("Unable to get server connection...");
}

simxFloat Simulator::readDistanceObject(const simxInt distanceHandle) const {
    simxFloat minDistance;

    if (clientID != -1) {
        if (simxReadDistance(clientID, distanceHandle, &minDistance, simx_opmode_oneshot_wait) == simx_return_ok) {
            return minDistance;
        }

        throw VRepException("Unable to connect with distance object...");
    }

    throw VRepException("Unable to get server connection...");
}

void Simulator::setObjectPosition(const simxInt objectHandle, const simxFloat coord[3]) const {
    if (clientID != -1) {
        if (simxSetObjectPosition(clientID, objectHandle, objectHandle, coord, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to change object distance...");
    }

    throw VRepException("Unable to get server connection...");
}

void Simulator::setObjectOrientation(const simxInt objectHandle, const simxFloat angles[3]) const {
    if (clientID != -1) {
        if (simxSetObjectOrientation(clientID, objectHandle, objectHandle, angles, simx_opmode_oneshot_wait) == simx_return_ok) {
            return;
        }

        throw VRepException("Unable to change object distance...");
    }

    throw VRepException("Unable to get server connection...");
}

void Simulator::pauseServer() const {
    if (clientID != -1) {
        simxPauseSimulation(clientID, simx_opmode_oneshot);
        return;
    }
    throw VRepException("Unable to get server connection...");
}
