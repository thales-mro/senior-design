//
// Created by previato on 02/04/17.
//

#ifndef ROBOT2_SIMULATOR_HPP
#define ROBOT2_SIMULATOR_HPP

#include <string>
#include <vector>
#include <iostream>

#include <vRepIncludes.hpp>

class Simulator {
private:
    const simxChar* ip;
    simxInt portNumber;
    simxInt clientID;

public:
    Simulator(const simxChar* ip, int port);

    void connectServer();

    void disconnectServer() const;

    void pauseServer() const;

    simxInt getHandle(const std::string name) const;

    simxUChar readProximitySensor(const simxInt sensorHandle, std::vector<simxFloat> &read, const int initial) const;

    void setJointVelocity(const simxInt jointHandle, simxFloat velocity) const;

    void getObjectPosition(const simxInt handle, simxFloat* coord) const;

    void getObjectOrientation(const simxInt sensorHandle, simxFloat* coord) const;

    void getJointPosition(const simxInt jointHandle, simxFloat* coord) const;

    simxInt getVisionSensorImage(const simxInt visionSensorHandle, simxInt* resolution, simxUChar** buffer) const;

    simxInt getDistanceHandle(const std::string name) const;

    simxFloat readDistanceObject(const simxInt distanceHandle) const;

    void setObjectPosition(const simxInt objectHandle, const simxFloat coord[3]) const;

    void setObjectOrientation(const simxInt objectHandle, const simxFloat angles[3]) const;

    simxInt getClientID() const;

    void reConnectServer();

};


#endif //ROBOT2_SIMULATOR_HPP
