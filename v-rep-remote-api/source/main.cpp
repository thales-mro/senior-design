#include <defines.hpp>
#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Utils.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <math.h>
#include "Simulator.hpp"
#include <cmath>
#include <iomanip>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <unistd.h>

#define MAX_X  6.0
#define MAX_Y  4.0
#define SCALE  0.25
#define MAP_X 48
#define MAP_Y 32

#define FIELD_X  4.5
#define FIELD_Y 3.0

#define TEAM_ROBOT 1
#define OPP_ROBOT 2
#define BALL 4
#define TEAM_GOAL 8
#define OPP_GOAL 9
#define FIELD_LIMITS 16

#define MAX_DISTANCE 4.5
#define GOAL_SIZE 1.50
#define N_MAP_ELEMENTS 6


void printInfo(std::string name, simxInt id, simxFloat coord[]) {
	std::cout  << name <<  ":\n\tid: " << id << "\n\tposition:\n\t\tx: " << coord[0]
			<< "\n\t\ty: " << coord[1] << "\n\t\tz: " << coord[2] << std::endl;
}

void printMapGroundTruth(int map[][MAP_Y][N_MAP_ELEMENTS]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++ ) {
			if(map[i][j] == 0) {
				std::cout << "-" << " ";
			}
			else {
				int sum = 0;
				for(int k = 0; k < N_MAP_ELEMENTS; k++) {	
					sum += map[i][j][k];			
				}
				std::cout << std::setw(2) << sum << " ";

				
			}
		}
		std::cout << std::endl;
	}
}


void printMap(int map[][MAP_Y][N_MAP_ELEMENTS],  float occupancy[][MAP_Y]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++ ) {
			if(occupancy[i][j] < 0.25) {
				std::cout << std::setw(2) << "-" << " ";
			}
			else {
				int sum = 0;
				for(int k = 0; k < N_MAP_ELEMENTS; k++) {	
					sum += map[i][j][k];			
				}
				std::cout << std::setw(2) << sum << " ";

			}
		}
		std::cout << std::endl;
	}
}

int convertX(simxFloat x) {
	int x_pos = (x + MAX_X)/SCALE;
	return x_pos;
}

int convertY(simxFloat y) { 
	int y_pos = (y + MAX_Y)/SCALE;
	return y_pos;
}

float xCoordToMeters(int x) {
	return (x*SCALE) - MAX_X;
}


float yCoordToMeters(int y) {
	return (y*SCALE) - MAX_Y;
}


void updateMap(int map[][MAP_Y][N_MAP_ELEMENTS], simxFloat coord[], int type) {
	int x = convertX(coord[0]);
	int y = convertY(coord[1]);
	switch(type) {
		case TEAM_ROBOT:
			map[x][y][0] = TEAM_ROBOT;
			break;
		case OPP_ROBOT:
			map[x][y][1] = OPP_ROBOT;
			break;
		case BALL:
			map[x][y][2] = BALL;
			break;
		case TEAM_GOAL:
			map[x][y][3] = TEAM_GOAL;
			break;
		case OPP_GOAL:
			map[x][y][4] = OPP_GOAL;
			break;
		case FIELD_LIMITS:
			map[x][y][5] = FIELD_LIMITS;
			break;
	}
}


void updateOccupancyGrid(float m[][MAP_Y], simxFloat robot[], simxFloat orientation[]) {
	simxFloat x_robot = robot[0];
	simxFloat y_robot = robot[1];
	simxFloat z_rotation = orientation[2];
	//if(orientation[2] < 0) 
		//z_rotation += 2*M_PI;
	simxFloat sin, cos, dist, x, y, diffX, diffY;
	for(int i = 0; i < MAP_X; i++) {
		x = xCoordToMeters(i);
		diffX = x - x_robot;
		for(int j = 0; j < MAP_Y; j++) {
			m[i][j] = (m[i][j] > 0)?m[i][j] - m[i][j]*0.05 : 0;
			y = yCoordToMeters(j);
			diffY = y - y_robot;
			dist = sqrt(pow(diffX, 2) + pow(diffY, 2));
			if (dist < MAX_DISTANCE) {
				simxFloat arcTg = atan2((double)diffY,(double)diffX);
				simxFloat arcSin = asin((double)diffY/ (double)dist);				
				//if((std::abs(z_rotation) - (M_PI/6) < std::abs(arcTg)) && (std::abs(z_rotation) + (M_PI/6) > std::abs(arcTg))) {	
				if(z_rotation - (M_PI/6) < arcTg) {
					//m[i][j] = 9;
					if(z_rotation + (M_PI/6) > arcTg) {

				 		m[i][j] = 1;
					}
				}
			}  
		}
	}
	m[convertX(x_robot)][convertY(y_robot)] = 1;
}


void printOccupancyGrid(float occupancy[][MAP_Y]) {
	std::cout << "Occupancy grid" << std::endl;
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++) {
			std::cout <<std::setw(2) << occupancy[i][j] <<  " ";
		}
		std::cout << std::endl;
	}
}


void resetOccupancyGrid(float m[][MAP_Y]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++) {
				m[i][j] = 0;
		}
	}
}

void resetMap(int m[][MAP_Y][N_MAP_ELEMENTS]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++ ) {
			for(int k = 0; k < N_MAP_ELEMENTS; k++)
				m[i][j][k] = 0;
		}
	}
}

int main(int argc, char** argv) {
	// First, we must connect to the v-rep simulator server
	// For local connection, the IP is "127.0.0.1"

	auto sim = new Simulator("127.0.0.1", PORT_NUMBER);
	Simulator &vrep = *sim;
	vrep.connectServer();	

	float max_x = 5.5;
	float max_y = 4.0;

	int map[MAP_X][MAP_Y][N_MAP_ELEMENTS];	
	float occupancy[MAP_X][MAP_Y];
	
	resetMap(map);
	resetOccupancyGrid(occupancy);
	
	
	simxInt self_id, robot0_id, robot1_id, robot2_id, robot3_id, robot4_id, robot5_id, robot6_id;
	simxInt ball_id, goal_left_id, goal_right_id;

	simxFloat self_coord[3], robot0_coord[3], robot1_coord[3], robot2_coord[3], robot3_coord[3], robot4_coord[3], robot5_coord[3], robot6_coord[3];
	simxFloat ball_coord[3], goal_left_coord[3], goal_right_coord[3];

	simxFloat self_orientation[3], robot5_orientation[3], robot6_orientation[3];

	self_id = vrep.getHandle("NAO");
	robot0_id = vrep.getHandle("NAO#0");
	robot1_id = vrep.getHandle("NAO#1");
	robot2_id = vrep.getHandle("NAO#2");
	robot3_id = vrep.getHandle("NAO#3");
	robot4_id = vrep.getHandle("NAO#4");
	robot5_id = vrep.getHandle("NAO#5");
	robot6_id = vrep.getHandle("NAO#6");
	ball_id = vrep.getHandle("Ball");
	goal_left_id = vrep.getHandle("Goal_left");
	goal_right_id = vrep.getHandle("Goal_right");

	vrep.getObjectPosition(self_id, self_coord);
	vrep.getObjectPosition(robot0_id, robot0_coord);
	vrep.getObjectPosition(robot1_id, robot1_coord);
	vrep.getObjectPosition(robot2_id, robot2_coord);
	vrep.getObjectPosition(robot3_id, robot3_coord);
	vrep.getObjectPosition(robot4_id, robot4_coord);
	vrep.getObjectPosition(robot5_id, robot5_coord);
	vrep.getObjectPosition(ball_id, ball_coord);
	vrep.getObjectPosition(goal_left_id, goal_left_coord);
	vrep.getObjectPosition(goal_right_id, goal_right_coord);
	vrep.getObjectPosition(robot6_id, robot6_coord);
	
	vrep.getObjectOrientation(self_id, self_orientation);
	vrep.getObjectOrientation(robot5_id, robot5_orientation);
	vrep.getObjectOrientation(robot6_id, robot6_orientation);

	std::cout << "----------------------LIST INFO-----------------------" << std::endl;
	printInfo("Robot self", self_id, self_coord );
	printInfo("Robot #0", robot0_id, robot0_coord );
	printInfo("Robot #1", robot1_id, robot1_coord );
	printInfo("Robot #2", robot2_id, robot2_coord );
	printInfo("Robot #3", robot3_id, robot3_coord );
	printInfo("Robot #4", robot4_id, robot4_coord );
	printInfo("Robot #5", robot5_id, robot5_coord );
	printInfo("Ball", ball_id, ball_coord );
	printInfo("Goal left ", goal_left_id, goal_left_coord );
	printInfo("Goal Right", goal_right_id, goal_right_coord );
	printInfo("Robot #5 Orientation", robot5_id, robot5_orientation);
	
	//initialize map with static elements
	simxFloat aux[3];
	for(float i = -FIELD_Y; i <= FIELD_Y; i+=SCALE) {
		aux[0] = -FIELD_X;
		aux[1] = i;
		updateMap(map, aux, FIELD_LIMITS);
		aux[0] = FIELD_X;
		updateMap(map, aux, FIELD_LIMITS);	
	}
	for(float i = -FIELD_X; i <= FIELD_X; i+=SCALE) {
		aux[0] = i;
		aux[1] = -FIELD_Y;
		updateMap(map, aux, FIELD_LIMITS);
		aux[1] = FIELD_Y;
		updateMap(map, aux, FIELD_LIMITS);
	}
	
	aux[0] = -4.5;
	for(float i = goal_left_coord[1] - (GOAL_SIZE)/2; i < goal_left_coord[1] + (GOAL_SIZE)/2; i += 0.25 ) {
		aux[1] = i;
		updateMap(map, aux, TEAM_GOAL);
	}
	
	aux[0] = goal_right_coord[0];
	for(float i = goal_right_coord[1] - (GOAL_SIZE)/2; i < goal_right_coord[1] + (GOAL_SIZE)/2; i += 0.25 ) {
		aux[1] = i;
		updateMap(map, aux, OPP_GOAL);
	}
	//initialize map with robots and ball`s inicial position 
	updateMap(map, self_coord, TEAM_ROBOT);
	updateMap(map, robot0_coord, TEAM_ROBOT);
	updateMap(map, robot1_coord, TEAM_ROBOT);
	updateMap(map, robot2_coord, TEAM_ROBOT);
	updateMap(map, robot3_coord, OPP_ROBOT);
	updateMap(map, robot4_coord, OPP_ROBOT);
	updateMap(map, robot5_coord, OPP_ROBOT);
	updateMap(map, robot6_coord, TEAM_ROBOT);
	
	updateMap(map, ball_coord, BALL);

	printMapGroundTruth(map);

	updateOccupancyGrid(occupancy, self_coord, self_orientation);
	
	printOccupancyGrid(occupancy);

	printMap(map, occupancy);

	self_coord[0] = 4.0;
	self_coord[1] = 3.0;
	std::cout.precision(2);
	while(true) {
		sleep(1);	
		updateMap(map, self_coord, TEAM_ROBOT);		
		updateOccupancyGrid(occupancy, self_coord, self_orientation);
		updateOccupancyGrid(occupancy, robot5_coord, robot5_orientation);
		updateOccupancyGrid(occupancy, robot6_coord, robot6_orientation);
		vrep.getObjectPosition(robot6_id, robot6_coord);
		vrep.getObjectOrientation(robot6_id, robot6_orientation);
		printOccupancyGrid(occupancy);
		printMap(map, occupancy);
	}
	
	vrep.disconnectServer();
	return 0;
}
