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

void printOccupancyGrid(float occupancy[][MAP_Y]) {
	std::cout << "Occupancy grid" << std::endl;
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++) {
			std::cout <<std::setw(2) << occupancy[i][j] <<  " ";
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

void updateOccupancyGrid2(float m[][MAP_Y], std::vector<simxFloat*>& robots_coords, std::vector<simxFloat*>& robots_orient) {
	float modified[MAP_X][MAP_Y];
	resetOccupancyGrid(modified);
	for(int index = 0; index < robots_coords.size(); index++) {
		simxFloat *coords = robots_coords.at(index);
		simxFloat x_robot = coords[0];
		simxFloat y_robot = coords[1];
		simxFloat *orientation = robots_orient.at(index);
		simxFloat z_rotation = orientation[2];
		simxFloat sin, cos, dist, x, y, diffX, diffY, upperLim, bottomLim;
		int flag = 0;
		if((z_rotation - (M_PI/6)) < -M_PI) {
			float diff = M_PI + z_rotation;
			bottomLim = M_PI - ((M_PI/6) - diff);
			flag = 1;
		}
		else 
			bottomLim = z_rotation - (M_PI/6);

		if((z_rotation + (M_PI/6)) > M_PI) {
			float diff = M_PI - z_rotation;
			upperLim = -M_PI + ((M_PI/6) - diff);
			flag = 1;
		}
		else 
			upperLim = z_rotation + (M_PI/6);
		for(int i = 0; i < MAP_X; i++) {
			x = xCoordToMeters(i);
			diffX = x - x_robot;
			for(int j = 0; j < MAP_Y; j++) {
				if(modified[i][j] != 1)
					m[i][j] = (m[i][j] > 0)?m[i][j] - m[i][j]*0.05 : 0;
				y = yCoordToMeters(j);
				diffY = y - y_robot;
				dist = sqrt(pow(diffX, 2) + pow(diffY, 2));
				if (dist < MAX_DISTANCE) {
					simxFloat arcCos = acos((double)diffX/ (double)dist);
					simxFloat arcSin = asin((double)diffY/ (double)dist);
					simxFloat alpha = 0.0;				
					if(arcSin > 0) 
						alpha = arcCos;
					else 
						alpha = -arcCos;	
				
					if(flag > 0) {
						if((alpha >= bottomLim) && (alpha <= M_PI)) {
							m[i][j] = 1;
							modified[i][j] = 1;
						}
						if((alpha >= -M_PI) && (alpha <= upperLim)) { 
							m[i][j] = 1;
							modified[i][j] = 1;
						}
					}
					else {
						if((alpha >= bottomLim) && (alpha <= upperLim)) { 
							m[i][j] = 1;
							modified[i][j] = 1;
						}
					}
				}  
			}
		}
		m[convertX(x_robot)][convertY(y_robot)] = 1;
				
	}	
} 

void updateOccupancyGrid(float m[][MAP_Y], simxFloat robot[], simxFloat orientation[]) {
	simxFloat x_robot = robot[0];
	simxFloat y_robot = robot[1];
	simxFloat z_rotation = orientation[2];
	std::cout << "Orientacao: " << z_rotation << std::endl;
	simxFloat sin, cos, dist, x, y, diffX, diffY, upperLim, bottomLim;

	int flag = 0;
				
	if((z_rotation - (M_PI/6)) < -M_PI) {
		float diff = M_PI + z_rotation;
		bottomLim = M_PI - ((M_PI/6) - diff);
		flag = 1;
	}
	else 
		bottomLim = z_rotation - (M_PI/6);

	if((z_rotation + (M_PI/6)) > M_PI) {
		float diff = M_PI - z_rotation;
		upperLim = -M_PI + ((M_PI/6) - diff);
		flag = 1;
	}
	else 
		upperLim = z_rotation + (M_PI/6);
				
	for(int i = 0; i < MAP_X; i++) {
		x = xCoordToMeters(i);
		diffX = x - x_robot;
		for(int j = 0; j < MAP_Y; j++) {
			m[i][j] = (m[i][j] > 0)?m[i][j] - m[i][j]*0.05 : 0;
			y = yCoordToMeters(j);
			diffY = y - y_robot;
			dist = sqrt(pow(diffX, 2) + pow(diffY, 2));
			if (dist < MAX_DISTANCE) {
				simxFloat arcCos = acos((double)diffX/ (double)dist);
				simxFloat arcSin = asin((double)diffY/ (double)dist);
				simxFloat alpha = 0.0;				
				if(arcSin > 0) 
					alpha = arcCos;
				else 
					alpha = -arcCos;	
				
				if(flag > 0) {
					if((alpha >= bottomLim) && (alpha <= M_PI)) 
						m[i][j] = 1;
					
					if((alpha >= -M_PI) && (alpha <= upperLim)) 
						m[i][j] = 1;
				}
				else {
					if((alpha >= bottomLim) && (alpha <= upperLim)) 
							m[i][j] = 1;
				}
			}  
		}
	}
	m[convertX(x_robot)][convertY(y_robot)] = 1;
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

	std::vector<simxInt> team_robots_ids;
	std::vector<simxFloat*> team_robots_coords;		
	std::vector<simxFloat*> team_robots_orient;	
	std::vector<simxInt> opp_robots_ids;
	std::vector<simxFloat*> opp_robots_coords;		
	std::vector<simxFloat*> opp_robots_orient;	

	simxInt self_id, robot0_id, robot1_id, robot2_id, robot3_id, robot4_id, robot5_id, robot6_id, robot7_id, robot8_id;
	simxInt ball_id, goal_left_id, goal_right_id;

	simxFloat self_coord[3], robot0_coord[3], robot1_coord[3], robot2_coord[3], robot3_coord[3], robot4_coord[3], robot5_coord[3], robot6_coord[3], robot7_coord[3], robot8_coord[3];
	simxFloat ball_coord[3], goal_left_coord[3], goal_right_coord[3];

	simxFloat self_orientation[3], robot0_orientation[3], robot1_orientation[3], robot2_orientation[3], robot3_orientation[3], robot4_orientation[3], robot5_orientation[3], robot6_orientation[3], robot7_orientation[3], robot8_orientation[3];
	
	
	team_robots_ids.push_back(vrep.getHandle("NAO"));
	team_robots_ids.push_back(vrep.getHandle("NAO#0"));
	team_robots_ids.push_back(vrep.getHandle("NAO#1"));
	team_robots_ids.push_back(vrep.getHandle("NAO#2"));
	opp_robots_ids.push_back(vrep.getHandle("NAO#3"));
	opp_robots_ids.push_back(vrep.getHandle("NAO#4"));
	opp_robots_ids.push_back(vrep.getHandle("NAO#5"));
	//robot6_id = vrep.getHandle("NAO#6");
	//team_robots_ids.push_back(vrep.getHandle("NAO#7"));
	//team_robots_ids.push_back(vrep.getHandle("NAO#8"));
	ball_id = vrep.getHandle("Ball");
	goal_left_id = vrep.getHandle("Goal_left");
	goal_right_id = vrep.getHandle("Goal_right");

	vrep.getObjectPosition(team_robots_ids.at(0), self_coord);
	team_robots_coords.push_back(self_coord);
	vrep.getObjectPosition(team_robots_ids.at(1), robot0_coord);
	team_robots_coords.push_back(robot0_coord);
	vrep.getObjectPosition(team_robots_ids.at(2), robot1_coord);
	team_robots_coords.push_back(robot1_coord);
	vrep.getObjectPosition(team_robots_ids.at(3), robot2_coord);
	team_robots_coords.push_back(robot2_coord);
	vrep.getObjectPosition(opp_robots_ids.at(0), robot3_coord);
	opp_robots_coords.push_back(robot3_coord);
	vrep.getObjectPosition(opp_robots_ids.at(1), robot4_coord);
	opp_robots_coords.push_back(robot4_coord);
	vrep.getObjectPosition(opp_robots_ids.at(2), robot5_coord);
	opp_robots_coords.push_back(robot5_coord);

	vrep.getObjectPosition(ball_id, ball_coord);
	vrep.getObjectPosition(goal_left_id, goal_left_coord);
	vrep.getObjectPosition(goal_right_id, goal_right_coord);
	//vrep.getObjectPosition(robot6_id, robot6_coord);
	//vrep.getObjectPosition(robot7_id, robot7_coord);
	//team_robots_coords.push_back(robot7_coord);
	//vrep.getObjectPosition(robot8_id, robot8_coord);	
	//team_robots_coords.push_back(robot8_coord);

	vrep.getObjectOrientation(team_robots_ids.at(0), self_orientation);
	team_robots_orient.push_back(self_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(1), robot0_orientation);
	team_robots_orient.push_back(robot0_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(2), robot1_orientation);
	team_robots_orient.push_back(robot1_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(3), robot2_orientation);
	team_robots_orient.push_back(robot2_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(0), robot3_orientation);
	opp_robots_orient.push_back(robot3_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(1), robot4_orientation);
	opp_robots_orient.push_back(robot4_orientation);
	vrep.getObjectOrientation(team_robots_ids.at(2), robot5_orientation);
	opp_robots_orient.push_back(robot5_orientation);
	//vrep.getObjectOrientation(robot6_id, robot6_orientation);
	//vrep.getObjectOrientation(robot7_id, robot7_orientation);
	//vrep.getObjectOrientation(robot8_id, robot8_orientation);


	std::cout << "----------------------LIST INFO-----------------------" << std::endl;
	printInfo("Robot self",team_robots_ids.at(0), self_coord );
	printInfo("Robot #0", team_robots_ids.at(1) , robot0_coord );
	printInfo("Robot #1", team_robots_ids.at(2), robot1_coord );
	printInfo("Robot #2", team_robots_ids.at(3), robot2_coord );
	printInfo("Robot #3", opp_robots_ids.at(0), robot3_coord );
	printInfo("Robot #4", opp_robots_ids.at(1), robot4_coord );
	printInfo("Robot #5", opp_robots_ids.at(2), robot5_coord );
	printInfo("Ball", ball_id, ball_coord );
	printInfo("Goal left ", goal_left_id, goal_left_coord);
	printInfo("Goal Right", goal_right_id, goal_right_coord);
	printInfo("Robot #4 orientation", opp_robots_ids.at(1), robot4_orientation);
	printInfo("Robot #5 Orientation", opp_robots_ids.at(2), robot5_orientation);
	//printInfo("Robot #7 orientation", robot7_id, robot7_orientation);
	//printInfo("Robot #8 orientation", robot8_id, robot8_orientation);
	
	
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

	//printMapGroundTruth(map);

	//updateOccupancyGrid(occupancy, self_coord, self_orientation);

	updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_orient);
	
	//printOccupancyGrid(occupancy);

	//printMap(map, occupancy);

	//self_coord[0] = 4.0;
	//self_coord[1] = 3.0;
	std::cout.precision(2);
	while(true) {
		sleep(1);	
		//updateMap(map, self_coord, TEAM_ROBOT);		
//		updateOccupancyGrid(occupancy, self_coord, self_orientation);
//		updateOccupancyGrid(occupancy, robot5_coord, robot5_orientation);
		updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_orient);		
		

		//printOccupancyGrid(occupancy);
		printMap(map, occupancy);
	}
	
	vrep.disconnectServer();
	return 0;
}
