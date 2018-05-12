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
#include <random>
#include <boost/tuple/tuple.hpp>
#include "/home/thales/gnuplot-iostream/gnuplot-iostream.h"
#include "fl/Headers.h"

using namespace std;
using namespace fl;

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

Gnuplot gp;
fl::Engine *engine = new fl::Engine;
const int nstepsRange = 1000;
const int nstepsFOV = 10000;
double rangeDistr[10] = {};
double FOVdistr[60] = {};
std::default_random_engine generator;
std::normal_distribution<double> distributionRange(2.5, 1.0);
std::normal_distribution<double> distributionFOV(0.0, 10.0);
//////////////////////////////////////////////////////////////////////////////////////////////////
void generateNormalDistributionRange() {
	for(int i = 0; i < nstepsRange; ++i) {
		double number = distributionRange(generator);
		if((number>=0.0)&&(number<10.0)) ++rangeDistr[int(number)];
	}
	int maxElement = 0;
	double max = 0;
	for(int i = 0; i < 10; i++) {
		if(rangeDistr[i] > max) {
			max = rangeDistr[i];
			maxElement = i;
		}
	}
	for(int i = 0; i < 10; i++) {
		rangeDistr[i] /= max;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void generateNormalDistributionFOV() {
	for(int i = 0; i < nstepsFOV; ++i) {
		double number = distributionFOV(generator);
		if((number>=-30.0)&&(number<30.0)){
			++FOVdistr[int(number + 30)];
		}
	}
	int maxElement = 0;
	double max = 0;
	for(int i = 0; i < 60; i++) {
		if(FOVdistr[i] > max) {
			max = FOVdistr[i];
			maxElement = i;
		}
	}
	for(int i = 0; i < 60; i++) {
		FOVdistr[i] /= max;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void resetOccupancyGrid(float m[][MAP_Y]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++) {
				m[i][j] = 0;
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void resetMap(int m[][MAP_Y][N_MAP_ELEMENTS]) {
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++ ) {
			for(int k = 0; k < N_MAP_ELEMENTS; k++)
				m[i][j][k] = 0;
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void printInfo(std::string name, simxInt id, simxFloat coord[]) {
	std::cout  << name <<  ":\n\tid: " << id << "\n\tposition:\n\t\tx: " << coord[0]
			<< "\n\t\ty: " << coord[1] << "\n\t\tz: " << coord[2] << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
int convertX(simxFloat x) {
	int x_pos = (x + MAX_X)/SCALE;
	return x_pos;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
int convertY(simxFloat y) {
	int y_pos = (MAP_Y/2) - (y/SCALE);
	return y_pos;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
float xCoordToMeters(int x) {
	return (x*SCALE) - MAX_X;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
float yCoordToMeters(int y) {
	return ((MAP_Y/2) - y)*SCALE;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////
void feedMap(int map[][MAP_Y][N_MAP_ELEMENTS]) {
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
}
//////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////
void printOccupancyGrid(float occupancy[][MAP_Y]) {
	std::cout << "Occupancy grid" << std::endl;
	for(int i = 0; i < MAP_X; i++) {
		for(int j = 0; j < MAP_Y; j++) {
			std::cout <<std::setw(5) << occupancy[i][j] <<  " ";
		}
		std::cout << std::endl;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////
simxFloat calculateBottomLim(simxFloat z_angle, int* flag) {
	if((z_angle - (M_PI/6)) < -M_PI) {
		float diff = M_PI + z_angle;
		*flag = 1;
		return (M_PI - ((M_PI/6) - diff));
	}
	return (z_angle - (M_PI/6));
}
//////////////////////////////////////////////////////////////////////////////////////////////////
simxFloat calculateUpperLim(simxFloat z_angle, int* flag) {
	if((z_angle + (M_PI/6)) > M_PI) {
		float diff = M_PI - z_angle;
		*flag = 1;
		return (-M_PI + ((M_PI/6) - diff));
	}
		return(z_angle + (M_PI/6));
}
//////////////////////////////////////////////////////////////////////////////////////////////////
simxFloat calculateAngle(simxFloat diffX, simxFloat diffY, simxFloat dist) {
	//simxFloat hip = sqrt(pow(x, 2) + pow(y, 2));
	simxFloat arcCos = acos((double)diffX/ (double)dist);
	simxFloat arcSin = asin((double)diffY/ (double)dist);
	simxFloat alpha = 0.0;
	if(arcSin > 0)
		return arcCos;
	return -1*arcCos;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
simxFloat calculateDistributionBasedOnAngleInDegrees(simxFloat alpha, simxFloat robotAngle, simxFloat dist) {
	simxFloat angleInDegrees = (std::abs(alpha - robotAngle)*180)/M_PI;
	simxFloat result = FOVdistr[int(angleInDegrees + 30)]*rangeDistr[int(dist)];
	return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void setOccupation(float m[][MAP_Y], float modified[][MAP_Y], int i, int j, simxFloat alpha, simxFloat z_angle, simxFloat dist) {
	//double angleInDegrees = (std::abs(alpha - z_angle)*180)/M_PI;
	double aux = calculateDistributionBasedOnAngleInDegrees(alpha, z_angle, dist);
	m[i][j] = (aux > m[i][j]?aux:m[i][j]);
	modified[i][j] = 1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void updateOccupancyGrid2(float m[][MAP_Y], const std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>& robots_coords, 	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>& robots_orient) {
	float modified[MAP_X][MAP_Y];
	resetOccupancyGrid(modified);
	for(int index = 0; index < robots_coords.size(); index++) {
		simxFloat dist, x, y, diffX, diffY, upperLim, bottomLim;
		int flag = 0;
		boost::tuple<simxFloat, simxFloat, simxFloat> coord = robots_coords.at(index);
		boost::tuple<simxFloat, simxFloat, simxFloat> orient = robots_orient.at(index);
		simxFloat x_coord = boost::get<0>(coord);
		simxFloat y_coord = boost::get<1>(coord);
		simxFloat z_angle = boost::get<2>(orient);
		bottomLim = calculateBottomLim(z_angle, &flag);
		upperLim = calculateUpperLim(z_angle, &flag);

		for(int i = 0; i < MAP_X; i++) {
			x = xCoordToMeters(i);
			diffX = x - x_coord;
			for(int j = 0; j < MAP_Y; j++) {
				if(modified[i][j] != 1)
					m[i][j] = (m[i][j] > 0)?m[i][j] - m[i][j]*0.05 : 0;
				y = yCoordToMeters(j);
				diffY = y - y_coord;
				dist = sqrt(pow(diffX, 2) + pow(diffY, 2));
				if (dist < MAX_DISTANCE) {
					simxFloat alpha = calculateAngle(diffX, diffY, dist);
					if(flag > 0) {
						if((alpha >= bottomLim) && (alpha <= M_PI))
							setOccupation(m, modified, i, j, alpha, z_angle, dist);
						if((alpha >= -M_PI) && (alpha <= upperLim))
							setOccupation(m, modified, i, j, alpha, z_angle, dist);
					}
					else
						if((alpha >= bottomLim) && (alpha <= upperLim))
							setOccupation(m, modified, i, j, alpha, z_angle, dist);
				}
			}
		}
		m[convertX(x_coord)][convertY(y_coord)] = 1;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void updateOccupancyGrid(float m[][MAP_Y], simxFloat robot[], simxFloat orientation[]) {
	simxFloat x_coord = robot[0];
	simxFloat y_coord = robot[1];
	simxFloat z_angle = orientation[2];
	std::cout << "Orientacao: " << z_angle << std::endl;
	simxFloat sin, cos, dist, x, y, diffX, diffY, upperLim, bottomLim;

	int flag = 0;

	if((z_angle - (M_PI/6)) < -M_PI) {
		float diff = M_PI + z_angle;
		bottomLim = M_PI - ((M_PI/6) - diff);
		flag = 1;
	}
	else
		bottomLim = z_angle - (M_PI/6);

	if((z_angle + (M_PI/6)) > M_PI) {
		float diff = M_PI - z_angle;
		upperLim = -M_PI + ((M_PI/6) - diff);
		flag = 1;
	}
	else
		upperLim = z_angle + (M_PI/6);

	for(int i = 0; i < MAP_X; i++) {
		x = xCoordToMeters(i);
		diffX = x - x_coord;
		for(int j = 0; j < MAP_Y; j++) {
			m[i][j] = (m[i][j] > 0)?m[i][j] - m[i][j]*0.05 : 0;
			y =
			(j);
			diffY = y - y_coord;
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
	m[convertX(x_coord)][convertY(y_coord)] = 1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void printGnuPlot(float occupancyGrid[][MAP_Y]) {

	gp << "set xrange [-6:6]\n";
	gp << "set yrange [-6:6]\n";
	gp << "set zrange [-1:1]\n";
	gp << "set hidden3d nooffset\n";
	gp << "splot ";
	{
		double step = SCALE;
		double startX = -FIELD_X;
		double startY = FIELD_Y;
		double stepX = startX, stepY, stepZ;
		std::vector<std::vector<boost::tuple<double,double,double> > > pts(MAP_X);
		for(int u = 0; u < MAP_X; u++) {
			pts[u].resize(MAP_Y);
			stepY = startY;
			for(int v = 0; v < MAP_Y; v++) {
				stepZ = occupancyGrid[u][v];
				pts[u][v] = boost::make_tuple(stepX, stepY, stepZ);
				stepY -= step;
			}
			stepX += step;
		}
		gp << gp.binFile2d(pts, "record") << "with lines title 'vec of vec of boost::tuple'";
	}
	gp << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
	simxInt ball_id, goal_left_id, goal_right_id;
	simxFloat ball_coord[3], goal_left_coord[3], goal_right_coord[3], aux_coord[3];
	std::vector<std::string> team_robots;
	std::vector<simxInt> team_robots_ids;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_coords;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_orient;
	std::vector<std::string> opp_robots;
	std::vector<simxInt> opp_robots_ids;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_coords;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_orient;

	// First, we must connect to the v-rep simulator server
	// For local connection, the IP is "127.0.0.1"
	generateNormalDistributionRange();
	generateNormalDistributionFOV();

	auto sim = new Simulator("127.0.0.1", PORT_NUMBER);
	Simulator &vrep = *sim;
	vrep.connectServer();

	int map[MAP_X][MAP_Y][N_MAP_ELEMENTS];
	float occupancy[MAP_X][MAP_Y];

	resetMap(map);
	resetOccupancyGrid(occupancy);

	team_robots.push_back("NAO");
	team_robots.push_back("NAO#0");
	team_robots.push_back("NAO#1");
	team_robots.push_back("NAO#2");
	opp_robots.push_back("NAO#3");
	opp_robots.push_back("NAO#4");
	opp_robots.push_back("NAO#5");

	for(const auto robotName : team_robots) {
		team_robots_ids.push_back(vrep.getHandle(robotName));
		std::cout << robotName << std::endl;
	}
	for(const auto robotName : opp_robots)
		opp_robots_ids.push_back(vrep.getHandle(robotName));

	ball_id = vrep.getHandle("Ball");
	goal_left_id = vrep.getHandle("Goal_left");
	goal_right_id = vrep.getHandle("Goal_right");

	for(auto robotId : team_robots_ids) {
		vrep.getObjectPosition(robotId, aux_coord);
		team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		vrep.getObjectOrientation(robotId, aux_coord);
		team_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
	}
	for(const auto robotId : opp_robots_ids) {
		vrep.getObjectPosition(robotId, aux_coord);
		opp_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		vrep.getObjectOrientation(robotId, aux_coord);
		opp_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
	}

	vrep.getObjectPosition(ball_id, ball_coord);
	vrep.getObjectPosition(goal_left_id, goal_left_coord);
	vrep.getObjectPosition(goal_right_id, goal_right_coord);

	feedMap(map);
	simxFloat aux[3];
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
	for(auto const pos : team_robots_coords) {
		simxFloat coords[3] = {boost::get<0>(pos), boost::get<1>(pos), boost::get<2>(pos)};
		updateMap(map, coords, TEAM_ROBOT);
	}
	for(auto const pos : opp_robots_coords) {
		simxFloat coords[3] = {boost::get<0>(pos), boost::get<1>(pos), boost::get<2>(pos)};
		updateMap(map, coords, OPP_ROBOT);
	}
	updateMap(map, ball_coord, BALL);
	updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_orient);
	printGnuPlot(occupancy);
	//std::cout.precision(2);

	while(true) {
		sleep(1);
		team_robots_coords.clear();
		team_robots_orient.clear();
		for(auto const& id : team_robots_ids) {
			vrep.getObjectPosition(id, aux_coord);
			team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
			vrep.getObjectOrientation(id, aux_coord);
			team_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}
		for(auto const& id : opp_robots_ids) {
			vrep.getObjectPosition(id, aux_coord);
			opp_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
			vrep.getObjectOrientation(id, aux_coord);
			opp_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}
		updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_orient);
		printGnuPlot(occupancy);
	}

	vrep.disconnectServer();
	return 0;
}
