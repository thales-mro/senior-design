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
#include <string>
#include <iostream>
#include <unistd.h>
#include <random>
#include <boost/tuple/tuple.hpp>
#include "/home/thales/gnuplot-iostream/gnuplot-iostream.h"
#include "fl/Headers.h"
#include "teste.hpp"
#include "Exceptions/VRepException.hpp"

using namespace std;
using namespace fl;

const bool IS_MAP_SHARED = true;

const double MAX_X = 6.0;
const double MAX_Y = 4.0;
const double SCALE = 0.25;
const int MAP_X = 48;
const int MAP_Y = 32;

const double FIELD_X = 4.5;
const double FIELD_Y = 3.0;

const int TEAM_ROBOT = 1;
const int OPP_ROBOT = 2;
const int BALL = 4;
const int TEAM_GOAL = 8;
const int OPP_GOAL = 9;
const int FIELD_LIMITS = 16;

const double MAX_DISTANCE = 4.5;
const double GOAL_SIZE = 1.50;
const int N_MAP_ELEMENTS = 6;

Gnuplot gp;
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
void startEngine(Engine *engine) {
	engine->setName("PanControl");
	engine->setDescription("Defines robot pan control (speed)");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configBallInputVariables(Engine *engine, InputVariable *ballDistance,InputVariable *ballAngle,InputVariable *ballConfidence) {
	ballDistance->setName("BallDistance");
	ballDistance->setDescription("Checks whether the robot have the ball or not");
	ballDistance->setEnabled(true);
	ballDistance->setRange(0.000, 15.000);
	ballDistance->setLockValueInRange(false);
	ballDistance->addTerm(new Triangle("haveBall", 0, 0.05, 0.2));
	ballDistance->addTerm(new Trapezoid("doNotHaveBall",0.15, 0.30, 14.5, 14.5));
	engine->addInputVariable(ballDistance);

	ballAngle->setName("BallAngle");
	ballAngle->setDescription("Checks the angle of the ball towards the robot");
	ballAngle->setEnabled(true);
	ballAngle->setRange(-3.14, 3.14);
	ballAngle->setLockValueInRange(false);
	ballAngle->addTerm(new Triangle("littleToLeft", 0, 0.502, 1.06));
	ballAngle->addTerm(new Trapezoid("moderateToLeft",0.4854, 0.8403, 1.297, 1.617));
	ballAngle->addTerm(new Trapezoid("aLotToLeft",1.057, 1.767, 3.017, 3.297));
	ballAngle->addTerm(new Triangle("littleToRight", -1.06, -0.502, 0));
	ballAngle->addTerm(new Trapezoid("moderateToRight", -1.617, -1.297, -0.8403, -0.4854));
	ballAngle->addTerm(new Trapezoid("aLotToRight", -3.297, -3.017, -1.767, -1.057));
	engine->addInputVariable(ballAngle);

	ballConfidence->setName("BallConfidence");
	ballConfidence->setDescription("Checks the confidence in ball's location");
	ballConfidence->setEnabled(true);
	ballConfidence->setRange(0.000, 1.000);
	ballConfidence->setLockValueInRange(false);
	ballConfidence->addTerm(new Trapezoid("lowConfidence", 0, 0, 0.4501, 0.7968));
	engine->addInputVariable(ballConfidence);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configOppPlayerInputVariables(Engine *engine, InputVariable *oppAngle, InputVariable *oppConfidence, int id) {
	oppAngle->setName("oppRobotAngle" + to_string(id));
	oppAngle->setDescription("Checks the angle of a opponent robot towards the robot");
	oppAngle->setEnabled(true);
	oppAngle->setRange(-3.15, 3.15);
	oppAngle->setLockValueInRange(false);
	oppAngle->addTerm(new Triangle("littleToLeft", 0, 0.502, 1.06));
	oppAngle->addTerm(new Trapezoid("moderateToLeft",0.4807, 0.8403, 3.017, 3.297));
	//oppAngle->addTerm(new Trapezoid("aLotToLeft",1.057, 1.767, 3.017, 3.297));
	oppAngle->addTerm(new Triangle("littleToRight", -1.06, -0.502, 0));
	oppAngle->addTerm(new Trapezoid("moderateToRight", -3.297, -3.017, -0.8403, -0.4807));
	//oppAngle->addTerm(new Trapezoid("aLotToRight", -3.297, -3.017, -1.767, -1.057));
	engine->addInputVariable(oppAngle);

	oppConfidence->setName("oppConfidence" + to_string(id));
	oppConfidence->setDescription("Checks the confidence in opponent's location");
	oppConfidence->setEnabled(true);
	oppConfidence->setRange(0.000, 1.000);
	oppConfidence->setLockValueInRange(false);
	oppConfidence->addTerm(new Trapezoid("lowConfidence", 0, 0, 0.4, 0.6958));
	engine->addInputVariable(oppConfidence);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configPanAngleOutputVariable(Engine *engine, OutputVariable *panAngle) {
	panAngle->setName("panAngle");
	panAngle->setDescription("Sets pan yaw velocity of head joint");
	panAngle->setEnabled(true);
	panAngle->setRange(-1.000, 1.000);
	panAngle->setLockValueInRange(false);
	panAngle->setAggregation(new Maximum);
	panAngle->setDefuzzifier(new Centroid(100));
	panAngle->setDefaultValue(0);
	panAngle->addTerm(new Trapezoid("panLittleToLeft", -0.06373, -0.003132, 0.05551, 0.1343));
	panAngle->addTerm(new Trapezoid("panModerateToLeft", 0.0151, 0.1573, 0.3401, 0.5271));
	panAngle->addTerm(new Trapezoid("panALotToLeft", 0.227, 0.435, 0.6723, 0.683));
	panAngle->addTerm(new Trapezoid("panToBallLittleToLeft", 0.2137, 0.3537, 0.5537, 0.6427));
	panAngle->addTerm(new Trapezoid("panToBallModerateToLeft", 0.3362, 0.4842, 0.6842, 0.7992));
	panAngle->addTerm(new Trapezoid("panToBallALotToLeft", 0.5855, 0.7355, 0.8605, 0.9975));
	panAngle->addTerm(new Trapezoid("panLittleToRight", -0.1343, -0.05551, 0.003132, 0.06373));
	panAngle->addTerm(new Trapezoid("panModerateToRight", -0.5271, -0.3401, -0.1573, -0.0151));
	panAngle->addTerm(new Trapezoid("panALotToRight", -0.683, -0.6723, -0.435, -0.227));
	panAngle->addTerm(new Trapezoid("panToBallLittleToRight", -0.6427, -0.5537,-0.3537, -0.2137));
	panAngle->addTerm(new Trapezoid("panToBallModerateToRight", -0.7992, -0.6842, -0.4842, -0.3362));
	panAngle->addTerm(new Trapezoid("panToBallALotToRight", -0.9975, -0.8605, -0.7355, -0.5855));
	engine->addOutputVariable(panAngle);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configPanRules(Engine *engine, RuleBlock *mamdani) {
	mamdani->setName("mamdani");
	mamdani->setDescription("");
	mamdani->setEnabled(true);
	mamdani->setConjunction(new Minimum);
	mamdani->setDisjunction(new Maximum);
	mamdani->setImplication(new AlgebraicProduct);
	mamdani->setActivation(new General);
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is littleToLeft and BallConfidence is lowConfidence then panAngle is panToBallLittleToLeft", engine));
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is moderateToLeft and BallConfidence is lowConfidence then panAngle is panToBallModerateToLeft", engine));
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is aLotToLeft and BallConfidence is lowConfidence then panAngle is panToBallALotToLeft", engine));
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is littleToRight and BallConfidence is lowConfidence then panAngle is panToBallLittleToRight", engine));
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is moderateToRight and BallConfidence is lowConfidence then panAngle is panToBallModerateToRight", engine));
	mamdani->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is aLotToRight and BallConfidence is lowConfidence then panAngle is panToBallALotToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is littleToLeft then panAngle is panLittleToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is littleToRight then panAngle is panLittleToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is moderateToLeft then panAngle is panModerateToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is moderateToRight then panAngle is panModerateToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is littleToLeft then panAngle is panLittleToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is littleToRight then panAngle is panLittleToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is moderateToLeft then panAngle is panModerateToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is moderateToRight then panAngle is panModerateToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is littleToLeft then panAngle is panLittleToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is littleToRight then panAngle is panLittleToRight", engine));
	mamdani->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is moderateToLeft then panAngle is panModerateToLeft", engine));
	mamdani->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is moderateToRight then panAngle is panModerateToRight", engine));
	engine->addRuleBlock(mamdani);
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
	if(arcSin > 0)
		return arcCos;
	return -1*arcCos;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
double calculateAngleForFuzzy(double angle, simxFloat z_angle) {
	z_angle = (z_angle < 0) ? z_angle + (2*M_PI): z_angle;
	angle = (angle < 0) ? angle + (2*M_PI): angle;
	double upperLim = z_angle + M_PI > (2*M_PI)? z_angle - M_PI : z_angle + M_PI;
	if(upperLim < z_angle)
		return (angle < upperLim) ? angle + ((2*M_PI) -z_angle) : angle - z_angle;
	else
		return (angle > upperLim) ? -(z_angle - (angle - (2*M_PI))) : angle - z_angle;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
double calculateAngleForOppRobotsFuzzy(const boost::tuple<simxFloat, simxFloat, simxFloat>& opp_coord, simxFloat x_coord, simxFloat y_coord, simxFloat z_angle) {
	simxFloat x_opp = boost::get<0>(opp_coord);
	simxFloat y_opp = boost::get<1>(opp_coord);
	double diffX = x_opp - x_coord;
	double diffY = y_opp - y_coord;
	double distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
	double angle = calculateAngle(diffX, diffY, distance);
	return calculateAngleForFuzzy(angle, z_angle);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void processFuzzy(Engine *engine, InputVariable *ballDistance, InputVariable *ballAngle, InputVariable *ballConfidence, InputVariable* opp1Confidence, InputVariable* opp1Angle,InputVariable* opp2Confidence, InputVariable* opp2Angle,InputVariable* opp3Confidence, InputVariable* opp3Angle, OutputVariable *panAngle, const boost::tuple<simxFloat, simxFloat, simxFloat>& robot_coord,  const boost::tuple<simxFloat, simxFloat, simxFloat> robot_orient, simxFloat* ball_coord, simxFloat ball_confidence, const std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>& opp_robots_coords, const std::vector<simxFloat> opp_robots_confidence) {
	string status;
	if(not engine->isReady(&status))
		throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

	simxFloat x_coord = boost::get<0>(robot_coord);
	simxFloat y_coord = boost::get<1>(robot_coord);
	simxFloat z_angle = boost::get<2>(robot_orient);

	double diffX = ball_coord[0] - x_coord;
	double diffY = ball_coord[1] - y_coord;
	double distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
	double angle = calculateAngle(diffX, diffY, distance);

	ballDistance->setValue(distance);
	ballAngle->setValue(calculateAngleForFuzzy(angle, z_angle));
	ballConfidence->setValue(ball_confidence);

	opp1Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(0), x_coord, y_coord, z_angle));
	opp2Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(1), x_coord, y_coord, z_angle));
	opp3Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(2), x_coord, y_coord, z_angle));
	opp1Confidence->setValue(opp_robots_confidence.at(0));
	opp2Confidence->setValue(opp_robots_confidence.at(1));
	opp3Confidence->setValue(opp_robots_confidence.at(2));
	//cout << "ball confidence: " << ballConfidence->getValue() << " ball distance: " << ballDistance->getValue() << " ball angle: "<< ballAngle->getValue() << endl;
	//cout << "angle Opp 1: " << opp1Angle->getValue() << "Angle Opp 2: " << opp2Angle->getValue() << " Angle Opp 3: " << opp3Angle->getValue() << endl;

	engine->process();
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
void zeroesRobotsJointsVelocity(const std::vector<simxInt>& team_robots_joints_ids, Simulator& vrep) {
	for(const auto jointId : team_robots_joints_ids) {
		vrep.setJointVelocity(jointId, 0.0);
	}

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
void printGnuPlot2(Gnuplot &gp2Aux, float occupancyGrid[][MAP_Y]) {

	gp2Aux << "set xrange [-6:6]\n";
	gp2Aux << "set yrange [-6:6]\n";
	gp2Aux << "set zrange [-1:1]\n";
	gp2Aux << "set hidden3d nooffset\n";
	gp2Aux << "splot ";
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
		gp2Aux << gp2Aux.binFile2d(pts, "record") << "with lines title 'vec of vec of boost::tuple'";
	}
	gp2Aux << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	Teste *teste = new Teste();

	simxInt ball_id, goal_left_id, goal_right_id;
	simxFloat ball_coord[3], goal_left_coord[3], goal_right_coord[3], aux_coord[3], ball_confidence;
	std::vector<std::string> team_robots_joints, team_robots_head, team_robots_body;
	std::vector<simxInt> team_robots_joints_ids, team_robots_head_ids, team_robots_body_ids;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_coords;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_head_orient, team_robots_body_orient;
	std::vector<std::string> opp_robots;
	std::vector<simxInt> opp_robots_ids;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_coords;
	std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_orient;
	std::vector<simxFloat> opp_robots_confidence;

	Gnuplot gpRobot1, gpRobot2, gpRobot3;

	Engine *engine = new Engine;
	InputVariable *ballAngle = new InputVariable;
	InputVariable *ballDistance = new InputVariable;
	InputVariable *ballConfidence = new InputVariable;
	InputVariable *opp1Confidence = new InputVariable;
	InputVariable *opp1Angle = new InputVariable;
	InputVariable *opp2Confidence = new InputVariable;
	InputVariable *opp2Angle = new InputVariable;
	InputVariable *opp3Confidence = new InputVariable;
	InputVariable *opp3Angle = new InputVariable;
	OutputVariable *panAngle = new OutputVariable;
	RuleBlock *mamdani = new RuleBlock;
	startEngine(engine);
	configBallInputVariables(engine, ballDistance, ballAngle, ballConfidence);
	configOppPlayerInputVariables(engine, opp1Angle, opp1Confidence, 1);
	configOppPlayerInputVariables(engine, opp2Angle, opp2Confidence, 2);
	configOppPlayerInputVariables(engine, opp3Angle, opp3Confidence, 3);
	configPanAngleOutputVariable(engine, panAngle);
	configPanRules(engine, mamdani);

	generateNormalDistributionRange();
	generateNormalDistributionFOV();

	auto sim = new Simulator("127.0.0.1", PORT_NUMBER);
	Simulator &vrep = *sim;
	vrep.connectServer();


	int map[MAP_X][MAP_Y][N_MAP_ELEMENTS];
	resetMap(map);

	team_robots_joints.push_back("HeadYaw#0");
	team_robots_joints.push_back("HeadYaw#1");
	team_robots_joints.push_back("HeadYaw#2");

	team_robots_body.push_back("NAO#0");
	team_robots_body.push_back("NAO#1");
	team_robots_body.push_back("NAO#2");
	team_robots_head.push_back("HeadPitch_link_respondable#0");
	team_robots_head.push_back("HeadPitch_link_respondable#1");
	team_robots_head.push_back("HeadPitch_link_respondable#2");
	opp_robots.push_back("NAO#3");
	opp_robots.push_back("NAO#4");
	opp_robots.push_back("NAO#5");

	for(const auto jointName : team_robots_joints) {
		team_robots_joints_ids.push_back(vrep.getHandle(jointName));
		std::cout << jointName << std::endl;
	}
	for(const auto headName : team_robots_head) {
		team_robots_head_ids.push_back(vrep.getHandle(headName));
	}
	for(const auto bodyName : team_robots_body) {
		team_robots_body_ids.push_back(vrep.getHandle(bodyName));
	}
	for(const auto robotName : opp_robots)
		opp_robots_ids.push_back(vrep.getHandle(robotName));

	ball_id = vrep.getHandle("Ball");
	goal_left_id = vrep.getHandle("Goal_left");
	goal_right_id = vrep.getHandle("Goal_right");

	/*for(auto robotId : team_robots_joints_ids) {
		vrep.getObjectPosition(robotId, aux_coord);
		team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		vrep.getObjectOrientation(robotId, aux_coord);
		team_robot_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
	}*/
	for(auto robotId : team_robots_body_ids) {
		vrep.getObjectPosition(robotId, aux_coord);
		team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		vrep.getObjectOrientation(robotId, aux_coord);
		team_robots_body_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
	}
	for(auto headId : team_robots_head_ids) {
		vrep.getObjectOrientation(headId, aux_coord);
		team_robots_head_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
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


	float occupancy[MAP_X][MAP_Y], occupancy2[MAP_X][MAP_Y], occupancy3[MAP_X][MAP_Y];
	resetOccupancyGrid(occupancy);
	if(!IS_MAP_SHARED) {
		resetOccupancyGrid(occupancy2);
		resetOccupancyGrid(occupancy3);
	}


	//updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_orient);
	//std::cout.precision(2);

	while(true) {


		team_robots_coords.clear();
		//team_robot_orient.clear();
		team_robots_head_orient.clear();
		team_robots_body_orient.clear();
		opp_robots_coords.clear();

		vrep.getObjectPosition(ball_id, ball_coord);
		/*for(auto const& id : team_robots_head_ids) {
			vrep.getObjectPosition(id, aux_coord);
			team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
			vrep.getObjectOrientation(id, aux_coord);
			team_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}*/
		for(auto const& id : team_robots_body_ids) {
			vrep.getObjectPosition(id, aux_coord);
			team_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
			vrep.getObjectOrientation(id, aux_coord);
			team_robots_body_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}
		for(auto const& id: team_robots_head_ids) {
			vrep.getObjectOrientation(id, aux_coord);
			team_robots_head_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}
		for(auto const& id : opp_robots_ids) {
			vrep.getObjectPosition(id, aux_coord);
			opp_robots_coords.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
			vrep.getObjectOrientation(id, aux_coord);
			opp_robots_orient.push_back(boost::make_tuple(aux_coord[0], aux_coord[1], aux_coord[2]));
		}
		if(IS_MAP_SHARED) {
			updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_head_orient);
			printGnuPlot(occupancy);
		}
		else {
			std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> aux;
			std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> aux2;
			aux.clear();
			aux2.clear();
			aux.push_back(team_robots_coords.at(0));
			aux2.push_back(team_robots_head_orient.at(0));
			updateOccupancyGrid2(occupancy, aux, aux2);
			aux.clear();
			aux2.clear();
			aux.push_back(team_robots_coords.at(1));
			aux2.push_back(team_robots_head_orient.at(1));
			updateOccupancyGrid2(occupancy2, aux, aux2);
			aux.clear();
			aux2.clear();
			aux.push_back(team_robots_coords.at(2));
			aux2.push_back(team_robots_head_orient.at(2));
			updateOccupancyGrid2(occupancy3, aux, aux2);
			printGnuPlot2(gpRobot1, occupancy);
			printGnuPlot2(gpRobot2, occupancy2);
			printGnuPlot2(gpRobot3, occupancy3);
		}
		if(IS_MAP_SHARED) {
			ball_confidence = occupancy[convertX(ball_coord[0])][convertY(ball_coord[1])];
			cout << "Ball confidence: " << ball_confidence << endl;
			opp_robots_confidence.clear();
			for(int i = 0; i < opp_robots_coords.size(); i++) {
				boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
				opp_robots_confidence.push_back(occupancy[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
				cout << "Outside fuzzy " << "Confidence robot " << i << ": " << occupancy[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))] << endl;
			}

			for(int i = 0; i < team_robots_coords.size(); i++) {
				cout << "For robot: " << i << endl;
				processFuzzy(engine, ballDistance, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp2Confidence, opp2Angle, opp3Confidence, opp3Angle, panAngle, team_robots_coords.at(i), team_robots_head_orient.at(i), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
				double speed = panAngle->getValue();
				cout << "calculated speed " << i << ": " << speed << endl;
				vrep.setJointVelocity(team_robots_joints_ids.at(i), speed);
			}
		}
		else {
			ball_confidence = occupancy[convertX(ball_coord[0])][convertY(ball_coord[1])];
			opp_robots_confidence.clear();
			for(int i = 0; i < opp_robots_coords.size(); i++) {
				boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
				opp_robots_confidence.push_back(occupancy[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
			}
			processFuzzy(engine, ballDistance, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(0), team_robots_head_orient.at(0), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
			double speed = panAngle->getValue();
			cout << "calculated speed "  << ": " << speed << endl;
			vrep.setJointVelocity(team_robots_joints_ids.at(0), speed);

			ball_confidence = occupancy2[convertX(ball_coord[0])][convertY(ball_coord[1])];
			opp_robots_confidence.clear();
			for(int i = 0; i < opp_robots_coords.size(); i++) {
				boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
				opp_robots_confidence.push_back(occupancy2[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
			}
			processFuzzy(engine, ballDistance, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(1), team_robots_head_orient.at(1), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
			speed = panAngle->getValue();
			cout << "calculated speed "  << ": " << speed << endl;
			vrep.setJointVelocity(team_robots_joints_ids.at(1), speed);

			ball_confidence = occupancy3[convertX(ball_coord[0])][convertY(ball_coord[1])];
			opp_robots_confidence.clear();
			for(int i = 0; i < opp_robots_coords.size(); i++) {
				boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
				opp_robots_confidence.push_back(occupancy3[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
			}
			processFuzzy(engine, ballDistance, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(2), team_robots_head_orient.at(2), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
			speed = panAngle->getValue();
			cout << "calculated speed "  << ": " << speed << endl;
			vrep.setJointVelocity(team_robots_joints_ids.at(2), speed);

		}
		sleep(5);
	}


	vrep.disconnectServer();
	return 0;
}
