#define _GLIBCXX_USE_CXX11_ABI 0
#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <stdlib.h>
#include <time.h>

#include <defines.hpp>
#include "Simulator.hpp"
#include "teste.hpp"
#include "Communication.hpp"
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
#include <thread>
#include <future>

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
void startGazeControlEngine(Engine *gazeControlEngine) {
	gazeControlEngine->setName("PanControl");
	gazeControlEngine->setDescription("Defines robot pan control (speed)");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void startNavigationControlEngine(Engine *navigationControlEngine) {
	navigationControlEngine->setName("Velocity Control");
	navigationControlEngine->setDescription("Defines robot velocity control (x-axis and z-rotation (relative to robot))");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configBallInputVariables(Engine *gazeControlEngine, Engine *navigationControlEngine, InputVariable *ballReferenceForGazeControl,InputVariable *ballAngle,InputVariable *ballConfidence, InputVariable *distanceBall, InputVariable *teamHaveBall, InputVariable *closestToBall) {
	ballReferenceForGazeControl->setName("BallDistance");
	ballReferenceForGazeControl->setDescription("Checks whether the robot have the ball or not");
	ballReferenceForGazeControl->setEnabled(true);
	ballReferenceForGazeControl->setRange(0.000, 15.000);
	ballReferenceForGazeControl->setLockValueInRange(false);
	ballReferenceForGazeControl->addTerm(new Triangle("haveBall", 0, 0.05, 0.2));
	ballReferenceForGazeControl->addTerm(new Trapezoid("doNotHaveBall",0.15, 0.30, 14.5, 14.5));
	gazeControlEngine->addInputVariable(ballReferenceForGazeControl);

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
	gazeControlEngine->addInputVariable(ballAngle);
	navigationControlEngine->addInputVariable(ballAngle);

	ballConfidence->setName("BallConfidence");
	ballConfidence->setDescription("Checks the confidence in ball's location");
	ballConfidence->setEnabled(true);
	ballConfidence->setRange(0.000, 1.000);
	ballConfidence->setLockValueInRange(false);
	ballConfidence->addTerm(new Trapezoid("lowConfidence", 0, 0, 0.4501, 0.7968));
	gazeControlEngine->addInputVariable(ballConfidence);

	distanceBall->setName("distanceBall");
	distanceBall->setDescription("Distance to Ball from main robot");
	distanceBall->setEnabled(true);
	distanceBall->setRange(0.000, 11.000);
	distanceBall->setLockValueInRange(false);
	distanceBall->addTerm(new Triangle("close", 0, 0.15, 0.3));
	distanceBall->addTerm(new Trapezoid("closeModerate", 0.2, 0.4, 0.6, 0.8));
	distanceBall->addTerm(new Trapezoid("moderate", 0.5, 0.8, 1.1, 1.4));
	distanceBall->addTerm(new Trapezoid("moderateFar", 1, 1.5, 2, 2.5));
	distanceBall->addTerm(new Trapezoid("far", 2, 2.5, 10.8, 10.8));
	navigationControlEngine->addInputVariable(distanceBall);

	teamHaveBall->setName("teamHaveBall");
	teamHaveBall->setDescription("Checks if team have ball possession");
	teamHaveBall->setEnabled(true);
	teamHaveBall->setRange(0.000, 2.000);
	teamHaveBall->setLockValueInRange(false);
	teamHaveBall->addTerm(new Triangle("definetelyHaveBall", 1, 1, 1));
	navigationControlEngine->addInputVariable(teamHaveBall);

	closestToBall->setName("closestToBall");
	closestToBall->setDescription("Checks if controlled robot is the closest to ball");
	closestToBall->setEnabled(true);
	closestToBall->setRange(0.000, 2.000);
	closestToBall->setLockValueInRange(false);
	closestToBall->addTerm(new Triangle("closestToBall", 1, 1, 1));
	navigationControlEngine->addInputVariable(closestToBall);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configOppPlayerInputVariables(Engine *gazeControlEngine, InputVariable *oppAngle, InputVariable *oppConfidence, int id) {
	oppAngle->setName("oppRobotAngle" + to_string(id));
	oppAngle->setDescription("Checks the angle of a opponent robot towards the robot");
	oppAngle->setEnabled(true);
	oppAngle->setRange(-3.15, 3.15);
	oppAngle->setLockValueInRange(false);
	oppAngle->addTerm(new Triangle("littleToLeft", 0, 0.502, 1.06));
	oppAngle->addTerm(new Trapezoid("moderateToLeft",0.4807, 0.8403, 3.017, 3.297));
	oppAngle->addTerm(new Triangle("littleToRight", -1.06, -0.502, 0));
	oppAngle->addTerm(new Trapezoid("moderateToRight", -3.297, -3.017, -0.8403, -0.4807));
	gazeControlEngine->addInputVariable(oppAngle);

	oppConfidence->setName("oppConfidence" + to_string(id));
	oppConfidence->setDescription("Checks the confidence in opponent's location");
	oppConfidence->setEnabled(true);
	oppConfidence->setRange(0.000, 1.000);
	oppConfidence->setLockValueInRange(false);
	oppConfidence->addTerm(new Trapezoid("lowConfidence", 0, 0, 0.4, 0.6958));
	gazeControlEngine->addInputVariable(oppConfidence);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void configOppGoalVariables(Engine *navigationControlEngine, InputVariable *distanceToOppGoal, InputVariable *angleToOppGoal) {

	distanceToOppGoal->setName("distanceToOppGoal");
	distanceToOppGoal->setDescription("Distance to Opponent`s goal from main robot");
	distanceToOppGoal->setEnabled(true);
	distanceToOppGoal->setRange(0.000, 10.000);
	distanceToOppGoal->setLockValueInRange(false);
	distanceToOppGoal->addTerm(new Triangle("close", 0, 0.15, 0.3));
	distanceToOppGoal->addTerm(new Trapezoid("closeModerate", 0.2, 0.4, 0.6, 0.8));
	distanceToOppGoal->addTerm(new Trapezoid("moderate", 0.5, 0.8, 1.1, 1.4));
	distanceToOppGoal->addTerm(new Trapezoid("moderateFar", 1, 1.5, 2, 2.5));
	distanceToOppGoal->addTerm(new Trapezoid("far", 2, 2.5, 9.5, 9.5));
	navigationControlEngine->addInputVariable(distanceToOppGoal);

	angleToOppGoal->setName("angleToOppGoal");
	angleToOppGoal->setDescription("Checks the angle of the opp`s goal towards the robot");
	angleToOppGoal->setEnabled(true);
	angleToOppGoal->setRange(-3.14, 3.14);
	angleToOppGoal->setLockValueInRange(false);
	angleToOppGoal->addTerm(new Triangle("littleToLeft", 0, 0.502, 1.06));
	angleToOppGoal->addTerm(new Trapezoid("moderateToLeft",0.4854, 0.8403, 1.297, 1.617));
	angleToOppGoal->addTerm(new Trapezoid("aLotToLeft",1.057, 1.767, 3.017, 3.297));
	angleToOppGoal->addTerm(new Triangle("littleToRight", -1.06, -0.502, 0));
	angleToOppGoal->addTerm(new Trapezoid("moderateToRight", -1.617, -1.297, -0.8403, -0.4854));
	angleToOppGoal->addTerm(new Trapezoid("aLotToRight", -3.297, -3.017, -1.767, -1.057));
	navigationControlEngine->addInputVariable(angleToOppGoal);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configTeamGoalVariables(Engine *navigationControlEngine, InputVariable *closestToGoal, InputVariable *distanceToOurGoal, InputVariable *angleToOurGoal) {
	closestToGoal->setName("closestToGoal");
	closestToGoal->setDescription("Checks if controlled robot is the closest to team`s goal");
	closestToGoal->setEnabled(true);
	closestToGoal->setRange(0.000, 2.000);
	closestToGoal->setLockValueInRange(false);
	closestToGoal->addTerm(new Triangle("closestToGoal", 1, 1, 1));
	navigationControlEngine->addInputVariable(closestToGoal);

	distanceToOurGoal->setName("distanceToOurGoal");
	distanceToOurGoal->setDescription("Distance to Opponent`s goal from main robot");
	distanceToOurGoal->setEnabled(true);
	distanceToOurGoal->setRange(0.000, 10.000);
	distanceToOurGoal->setLockValueInRange(false);
	distanceToOurGoal->addTerm(new Trapezoid("inGoal", 0, 0, 0.1, 0.1));
	distanceToOurGoal->addTerm(new Triangle("close", 0.1, 0.15, 0.3));
	distanceToOurGoal->addTerm(new Trapezoid("closeModerate", 0.2, 0.4, 0.6, 0.8));
	distanceToOurGoal->addTerm(new Trapezoid("moderate", 0.5, 0.8, 1.1, 1.4));
	distanceToOurGoal->addTerm(new Trapezoid("moderateFar", 1, 1.5, 2, 2.5));
	distanceToOurGoal->addTerm(new Trapezoid("far", 2, 2.5, 9.5, 9.5));
	navigationControlEngine->addInputVariable(distanceToOurGoal);

	angleToOurGoal->setName("angleToOurGoal");
	angleToOurGoal->setDescription("Checks the angle of team`s goal towards the robot");
	angleToOurGoal->setEnabled(true);
	angleToOurGoal->setRange(-3.14, 3.14);
	angleToOurGoal->setLockValueInRange(false);
	angleToOurGoal->addTerm(new Triangle("littleToLeft", 0, 0.502, 1.06));
	angleToOurGoal->addTerm(new Trapezoid("moderateToLeft",0.4854, 0.8403, 1.297, 1.617));
	angleToOurGoal->addTerm(new Trapezoid("aLotToLeft",1.057, 1.767, 3.017, 3.297));
	angleToOurGoal->addTerm(new Triangle("littleToRight", -1.06, -0.502, 0));
	angleToOurGoal->addTerm(new Trapezoid("moderateToRight", -1.617, -1.297, -0.8403, -0.4854));
	angleToOurGoal->addTerm(new Trapezoid("aLotToRight", -3.297, -3.017, -1.767, -1.057));
	navigationControlEngine->addInputVariable(angleToOurGoal);

}

//////////////////////////////////////////////////////////////////////////////////////////////////
void configPanAngleOutputVariable(Engine *gazeControlEngine, OutputVariable *panAngle) {
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
	gazeControlEngine->addOutputVariable(panAngle);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configNavigationOutputVariables(Engine *navigationControlEngine, OutputVariable *velocityX, OutputVariable *velocityTheta) {
	velocityX->setName("velocityX");
	velocityX->setDescription("Sets robot`s x-axis velocity (relative to robot)");
	velocityX->setEnabled(true);
	velocityX->setRange(-1.000, 1.000);
	velocityX->setLockValueInRange(false);
	velocityX->setAggregation(new Maximum);
	velocityX->setDefuzzifier(new Centroid(100));
	velocityX->setDefaultValue(0);
	velocityX->addTerm(new Triangle("slowForward", 0, 0.1, 0.2));
	velocityX->addTerm(new Triangle("slowModerateForward", 0.1, 0.2, 0.3));
	velocityX->addTerm(new Trapezoid("moderateForward", 0.2, 0.3, 0.4, 0.5));
	velocityX->addTerm(new Trapezoid("moderateFastForward", 0.4, 0.5, 0.7, 0.8));
	velocityX->addTerm(new Triangle("fastForward", 0.7, 0.85, 1.0));
	navigationControlEngine->addOutputVariable(velocityX);

	velocityTheta->setName("velocityTheta");
	velocityTheta->setDescription("Sets robot`s z-rotation velocity (relative to robot)");
	velocityTheta->setEnabled(true);
	velocityTheta->setRange(-1.000, 1.000);
	velocityTheta->setLockValueInRange(false);
	velocityTheta->setAggregation(new Maximum);
	velocityTheta->setDefuzzifier(new Centroid(100));
	velocityTheta->setDefaultValue(0);
	velocityTheta->addTerm(new Trapezoid("littleToLeft", 0, 0.1, 0.2, 0.3));
	velocityTheta->addTerm(new Trapezoid("moderateToLeft", 0.15, 0.35, 0.55, 0.75));
	velocityTheta->addTerm(new Trapezoid("aLotToLeft", 0.5, 0.8, 0.9, 1.0));
	velocityTheta->addTerm(new Trapezoid("littleToRight", -0.3, -0.2, -0.1, 0));
	velocityTheta->addTerm(new Trapezoid("moderateToRight", -0.75, -0.55, -0.35, -0.15));
	velocityTheta->addTerm(new Trapezoid("aLotToRight", -1.0, -0.9, -0.8, -0.5));
	navigationControlEngine->addOutputVariable(velocityTheta);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configPanRules(Engine *gazeControlEngine, RuleBlock *gazeControlRuleBlock) {
	gazeControlRuleBlock->setName("gazeControlRuleBlock");
	gazeControlRuleBlock->setDescription("");
	gazeControlRuleBlock->setEnabled(true);
	gazeControlRuleBlock->setConjunction(new Minimum);
	gazeControlRuleBlock->setDisjunction(new Maximum);
	gazeControlRuleBlock->setImplication(new AlgebraicProduct);
	gazeControlRuleBlock->setActivation(new General);
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is littleToLeft and BallConfidence is lowConfidence then panAngle is panToBallLittleToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is moderateToLeft and BallConfidence is lowConfidence then panAngle is panToBallModerateToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is aLotToLeft and BallConfidence is lowConfidence then panAngle is panToBallALotToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is littleToRight and BallConfidence is lowConfidence then panAngle is panToBallLittleToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is moderateToRight and BallConfidence is lowConfidence then panAngle is panToBallModerateToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if BallDistance is doNotHaveBall and BallAngle is aLotToRight and BallConfidence is lowConfidence then panAngle is panToBallALotToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is littleToLeft then panAngle is panLittleToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is littleToRight then panAngle is panLittleToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is moderateToLeft then panAngle is panModerateToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence1 is lowConfidence and oppRobotAngle1 is moderateToRight then panAngle is panModerateToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is littleToLeft then panAngle is panLittleToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is littleToRight then panAngle is panLittleToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is moderateToLeft then panAngle is panModerateToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence2 is lowConfidence and oppRobotAngle2 is moderateToRight then panAngle is panModerateToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is littleToLeft then panAngle is panLittleToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is littleToRight then panAngle is panLittleToRight", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is moderateToLeft then panAngle is panModerateToLeft", gazeControlEngine));
	gazeControlRuleBlock->addRule(Rule::parse("if oppConfidence3 is lowConfidence and oppRobotAngle3 is moderateToRight then panAngle is panModerateToRight", gazeControlEngine));
	gazeControlEngine->addRuleBlock(gazeControlRuleBlock);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void configNavigationRules(Engine *navigationControlEngine, RuleBlock *navigationControlRuleBlock) {
	navigationControlRuleBlock->setName("navigationControlRuleBlock");
	navigationControlRuleBlock->setDescription("");
	navigationControlRuleBlock->setEnabled(true);
	navigationControlRuleBlock->setConjunction(new Minimum);
	navigationControlRuleBlock->setDisjunction(new Maximum);
	navigationControlRuleBlock->setImplication(new AlgebraicProduct);
	navigationControlRuleBlock->setActivation(new General);

	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is littleToLeft and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is littleToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is littleToRight and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is littleToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is moderateToLeft and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is moderateToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is moderateToRight and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is moderateToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is aLotToLeft and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is aLotToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if BallAngle is aLotToRight and teamHaveBall is not definetelyHaveBall and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityTheta is aLotToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and distanceBall is close and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityX is slowForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and distanceBall is closeModerate and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityX is slowModerateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and distanceBall is moderate and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityX is moderateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and distanceBall is moderateFar and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityX is moderateFastForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and distanceBall is far and closestToBall is closestToBall and closestToGoal is not closestToGoal then velocityX is fastForward", navigationControlEngine));

	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is littleToLeft then velocityTheta is littleToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is littleToRight then velocityTheta is littleToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is moderateToLeft then velocityTheta is moderateToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is moderateToRight then velocityTheta is moderateToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is aLotToLeft then velocityTheta is aLotToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and angleToOppGoal is aLotToRight then velocityTheta is aLotToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and distanceToOppGoal is close then velocityX is slowForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and distanceToOppGoal is closeModerate then velocityX is slowModerateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and distanceToOppGoal is moderate then velocityX is moderateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and distanceToOppGoal is moderateFar then velocityX is moderateFastForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is definetelyHaveBall and distanceToOppGoal is far then velocityX is fastForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is littleToLeft then velocityTheta is littleToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is littleToRight then velocityTheta is littleToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is moderateToLeft then velocityTheta is moderateToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is moderateToRight then velocityTheta is moderateToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is aLotToLeft then velocityTheta is aLotToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is not inGoal and angleToOurGoal is aLotToRight then velocityTheta is aLotToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is littleToLeft then velocityTheta is aLotToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is littleToRight then velocityTheta is aLotToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is moderateToLeft then velocityTheta is moderateToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is moderateToRight then velocityTheta is moderateToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is aLotToLeft then velocityTheta is littleToLeft", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is inGoal and angleToOurGoal is aLotToRight then velocityTheta is littleToRight", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is close then velocityX is slowForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is closeModerate then velocityX is slowModerateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is moderate then velocityX is moderateForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is moderateFar then velocityX is moderateFastForward", navigationControlEngine));
	navigationControlRuleBlock->addRule(Rule::parse("if teamHaveBall is not definetelyHaveBall and closestToGoal is closestToGoal and distanceToOurGoal is far then velocityX is fastForward", navigationControlEngine));


	navigationControlEngine->addRuleBlock(navigationControlRuleBlock);
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
void processFuzzy(Engine *gazeControlEngine, InputVariable *ballReferenceForGazeControl, InputVariable *ballAngle, InputVariable *ballConfidence, InputVariable* opp1Confidence, InputVariable* opp1Angle,InputVariable* opp2Confidence, InputVariable* opp2Angle,InputVariable* opp3Confidence, InputVariable* opp3Angle, OutputVariable *panAngle, const boost::tuple<simxFloat, simxFloat, simxFloat>& robot_coord,  const boost::tuple<simxFloat, simxFloat, simxFloat> robot_orient, simxFloat* ball_coord, simxFloat ball_confidence, const std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>& opp_robots_coords, const std::vector<simxFloat> opp_robots_confidence) {
	string status;
	if(not gazeControlEngine->isReady(&status))
		throw Exception("[gazeControlEngine error] gazeControlEngine is not ready:n" + status, FL_AT);

	simxFloat x_coord = boost::get<0>(robot_coord);
	simxFloat y_coord = boost::get<1>(robot_coord);
	simxFloat z_angle = boost::get<2>(robot_orient);

	double diffX = ball_coord[0] - x_coord;
	double diffY = ball_coord[1] - y_coord;
	double distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
	double angle = calculateAngle(diffX, diffY, distance);

	ballReferenceForGazeControl->setValue(distance);
	ballAngle->setValue(calculateAngleForFuzzy(angle, z_angle));
	ballConfidence->setValue(ball_confidence);

	opp1Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(0), x_coord, y_coord, z_angle));
	opp2Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(1), x_coord, y_coord, z_angle));
	opp3Angle->setValue(calculateAngleForOppRobotsFuzzy(opp_robots_coords.at(2), x_coord, y_coord, z_angle));
	opp1Confidence->setValue(opp_robots_confidence.at(0));
	opp2Confidence->setValue(opp_robots_confidence.at(1));
	opp3Confidence->setValue(opp_robots_confidence.at(2));

	gazeControlEngine->process();
}
//////////////////////////////////////////////////////////////////////////////////////////////////
simxFloat calculateDistributionBasedOnAngleInDegrees(simxFloat alpha, simxFloat robotAngle, simxFloat dist) {
	simxFloat angleInDegrees = (std::abs(alpha - robotAngle)*180)/M_PI;
	simxFloat result = FOVdistr[int(angleInDegrees + 30)]*rangeDistr[int(dist)];
	return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void setOccupation(float m[][MAP_Y], float modified[][MAP_Y], int i, int j, simxFloat alpha, simxFloat z_angle, simxFloat dist) {
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
		gp << gp.binFile2d(pts, "record") << "with lines title 'confidence versus map coordinates'";
	}
	gp << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {

  try {
    /** Create a ALMotionProxy to call the methods to move NAO's head.
    * Arguments for the constructor are:
    * - IP adress of the robot
    * - port on which NAOqi is listening, by default 9559
    */

    AL::ALMotionProxy motion("127.0.0.1:39501", 9559);

    std::vector<float> listAngles;
    std::vector<float> parallel;

    AL::ALValue x = 0.5f;
    AL::ALValue y = 0.5f;
    AL::ALValue theta = 0.5f;

    simxInt ball_id, goal_left_id, goal_right_id;
    simxFloat ball_coord[3], goal_left_coord[3], goal_right_coord[3], aux_coord[3], ball_confidence;
    std::vector<std::string> team_robots_joints, team_robots_head;
    std::vector<simxInt> team_robots_joints_ids, team_robots_head_ids;
    std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_coords;
    std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_orient;
    std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> team_robots_head_orient;
    std::vector<std::string> opp_robots;
    std::vector<simxInt> opp_robots_ids;
    std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_coords;
    std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>>  opp_robots_orient;
    std::vector<simxFloat> opp_robots_confidence;

    Gnuplot gpRobot1, gpRobot2, gpRobot3;

    Engine *gazeControlEngine = new Engine;
    InputVariable *ballAngle = new InputVariable;
    InputVariable *ballReferenceForGazeControl = new InputVariable;
    InputVariable *ballConfidence = new InputVariable;
    InputVariable *opp1Confidence = new InputVariable;
    InputVariable *opp1Angle = new InputVariable;
    InputVariable *opp2Confidence = new InputVariable;
    InputVariable *opp2Angle = new InputVariable;
    InputVariable *opp3Confidence = new InputVariable;
    InputVariable *opp3Angle = new InputVariable;
    OutputVariable *panAngle = new OutputVariable;
    RuleBlock *gazeControlRuleBlock = new RuleBlock;

		Engine *navigationControlEngine = new Engine;
		InputVariable *distanceBall = new InputVariable;
		InputVariable *teamHaveBall = new InputVariable;
		InputVariable *closestToBall = new InputVariable;
		InputVariable *distanceToOppGoal = new InputVariable;
		InputVariable *angleToOppGoal = new InputVariable;
		InputVariable *closestToGoal = new InputVariable;
		InputVariable *distanceToOurGoal = new InputVariable;
		InputVariable *angleToOurGoal = new InputVariable;
		OutputVariable *velocityX = new OutputVariable;
		OutputVariable *velocityTheta = new OutputVariable;
		RuleBlock *navigationControlRuleBlock = new RuleBlock;

    startGazeControlEngine(gazeControlEngine);
		startNavigationControlEngine(navigationControlEngine);

    configBallInputVariables(gazeControlEngine, navigationControlEngine, ballReferenceForGazeControl, ballAngle, ballConfidence, distanceBall, teamHaveBall, closestToBall);
    configOppPlayerInputVariables(gazeControlEngine, opp1Angle, opp1Confidence, 1);
    configOppPlayerInputVariables(gazeControlEngine, opp2Angle, opp2Confidence, 2);
    configOppPlayerInputVariables(gazeControlEngine, opp3Angle, opp3Confidence, 3);
    configPanAngleOutputVariable(gazeControlEngine, panAngle);
    configPanRules(gazeControlEngine, gazeControlRuleBlock);
		configOppGoalVariables(navigationControlEngine, distanceToOppGoal, angleToOppGoal);
		configTeamGoalVariables(navigationControlEngine, closestToGoal, distanceToOurGoal, angleToOurGoal);
		configNavigationOutputVariables(navigationControlEngine, velocityX, velocityTheta);
		configNavigationRules(navigationControlEngine, navigationControlRuleBlock);


    generateNormalDistributionRange();
    generateNormalDistributionFOV();

    auto sim = new Simulator("127.0.0.1", PORT_NUMBER);
    Simulator &vrep = *sim;
    vrep.connectServer();
    Communication comm(vrep.getClientID());
		auto f = std::async(std::launch::async, &Communication::updateJointsPositions, comm);

    simxInt naoJointsIds[24];
    int map[MAP_X][MAP_Y][N_MAP_ELEMENTS];
    resetMap(map);

		std::vector<simxFloat> auxCoord;
		auxCoord = comm.getOurGoalCoords();
		goal_left_coord[0] = auxCoord.at(0);
		goal_left_coord[1] = auxCoord.at(1);
		auxCoord.clear();
		auxCoord = comm.getOppGoalCoords();
		goal_right_coord[0] = auxCoord.at(0);
		goal_right_coord[1] = auxCoord.at(1);
		auxCoord.clear();
		auxCoord = comm.getBallCoords();
		ball_coord[0] = auxCoord.at(0);
		ball_coord[1] = auxCoord.at(1);

    float occupancy[MAP_X][MAP_Y], occupancy2[MAP_X][MAP_Y], occupancy3[MAP_X][MAP_Y];
    resetOccupancyGrid(occupancy);
    if(!IS_MAP_SHARED) {
      resetOccupancyGrid(occupancy2);
      resetOccupancyGrid(occupancy3);
    }

    bool valid;
		bool first = true;
		int time = 0;
    while(true) {
      sleep(1);
			if(first) {
				sleep(30);
				first = false;

			}
      team_robots_coords.clear();
      team_robots_orient.clear();
      team_robots_head_orient.clear();

			std::vector<simxFloat> auxCoords;

			auxCoords.clear();

			auxCoords = comm.getRobot0Coords();
      team_robots_coords.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			auxCoords.clear();
			auxCoords = comm.getOpp0Coords();
			opp_robots_coords.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			auxCoords.clear();
			auxCoords = comm.getOpp1Coords();
			opp_robots_coords.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			auxCoords.clear();
			auxCoords = comm.getOpp2Coords();
			opp_robots_coords.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			auxCoords.clear();

			auxCoords = comm.getRobot0Orientation();
			team_robots_orient.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			cout << auxCoords[2];
			auxCoords.clear();

			auxCoords = comm.getRobot0HeadOrientation();
			team_robots_head_orient.push_back(boost::make_tuple(auxCoords.at(0),auxCoords.at(1),auxCoords.at(2)));
			auxCoords.clear();

      if(IS_MAP_SHARED) {
        updateOccupancyGrid2(occupancy, team_robots_coords, team_robots_head_orient);
      }
      else {
        std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> aux;
        std::vector<boost::tuple<simxFloat, simxFloat, simxFloat>> aux2;
        aux.clear();
        aux2.clear();
        aux.push_back(team_robots_coords.at(0));
        aux2.push_back(team_robots_orient.at(0));
        updateOccupancyGrid2(occupancy, aux, aux2);
        aux.clear();
        aux2.clear();
        aux.push_back(team_robots_coords.at(1));
        aux2.push_back(team_robots_orient.at(1));
        updateOccupancyGrid2(occupancy2, aux, aux2);
        aux.clear();
        aux2.clear();
        aux.push_back(team_robots_coords.at(2));
        aux2.push_back(team_robots_orient.at(2));
        updateOccupancyGrid2(occupancy3, aux, aux2);
      }
			/*  -- GAZE CONTROL ROUTINE
      if(IS_MAP_SHARED) {
        ball_confidence = occupancy[convertX(ball_coord[0])][convertY(ball_coord[1])];
        opp_robots_confidence.clear();
        for(int i = 0; i < opp_robots_coords.size(); i++) {
          boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
          opp_robots_confidence.push_back(occupancy[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
        }

        for(int i = 0; i < team_robots_coords.size(); i++) {
          processFuzzy(gazeControlEngine, ballReferenceForGazeControl, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(i), team_robots_orient.at(i), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
          double speed = panAngle->getValue();
          cout << "calculated speed " << i << ": " << speed << endl;
          valid = false;
          while(!valid) {
            auto result = simxSetJointTargetVelocity(vrep.getClientID(), team_robots_joints_ids.at(i), speed, simx_opmode_oneshot_wait);
            if(result == simx_return_ok)
            valid = true;
          }
          //vrep.setJointVelocity(team_robots_joints_ids.at(i), 0.5);
        }
      }
      else {
        ball_confidence = occupancy[convertX(ball_coord[0])][convertY(ball_coord[1])];
        opp_robots_confidence.clear();
        for(int i = 0; i < opp_robots_coords.size(); i++) {
          boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
          opp_robots_confidence.push_back(occupancy[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
        }
        processFuzzy(gazeControlEngine, ballReferenceForGazeControl, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(0), team_robots_orient.at(0), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
        double speed = panAngle->getValue();
        cout << "calculated speed "  << ": " << speed << endl;
        valid = false;
        while(!valid) {
          auto result = simxSetJointTargetVelocity(vrep.getClientID(), team_robots_joints_ids.at(0), speed, simx_opmode_oneshot_wait);
          if(result == simx_return_ok)
            valid = true;
        }
        //vrep.setJointVelocity(team_robots_joints_ids.at(0), speed);

        ball_confidence = occupancy2[convertX(ball_coord[0])][convertY(ball_coord[1])];
        opp_robots_confidence.clear();
        for(int i = 0; i < opp_robots_coords.size(); i++) {
          boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
          opp_robots_confidence.push_back(occupancy2[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
        }
        processFuzzy(gazeControlEngine, ballReferenceForGazeControl, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(1), team_robots_orient.at(1), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
        speed = panAngle->getValue();
        cout << "calculated speed "  << ": " << speed << endl;
        valid = false;
        while(!valid) {
          auto result = simxSetJointTargetVelocity(vrep.getClientID(), team_robots_joints_ids.at(1), speed, simx_opmode_oneshot_wait);
          if(result == simx_return_ok)
            valid = true;
        }
        //vrep.setJointVelocity(team_robots_joints_ids.at(1), speed);

        ball_confidence = occupancy3[convertX(ball_coord[0])][convertY(ball_coord[1])];
        opp_robots_confidence.clear();
        for(int i = 0; i < opp_robots_coords.size(); i++) {
          boost::tuple<simxFloat, simxFloat, simxFloat> coord = opp_robots_coords.at(i);
          opp_robots_confidence.push_back(occupancy3[convertX(boost::get<0>(coord))][convertY(boost::get<1>(coord))]);
        }
        processFuzzy(gazeControlEngine, ballReferenceForGazeControl, ballAngle, ballConfidence, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, opp1Confidence, opp1Angle, panAngle, team_robots_coords.at(2), team_robots_orient.at(2), ball_coord, ball_confidence, opp_robots_coords, opp_robots_confidence);
        speed = panAngle->getValue();
        cout << "calculated speed "  << ": " << speed << endl;
        valid = false;
        while(!valid) {
          auto result = simxSetJointTargetVelocity(vrep.getClientID(), team_robots_joints_ids.at(2), speed, simx_opmode_oneshot_wait);
          if(result == simx_return_ok)
            valid = true;
        }
      }
			*/
			string status;
			if(not navigationControlEngine->isReady(&status))
				throw Exception("[gazeControlEngine error] gazeControlEngine is not ready:n" + status, FL_AT);

			auxCoord.clear();
			auxCoord = comm.getBallCoords();
			ball_coord[0] = auxCoord.at(0);
			ball_coord[1] = auxCoord.at(1);

			//init fuzzy variables
			simxFloat x_coord = boost::get<0>(team_robots_coords.at(0));
			simxFloat y_coord = boost::get<1>(team_robots_coords.at(0));
			simxFloat z_angle = boost::get<2>(team_robots_orient.at(0));

			double diffX = ball_coord[0] - x_coord;
			double diffY = ball_coord[1] - y_coord;
			double distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
			double angle = calculateAngle(diffX, diffY, distance);
			ballReferenceForGazeControl->setValue(distance);
			double angleForFuzzy = calculateAngleForFuzzy(angle, z_angle);
			ballAngle->setValue(calculateAngleForFuzzy(angle, z_angle));

			distanceBall->setValue(distance);

			diffX = goal_left_coord[0] - x_coord;
			diffY = goal_left_coord[1] - y_coord;
			distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
			angle = calculateAngle(diffX, diffY, distance);
			angleForFuzzy = calculateAngleForFuzzy(angle, z_angle);

			distanceToOurGoal->setValue(distance);
			angleToOurGoal->setValue(angleForFuzzy);

			diffX = goal_right_coord[0] - x_coord;
			diffY = goal_right_coord[1] - y_coord;
			distance = sqrt(pow(diffX, 2) + pow(diffY, 2));
			angle = calculateAngle(diffX, diffY, distance);
			angleForFuzzy = calculateAngleForFuzzy(angle, z_angle);

			distanceToOppGoal->setValue(distance);
			angleToOppGoal->setValue(angleForFuzzy);
			teamHaveBall->setValue(0);
			closestToBall->setValue(1);
			closestToGoal->setValue(0);
			navigationControlEngine->process();
			cout << " "<< velocityX->getValue();
			cout << " " << velocityTheta->getValue();

			cout << endl;
			motion.moveToward(velocityX->getValue(), 0, velocityTheta->getValue());

			time++;
    }

    vrep.disconnectServer();
  }
  catch (const AL::ALError& e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(1);
  }
  exit(0);
}
