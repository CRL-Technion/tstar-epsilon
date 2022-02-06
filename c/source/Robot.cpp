//
// Created by doron.pinsky on 29/01/2020.
//

#include "Robot.h"
#include <iostream>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

Robot::Robot()
{
    uMax = 1;
    vMin = 0.5;
    vMax = 1;
    eta = M_PI / 2;
    numOfOreintations = 8;
    m_stepsize = 0.05;
    m_r = vMin / uMax;
    m_R = vMax / uMax;
    m_speed = 0; 
}

void Robot::setuMax(double uMaxIn)
{
    uMax = uMaxIn;
    m_r = vMin / uMax;
    m_R = vMax / uMax;
}

void Robot::setvMax(double vMaxIn)
{
    vMax = vMaxIn;
    m_R = vMax / uMax;
}

void Robot::setvMin(double vMinIn)
{
    vMin = vMinIn;
    m_r = vMin / uMax;
}

void Robot::setEta(double etaIn)
{
    eta = etaIn;
}

void Robot::setNumOfOreintations(int orientationIn)
{
    numOfOreintations = orientationIn;
}

void Robot::setSpeed(int speed) {
    m_speed = speed;
}

int Robot::getSpeed() {
    return m_speed;
}

double Robot::get_r() {
    return m_r;
}
double Robot::get_R() {
    return m_R;
}

double Robot::getuMax()
{
    return uMax;
}

double Robot::getvMax()
{
    return vMax;
}

double Robot::getvMin()
{
    return vMin;
}

double Robot::getEta()
{
    return eta;
}

double Robot::getStepSize(){
    return m_stepsize;
}

int Robot::getNumOfOrientation()
{
    return numOfOreintations;
}

void Robot::printInfo()
{
    std::cout << "uMax = " << uMax << " [r/s]\n" << "vMax = " << vMax << " [m/s]\n" << "vMin = " << vMin << " [m/s]\n";
    std::cout << "eta = " << eta << "\n" << "number of orientations = " << numOfOreintations << "\n";
}

void Robot::setOcpsStatus(bool ocps) {
	_ocpsStatus = ocps;
}

bool Robot::getOcpsStatus(){
	return _ocpsStatus;
}

