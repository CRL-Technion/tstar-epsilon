#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

// custom
#include<VSDUtils.h>
#include<VSDPath.h>
#include<VSDProblem.h>
#include<VSDSolver.h>
#include<VSDNLP.h>
#include "VarSpeed.h"
#include "VarSpeed_utils.h"

enum DUBINS_TYPE {
    LSL = 0,
    LSR,
    RSL,
    RSR,
    RLR,
    LRL
};

enum SEG_TYPE {
    L_SEG = 1,
    S_SEG,
    R_SEG
};

//typedef struct Candidate {
//	int valid;
//	long key;
//	double cost;
//	Coordinate start;
//	int h_initial;
//	Coordinate target;
//	int h_final;
//	std::string path_type;
//	std::string path_orientation;
//	std::vector<Coordinate>* reduced_path = new std::vector<Coordinate>;
//} Candidate;

typedef struct DubinsState {
    int row;
    int col;
    int theta;
    int v;
} DubinsState;

double groundSpeedFromHeading(double heading, Wind wind, double speed) {
    // max speed
    double dir_x = cos(heading) * speed + wind.x;
    double dir_y = sin(heading) * speed + wind.y;
    return sqrt(dir_x * dir_x + dir_y * dir_y);
}

double getGroundSpeedDubins(double dir_x, double dir_y, double wind_x, double wind_y, double speed){
    // normalize dir vector
    double tmp_len = sqrt(dir_x * dir_x + dir_y * dir_y);
    dir_x /= tmp_len;
    dir_y /= tmp_len;

    double b = wind_x * dir_x + wind_y * dir_y;
    double c = wind_x * wind_x + wind_y * wind_y - speed * speed;
    return (b + sqrt(b*b - c));
}

void dubinsLSL(double alpha, double beta, double d, double r, double* out)
{
    double pSquared = (d * d) + 2 * r * r * (1 - cos(alpha - beta)) + 2 * d * (r * sin(alpha) - r * sin(beta));
    if (pSquared < 0) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = sqrt(pSquared);
        double num1 = d + r * sin(alpha) - r * sin(beta);
        double num2 = r * cos(beta) - r * cos(alpha);
        double tmp1 = fmod(atan2(p*num2, p*num1) + 2*M_PI, 2*M_PI);
        double t = fmod(-alpha + tmp1 + 2*M_PI, 2*M_PI);
        double q = fmod(beta - tmp1 + 2 * M_PI, 2 * M_PI);

        if (p == 0) {
            double temp = fmod(t * r + q * r, 2 * M_PI) / 2;
            out[0] = temp;
            out[1] = p;
            out[2] = temp;;

        }
        else {
            out[0] = t * r;
            out[1] = p;
            out[2] = q * r;
        }
    }
    return;
}

void dubinsLSR(double alpha, double beta, double d, double r, double* out)
{
    double pSquared = (d * d) + 2 * r * r * (cos(alpha - beta) - 1) + 2 * d * (r * sin(alpha) + r * sin(beta));
    if (pSquared < 0) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = sqrt(pSquared);
        double num1 = d + r * sin(alpha) + r * sin(beta);
        double num2 = r * cos(beta) + r * cos(alpha);
        double tmp1 = fmod(atan2(2 * r * num1 - p * num2, p * num1 + 2 * r * num2) + 2 * M_PI, 2 * M_PI);
        double t = fmod(-alpha + tmp1 + 2 * M_PI, 2 * M_PI);
        double q = fmod(-beta + tmp1 + 2 * M_PI, 2 * M_PI);

        out[0] = t * r;
        out[1] = p;
        out[2] = q * r;
    }
    return;
}

void dubinsRSL(double alpha, double beta, double d, double r, double* out)
{
    double pSquared = (d * d) + 2 * r * r * (cos(alpha - beta) - 1) - 2 * d * (r * sin(alpha) + r * sin(beta));
    if (pSquared < 0) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = sqrt(pSquared);
        double num1 = d - r * sin(alpha) - r * sin(beta);
        double num2 = r * cos(beta) + r * cos(alpha);
        double tmp1 = fmod(atan2(-2 * r * num1 + p * num2, p * num1 + 2 * r * num2) + 2 * M_PI, 2 * M_PI);
        double t = fmod(alpha - tmp1 + 2 * M_PI, 2 * M_PI);
        double q = fmod(beta - tmp1 + 2 * M_PI, 2 * M_PI);

        out[0] = t * r;
        out[1] = p;
        out[2] = q * r;
    }
    return;
}

void dubinsRSR(double alpha, double beta, double d, double r, double* out)
{
    double pSquared = (d * d) + 2 * r * r * (1 - cos(alpha - beta)) + 2 * d * (-r * sin(alpha) + r * sin(beta));
    if (pSquared < 0) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = sqrt(pSquared);
        double num1 = d - r * sin(alpha) + r * sin(beta);
        double num2 = -r * cos(beta) + r * cos(alpha);
        double tmp1 = fmod(atan2(p * num2, p * num1) + 2 * M_PI, 2 * M_PI);
        double t = fmod(alpha - tmp1 + 2 * M_PI, 2 * M_PI);
        double q = fmod(-beta + tmp1 + 2 * M_PI, 2 * M_PI);

        if (p == 0) {
            double temp = fmod(t * r + q * r,2*M_PI)/2;
            out[0] = temp;
            out[1] = p;
            out[2] = temp;;

        }
        else {
            out[0] = t * r;
            out[1] = p;
            out[2] = q * r;
        }
    }
    return;
}

void dubinsRLR(double alpha, double beta, double d, double r, double* out)
{
    double _d = d / r;
    double tmp_rlr = (6 - _d * _d + 2 * cos(alpha - beta) + 2 * _d * (sin(alpha) - sin(beta))) / 8;
    if (fabs(tmp_rlr) > 1) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = fmod(2 * M_PI - acos(tmp_rlr), 2 * M_PI);
        double t = fmod(atan2(cos(alpha) - cos(beta), _d - sin(alpha) + sin(beta)) + 2 * M_PI, 2 * M_PI);
        t = fmod(alpha - t + 2 * M_PI, 2 * M_PI);
        t = fmod(t + p / 2, 2 * M_PI);
        double q = fmod(alpha - beta + 2 * M_PI, 2 * M_PI);
        q = fmod(q - t + 2 * M_PI, 2 * M_PI);
        q = fmod(q + p, 2 * M_PI);

        out[0] = t * r;
        out[1] = p * r;
        out[2] = q * r;
    }
    return;
}

void dubinsLRL(double alpha, double beta, double d, double r, double* out)
{
    double _d = d / r;
    double tmp_lrl = (6 - _d * _d + 2 * cos(alpha - beta) + 2 * _d * (-sin(alpha) + sin(beta))) / 8;
    if (fabs(tmp_lrl) > 1) {
        for (int i = 0; i < 3; ++i) {
            out[i] = -1;
        }
    }
    else {
        double p = 2 * M_PI - acos(tmp_lrl);
        double t = fmod(atan2(cos(alpha) - cos(beta), _d + sin(alpha) - sin(beta)) + 2 * M_PI, 2 * M_PI);
        t = fmod(-alpha - t + 2 * M_PI, 2 * M_PI);
        t = fmod(t + p / 2, 2 * M_PI);
        double q = fmod(beta - alpha + 2 * M_PI, 2 * M_PI);
        q = fmod(q - t + 2 * M_PI, 2 * M_PI);
        q = fmod(q + p, 2 * M_PI);

        out[0] = t * r;
        out[1] = p * r;
        out[2] = q * r;
    }
    return;
}

double dubinsLength(double* path){
    return path[0] + path[1] + path[2];

}

void dubinsSegment(double param, double segInit[3], int seg_type, double r, double* out){
    if (seg_type == L_SEG)
    {
        out[0] = segInit[0] + r * sin(segInit[2] + param/r) - r * sin(segInit[2]);
        out[1] = segInit[1] - r * cos(segInit[2] + param/r) + r * cos(segInit[2]);
        out[2] = segInit[2] + param/r;
    }
    else if (seg_type == R_SEG)
    {
        out[0] = segInit[0] - r * sin(segInit[2] - param / r) + r * sin(segInit[2]);
        out[1] = segInit[1] + r * cos(segInit[2] - param / r) - r * cos(segInit[2]);
        out[2] = segInit[2] - param / r;
    }
    else if (seg_type == S_SEG)
    {
        out[0] = segInit[0] + cos(segInit[2]) * param;
        out[1] = segInit[1] + sin(segInit[2]) * param;
        out[2] = segInit[2];
    }
    return;
}

void dubinsPathSample(DubinsState s, double* path, double t, double* out, double r)
{
//    double r = robot->get_r();
//    if (robot->getSpeed() == 1) {
//        r = robot->get_R();
//    }
    double tPrime = t;
    double absThStart = (double)s.theta * (2 * M_PI) / ORIENTATIONS;
    double pInit[3] = { 0, 0, absThStart };
    int dirData[6][3] = { L_SEG, S_SEG, L_SEG,
                          L_SEG, S_SEG, R_SEG,
                          R_SEG, S_SEG, L_SEG,
                          R_SEG, S_SEG, R_SEG,
                          R_SEG, L_SEG, R_SEG,
                          L_SEG, R_SEG, L_SEG};

    int types[3] = { dirData[(int)path[3]][0], dirData[(int)path[3]][1], dirData[(int)path[3]][2] };
    double param1 = path[0];
    double param2 = path[1];
    double midPt1[3] = {};
    dubinsSegment(param1, pInit, types[0], r, midPt1);

    double midPt2[3] = {};
    dubinsSegment(param2, midPt1, types[1], r, midPt2);

    double endPt[3] = {};
    if (tPrime < param1) {
        dubinsSegment(tPrime, pInit, types[0], r, endPt);
    }
    else if (tPrime < (param1 + param2)) {
        dubinsSegment(tPrime - param1, midPt1, types[1], r, endPt);
    }
    else {
        dubinsSegment(tPrime - param1 - param2, midPt2, types[2], r, endPt);
    }

    out[0] = endPt[1] + s.row;
    out[1] = endPt[0] + s.col;
    out[2] = fmod(endPt[2] + 2 * M_PI, 2 * M_PI);

    return;
}

double pathValidityCheck(DubinsState s, double* path, double cost, double r, Wind wind)
{
    double step = 0.001;
    double x = 0;
    double length = cost;
    int temp = (int)floor(length / step);
    int pathRows = (int)temp + 1;
    double** pathMid = new double* [pathRows]; //descretizied path in row,col,theta
    for (int i = 0; i < pathRows; ++i) {
        pathMid[i] = new double[3];
    }

    // build all the mid points of the path
    int i = 0;

    while (x <= length) {
        double tempPath[3] = { -1, -1, -1 };
        dubinsPathSample(s, path, x, tempPath, r);
        for (int j = 0; j < 3; j++) {
            pathMid[i][j] = tempPath[j];
        }
        /*for (int k = 0; k < 3; k++){
            cout<<"["<<tempPath[0]<<", "<<tempPath[1]<<", "<<tempPath[2]<<"]"<<endl;
        }*/
        x += step;
        i++;
    }
    double lower_bound = 0;
    for (int k = 0; k < i; ++k) {
        int row = (int)round(pathMid[k][0]);
        int col = (int)round(pathMid[k][1]);
        double heading = pathMid[k][2];
        double gs = groundSpeedFromHeading(heading, wind, 1);
        lower_bound += step / gs;
//        if (!map->isValid(row,col) || !map->collisionFree(row,col)){
//            return false;
//        }
//        double buffer_size = map->getObstacleClearance();
//        for (int theta = 0; theta < 8; ++theta) {
//            double angle = theta * 2 * M_PI / 8;
//            double x_circle = buffer_size * cos(angle) + pathMid[k][1];
//            double y_circle = buffer_size * sin(angle) + pathMid[k][0];
//            int c_row = (int)round(y_circle);
//            int c_col = (int)round(x_circle);
//            if (!map->isValid(c_row, c_col) || !map->collisionFree(c_row, c_col)) {
//                return false;
//            }
//        }
    }

    // pathCells[0] = i - 1; // the actual length of the array
    for (i = 0; i < pathRows; ++i) {
        delete[] pathMid[i];
    }
    delete[] pathMid;
    return lower_bound;
}

double obstacleDubinsPath(DubinsState s, DubinsState g, double* out, double r, Wind wind)
{
//    double r = robot->get_r();
//    if (robot->getSpeed() == 1) {
//        r = robot->get_R();
//    }
    double dx, dy, D, d;
    double wayLength[6][3] = { };
    double theta, alpha, beta;
    double array[3];
    double cost = 0;
    int i, bestWay = -1;

    dx = ((double)g.col - (double)s.col);
    dy = ((double)g.row - (double)s.row);
    D = sqrt(pow(dx, 2) + pow(dy, 2));
    d = D;
    if (r <= 0)
    {
        for (i = 0; i < 4; ++i) {
            out[i] = -1;
        }
        std::cout << "r is smaller than 0" << std::endl;
        return -1;
    }

    double absThStart, absThGoal;
    absThStart = (double)s.theta * (2 * M_PI) / ORIENTATIONS;
    absThGoal = (double)g.theta * (2 * M_PI) / ORIENTATIONS;

    theta = fmod(atan2(dy, dx) + (2 * M_PI), 2 * M_PI); // difference between stat and goal orientation
    alpha = fmod(absThStart - theta + (2 * M_PI), 2 * M_PI); // fix start orientation to the difference
    beta = fmod(absThGoal - theta + (2 * M_PI), 2 * M_PI); //fix goal orientation to the difference

    dubinsLSL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[LSL][i] = array[i];
    }
    dubinsLSR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[LSR][i] = array[i];
    }
    dubinsRSL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[RSL][i] = array[i];
    }
    dubinsRSR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[RSR][i] = array[i];
    }
    dubinsRLR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[RLR][i] = array[i];
    }
    dubinsLRL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++) {
        wayLength[LRL][i] = array[i];
    }

    // find shortest path
    double bestCost = -1;
    for (i = 0; i < 6; i++) {
        if (wayLength[i][0] != -1) {
            double tempPath[4] = { wayLength[i][0], wayLength[i][1], wayLength[i][2], (double)i };
            double cost = wayLength[i][0] + wayLength[i][1] + wayLength[i][2]; //TODO SHOULD divide by speed?
            double ground_speed_by_heading = pathValidityCheck(s, tempPath, cost, r, wind);
            cost = ground_speed_by_heading;
            if ((cost < bestCost) || (bestCost == -1)) {
//                double tempPath[4] = { wayLength[i][0], wayLength[i][1], wayLength[i][2], (double)i };
//                if (pathValidityCheck(s, tempPath, cost, r)) {
                    bestWay = i;
                    bestCost = cost;
//                }
            }
        }
    }
    if (bestWay == -1) {
        for (i = 0; i < 4; ++i) {
            out[i] = -1;
        }
    }
    else {
        for (i = 0; i < 3; i++) {
            out[i] = wayLength[bestWay][i];
        }
        out[3] = bestWay;
    }
    return bestCost;

}

void vanilaDubinsPath(double vMin, double r, DubinsState s, DubinsState g, Wind mapWind, double* out)
{
//    double r = robot->get_r();
    double modWind = sqrt(mapWind.x * mapWind.x + mapWind.y * mapWind.y);
    r = r * (vMin - modWind) / vMin;
    double dx, dy, D, d;
    double wayLength[6][3] = { };
    double theta, alpha, beta;
    double array[3];
    int i, bestWay = -1;

    dx = ((double)g.col - (double)s.col);
    dy = ((double)g.row - (double)s.row);
    D = sqrt(pow(dx, 2) + pow(dy, 2));
    d = D;
    if (r <= 0)
    {
        for (i = 0; i < 4; ++i) {
            out[i] = -1;
        }
        return;
    }

    double absThStart, absThGoal;
    absThStart = (double)s.theta * (2 * M_PI) / ORIENTATIONS;
    absThGoal = (double)g.theta * (2 * M_PI) / ORIENTATIONS;

    theta = fmod(atan2(dy, dx) + (2 * M_PI), 2*M_PI); // difference between stat and goal orientation
    alpha = fmod(absThStart - theta + (2 * M_PI), 2 * M_PI); // fix start orientation to the difference
    beta = fmod(absThGoal - theta + (2 * M_PI), 2 * M_PI); //fix goal orientation to the difference

    dubinsLSL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[LSL][i] = array[i];
    }
    dubinsLSR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[LSR][i] = array[i];
    }
    dubinsRSL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[RSL][i] = array[i];
    }
    dubinsRSR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[RSR][i] = array[i];
    }
    dubinsRLR(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[RLR][i] = array[i];
    }
    dubinsLRL(alpha, beta, d, r, array);
    for (i = 0; i < 3; i++){
        wayLength[LRL][i] = array[i];
    }

    // find shortest path
    double bestCost = -1;
    for (i = 0; i < 6; i++){
        if (wayLength[i][0] != -1){
            double cost = wayLength[i][0] + wayLength[i][1] + wayLength[i][2];
            if ((cost < bestCost) || (bestCost == -1)){
                bestWay = i;
                bestCost = cost;
            }
        }
    }
    if (bestWay == -1){
        for (i = 0; i < 4; ++i) {
            out[i] = -1;
        }
    }
    else{
        for (i = 0; i < 3; i++){
            out[i] = wayLength[bestWay][i];
        }
        out[3] = bestWay;
    }
    return;
}

void save_to_file(std::map <long, Candidate>& ocps_table, double v_min, double v_max, double u_max) {
	std::cout << "found " << ocps_table.size() << " candidate paths" << std::endl;

//	std::ofstream myfile("ocps.txt", std::ios_base::app | std::ios_base::out);  // if ocps.txt already create add to it
    std::ofstream myfile("ocps.txt");
    if (myfile.is_open()) {
		myfile << "v_min=" << v_min << std::endl;
		myfile << "v_max=" << v_max << std::endl;
		myfile << "u_max=" << u_max << std::endl << std::endl;

		for (std::map <long, Candidate>::iterator mit = ocps_table.begin(); mit != ocps_table.end(); ++mit) {
			myfile << "key=" << mit->first << std::endl;
			myfile << "pathType=" << mit->second.path_type << std::endl;
			myfile << "pathOrientation=" << mit->second.path_orientation << std::endl;
			myfile << "valid=" << mit->second.valid << std::endl;
			myfile << "cost=" << mit->second.cost << std::endl;
			myfile << "start (x,y,theta) = (" << mit->second.start.col << "," << mit->second.start.row << "," << mit->second.h_initial << ")" << std::endl;
			myfile << "target (x,y,theta) = (" << mit->second.target.col << "," << mit->second.target.row << "," << mit->second.h_final <<")"<< std::endl;

			myfile << "speeds=[";
			for (std::vector<Coordinate>::iterator vit = mit->second.reduced_path->begin(); vit != mit->second.reduced_path->end(); ++vit) {
				if (vit == mit->second.reduced_path->begin()) {
					myfile << (*vit).speed;
				}
				else {
					myfile << "," << (*vit).speed;
				}
			}
			myfile << "]" << std::endl;

			myfile << "x=[";
			for (std::vector<Coordinate>::iterator vit = mit->second.reduced_path->begin(); vit != mit->second.reduced_path->end(); ++vit) {
				if (vit == mit->second.reduced_path->begin()) {
					myfile << (*vit).col;
				}
				else {
					myfile << "," << (*vit).col;
				}	
			}
			myfile << "]" << std::endl;
			
			myfile << "y=[";
			for (std::vector<Coordinate>::iterator vit = mit->second.reduced_path->begin(); vit != mit->second.reduced_path->end(); ++vit) {
				if (vit == mit->second.reduced_path->begin()) {
					myfile << (*vit).row;
				}
				else {
					myfile << "," << (*vit).row;
				}
			}
			myfile << "]" << std::endl << std::endl;
 
		}

		myfile.close();
	}
	return;
}

Wind rotateWind(double theta, Wind windIn) {
    Wind windOut(0, 0);
    windOut.x = windIn.x * cos(theta) - windIn.y * sin(theta);
    windOut.y = windIn.x * sin(theta) + windIn.y * cos(theta);
    return windOut;
}

void copy_candidate(Candidate& source, Candidate& target) {
	target.valid = source.valid;
	target.key = source.key;
	target.cost = source.cost;
	target.start.row = source.start.row;
	target.start.col = source.start.col;
	target.h_initial = source.h_initial;
	target.target.row = source.target.row;
	target.target.col = source.target.col;
	target.h_final = source.h_final;
	target.path_type = source.path_type;
	target.path_orientation = source.path_orientation;
	for (std::vector<Coordinate>::iterator it = source.reduced_path->begin(); it != source.reduced_path->end(); ++it) {
		target.reduced_path->push_back(*it);
	}
}

double dubins_cost(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn) {
    double best_cost = 1000;
    /// All the paths calculate using the minimum radius
    double r = v_min / u_max;
    double R = v_min / u_max;

    double modWind = sqrt(windIn.x * windIn.x + windIn.y * windIn.y);
    r = (v_min - modWind) / u_max;
    R = r;


    //define the possible paths not including dubins for optimization
    const int CandidateIDs[CANDIDATES] = {4, 5, 6, 7, // (B)S(B) Robust dubins paths with LSL, LSR, RSL, RSR
                                          40, 41, 42, 43, // (BCB)(B) with LL,LR,RL,RR
                                          48, 49, 50, 51, // (B)(BCB) with LL,LR,RL,RR
                                          36, 37, 38, 39, // (BCB)(BC) with LL,LR,RL,RR
                                          28, 29, 30, 31, // (B)S(BC) with LSL,LSR,RSL,RSR
                                          44, 45, 46, 47, // (CB)(BCB) with LL,LR,RL,RR
                                          20, 21, 22, 23, // (CB)S(B) with LSL,LSR,RSL,RSR
                                          8, 9,         // (C)(C)(C) Robust dubins paths with LRL, RLR
                                          16, 17, 18, 19};     // (CB)S(BC) with LSL,LSR,RSL,RSR

    int numSamplesMax = 1000;
    std::string samplingAlgorithm = "barycentric";
    std::vector<std::vector<std::string> > candList = VarSpeedDubins::candidateList();
    std::vector<State> *symmetric_targets = new std::vector<State>;
    std::vector<int> *subset = new std::vector<int>;
    std::vector<double> *robustX = new std::vector<double>;
    std::vector<double> *robustY = new std::vector<double>;

    int count = 0;
    State curr_state;
    curr_state.orientation = startOrientation;
    Coordinate locCoor;
    locCoor.row = yFinalIn;
    locCoor.col = xFinalIn;

    double hFinal;

    // TODO Rotate final position to correct place instead of using  yFinalIn and xFinalIn as is
    if (startOrientation % 2 == 0) {
        if ((orientFinalIn - startOrientation) < 0) {
            hFinal = (ORIENTATIONS + (orientFinalIn - startOrientation)) * 2 * M_PI / ORIENTATIONS;
            //hFinal = 7 * 2 * M_PI / ORIENTATIONS;
        } else {
            hFinal = (orientFinalIn - startOrientation) * 2 * M_PI / ORIENTATIONS;
        }
        locCoor = rotate(-startOrientation, locCoor);
    } else { // startOrientation % 2 == 1
        if ((orientFinalIn - startOrientation) < 0) {
            hFinal = (ORIENTATIONS + (orientFinalIn - startOrientation)) * 2 * M_PI / ORIENTATIONS;
            //hFinal = 7 * 2 * M_PI / ORIENTATIONS;
        } else {
            hFinal = (orientFinalIn - startOrientation) * 2 * M_PI / ORIENTATIONS;
        }
        locCoor = rotate(-(startOrientation - 1), locCoor);
    }

    locCoor.row = round(locCoor.row);
    locCoor.col = round(locCoor.col);

    double startTheta = startOrientation * 2 * M_PI / ORIENTATIONS;
    Wind problemWind = rotateWind(-startTheta, windIn);

    State target_it;
    target_it.neighbour_id = coordinate_to_id(locCoor);
    target_it.orientation = orientFinalIn;
    Coordinate target_coordinate = id_to_coordinate(target_it.neighbour_id, startOrientation);
    VarSpeedDubins::ProblemStatement vsdprob;

    vsdprob.set_wind(problemWind.x, problemWind.y);

    double xFinal = target_coordinate.col;
    double yFinal = target_coordinate.row;

    ///
    Coordinate finalCoor;
    finalCoor.col = xFinalIn;
    finalCoor.row = yFinalIn;
    target_it.neighbour_id = coordinate_to_id(finalCoor);
    ///
//    double hFinal;

//    if (startOrientation % 2 == 0) {
//        hFinal = orientFinalIn * 2 * M_PI / ORIENTATIONS;
//    } else if (startOrientation % 2 == 1) {
//        if (orientFinalIn == 0) {
//            hFinal = 7 * 2 * M_PI / ORIENTATIONS;
//        } else {
//            hFinal = (orientFinalIn - 1) * 2 * M_PI / ORIENTATIONS;
//        }
//    }
//			double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;

    vsdprob.set_xFinal(xFinal);
    vsdprob.set_yFinal(yFinal);
    vsdprob.set_hFinal(hFinal);
    vsdprob.set_R(R);
    vsdprob.set_r(r);

//	int orientation = 0; // throw out after fix
    for (int curr_speed = 0; curr_speed < SPEEDS; ++curr_speed) {

//        State curr_state;
        curr_state.speed = curr_speed;
//        curr_state.orientation = startOrientation;
//        Coordinate locCoor;
//        locCoor.row = yFinal;
//        locCoor.col = xFinal;
//        State target_it;
//        target_it.neighbour_id = coordinate_to_id(locCoor);
//        target_it.orientation = orientFinal;
        for (int target_speed = 0; target_speed < SPEEDS; ++target_speed) {
//            State target_it;
            target_it.speed = target_speed;
//            Coordinate locCoor;
//            locCoor.row = yFinal;
//            locCoor.col = xFinal;
//            target_it.neighbour_id = coordinate_to_id(locCoor);
//            target_it.orientation = orientFinal;

            //symmetric_targets->clear();
            //get_basic_symmetric_targets(symmetric_targets, orientation);
            //for (std::vector<State>::iterator target_it = symmetric_targets->begin(); target_it != symmetric_targets->end(); ++target_it) {
//            Coordinate target_coordinate = id_to_coordinate(target_it.neighbour_id, startOrientation);
            //Coordinate target_coordinate_rot = id_to_coordinate((*target_it).neighbour_id, 0);
//			Coordinate target_coordinate_rot = id_to_coordinate(orientFinal, orientFinal);

            // create a problem statement
//            VarSpeedDubins::ProblemStatement vsdprob;
//
//            xFinal = target_coordinate.col;
//			yFinal = target_coordinate.row;
//            double hFinal;
//
//            if (startOrientation == 0) {
//                hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;
//            } else if (startOrientation == 1) {
//                if (orientFinal == 0) {
//                    hFinal = 7 * 2 * M_PI / ORIENTATIONS;
//                } else {
//                    hFinal = (orientFinal - 1) * 2 * M_PI / ORIENTATIONS;
//                }
//            }
//			  double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;
//
//            vsdprob.set_xFinal(xFinal);
//            vsdprob.set_yFinal(yFinal);
//            vsdprob.set_hFinal(hFinal);
//            vsdprob.set_R(R);
//            vsdprob.set_r(r);

            subset->clear();
            get_candidate_subset(subset, curr_state, target_it);
            for (std::vector<int>::iterator s_it = subset->begin(); s_it != subset->end(); ++s_it) {
                // create a solver
                VarSpeedDubins::Solver vsdsolver;

                int candidateID = CandidateIDs[*s_it];
                Candidate candidate;
                candidate.start.row = 0;
                candidate.start.col = 0;
                //candidate.h_initial = curr_state.orientation;
                candidate.target.row = yFinalIn;
                candidate.target.col = xFinalIn;
                candidate.h_final = orientFinalIn;
                candidate.h_initial = curr_state.orientation;

                int solveStatus = 0;

                if (candidateID < 10) {
                    // ------------------------------------------------------------------
                    // Robust Dubins path.
                    // ------------------------------------------------------------------

                    robustX->clear();
                    robustY->clear();
                    vsdsolver.set_problemStatement(vsdprob);
                    double cost = vsdsolver.solveRobust(candidate.path_type, candidate.path_orientation, candidateID,
                                                        robustX, robustY);
                    if (cost > -1) { // has solution
                        candidate.valid = 1;
                        candidate.cost = cost;
                        if (cost < best_cost) {
                            best_cost = cost;
                        }
//                        get_path_collision(candidate.reduced_path, *robustX, *robustY, candidate.path_type,
//                                           startOrientation);
//                        if (startOrientation > 1) {
//                            for (std::vector<Coordinate>::iterator it = candidate.reduced_path->begin(); it != candidate.reduced_path->end(); ++it) {
//                                Coordinate rotated = rotate(startOrientation, *it);
//                                (*it).row = rotated.row;
//                                (*it).col = rotated.col;
//                            }
//                        }

                        solveStatus = 1;
                    }
                }
            }
        }
    }
    return best_cost;
}

double build_ocps_table(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, std::map<long, Candidate>& ocps_table) {
    double r = v_min / u_max;
    double R = v_max / u_max;
    double best_cost = 1000;
//    std::map<long, Candidate> ocps_table;

    //define the possible paths not including dubins for optimization
    const int CandidateIDs[CANDIDATES] = {4, 5, 6, 7, // (B)S(B) Robust dubins paths with LSL, LSR, RSL, RSR
                                          40, 41, 42, 43, // (BCB)(B) with LL,LR,RL,RR
                                          48, 49, 50, 51, // (B)(BCB) with LL,LR,RL,RR
                                          36, 37, 38, 39, // (BCB)(BC) with LL,LR,RL,RR
                                          28, 29, 30, 31, // (B)S(BC) with LSL,LSR,RSL,RSR
                                          44, 45, 46, 47, // (CB)(BCB) with LL,LR,RL,RR
                                          20, 21, 22, 23, // (CB)S(B) with LSL,LSR,RSL,RSR
                                          8, 9,         // (C)(C)(C) Robust dubins paths with LRL, RLR
                                          16, 17, 18, 19};     // (CB)S(BC) with LSL,LSR,RSL,RSR

    int numSamplesMax = 1000;
    std::string samplingAlgorithm = "barycentric";
    std::vector<std::vector<std::string> > candList = VarSpeedDubins::candidateList();
    std::vector<State> *symmetric_targets = new std::vector<State>;
    std::vector<int> *subset = new std::vector<int>;
    std::vector<double> *robustX = new std::vector<double>;
    std::vector<double> *robustY = new std::vector<double>;

    int count = 0;
    State curr_state;
    curr_state.orientation = startOrientation;
    Coordinate locCoor;
    locCoor.row = yFinalIn;
    locCoor.col = xFinalIn;

    double hFinal;

    // TODO Rotate final position to correct place instead of using  yFinalIn and xFinalIn as is
    if (startOrientation % 2 == 0) {
        if ((orientFinalIn - startOrientation) < 0) {
            hFinal = (ORIENTATIONS + (orientFinalIn - startOrientation)) * 2 * M_PI / ORIENTATIONS;
            //hFinal = 7 * 2 * M_PI / ORIENTATIONS;
        } else {
            hFinal = (orientFinalIn - startOrientation) * 2 * M_PI / ORIENTATIONS;
        }
        locCoor = rotate(-startOrientation, locCoor);
    } else { // startOrientation % 2 == 1
        if ((orientFinalIn - startOrientation) < 0) {
            hFinal = (ORIENTATIONS + (orientFinalIn - startOrientation)) * 2 * M_PI / ORIENTATIONS;
            //hFinal = 7 * 2 * M_PI / ORIENTATIONS;
        } else {
            hFinal = (orientFinalIn - startOrientation) * 2 * M_PI / ORIENTATIONS;
        }
        locCoor = rotate(-(startOrientation - 1), locCoor);
    }

    locCoor.row = round(locCoor.row);
    locCoor.col = round(locCoor.col);

    double startTheta = startOrientation * 2 * M_PI / ORIENTATIONS;
    Wind problemWind = rotateWind(-startTheta, windIn);

    State target_it;
    target_it.neighbour_id = coordinate_to_id(locCoor);
    target_it.orientation = orientFinalIn;
    Coordinate target_coordinate = id_to_coordinate(target_it.neighbour_id, startOrientation);
    VarSpeedDubins::ProblemStatement vsdprob;

    vsdprob.set_wind(problemWind.x, problemWind.y);

    double xFinal = target_coordinate.col;
    double yFinal = target_coordinate.row;

    ///
    Coordinate finalCoor;
    finalCoor.col = xFinalIn;
    finalCoor.row = yFinalIn;
    target_it.neighbour_id = coordinate_to_id(finalCoor);
    ///
//    double hFinal;

//    if (startOrientation % 2 == 0) {
//        hFinal = orientFinalIn * 2 * M_PI / ORIENTATIONS;
//    } else if (startOrientation % 2 == 1) {
//        if (orientFinalIn == 0) {
//            hFinal = 7 * 2 * M_PI / ORIENTATIONS;
//        } else {
//            hFinal = (orientFinalIn - 1) * 2 * M_PI / ORIENTATIONS;
//        }
//    }
//			double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;

    vsdprob.set_xFinal(xFinal);
    vsdprob.set_yFinal(yFinal);
    vsdprob.set_hFinal(hFinal);
    vsdprob.set_R(R);
    vsdprob.set_r(r);

//	int orientation = 0; // throw out after fix
    for (int curr_speed = 0; curr_speed < SPEEDS; ++curr_speed) {

//        State curr_state;
        curr_state.speed = curr_speed;
//        curr_state.orientation = startOrientation;
//        Coordinate locCoor;
//        locCoor.row = yFinal;
//        locCoor.col = xFinal;
//        State target_it;
//        target_it.neighbour_id = coordinate_to_id(locCoor);
//        target_it.orientation = orientFinal;
        for (int target_speed = 0; target_speed < SPEEDS; ++target_speed) {
//            State target_it;
            target_it.speed = target_speed;
//            Coordinate locCoor;
//            locCoor.row = yFinal;
//            locCoor.col = xFinal;
//            target_it.neighbour_id = coordinate_to_id(locCoor);
//            target_it.orientation = orientFinal;

            //symmetric_targets->clear();
            //get_basic_symmetric_targets(symmetric_targets, orientation);
            //for (std::vector<State>::iterator target_it = symmetric_targets->begin(); target_it != symmetric_targets->end(); ++target_it) {
//            Coordinate target_coordinate = id_to_coordinate(target_it.neighbour_id, startOrientation);
            //Coordinate target_coordinate_rot = id_to_coordinate((*target_it).neighbour_id, 0);
//			Coordinate target_coordinate_rot = id_to_coordinate(orientFinal, orientFinal);

            // create a problem statement
//            VarSpeedDubins::ProblemStatement vsdprob;
//
//            xFinal = target_coordinate.col;
//			yFinal = target_coordinate.row;
//            double hFinal;
//
//            if (startOrientation == 0) {
//                hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;
//            } else if (startOrientation == 1) {
//                if (orientFinal == 0) {
//                    hFinal = 7 * 2 * M_PI / ORIENTATIONS;
//                } else {
//                    hFinal = (orientFinal - 1) * 2 * M_PI / ORIENTATIONS;
//                }
//            }
//			  double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;
//
//            vsdprob.set_xFinal(xFinal);
//            vsdprob.set_yFinal(yFinal);
//            vsdprob.set_hFinal(hFinal);
//            vsdprob.set_R(R);
//            vsdprob.set_r(r);

            subset->clear();
            get_candidate_subset(subset, curr_state, target_it);
            for (std::vector<int>::iterator s_it = subset->begin(); s_it != subset->end(); ++s_it) {
                // create a solver
                VarSpeedDubins::Solver vsdsolver;

                int candidateID = CandidateIDs[*s_it];
                Candidate candidate;
                candidate.start.row = 0;
                candidate.start.col = 0;
                //candidate.h_initial = curr_state.orientation;
                candidate.target.row = yFinalIn;
                candidate.target.col = xFinalIn;
                candidate.h_final = orientFinalIn;
                candidate.h_initial = curr_state.orientation;

                int solveStatus = 0;

                if (candidateID < 10) {
                    // ------------------------------------------------------------------
                    // Robust Dubins path.
                    // ------------------------------------------------------------------

                    robustX->clear();
                    robustY->clear();
                    vsdsolver.set_problemStatement(vsdprob);
                    double cost = vsdsolver.solveRobust(candidate.path_type, candidate.path_orientation, candidateID,
                                                        robustX, robustY);
                    if (cost > -1) { // has solution
                        candidate.valid = 1;
                        candidate.cost = cost;
                        get_path_collision(candidate.reduced_path, *robustX, *robustY, candidate.path_type,
                                           startOrientation);
						if (startOrientation > 1) {
                            for (std::vector<Coordinate>::iterator it = candidate.reduced_path->begin(); it != candidate.reduced_path->end(); ++it) {
                                Coordinate rotated = rotate(startOrientation, *it);
                                (*it).row = rotated.row;
                                (*it).col = rotated.col;
                            }
						}

                        solveStatus = 1;
                    }

                } else {
                    // ------------------------------------------------------------------
                    // Solve Variable Speed Dubins Problem
                    // ------------------------------------------------------------------
                    std::vector<std::string> cand = candList[candidateID];
                    std::string pathClass = cand[0];
                    std::string pathType = cand[1];
                    std::string pathOrientation = cand[2];

                    VarSpeedDubins::Path vsdcand;
                    vsdcand.set_turnRadii(r, R);

//                    double startTheta = startOrientation * 2 * M_PI / ORIENTATIONS;
//                    Wind problemWind = rotateWind(-startTheta, windIn);
                    vsdcand.set_wind(problemWind.x, problemWind.y);

                    vsdcand.set_pathClass(pathClass);
                    vsdcand.set_pathType(pathType);
                    vsdcand.set_pathOrientation(pathOrientation);

                    // create a VSDNLP from the problem statement + candidate
                    SmartPtr<VSDNLP> vsdnlp = new VSDNLP();
                    vsdnlp->set_problemStatement(vsdprob);
                    vsdnlp->set_candidate(vsdcand);
                    vsdnlp->set_Lmax(10.0);
                    vsdnlp->set_samplingAlgorithm(samplingAlgorithm);

                    // pass nlp to solver and solve
                    vsdsolver.set_vsdnlp(vsdnlp);
                    vsdsolver.set_numSamplesMax(numSamplesMax);
                    vsdsolver.set_samplingAlgorithm(samplingAlgorithm);
                    vsdsolver.solveGivenNLP();

                    solveStatus = vsdsolver.get_solveStatus();
                    if (solveStatus) {
                        arma::vec optSoln = vsdnlp->get_optSoln();
                        vsdcand.set_paramsShort(optSoln);
                        vsdcand.computePathHistory();
                        vsdcand.computeEndpoint();
                        //vsdcand.print();

                        double costOpt = vsdcand.get_cost();
                        //std::cout << "costOpt : " << costOpt << std::endl;

                        candidate.valid = solveStatus;
                        candidate.cost = costOpt;
                        candidate.path_type = pathType;
                        candidate.path_orientation = pathOrientation;
                        arma::mat pathHistory = vsdcand.get_pathHistory();
                        arma::mat switchingPts = vsdcand.get_switchingPts();
                        get_path_collision(candidate.reduced_path, pathHistory, switchingPts, pathType,
                                           startOrientation);
                        // TODO insert rotation of reduced_path if startOrientation is not 0 or 1
                        // Update the path for rotated direction
                        if (startOrientation > 1) {
                            for (std::vector<Coordinate>::iterator it = candidate.reduced_path->begin(); it != candidate.reduced_path->end(); ++it) {
                                Coordinate rotated = rotate(startOrientation, *it);
                                (*it).row = rotated.row;
                                (*it).col = rotated.col;
                            }
                        }
                    }
                }

                if (solveStatus) {
                    /// insert candidate to ocps table
                    if (candidate.cost < best_cost) {
                        best_cost = candidate.cost;
                    }
                    candidate.key = ocps_key(curr_state, target_it, candidateID);
                    ocps_table[candidate.key] = candidate;

//					/// rotate to all other directions and insert to ocps
//					Candidate candidate_rot_arr[3];
//					copy_candidate(candidate, candidate_rot_arr[0]);
//					State curr_state_rot = curr_state;
//					State target_rot = target_it;
//					for (int rot = 1; rot < 4; ++rot) {
//						rotate_target(target_rot);
//						// Update the path for rotated direction
//						for (std::vector<Coordinate>::iterator it = candidate_rot_arr[rot -1].reduced_path->begin(); it != candidate_rot_arr[rot -1].reduced_path->end(); ++it) {
//							Coordinate rotated = rotate(2, *it);
//							(*it).row = rotated.row;
//							(*it).col = rotated.col;
//						}
//
//						curr_state_rot.orientation += 2;
//						Coordinate target_coordinate_rot = id_to_coordinate(target_rot.neighbour_id, 0);
//						candidate_rot_arr[rot - 1].target.row = target_coordinate_rot.row;
//						candidate_rot_arr[rot - 1].target.col = target_coordinate_rot.col;
//						candidate_rot_arr[rot - 1].h_final = target_rot.orientation;
//						candidate_rot_arr[rot - 1].h_initial = curr_state_rot.orientation;
//
//						candidate_rot_arr[rot - 1].key = ocps_key(curr_state_rot, target_rot, candidateID);
//						ocps_table[candidate_rot_arr[rot - 1].key] = candidate_rot_arr[rot - 1];
//						if (rot < 3){
//							copy_candidate(candidate_rot_arr[rot - 1], candidate_rot_arr[rot]);
//						}
//					}

//					/// mirror the solution
//					Candidate candidate_mirror;
//					copy_candidate(candidate, candidate_mirror);
//
//					int candidateID_mirror = mirror_candidateID(candidateID);
//					if (candidateID < 10) {
//						flip_path_orientation(candidate_mirror.path_orientation);
//					}
//					else {
//						std::vector<std::string> mirror_cand = candList[candidateID_mirror];
//						candidate_mirror.path_orientation = mirror_cand[2];
//					}
//					State target_mirror = target_it;
//					/// Edited function find in VarSpeed_utils.h
//					mirror_state(target_it, target_mirror, curr_state.orientation);
//                    /// Edited function find in VarSpeed_utils.h
//
//					if (curr_state.orientation == 0) {
//						// mirror path over the x axis
//						for (std::vector<Coordinate>::iterator it = candidate_mirror.reduced_path->begin(); it != candidate_mirror.reduced_path->end(); ++it) {
//							(*it).row *= -1;
//						}
//					}
//					else if (curr_state.orientation == 1) {
//						// mirror path over y=x line
//						for (std::vector<Coordinate>::iterator it = candidate_mirror.reduced_path->begin(); it != candidate_mirror.reduced_path->end(); ++it) {
//							double temp = (*it).row;
//							(*it).row = (*it).col;
//							(*it).col = temp;
//						}
//					}
//
//					Coordinate target_coordinate_mirror = id_to_coordinate(target_mirror.neighbour_id, curr_state.orientation);
//					candidate_mirror.target.row = target_coordinate_mirror.row;
//					candidate_mirror.target.col = target_coordinate_mirror.col;
//					candidate_mirror.h_final = target_mirror.orientation;
//					candidate_mirror.h_initial = curr_state.orientation;
//					candidate_mirror.key = ocps_key(curr_state, target_mirror, candidateID_mirror);
//					ocps_table[candidate_mirror.key] = candidate_mirror;
//
//					/// rotate the mirror to all other directions and insert to ocps
//					Candidate candidate_mirror_rot_arr[3];
//					copy_candidate(candidate_mirror, candidate_mirror_rot_arr[0]);
//					curr_state_rot = curr_state;
//					target_rot = target_mirror;
//					for (int rot = 1; rot < 4; ++rot) {
//						rotate_target(target_rot);
//						for (std::vector<Coordinate>::iterator it = candidate_mirror_rot_arr[rot - 1].reduced_path->begin(); it != candidate_mirror_rot_arr[rot - 1].reduced_path->end(); ++it) {
//							Coordinate rotated = rotate(2, *it);
//							(*it).row = rotated.row;
//							(*it).col = rotated.col;
//						}
//						curr_state_rot.orientation += 2;
//						Coordinate target_coordinate_rot = id_to_coordinate(target_rot.neighbour_id, 0);
//						candidate_mirror_rot_arr[rot - 1].target.row = target_coordinate_rot.row;
//						candidate_mirror_rot_arr[rot - 1].target.col = target_coordinate_rot.col;
//						candidate_mirror_rot_arr[rot - 1].h_final = target_rot.orientation;
//						candidate_mirror_rot_arr[rot - 1].h_initial = curr_state_rot.orientation;
//						candidate_mirror_rot_arr[rot - 1].key = ocps_key(curr_state_rot, target_rot, candidateID_mirror);
//						ocps_table[candidate_mirror_rot_arr[rot - 1].key] = candidate_mirror_rot_arr[rot - 1];
//						if (rot < 3) {
//							copy_candidate(candidate_mirror_rot_arr[rot - 1], candidate_mirror_rot_arr[rot]);
//						}
//                    }

                }
                count++;
                std::cout << count << "/2312 Done" << std::endl;
            }
            //}
            //}
        }
//        save_to_file(ocps_table, v_min, v_max, u_max);
    }
//    save_to_file(ocps_table, v_min, v_max, u_max);
    return best_cost;
}

void printMod(Wind wind) {
    double mod = sqrt(wind.x * wind.x + wind.y * wind.y);
    std::cout << "mod(" << wind.x << ", " << wind.y << ") = " << mod << std::endl;
}

int main() {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    double neighbors [8][2] = {
            {1, 0},
            {1, 1},
            {0, 1},
            {-1, 1},
            {-1, 0},
            {-1, -1},
            {0, -1},
            {1, -1}
    };
	double v_min = 0.5;
	double v_max = 1;
	double u_max = 1;
//	double xFinal = 1;
//	double yFinal = 1;
//	int orientFinal = 0;
//	int startOrient = 0;
Wind winds[] = {{0, 0}, {.05, 0.05}, {.1, 0}, {.1, .1}, {.2, 0}, {.2, .1}, {.2, .2}, {.35, 0},
                    {.3, .2}, {.4, .1}, {.4, .2}};
//    for (auto &wi: winds) {
//        printMod(wi);
//    }
	Wind wi;
	wi.x = 0.1;
	wi.y = 0;
	std::map<long, Candidate> ocps;
	int transitions = 1;
	int num_of_transitions = 0;
	double sum_of_averages = 0;
//	double mod_of_wind = sqrt(wi.x * wi.x + wi.y * wi.y);
	double path[4] = {-1, -1, -1, -1};

//	DubinsState startState = {0, 0, 1, 0};
//	DubinsState goalState = {1, 0, 4, 0};

//    Coordinate dubinsrotate = {1, 0};
//    double rotateTheta = startState.theta * 2 * M_PI / ORIENTATIONS;
//    Coordinate finalCoor = rotate(-rotateTheta, dubinsrotate);
//    finalCoor.row = round(finalCoor.row);
//    finalCoor.col = round(finalCoor.col);
//
//    goalState = {(int) finalCoor.row, (int) finalCoor.col, goalState.theta - startState.theta, 0};
//    startState = {0, 0, 0, 0};
//
//	double var = build_ocps_table(v_min, v_max, u_max, 0, 1, 4, 1, wi, ocps);
//
////	wi = rotateWind(-rotateTheta, wi);
////    vanilaDubinsPath(v_min, v_min / u_max, startState, goalState, wi, path);
//    double lower = obstacleDubinsPath(startState, goalState, path, v_min / u_max, wi);
//    double cost_of_dubins = dubinsLength(path);
//	double dubins = dubins_cost(v_min, v_max, u_max, 1, -1, 7, 7, wi);
//
//    std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//    std::cout << "Real cost smaller than lower bound" << std::endl;
//    std::cout << "(x, y, final theta, start theta) ";
//    std::cout << goalState.col << ", " << goalState.row << ", " << goalState.theta << ", " << startState.theta << std::endl;
//    std::cout << "wind = (" << wi.x << ", " << wi.y << ")" << std::endl;
//    std::cout << "Lower bound cost: " << lower << std::endl;
//    std::cout << "Time-optimal cost: " << var << std::endl;
//    std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;

	for (auto & wi : winds) {
        std::cout << "start wind conditions" << std::endl;
        double minimum_lower = 100;
        double maximum_lower = 0;
        double mod_of_wind = sqrt(wi.x * wi.x + wi.y * wi.y);
        double wind_sqr = wi.x * wi.x + wi.y * wi.y;
        for (auto & neighbor : neighbors) {
            for (int startOrient = 0; startOrient < ORIENTATIONS; startOrient++) {
                for (int orientFinal = 0; orientFinal < ORIENTATIONS; orientFinal++) {
                    path[0] = -1; // initialize it
                    DubinsState startState = {0, 0, startOrient, 0};
                    DubinsState goalState = {(int)neighbor[1], (int)neighbor[0], orientFinal, 0};
//                    if (neighbor[0] == 0 && neighbor[1] == 1 && orientFinal == 4 && startOrient == 1) {
//                        int bla = 0;
//                    }
                    std::cout << "************************************************" << std::endl;
                    std::cout << "************************************************" << std::endl;
                    std::cout << "Transition " << transitions << "/512" << std::endl;
                    std::cout << "************************************************" << std::endl;
                    std::cout << "************************************************" << std::endl;
                    double var_speed_cost = build_ocps_table(v_min, v_max, u_max, neighbor[0], neighbor[1], orientFinal, startOrient, wi, ocps);
                    vanilaDubinsPath(v_min, v_min / u_max, startState, goalState, wi, path);
                    /// Lower-bound for ground speed with heading
//                    double new_lower_bound = obstacleDubinsPath(startState, goalState, path, v_min / u_max, wi);
//                    double cost_of_dubins = new_lower_bound;
//                double cost_of_dubins = dubins_cost(v_min, v_max, u_max, neighbor[0], neighbor[1], orientFinal, startOrient, wi);
                    /// Calculate average
                    if (var_speed_cost != 1000 && path[0] != -1) {
                        /// Divided by sqrt(1 + wind.x^2 + wind.y^2)
//                        double cost_of_dubins = dubinsLength(path) / sqrt(1 + wind_sqr);

                        /// Divided by 1 + modulus of wind
                        double cost_of_dubins = dubinsLength(path) / (1 + mod_of_wind); // Divided by 1 + modulus of wind

                        /// Ground speed by direction
//                        double cost_of_dubins = dubinsLength(path) / getGroundSpeedDubins(neighbor[0], neighbor[1], wi.x, wi.y, v_max); // Ground speed by direction
//                        double cost_of_dubins = dubinsLength(path) / obstacleDubinsPath(startState, goalState, path, v_min / u_max, wi);

                        double relative = (cost_of_dubins / var_speed_cost) * 100;

                        if (relative < minimum_lower){
                            minimum_lower = relative;
                        }
                        else if ((relative > maximum_lower) && (relative <= 100)) {
                            maximum_lower = relative;
                        }

                        if (relative > 100) {
                            relative = 100;
                        }
                        std::cout << "local lower = " << relative << std::endl;
//                        if (relative > 101) {
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                            std::cout << "Real cost smaller than lower bound" << std::endl;
//                            std::cout << "(x, y, final theta, start theta) ";
//                            std::cout << neighbor[0] << ", " << neighbor[1] << ", " << orientFinal << ", " << startOrient << std::endl;
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        }
                        sum_of_averages += relative;
                        num_of_transitions++;
                    }
                    transitions++;
                }
            }
        }
        std::cout << "wind = (" << wi.x << ", " << wi.y << ")" << std::endl;
        double total_average = sum_of_averages / num_of_transitions;
        std::cout << "Lower bound / time-optimal cost = " << total_average << "%" << std::endl;
        std::cout << "Max_Lower bound / time-optimal cost = " << maximum_lower << "%" << std::endl;
        std::cout << "Min_Lower bound / time-optimal cost = " << minimum_lower << "%" << std::endl;

        num_of_transitions = 0;
        sum_of_averages = 0;
        transitions = 1;
        std::cout << "end wind conditions" << std::endl;
	}
//    for (auto & neighbor : neighbors) {
//        for (int startOrient = 0; startOrient < ORIENTATIONS; startOrient++) {
//            for (int orientFinal = 0; orientFinal < ORIENTATIONS; orientFinal++) {
//                path[0] = -1; // initialize it
//                startState = {0, 0, startOrient, 0};
//                goalState = {(int)neighbor[1], (int)neighbor[0], orientFinal, 0};
//                std::cout << "************************************************" << std::endl;
//                std::cout << "************************************************" << std::endl;
//                std::cout << "Transition " << transitions << "/512" << std::endl;
//                std::cout << "************************************************" << std::endl;
//                std::cout << "************************************************" << std::endl;
//                double var_speed_cost = build_ocps_table(v_min, v_max, u_max, neighbor[0], neighbor[1], orientFinal, startOrient, wi, ocps);
//                vanilaDubinsPath(v_min, v_min / u_max, startState, goalState, wi, path);
////                double cost_of_dubins = dubins_cost(v_min, v_max, u_max, neighbor[0], neighbor[1], orientFinal, startOrient, wi);
//                /// Calculate average
//                if (var_speed_cost != 1000 && path[0] != -1) {
//                    cost_of_dubins = dubinsLength(path) / (1 + mod_of_wind);
//                    double relative = (cost_of_dubins / var_speed_cost) * 100;
//                    if (relative > 101) {
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        std::cout << "Real cost smaller than lower bound" << std::endl;
//                        std::cout << "(x, y, final theta, start theta) ";
//                        std::cout << neighbor[0] << ", " << neighbor[1] << ", " << orientFinal << ", " << startOrient << std::endl;
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//                    }
//                    sum_of_averages += relative;
//                    num_of_transitions++;
//                }
//                transitions++;
//            }
//        }
//    }

//	build_ocps_table(v_min, v_max, u_max, xFinal, yFinal, orientFinal, startOrient, wi, ocps);
//    std::cout << "wind = (" << wi.x << ", " << wi.y << ")" << std::endl;
//    double total_average = sum_of_averages / num_of_transitions;
//    std::cout << "Lower bound / time-optimal cost = " << total_average << "%" << std::endl;
//    save_to_file(ocps, v_min, v_max, u_max);
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    double duration = elapsed_seconds.count();
    std::cout << "Time of calculate: " << duration;
	return 0;
}