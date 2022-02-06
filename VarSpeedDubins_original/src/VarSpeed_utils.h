#ifndef VARSPEEDPATHS_UTILS_H
#define VARSPEEDPATHS_UTILS_H

// external
#include<armadillo>

#include <vector>
#include <iostream>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#define ORIENTATIONS 8
#define SPEEDS 2
#define NEIGHBOURS 8
#define CANDIDATES 34

typedef struct Coordinate {
	double row;
	double col;
	int speed; // only to be used when we return the reduced path 
} Coordinate;

typedef struct VS_State {
	int orientation;
	int speed;
	int neighbour_id = -1; //if state is a target than neighbour_id >= 0
} VS_State;

int rotate_neighbours(int n_id) {
	if (n_id == 0) return 3;
	else if (n_id == 1) return 0;
	else if (n_id == 2) return 1;
	else if (n_id == 3) return 5;
	else if (n_id == 4) return 2;
	else if (n_id == 5) return 6;
	else if (n_id == 6) return 7;
	else if (n_id == 7) return 4;
}

Coordinate rotate(int orientation, Coordinate coordinate) {
	double theta = orientation * 2 * M_PI / ORIENTATIONS;
	Coordinate rotated;
	rotated.col = coordinate.col * cos(theta) - coordinate.row * sin(theta);
	rotated.row = coordinate.col * sin(theta) + coordinate.row * cos(theta);
	return rotated;
}

int coordinate_to_id(Coordinate coordinate) {
	if (coordinate.row == 1 && coordinate.col == -1) return 0;
	else if (coordinate.row == 1 && coordinate.col == 0) return 1;
	else if (coordinate.row == 1 && coordinate.col == 1) return 2;
	else if (coordinate.row == 0 && coordinate.col == -1) return 3;
	else if (coordinate.row == 0 && coordinate.col == 1) return 4;
	else if (coordinate.row == -1 && coordinate.col == -1) return 5;
	else if (coordinate.row == -1 && coordinate.col == 0) return 6;
	else if (coordinate.row == -1 && coordinate.col == 1) return 7;
}

Coordinate id_to_coordinate(int id, int orientation) {
	/*
	a neighbour is id of {0 1 2
						  3 * 4
						  5 6 7}
	*/
	Coordinate coordinate;
	if (id == 0) {
		coordinate.row = 1;
		coordinate.col = -1;
	}
	if (id == 1) {
		coordinate.row = 1;
		coordinate.col = 0;
	}
	else if (id == 2) {
		coordinate.row = 1;
		coordinate.col = 1;
	}
	else if (id == 3) {
		coordinate.row = 0;
		coordinate.col = -1;
	}
	else if (id == 4) {
		coordinate.row = 0;
		coordinate.col = 1;
	}
	else if (id == 5) {
		coordinate.row = -1;
		coordinate.col = -1;
	}
	else if (id == 6) {
		coordinate.row = -1;
		coordinate.col = 0;
	}
	else if (id == 7) {
		coordinate.row = -1;
		coordinate.col = 1;
	}
	
	if ((orientation % 2) == 1) {
		//we rotate the coordinates 45 degrees to the right
		return rotate(-1, coordinate);
	}

	return coordinate;
}

long ocps_key(VS_State curr, VS_State target, int path_id) {
	/*
	curr state has 16 options : 2 speeds * 8 orientations
	target state has 128 options : 2 speeds * 8 orientations * 8 neighbours
	path_id has 34 options
	*/
	return (curr.speed + SPEEDS * (curr.orientation + ORIENTATIONS * (target.speed + SPEEDS * (target.orientation + ORIENTATIONS * (target.neighbour_id + NEIGHBOURS * path_id)))));
}

int mirror_candidateID(int candidateID) {
	if (candidateID == 8) return 9;
	else if (candidateID == 9) return 8;
	else {
		if (candidateID % 4 == 0) return candidateID + 3;
		else if (candidateID % 4 == 1) return candidateID + 1;
		else if (candidateID % 4 == 2) return candidateID - 1;
		else return candidateID - 3;
	}
}

void flip_path_orientation(std::string& pathOrientation){
	if (pathOrientation.compare("LSL") == 0) {
		pathOrientation = "RSR";
	}
	else if (pathOrientation.compare("LSR") == 0) {
		pathOrientation = "RSL";
	}
	else if (pathOrientation.compare("RSL") == 0) {
		pathOrientation = "LSR";
	}
	else if (pathOrientation.compare("RSR") == 0) {
		pathOrientation = "LSL";
	}
	else if (pathOrientation.compare("LRL") == 0) {
		pathOrientation = "RLR";
	}
	else if (pathOrientation.compare("RLR") == 0) {
		pathOrientation = "LRL";
	}
}

void mirror_state(VS_State source, VS_State& target, int orientation) {
	if (orientation == 0) {
		// mirror over the x axis
		if (source.orientation == 1) {
			target.orientation = 7;
		}
		else if (source.orientation == 2) {
			target.orientation = 6;
		}
		else if (source.orientation == 3) {
			target.orientation = 5;
		}
		 
		target.speed = source.speed;

		if (source.neighbour_id >= 0 && source.neighbour_id < 3) {
			target.neighbour_id += 5;
		}
		else {
			target.neighbour_id = source.neighbour_id;
		}
	}

	else if (orientation == 1) {
		// mirror over y=x line 
		if (source.orientation == 2) {
			target.orientation = 0;
		}
		else if (source.orientation == 3) {
			target.orientation = 7;
		}
		else if (source.orientation == 4) {
			target.orientation = 6;
		}
		target.speed = source.speed;

		if (source.neighbour_id == 0) target.neighbour_id = 7;
		else if (source.neighbour_id == 1) target.neighbour_id = 4;
		else if (source.neighbour_id == 2) target.neighbour_id = 2;
		else if (source.neighbour_id == 3) target.neighbour_id = 6;
		else if (source.neighbour_id == 5) target.neighbour_id = 5;

	}
}

bool compare_points(double x_ref, double x_new, double y_ref, double y_new) {
	double tol = 0.0001;
	double deltaX = x_ref - x_new;
	double deltaY = y_ref - y_new;

	if (std::abs(deltaX) <= tol && std::abs(deltaY) <= tol) {
		return true;
	}

	return false;
}

void get_path_collision(std::vector<Coordinate>* &coordinates, arma::mat& history, arma::mat& switchingPts, std::string& pathType,  int orientation) {
	std::vector<int> speeds;
	std::vector<double> x_switch;
	std::vector<double> y_switch;
	arma::vec xSwitchPts = switchingPts.row(0).t();
	arma::vec ySwitchPts = switchingPts.row(1).t();

	if (pathType.compare("BCB-B") == 0) {
		speeds.push_back(1);
		speeds.push_back(0);
		speeds.push_back(1);
		speeds.push_back(1);
		x_switch.push_back(xSwitchPts[0]);
		x_switch.push_back(xSwitchPts[1]);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		y_switch.push_back(ySwitchPts[0]);
		y_switch.push_back(ySwitchPts[1]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
	}
	else if (pathType.compare("B-BCB") == 0) {
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(0);
		speeds.push_back(1);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		x_switch.push_back(xSwitchPts[5]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
		y_switch.push_back(ySwitchPts[5]);
	}
	else if (pathType.compare("BCB-BC") == 0) {
		speeds.push_back(1);
		speeds.push_back(0);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(0);
		x_switch.push_back(xSwitchPts[0]);
		x_switch.push_back(xSwitchPts[1]);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		y_switch.push_back(ySwitchPts[0]);
		y_switch.push_back(ySwitchPts[1]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
	}
	else if (pathType.compare("B-S-BC") == 0) {
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(0);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		x_switch.push_back(xSwitchPts[5]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
		y_switch.push_back(ySwitchPts[5]);
	}
	else if (pathType.compare("CB-BCB") == 0) {
		speeds.push_back(0);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(0);
		speeds.push_back(1);
		x_switch.push_back(xSwitchPts[1]);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		x_switch.push_back(xSwitchPts[5]);
		y_switch.push_back(ySwitchPts[1]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
		y_switch.push_back(ySwitchPts[5]);
	}
	else if (pathType.compare("CB-S-B") == 0) {
		speeds.push_back(0);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(1);
		x_switch.push_back(xSwitchPts[1]);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		y_switch.push_back(ySwitchPts[1]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
	}
	else if (pathType.compare("CB-S-BC") == 0) {
		speeds.push_back(0);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(1);
		speeds.push_back(0);
		x_switch.push_back(xSwitchPts[1]);
		x_switch.push_back(xSwitchPts[2]);
		x_switch.push_back(xSwitchPts[3]);
		x_switch.push_back(xSwitchPts[4]);
		x_switch.push_back(xSwitchPts[5]);
		y_switch.push_back(ySwitchPts[1]);
		y_switch.push_back(ySwitchPts[2]);
		y_switch.push_back(ySwitchPts[3]);
		y_switch.push_back(ySwitchPts[4]);
		y_switch.push_back(ySwitchPts[5]);
	}

	arma::colvec xHistory = history.col(0);
	arma::colvec yHistory = history.col(1);
	
	int curr_speed = speeds.front();
	speeds.erase(speeds.begin());
	double curr_x_switch = x_switch.front();
	x_switch.erase(x_switch.begin());
	double curr_y_switch = y_switch.front();
	y_switch.erase(y_switch.begin());

	Coordinate coordinate;
	for (int i = 0; i < xHistory.size(); ++i) {
		coordinate.row = yHistory[i];
		coordinate.col = xHistory[i];
		coordinate.speed = curr_speed;

		if (compare_points(curr_x_switch, coordinate.col, curr_y_switch, coordinate.row)){

			if (!speeds.empty()) {
				curr_speed = speeds.front();
				speeds.erase(speeds.begin());
				curr_x_switch = x_switch.front();
				x_switch.erase(x_switch.begin());
				curr_y_switch = y_switch.front();
				y_switch.erase(y_switch.begin());
			}
		}

		if (orientation == 1) {
			//we rotate the coordinates 45 degrees back to the left
			Coordinate rotated = rotate(1, coordinate);
			coordinate.row = rotated.row;
			coordinate.col = rotated.col;
		}
		coordinates->push_back(coordinate);
	}
	
	return;
}

void get_path_collision(std::vector<Coordinate>*& coordinates, std::vector<double>& xHistory, std::vector<double>& yHistory, std::string& pathType, int orientation) {
	int curr_speed;
	if (pathType.compare("B-S-B") == 0) {
		curr_speed = 1;
	}
	else if (pathType.compare("C-C-C") == 0) {
		curr_speed = 0;
	}

	Coordinate coordinate;
	for (int i = 0; i < xHistory.size(); ++i) {
		coordinate.row = yHistory[i];
		coordinate.col = xHistory[i];
		coordinate.speed = curr_speed;

		if (orientation == 1) {
			//we rotate the coordinates 45 degrees back to the left
			Coordinate rotated = rotate(1, coordinate);
			coordinate.row = rotated.row;
			coordinate.col = rotated.col;
		}
		coordinates->push_back(coordinate);
	}

	return;
}

void get_candidate_subset(std::vector<int>* &subset, VS_State curr, VS_State target) {
	if (curr.speed == 1 and target.speed == 1) {
		// Starts and ends with B
		for (int i = 0; i < 12; ++i) {
			subset->push_back(i);
		}
	}
	else if (curr.speed == 1 and target.speed == 0) {
		// Starts with B and ends with C
		for (int i = 12; i < 20; ++i) {
			subset->push_back(i);
		}
	}
	else if (curr.speed == 0 and target.speed == 1) {
		// Starts with C and ends with B
		for (int i = 20; i < 28; ++i) {
			subset->push_back(i);
		}
	}
	else if (curr.speed == 0 and target.speed == 0) {
		// Starts and ends with C
		for (int i = 28; i < 34; ++i) {
			subset->push_back(i);
		}
	}
}

void rotate_target(VS_State& target) {
	// rotate 2 cells counter-clockwise
	if (target.neighbour_id == 0) {
		target.neighbour_id = 5;		
	}
	else if (target.neighbour_id == 1) {
		target.neighbour_id = 3;
	}
	else if (target.neighbour_id == 2) {
		target.neighbour_id = 0;
	}
	else if (target.neighbour_id == 3) {
		target.neighbour_id = 6;
	}
	else if (target.neighbour_id == 4) {
		target.neighbour_id = 1;
	}
	else if (target.neighbour_id == 5) {
		target.neighbour_id = 7;
	}
	else if (target.neighbour_id == 6) {
		target.neighbour_id = 4;
	}
	else if (target.neighbour_id == 7) {
		target.neighbour_id = 2;
	}
	
	target.orientation = (target.orientation + 2) % ORIENTATIONS;
	return;
}

void get_basic_symmetric_targets(std::vector<VS_State>* &targets, int curr_orientation) {
	
	/*
	State target;
	target.neighbour_id = 4;
	target.speed = 0;
	target.orientation = 0;
	targets->push_back(target);
	return;
	*/

	//orientation = right	
	int full_neighbours[3] = { 0, 1, 2 };
	int semi_neighbours[2] = { 3, 4 };
	
	if (curr_orientation == 1) {
		//orientation = right-up
		full_neighbours[2] = 3;
		semi_neighbours[0] = 2;
		semi_neighbours[1] = 5; 
	}

	for (int n_id = 0; n_id < 3; ++n_id) {
		VS_State target;
		target.neighbour_id = full_neighbours[n_id];
		for (int speed = 0; speed < SPEEDS; ++speed) {
			target.speed = speed;
			for (int orientation = 0; orientation < ORIENTATIONS; ++orientation) {
				target.orientation = orientation;
				targets->push_back(target);
			}
		}
	}

	for (int n_id = 0; n_id < 2; ++n_id) {
		VS_State target;
		target.neighbour_id = semi_neighbours[n_id];
		for (int speed = 0; speed < SPEEDS; ++speed) {
			target.speed = speed;
			for (int orientation = 0; orientation < 5; ++orientation) {
				if (curr_orientation == 0) {
					target.orientation = orientation;
				}
				else if (curr_orientation == 1) {
					target.orientation = orientation + 1;
				}
				targets->push_back(target);
			}
		}
	}
}
	
#endif // VARSPEEDPATHS_UTILS_H
