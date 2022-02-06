#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include "map_handler.h"

Map::Map(int* map, int width, int height, State start, State goal) {
	m_width = width;
	m_height = height;
	m_start = start;
	m_goal = goal;

	int** _map = new int* [m_height];
	for (int i = 0; i < m_height; ++i)
		_map[i] = new int[m_width];

	for (int i = 0; i < m_height; i++) {
		for (int j = 0; j < m_width; j++) {
			_map[i][j] =  *((map + i * width) + j);
		}
	}
	m_map = _map;
}

Map::~Map()
{
	for (int i = 0; i < m_height; ++i) {
		delete[] m_map[i];
	}
	delete[] m_map;
}

int** Map::get_map() {
	return m_map;
}

State Map::get_start() {
	return m_start;
}

State Map::get_goal() {
	return m_goal;
}

double Map::get_eta() {
	return m_eta;
}

int Map::get_map_width() {
	return m_width;
}

int Map::get_map_height() {
	return m_height;
}

double Map::getObstacleClearance() {
	return m_clearance;
}

void Map::setObstacleClearance(double clearance) {
	m_clearance = clearance;
}

double Map::getEpsilon() {
	return m_epsilon;
}

void Map::setEpsilon(double epsilon) {
	m_epsilon = epsilon;
}

bool Map::isValid(int row, int col) {
	return (row >= 0) && (row < m_height) && (col >= 0) && (col < m_width);
}

bool Map::collisionFree(int row, int col){
	return m_map[row][col] == 0;
}

bool Map::isGoal(State state) {
	return ((state.row == m_goal.row) && (state.col == m_goal.col) && (state.theta == m_goal.theta) && (state.v == m_goal.v));
}

bool Map::partialIsGoal(State state) {
	return ((state.row == m_goal.row) && (state.col == m_goal.col) && (state.theta == m_goal.theta));
}

void Map::setWind(Wind wind) {
	m_wind = wind;
}

Wind Map::getWind() {
	return m_wind;
}

void Map::setVelocities(int v) {
    m_goal.v = v;
    m_start.v = v;
    return;
}


std::vector<State> Map::get_neighbours(State state, int n_orients, int n_velocity, bool extended=false) {
	// TODO make sure to return neighbours considering the reached speed and going out speed
	std::vector<State> neighbours;
	int rel_candidates[8][2] = {{-1,-1}, {-1,0},{-1,1},
								{0,-1} ,        {0,1},
							    {1,-1} , {1,0} ,{1,1}};
	
	for (int i = 0; i < 8; ++i) {
		int row = state.row + rel_candidates[i][0];
		int col = state.col + rel_candidates[i][1];
		if (isValid(row, col) && collisionFree(row, col)) {
			State new_neighbour;
			new_neighbour.col = col;
			new_neighbour.row = row;
			if (extended) {
//				bool flag = false;
				for (int theta = 0; theta < n_orients; ++theta) {
					new_neighbour.theta = theta;
					for (int v = 0; v < n_velocity; ++v) {
						new_neighbour.v = v;
						neighbours.push_back(new_neighbour);
					}
					// pruning
					/*
					if (partialIsGoal(new_neighbour)) {
						for (int v = 0; v < n_velocity; ++v) {
						new_neighbour.v = v;
						std::cout << "Insert state: [" << new_neighbour.row << ", " << new_neighbour.col << ", " << new_neighbour.theta << ", ";
						std::cout << new_neighbour.v << "]" << std::endl;
						neighbours.push_back(new_neighbour);
						}
					}
					else if (obstacleBasedPruning(new_neighbour, n_orients) &&
							headingBasedPruning(new_neighbour, n_orients)) {
						for (int v = 0; v < n_velocity; ++v) {
							new_neighbour.v = v;
							// insert here speedBasedPruning
							
							if (flag && (v == 0)) {
								std::cout << "Pruning state: [" << new_neighbour.row << ", " << new_neighbour.col << ", " << new_neighbour.theta << ", ";
								std::cout << new_neighbour.v << "]" << std::endl;
								continue;
							}
							if ((v == 0) && (speedBasedPruning(new_neighbour, n_orients))) {
								std::cout << "Pruning state: [" << new_neighbour.row << ", " << new_neighbour.col << ", " << new_neighbour.theta << ", ";
								std::cout << new_neighbour.v << "]" << std::endl;
								flag = true;
								continue;
							}
							std::cout << "Insert state: [" << new_neighbour.row << ", " << new_neighbour.col << ", " << new_neighbour.theta << ", ";
							std::cout << new_neighbour.v << "]" << std::endl;
							neighbours.push_back(new_neighbour);
						}
					}*/
					 // original
					//if ((obstacleBasedPruning(new_neighbour, n_orients)) && 
						//(headingBasedPruning(new_neighbour, n_orients))) {
					/*for (int v = 0; v < n_velocity; ++v) {
						// v should be vmin for pruning
						// v = 1 for vmin
						//if (flag && (v == 0)) {
						//	continue;
						//}
						//if ((v == 0) && (!speedBasedPruning(new_neighbour, n_orients))) {
						//	flag = true;
						//	continue;
						//}
						new_neighbour.v = v;
						neighbours.push_back(new_neighbour);
					} // */
					//}
				}
			}
			else {
				new_neighbour.theta = 0;
				new_neighbour.v = 0;
				neighbours.push_back(new_neighbour);
			}
			
		}
	}
	return neighbours;
}

bool Map::obstacleBasedPruning(State state, int n_orients) {
	double theta = (double) state.theta / (double) n_orients * 2 * M_PI;
	int tempRow = (int) round((double) state.row + sin(theta));
	int tempCol = (int) round((double) state.col + cos(theta));
	if (!isValid(tempRow, tempCol) || !collisionFree(tempRow, tempCol)) {
		return false;
	}
	else {
		return true;
	}
}

//bool Map::speedBasedPruning(State state, int n_orients) {
//	int rel_candidates[8][2] = { 	{-1,-1}, {-1,0},{-1,1},
//									{0,-1} ,        {0,1},
//									{1,-1} , {1,0} ,{1,1} };
//	for (int i = 0; i < 8; ++i) {
//		int row = state.row + rel_candidates[i][0];
//		int col = state.col + rel_candidates[i][1];
//		if (!isValid(row, col) || !collisionFree(row, col)) {
//			return false;
//		}
//	}
//	return true;
//}

bool Map::headingBasedPruning(State state, int n_orients) {
	double m_eta = M_PI / 2;
	double theta = (double)state.theta / (double)n_orients * 2 * M_PI;
	if ((fabs(sin(theta)) < 1e-8) || (fabs(cos(theta)) < 1e-8)) { // make sure is diagonal state
		return true;
	}
	double alpha = atan2((double)(m_goal.row - state.row), (double)(m_goal.col - state.col));
	double relative = fabs(fmod(alpha - theta, 2 * M_PI));
	double ksi = std::min(relative, 2 * M_PI - relative);
	if (ksi > m_eta) {
		return false;
	}
	return true;
}
