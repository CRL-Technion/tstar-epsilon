<<<<<<< HEAD
#define _USE_MATH_DEFINES
#include <cmath>
#include "map_handler.h"

bool pruning(State state, int n_orients) {
	double theta = (double)state.theta / (double)n_orients * 2 * M_PI;
	double tempRow = round(state.row + sin(theta));
	double tempCol = round(state.col + cos(theta));
	// #1 obstacle based pruning
	if (!isValid((int) tempRow, (int)tempCol) || !collisionFree((int)tempRow, (int)tempCol)) {
		return false;
	}
	// #2 speed based pruning
	if (state.v == 1) { // v = vmin
		// TODO adjust the value of v = vmin
		int rel_candidates[8][2] = { {-1,-1}, {-1,0},{-1,1},
									{0,-1} ,        {0,1},
									{1,-1} , {1,0} ,{1,1} };

		int j = 0;
		for (int i = 0; i < 8; ++i) {
			int row = state.row + rel_candidates[i][0];
			int col = state.col + rel_candidates[i][1];

			if (isValid(row, col) && collisionFree(row, col)) {
				j++;
			}
			else { // mean at least one neighbor is not valid or not collision free
				break;
			}
		}
		if (j == 8) { // all the neighbors are valid, no need for minimum speed
			return false;
		}
	}
	// #3 heading based pruning
	// TODO
	double theta = (double)state.theta / (double)n_orients * 2 * M_PI;
	if ((sin(theta) == 0) || (cos(theta) == 0)) { // make sure is diagonal state
		return true;
	}
	double ksi = atan2((m_goal.row - state.row), m_goal.col - state.col);
	if (fabs(ksi) > eta) {
		return false;
	}

=======
#define _USE_MATH_DEFINES
#include <cmath>
#include "map_handler.h"

bool pruning(State state, int n_orients) {
	double theta = (double)state.theta / (double)n_orients * 2 * M_PI;
	double tempRow = ceil(state.row + sin(theta));
	double tempCol = ceil(state.col + cos(theta));
	// #1 obstacle based pruning
	if (!isValid((int) tempRow, (int)tempCol) || !collisionFree((int)tempRow, (int)tempCol)) {
		return false;
	}
	// #2 speed based pruning
	if (state.v == 1) { // v = vmin
		// TODO adjust the value of v = vmin
		int rel_candidates[8][2] = { {-1,-1}, {-1,0},{-1,1},
									{0,-1} ,        {0,1},
									{1,-1} , {1,0} ,{1,1} };

		int j = 0;
		for (int i = 0; i < 8; ++i) {
			int row = state.row + rel_candidates[i][0];
			int col = state.col + rel_candidates[i][1];

			if (isValid(row, col) && collisionFree(row, col)) {
				j++;
			}
			else { // mean at least one neighbor is not valid or not collision free
				break;
			}
		}
		if (j == 8) { // all the neighbors are valid, no need for minimum speed
			return false;
		}
	}
	// #3 heading based pruning
	// TODO
>>>>>>> 9d680c98e6ba596ed860a5dd17db0415445fe0ea
}