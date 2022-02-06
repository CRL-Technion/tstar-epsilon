//#include <iostream>
//#include <set>
//#include <map>
//#include <vector>
#include <bits/stdc++.h>
#include <queue>
//#include <cassert>
//#include <cmath>
//#include <chrono>
//#include <queue>

//#include <boost/heap/d_ary_heap.hpp>



#include "tstar_imp.h"

//void build_ocps_table(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn);


using namespace std;

bool operator<(const Node& n1, const Node& n2)
{
    ///
	//return n1.f < n2.f;
	return n1.f > n2.f; // To sort OPEN from smallest to largest
	///
}

bool compareNodes(const Node& n1, const Node& n2) {
    return n1.f < n2.f;
}

bool operator==(const State& s1, const State& s2){
    return ((s1.row == s2.row) && (s1.col == s2.col) && (s1.theta == s2.theta) && (s1.v == s2.v));
}

/*struct compareNodes_new {
    bool operator()(const Node & n1, const Node & n2) const
    {
        return n1.f > n2.f;
    }
};*/

//using dArray = boost::heap::d_ary_heap<Node, boost::heap::arity<8>, boost::heap::mutable_<true>, boost::heap::compare<compareNodes_new>>;
//using binomialHeap = boost::heap::binomial_heap<Node,boost::heap::compare<compareNodes_new>>;

void TStarImp::printMap(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient) {
	int num_of_translations = 0;
	std::cout << "======================= start printing map =======================" << std::endl;
	std::cout << "[x, y, start theta]" << std::endl;
	/*double neighbors [16][3] = { // indicates {final x, final y, initial orientation}
								{1, 0, 0},
								{1, 1, 0},
                                {0, 1, 0},
								{-1, 1, 0},
								{-1, 0, 0},
								{-1, -1, 0},
                                {0, -1, 0},
                                {1, -1, 0},
								{1, 1, 1},
								{0, 1, 1},
								{-1, 1, 1},
								{-1, 0, 1},
								{-1, -1, 1},
                                {0, -1, 1},
                                {1, -1, 1},
                                {1, 0, 1}
								};*/
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
	for (auto & neighbor : neighbors) {
		for (int i = 0; i < numOfOrient; i++) {
			std::cout << "[" << neighbor[0] << ", " << neighbor[1] << ", " << i << "]" << std::endl << "[";
			// Prints the numbers of orientations
			// e.g [0, 1, 2, 3, 4, 5, 6, 7] for 8 orientations
			for (int j = 0; j < numOfOrient; j++) {
				if (j == numOfOrient - 1) {
					std::cout << j << "]" << std::endl << "[";
				}
				else {
					std::cout << j << ", ";
				}
			}
			for (int j = 0; j < numOfOrient; j++) {
				std::cout << tempMap[{neighbor[0], neighbor[1], (double)i}][j] << ", ";
				if (tempMap[{neighbor[0], neighbor[1], (double)i}][j]) {
					num_of_translations++;
				}
			}
            std::cout << "]" << std::endl << std::endl;
		}
	}
    std::cout << "Number of translations: " << num_of_translations << std::endl;
    std::cout << "======================= end printing map =======================" << std::endl;
	/*
	for (int i = 0; i < 8; i ++) {
		std::cout << "[" << neighbors[i][0] << ", " << neighbors[i][1] << ", " << neighbors[i][2] << "]" << std::endl;
		std::cout << "[0, 1, 2, 3, 4, 5, 6, 7]" << std::endl;
		std::cout << "[";
        for (int j = 0; j < numOfOrient; j++) {
            std::cout << tempMap[{neighbors[i][0], neighbors[i][1], neighbors[i][2]}][j] << ", ";
			if (tempMap[{neighbors[i][0], neighbors[i][1], neighbors[i][2]}][j]) {
				num_of_translations++;
			}
		}
		std::cout << "]" << std::endl;
    }
	std::cout << "Number of translations: " << num_of_translations << std::endl;
	std::cout << "======================= end printing map =======================" << std::endl;
	*/
}

void TStarImp::calcDubinsPaths(double alpha, double beta, double d, double r, int type, dubinsCandidate candidate){
	double array[3];
	if (type == LSL) {
        m_dubins->dubinsLSL(alpha, beta, d, r, array);
	}
	else if (type == LSR) {
        m_dubins->dubinsLSR(alpha, beta, d, r, array);
	}
	else if (type == RSL) {
        m_dubins->dubinsRSL(alpha, beta, d, r, array);
	}
	else if (type == RSR) {
        m_dubins->dubinsRSR(alpha, beta, d, r, array);
	}
	else if (type == RLR) {
        m_dubins->dubinsRLR(alpha, beta, d, r, array);
	}
	else {
        m_dubins->dubinsLRL(alpha, beta, d, r, array);
	}
	
	for (int i = 0; i < 3; i++) {
		candidate.path[i] = array[i];
	}
	if (array[0] == -1) {
        return;
    }
	
	candidate.path[3] = type;
	candidate.cost = m_dubins->dubinsLength(array);
	if (candidate.cost != -1) {
		candidate.valid = true;
	}
	else {
		return;
	}
	long candKey = m_dubins->dubinsKey(candidate.trans, type);
	candidate.key = candKey;
	//std::cout << "2.3" << std::endl;
	State stateToFindCells = {0,0,(int)candidate.trans.startTheta,0};
    m_dubins->findCellsOfPath(m_robot, m_map, stateToFindCells, candidate.path, candidate.cost, candidate.reduced_path);
	
	m_dubins_table[candKey] = candidate;
	
}

void TStarImp::calculateDubinsTransitions() {
	dubinsCandidate candidate;
	/*double neighbors [10][3] = { 
								{1, 0, 0},
								{1, 1, 0},
								{0, 1, 0},
								{-1, 1, 0},
								{-1, 0, 0},
								{1, 1, 1},
								{0, 1, 1},
								{-1, 1, 1},
								{-1, 0, 1},
								{-1, -1, 1}
								};*/
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
	int numOfOrientations = m_robot->getNumOfOrientation();
	int n_dubins = 6;
	double r = m_robot->get_r();
    if (m_robot->getSpeed() == 1) {
        r = m_robot->get_R();
    }
    double dx, dy, D, d;
    double theta, alpha, beta;
    //double array[3];

    double absThetaStart, absThetaGoal;
    /*absThetaStart = (double)s.theta * (2 * M_PI) / robot->getNumOfOrientation();
    absThetaGoal = (double)g.theta * (2 * M_PI) / robot->getNumOfOrientation();

    theta = fmod(atan2(dy, dx) + (2 * M_PI), 2*M_PI); // difference between stat and goal orientation
    alpha = fmod(absThetaStart - theta + (2 * M_PI), 2 * M_PI); // fix start orientation to the difference
    beta = fmod(absThetaGoal - theta + (2 * M_PI), 2 * M_PI); //fix goal orientation to the difference*/
	for (auto & neighbor : neighbors) {
		//std::cout << "1" << std::endl;
		dx = neighbor[0];
		dy = neighbor[1];
		theta = fmod(atan2(dy, dx) + (2 * M_PI), 2*M_PI); // difference between stat and goal orientation
		D = sqrt(pow(dx, 2) + pow(dy, 2));
		d = D;
		for (int startOrient = 0; startOrient < numOfOrientations; startOrient++) {
			absThetaStart = (double)startOrient * (2 * M_PI) / numOfOrientations;
			alpha = fmod(absThetaStart - theta + (2 * M_PI), 2 * M_PI); // fix start orientation to the difference
			for (int endOrient = 0; endOrient < numOfOrientations; endOrient++) {
				//std::cout << "1.1" << std::endl;
				absThetaGoal = (double)endOrient * (2 * M_PI) / numOfOrientations;
				beta = fmod(absThetaGoal - theta + (2 * M_PI), 2 * M_PI); //fix goal orientation to the difference
				for (int i = 0; i < n_dubins; i++) {
					candidate.trans = {dx, dy, endOrient, startOrient};
                    calcDubinsPaths(alpha, beta, d, r, i, candidate);
					candidate = {};
				}
			}
        }
    }
}

void TStarImp::dubinsGetBestCandidate(State s, transition trans, dubinsCandidate &bestCandidate) {
	int dubinsTypes = 6;
	bestCandidate.cost = -1;
	
	for (int i = 0; i < dubinsTypes; i++) {
		long tableKey = m_dubins->dubinsKey(trans, i);
		std::map<long,dubinsCandidate>::iterator it = m_dubins_table.find(tableKey);
		
		if (it != m_dubins_table.end()) {
			dubinsCandidate tempCand = it->second;
			if (m_dubins->pathValidityByCells(m_map, s, tempCand)) {
				if (bestCandidate.cost == -1 || tempCand.cost < bestCandidate.cost) {
                    m_dubins->copyDubinsCandidate(tempCand, bestCandidate);
				}
			}
		}
	}
	return;
}


void TStarImp::initMap(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient) {
    /*double neighbors [16][3] = { // indicates {final x, final y, initial orientation}
                                {1, 0, 0},
                                {1, 1, 0},
                                {0, 1, 0},
                                {-1, 1, 0},
                                {-1, 0, 0},
                                {-1, -1, 0},
                                {0, -1, 0},
                                {1, -1, 0},
                                {1, 1, 1},
                                {0, 1, 1},
                                {-1, 1, 1},
                                {-1, 0, 1},
                                {-1, -1, 1},
                                {0, -1, 1},
                                {1, -1, 1},
                                {1, 0, 1}
                                };*/
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
	for (auto & neighbor : neighbors) {
		//std::cout << "[" << neighbor[0] << ", " << neighbor[1] << ", " << neighbor[2] << "]" << std::endl;
        // Going over all the start orientations
		for (int i = 0; i < numOfOrient; i++) {
		    // Going over all the final orientations
			for (int j = 0; j < numOfOrient; j++) {
				tempMap[{neighbor[0], neighbor[1], (double)i}].push_back(false);
			}
        }
    }
}


TStarImp::TStarImp(int* map, int width, int height, State start, State goal, Problem problem) {
    Map* _map = new Map(map, width, height, start, goal);
    _map->setObstacleClearance(problem.obstacle_clearance);
	
	_map->setEpsilon(problem.epsilon);
	
	_map->setWind({problem.wind_x, problem.wind_y});

    std::cout << "uMax = " << problem.uMax << std::endl;
    std::cout << "vMax = " << problem.vMax << std::endl;
    std::cout << "vMin = " << problem.vMin << std::endl;
    std::cout << "speed = " << problem.speed << std::endl;
    std::cout << "obstacle_clearance = " << problem.obstacle_clearance << std::endl;
    std::cout << "h_function = " << problem.h << std::endl;
    std::cout << "g_function = " << problem.g << std::endl;
    std::cout << "epsilon = " << problem.epsilon << std::endl;
    std::cout << "wind_x = " << problem.wind_x << std::endl;
    std::cout << "wind_y = " << problem.wind_y << std::endl;
    std::cout << "alg = " << problem.alg << std::endl;

	std::cout << std::endl << "ε = " << _map->getEpsilon() << std::endl << std::endl;
    m_map = _map;

    Dubins* _dubins = new Dubins();
    m_dubins = _dubins;
	
	std::cout << "wind = (" << m_map->getWind().x << ", " << m_map->getWind().y << ")" << std::endl;
	if (m_map->getWind().x || m_map->getWind().y) {
		this->set_no_wind(false);
	}
	
	this -> set_epsilon_alg(problem.alg);
	
    Robot* _robot = new Robot;
    _robot->setuMax(problem.uMax);
    _robot->setvMax(problem.vMax);
    _robot->setvMin(problem.vMin);
    _robot->setSpeed(problem.speed);
    
    m_robot = _robot;
    cout << "uMax: " << m_robot->getuMax() << endl;
    cout << "vMin: " << m_robot->getvMin() << endl;
    cout << "vMax: " << m_robot->getvMax() << endl;
    cout << "obstacle clearance: " << m_map->getObstacleClearance() << endl;

    if (m_robot->getSpeed() == 0) {
        cout << "speed: low" << endl;
    }
    else {
        cout << "speed: high" << endl;
    }

    if (problem.h == h_dubins) {
        m_heuristic = &TStarImp::dubins_heuristic;
        m_extended_state = true;
        cout << "Heuristic function: Dubins" << endl;
    }
    else if (problem.h == h_euclidean) {
        m_heuristic = &TStarImp::euclidean_heuristic;
        cout << "Heuristic function: Euclidean" << endl;
    }
	
	
	/*m_cost2 = &TStarImp::dubins_cost;
	m_points_sample2 = g_dubins;
	
	Ocps* _ocps = new Ocps;
	m_ocps = _ocps;
	cout << "Loading ocps table from ocps.txt" << endl;
	m_ocps->load_from_file();
	//m_ocps->build_ocps(0.5, 1, 1, 1, 1, 2, 1);
	m_cost = &TStarImp::varspeed_cost;
	m_points_sample = g_varspeed;*/
	
    if (problem.g == g_dubins) {
		/// edit
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
		std::cout << "calc Dubins begin" << std::endl;
		calculateDubinsTransitions();
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> time = end - start;
		std::cout << "calc Dubins secssus Dubins table size: " << m_dubins_table.size() << std::endl;		
		std::cout << "duration of calc Dubibs paths = " << time.count() << std::endl;
		/// edit
		
        m_cost = &TStarImp::dubins_cost;
        m_points_sample = g_dubins;
        cout << "Cost function: Dubins" << endl;
    }
    else if (problem.g == g_euclidean) {
        m_cost = &TStarImp::euclidean_cost;
        m_points_sample = g_euclidean;
        cout << "Cost function: Euclidean" << endl;
    }
    else if (problem.g == g_varspeed) {
		//m_robot->setOcpsStatus(true);
        Ocps* _ocps = new Ocps;
        m_ocps = _ocps;
        cout << "Loading ocps table from ocps.txt" << endl;
        m_ocps->load_from_file(m_map);
        m_cost = &TStarImp::varspeed_cost;
        m_points_sample = g_varspeed;
        cout << "Cost function: VarSpeed" << endl;
    }

}


TStarImp::~TStarImp()
{
	if (m_robot->getOcpsStatus()) {
		delete m_ocps;
	}
    delete m_robot;
    delete m_map;
    delete m_dubins;
}

void TStarImp::set_epsilon_alg(int alg) {
	if (alg == alg_tstar) {
		m_epsilon_algo = false;
	}
	else {
		m_epsilon_algo = true;
	}
}

bool TStarImp::get_epsilon_alg() {
	return m_epsilon_algo;
}

void TStarImp::set_no_wind(bool wind) {
	m_no_wind = wind;
}

bool TStarImp::get_no_wind() {
	return m_no_wind;
}

void TStarImp::calc_first_time_optimal_trajectories(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient) {
	double v_min = m_robot->getvMin();
	double v_max = m_robot->getvMax();
	double u_max = m_robot->getuMax();

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
	for (auto & neighbor : neighbors) {
		for(int startOrient = 0; startOrient < numOfOrient; startOrient++) {
			for(int finalOrient = 0; finalOrient < numOfOrient; finalOrient++) {
				if (tempMap[{neighbor[0], neighbor[1], (double)startOrient}][finalOrient]) {
					double xFinal = neighbor[0];
					double yFinal = neighbor[1];
//					build_ocps_table(v_min, v_max, u_max, xFinal, yFinal, finalOrient, startOrient, m_map->getWind());
                    m_ocps->build_ocps(v_min, v_max, u_max, xFinal, yFinal, finalOrient, startOrient, m_map->getWind(), m_map->getObstacleClearance());
//					m_ocps->load_from_file(m_map);
				}
			}
		}
	}
    /*double neighbors [16][3] = { // indicates {final x, final y, initial orientation}
                                {1, 0, 0},
                                {1, 1, 0},
                                {0, 1, 0},
                                {-1, 1, 0},
                                {-1, 0, 0},
                                {-1, -1, 0},
                                {0, -1, 0},
                                {1, -1, 0},
                                {1, 1, 1},
                                {0, 1, 1},
                                {-1, 1, 1},
                                {-1, 0, 1},
                                {-1, -1, 1},
                                {0, -1, 1},
                                {1, -1, 1},
                                {1, 0, 1}
                                };

    for (int i = 0; i < 16; i ++) {
        for (int j = 0; j < numOfOrient; j++) {
			if (tempMap[{neighbors[i][0], neighbors[i][1], neighbors[i][2]}][j]) {
				double xFinal = neighbors[i][0];
				double yFinal = neighbors[i][1];
				int orientFinal = j;
				int startOrient = neighbors[i][2];
				build_ocps_table(v_min, v_max, u_max, xFinal, yFinal, orientFinal, startOrient, m_map->getWind());
				m_ocps->load_from_file(m_map);
			}
		}
	}*/
	//m_ocps->load_from_file();
}

void TStarImp::calc_new_translation(std::map<cells, std::vector<bool>> &tempMap, relativeTrajectory vec, double& ocps_time) {
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	/*if (tempMap[{vec.dx, vec.dy, vec.initialTheta}][vec.finalTheta] == true) {
		std::cout << "===================================" << std::endl;
		std::cout << "===================================" << std::endl;
		std::cout << "===================================" << std::endl;
		std::cout << "Calculate already exist translation" << std::endl;
		std::cout << "===================================" << std::endl;
		std::cout << "===================================" << std::endl;
		std::cout << "===================================" << std::endl;
	}*/
	tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] = true;
	double v_min = m_robot->getvMin();
	double v_max = m_robot->getvMax();
	double u_max = m_robot->getuMax();
	/*int startOrient;
	vec.diagonal ? startOrient = 1: startOrient = 0;*/
//	build_ocps_table(v_min, v_max, u_max, vec.dx, vec.dy, vec.finalTheta, vec.initialTheta, m_map->getWind());
//	m_ocps->load_from_file(m_map);
    m_ocps->build_ocps(v_min, v_max, u_max, vec.dx, vec.dy, vec.finalTheta, vec.initialTheta, m_map->getWind(), m_map->getObstacleClearance());
	end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
	ocps_time += elapsed_seconds.count();
}



void TStarImp::set_cost(int num) {
	if (num == 1) {
		m_heuristic = &TStarImp::dubins_heuristic;
        m_extended_state = true;
        cout << "Heuristic function: Dubins" << endl;
		m_cost = &TStarImp::dubins_cost;
		m_points_sample = g_dubins;
	}
	else if (num == 2) {
		//m_robot->setOcpsStatus(true);
		//Ocps* _ocps = new Ocps;
		//m_ocps = _ocps;
		//cout << "Loading ocps table from ocps.txt" << endl;
		//m_ocps->load_from_file(m_map);
		m_cost = &TStarImp::varspeed_cost;
		m_points_sample = g_varspeed;
		
	}
}

int TStarImp::run(double* solution, int* solution_length) {

	// First step: find the trajectories of low speed Dubins
	bool epsilon = true;
	if (this ->get_epsilon_alg()) {
        std::cout << "T*-eps" << std::endl;
		
		/// edit
//		// Make sure the first run done with minimum-speed Dubins
//		State tempState = m_map->get_goal();
//        std::cout << "[" << tempState.row << ", " << tempState.col << ", " << tempState.theta << ", " << tempState.v << "]\n";
//        int original_v = tempState.v;
//        m_map->setVelocities(0);
//        m_robot->setSpeed(0);
//        tempState = m_map->get_goal();
//        std::cout << "[" << tempState.row << ", " << tempState.col << ", " << tempState.theta << ", " << tempState.v << "]\n";
//        State a = m_map->get_start();
//        std::cout << a.v << std::endl;
		m_robot->setOcpsStatus(true);
		Ocps* _ocps = new Ocps;
		m_ocps = _ocps;
		cout << "Loading ocps table from ocps.txt" << endl;
		m_ocps->load_from_file(m_map);
		/// edit
		
		int n_orient = m_robot->getNumOfOrientation();
		std::map<cells , std::vector<bool>> translationTable;
		initMap(translationTable, n_orient);
		cout << "first step" << endl;
		set_cost(1);
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
        bool dubins_solution = false;
        dubins_solution = firstAStar(translationTable);
        if (!dubins_solution) {
            return 1;
        }
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << "Time to calculate the first step = " << elapsed_seconds.count() << " sec" << std::endl;

//        // Set the velocity to its original value for T*-epsilon
//        m_map->setVelocities(original_v);
//        m_robot->setSpeed(original_v);

		cout << "second step" << endl;
		set_cost(2);
		std::chrono::time_point<std::chrono::system_clock> start_ocps, end_ocps;
		start_ocps = std::chrono::system_clock::now();
		/// edit
		//calc_first_time_optimal_trajectories(translationTable, n_orient);
		/// edit
		end_ocps = std::chrono::system_clock::now();
		std::chrono::duration<double> ocps_time = end_ocps - start_ocps;
		
		return AStarEpsilon(solution, solution_length, translationTable, ocps_time.count());
	}
	else { // T*/A* Dubins algorithm
        std::cout << "T*" << std::endl;
		return AStar(solution, solution_length);
	}
}

double TStarImp::dubins_heuristic(State state) {
    State goal = m_map->get_goal();

    /// add ground speed
    double dx = ((double)goal.col - (double)state.col);
    double dy = ((double)goal.row - (double)state.row);
    Wind windDubins = m_map->getWind();
    double wx = windDubins.x;
    double wy = windDubins.y;
    double max_v = m_robot->getvMax();
//    double groundSpeed = m_dubins->getGroundSpeed(wx, wy, wx, wy, max_v);
    ///


    double path[4] = { -1, -1, -1, -1 }; //{1-3 means how long in each segment, 4 means dubins segment type
    m_dubins->vanilaDubinsPath(m_robot, state, goal, m_map->getWind(), path);
    //cout << "heuristic: " << dubinsLength(path) / m_robot->getvMax() << endl;
//    return m_dubins->dubinsLength(path) / groundSpeed;
    return m_dubins->dubinsLength(path) / m_robot->getvMax();
}

double TStarImp::euclidean_heuristic(State state) {
    State goal = m_map->get_goal();
    double distance = sqrt(pow(goal.col - state.col, 2) + pow(goal.row - state.row, 2));
    return distance;
}

double TStarImp::dubins_cost(State s1, State s2, double* path) {
    if (s1.v != s2.v) {
        return -1;
    }
    
	/*obstacleDubinsPath(m_robot, m_map, s1, s2, path);
	if (path[0] == -1) {
		return -1;
	}
	return dubinsLength(path);*/
	dubinsCandidate bestCand;
//    relativeTrajectory transition = calculateRelativeTrajectory(s1, s2, 8);
    double dx = s2.col - s1.col;
    double dy = s2.row - s1.row;
    transition trans = {dx,dy,s2.theta,s1.theta};
	dubinsGetBestCandidate(s1, trans, bestCand);
    if (bestCand.cost == -1) {
		return -1;
    }
	path[0] = bestCand.path[0];
	path[1] = bestCand.path[1];
	path[2] = bestCand.path[2];
	path[3] = bestCand.path[3];
	//path[3] = (double)bestCand.key;
	return bestCand.cost;
	
}

double TStarImp::varspeed_cost(State s1, State s2, double* path) {
    //Find the best VarSpeed path
    Candidate best_varspeed;
    m_ocps->VarSpeedGetBestCandidate(m_map, s1.row, s1.col, s1.theta, s1.v, s2.row - s1.row, s2.col - s1.col, s2.theta, s2.v, best_varspeed);

    if (best_varspeed.cost != -1) {
        path[3] = (double)best_varspeed.key;
        return best_varspeed.cost;
    }
    return -1;
}

double TStarImp::euclidean_cost(State s1, State s2, double* path) {
    double distance = sqrt(pow(s1.col - s2.col, 2) + pow(s1.row - s2.row, 2));
    return distance;
}

int TStarImp::close_key(State s) {
    int width = m_map->get_map_width();
    int height = m_map->get_map_height();
    int orient = m_robot->getNumOfOrientation();

    return (s.col + width * (s.row + height * (s.theta + orient * s.v)));
}
void TStarImp::solution_add(double* solution, int curr_pos, Node node) {
    solution[curr_pos * 8] = (double)node.state.row;
    solution[curr_pos * 8 + 1] = (double)node.state.col;
    solution[curr_pos * 8 + 2] = (double)node.state.theta;
    solution[curr_pos * 8 + 3] = (double)node.state.v;
    solution[curr_pos * 8 + 4] = node.pathFromParent[0];
    solution[curr_pos * 8 + 5] = node.pathFromParent[1];
    solution[curr_pos * 8 + 6] = node.pathFromParent[2];
    solution[curr_pos * 8 + 7] = node.pathFromParent[3];
}

int TStarImp::AStar(double* solution, int* solution_length) {
    int** map = m_map->get_map();
    int width = m_map->get_map_width();
    int height = m_map->get_map_height();
    State start = m_map->get_start();
    State goal = m_map->get_goal();

    if (!m_map->isValid(start.row, start.col) || !m_map->isValid(goal.row, goal.col)){
        printf("start or goal states out of range\n");
        return false;
    }
    if (!m_map->collisionFree(start.row, start.col) || !m_map->collisionFree(goal.row, goal.col)){
        printf("start or goal states are not collision free\n");
        return false;
    }
    int orient = m_robot->getNumOfOrientation(); 
    

    //std::set <std::pair<double, Node>> open;
    std::priority_queue<Node> open;
    std::map <int, Node> close;

    Node start_node;
    start_node.state = start;
    start_node.g = 0;
    start_node.f = (this->*m_heuristic)(start);
    //open.insert(make_pair(start_node.f, start_node));
    open.push(start_node);
	int iter = 0;
	
    while (!open.empty()) {
        //pair<double, Node> open_top = *open.begin();
        //Node next_node = open_top.second;
        Node next_node = open.top();
		open.pop();
		std::map<int, Node>::iterator closed_it = close.find(close_key(next_node.state));
		if (closed_it != close.end()) {
		    continue;
		}
        //open.erase(open.begin());
        close[close_key(next_node.state)] = next_node;
		
        
        //std::cout << "next node f " << next_node.f << std::endl;
        //std::cout << "row,col,theta,v " << next_node.state.row << "," << next_node.state.col << "," << next_node.state.theta << "," << next_node.state.v << std::endl;
        //if (next_node.pathFromParent[3] == 95280) {
        //    std::cout << "trying :" << next_node.state.row << "," << next_node.state.col << "," << next_node.state.v << "," << next_node.state.theta << std::endl;
        //    std::cout << "cost is: " << next_node.g << std::endl;
        //}
        if (m_map->isGoal(next_node.state)){
            int num_of_states_in_path = 0;
			// reconsruct the path
			//cout << "number of iterations: " << iter << endl;
			//cout << "size of open list: " << open.size() << endl;
            double absTheta = (double)next_node.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            cout << "Cost to path: " << next_node.g * 2<< endl; // we mlutiply by two since the article did it as well
            cout << "The path from goal to start:" << endl;
            cout << "["<<next_node.state.row<<", "<< next_node.state.col<<", "<<absTheta<<", "<<next_node.state.v << "]" <<endl;
            num_of_states_in_path++;

            int sol_len = 0;
            solution_add(solution, sol_len, next_node);
			
			relativeTrajectory vec = calculateRelativeTrajectory(next_node.parent, next_node.state, m_robot->getNumOfOrientation());
            std::cout << "[" << next_node.parent.row << ", " << next_node.parent.col << ", " << next_node.parent.theta << ", " << next_node.parent.v << "] -->";
            std::cout << "[" << next_node.state.row << ", " << next_node.state.col << ", " << next_node.state.theta << ", " << next_node.state.theta << "] ";
            std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.finalTheta << ", " << vec.initialTheta << "]" << std::endl;

            Node temp = close[close_key(next_node.parent)];
            while (!(temp.state == start)) {
                sol_len += 1;
                solution_add(solution, sol_len, temp);
                
				vec = calculateRelativeTrajectory(temp.parent, temp.state, m_robot->getNumOfOrientation());
				
                absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
                cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "]" << endl;
				
				std::cout << "[" << temp.parent.row << ", " << temp.parent.col << ", " << temp.parent.theta << ", " << temp.parent.v <<  "] -->";
                std::cout << "[" << temp.state.row << ", " << temp.state.col << ", " << temp.state.theta << ", " << temp.state.v << "] ";
                std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.finalTheta << ", " << vec.initialTheta << "]" << std::endl;
                num_of_states_in_path++;

                temp = close[close_key(temp.parent)];
            }
            absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "]" << endl;
            num_of_states_in_path++;
            std::cout << "Number of states in the path: " << num_of_states_in_path << std::endl;

			sol_len += 1;
            solution_add(solution, sol_len, temp);
            *solution_length = sol_len+1;
            return 1;
        }
        std::vector<State> neighbours = m_map->get_neighbours(next_node.state, orient, m_numOfVelocity, m_extended_state);
        for (std::vector<State>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			//cout << (*it).row << endl;
            double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
            double cost = (this->*m_cost)(next_node.state, *it, path_from_parent); //cost==-1 means no path
            // check there is a valid path between two states
            if (cost != -1){
                
                double gNew = next_node.g + cost;
                double hNew = (this->*m_heuristic)(*it);
                double fNew = gNew + hNew;
                
                Node successor_node;
                successor_node.state = *it;
                successor_node.parent = next_node.state;
                successor_node.g = gNew;
                successor_node.f = fNew;
                successor_node.pathFromParent[0] = path_from_parent[0];
                successor_node.pathFromParent[1] = path_from_parent[1];
                successor_node.pathFromParent[2] = path_from_parent[2];
                successor_node.pathFromParent[3] = path_from_parent[3];
                //check if the node is already in open
                bool in_open = false;
                //for (std::set <std::pair<double, Node>>::iterator oit = open.begin(); oit != open.end(); ++oit) {
//                for (std::vector <Node>::iterator oit = open.begin(); oit != open.end(); ++oit) {
//                    //pair<double, Node> temp_pair = *oit;
//                    Node temp = *oit;
//                    //Node temp = temp_pair.second;
//                    if (temp.state == successor_node.state) {
//                        in_open = true;
//                        if (successor_node.g < temp.g) { // new parent is better
//                            open.erase(oit);
//                            temp.g = gNew;
//                            temp.parent = successor_node.parent;
//                            temp.f = successor_node.f;
//                            temp.pathFromParent[0] = successor_node.pathFromParent[0];
//                            temp.pathFromParent[1] = successor_node.pathFromParent[1];
//                            temp.pathFromParent[2] = successor_node.pathFromParent[2];
//                            temp.pathFromParent[3] = successor_node.pathFromParent[3];
//                            //open.insert(make_pair(temp.f, temp));
//                            open.push_back(temp);
//                            std::sort(open.begin(), open.end(), compareNodes);
//                        }
//                    }
//                }
                if (!in_open) { //state not in OPEN maybe in CLOSED
                    bool in_close = false;
                    std::map<int, Node>::iterator cit = close.find(close_key(successor_node.state));
                    if (cit != close.end()) { //this node exists in CLOSED
                        in_close = true;
                        Node temp = cit->second;
                        if (successor_node.g < temp.g) { // new parent is better
                            close.erase(cit);
                            temp.g = gNew;
                            temp.parent = successor_node.parent;
                            temp.f = successor_node.f;
                            temp.pathFromParent[0] = successor_node.pathFromParent[0];
                            temp.pathFromParent[1] = successor_node.pathFromParent[1];
                            temp.pathFromParent[2] = successor_node.pathFromParent[2];
                            temp.pathFromParent[3] = successor_node.pathFromParent[3];
                            //open.insert(make_pair(temp.f, temp));
                            open.push(temp);
//                            std::sort(open.begin(), open.end(), compareNodes);
                        }
                    }
                    else { // this is a new state - create a new node = insert new node to OPEN
                        //if (path_from_parent[3] == 95280) {
                        //    std::cout << "95280 is a new state in open for sure! " << std::endl;
                        //    std::cout << "with g,h,f " << gNew << "," << hNew << "," << fNew << std::endl;
                        //    std::cout << successor_node.state.row << "," << successor_node.state.col << "," << successor_node.state.theta << "," << successor_node.state.v << std::endl;
                        //}
                        //if (path_from_parent[3] == 37920) {
                        //    std::cout << "37920 is a new state in open for sure! " << std::endl;
                        //    std::cout << "with g,h,f " << gNew << "," << hNew << "," << fNew << std::endl;
                        //    std::cout << successor_node.state.row << ","<< successor_node.state.col << "," << successor_node.state.theta << "," << successor_node.state.v << std::endl;
                        //}
                        //open.insert(make_pair(successor_node.f, successor_node));
                        open.push(successor_node);
//                        std::sort(open.begin(), open.end(), compareNodes);
                        //if (path_from_parent[3] == 37920 || path_from_parent[3] == 95280) {
                            //for (std::set <std::pair<double, Node>>::iterator it = open.begin(); it != open.end(); ++it) {
                            //for (std::vector <Node>::iterator it = open.begin(); it != open.end(); ++it) {
                                //Node n = it->second;
                            //    Node n = *it;
                            //    std::cout << n.f << "," << n.state.row << "," << n.state.col << "," << n.state.theta << " " << n.state.v << std::endl;
                            //}
                        //}
                    }
                }
            }
        }
        iter++;
        if (iter % 1000 == 0) {
            std::cout << "iteration: " << iter << std::endl;
        }
	}
	std::cout << "No solution" << std::endl;
    return 0;
}

bool TStarImp::firstAStar(std::map<cells, std::vector<bool>> &tempMap) {
    int** map = m_map->get_map();
    int width = m_map->get_map_width();
    int height = m_map->get_map_height();
    State start = m_map->get_start();
    State goal = m_map->get_goal();

    Wind dubinsNoWind = {0, 0};

    if (!m_map->isValid(start.row, start.col) || !m_map->isValid(goal.row, goal.col)){
        printf("start or goal states out of range\n");
        return false;
		//return false;
    }
    if (!m_map->collisionFree(start.row, start.col) || !m_map->collisionFree(goal.row, goal.col)){
        printf("start or goal states are not collision free\n");
        return false;
		//return false;
    }
    int orient = m_robot->getNumOfOrientation(); 
    

    //std::set <std::pair<double, Node>> open;
    //std::vector<Node> open;
	///
    //std::vector<Node> open;
	std::priority_queue<Node> open;
	///
    std::map <int, Node> close;

    Node start_node;
    start_node.state = start;
    start_node.g = 0;
    start_node.f = (this->*m_heuristic)(start);
    //open.insert(make_pair(start_node.f, start_node));
    open.push(start_node);
	int iter = 0;
	
    while (!open.empty()) {
        //pair<double, Node> open_top = *open.begin();
        //Node next_node = open_top.second;
        /*Node next_node = open.front();
		open.erase(open.begin());*/
		
		Node next_node = open.top();
		open.pop();
		
		std::map<int, Node>::iterator closed_it = close.find(close_key(next_node.state));
        if (closed_it != close.end()) {
			continue;
		}
        //open.erase(open.begin());
        close[close_key(next_node.state)] = next_node;
		
        
        //std::cout << "next node f " << next_node.f << std::endl;
        //std::cout << "row,col,theta,v " << next_node.state.row << "," << next_node.state.col << "," << next_node.state.theta << "," << next_node.state.v << std::endl;

		
		int num_of_translations = 0;
        if (m_map->isGoal(next_node.state)){
			// Reconstruct the path
			//cout << "number of iterations: " << iter << endl;
			//cout << "size of open list: " << open.size() << endl;
            double absTheta = (double)next_node.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            cout << "Cost to path: " << next_node.g * 2 << endl; // We multiply by two since the article did it as well
            cout << "The path from goal to start:" << endl;
            cout << "["<<next_node.state.row<<", "<< next_node.state.col<<", "<<absTheta<<", "<<next_node.state.v << "]" <<endl;
            int sol_len = 0;
            //solution_add(solution, sol_len, next_node);
			
			// fill up the map, what we actually want
			//std::cout << "before rotate" << std::endl;
			/*std::cout << "before fill" << std::endl;
			std::cout << "parent" << std::endl;
			std::cout << "x = " << next_node.parent.col << " y = " << next_node.parent.row;
			std::cout << " theta = " << next_node.parent.theta << std::endl;
			
			std::cout << "child" << std::endl;
			std::cout << "x = " << next_node.state.col << " y = " << next_node.state.row;
			std::cout << " theta = " << next_node.state.theta << std::endl;*/
			relativeTrajectory vec = calculateRelativeTrajectory(next_node.parent, next_node.state, orient);
			
			std::cout << "[" << next_node.parent.row << ", " << next_node.parent.col << ", " << next_node.parent.theta << "] -->";
			std::cout << "[" << next_node.state.row << ", " << next_node.state.col << ", " << next_node.state.theta << "] ";
			std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.finalTheta << ", " << vec.initialTheta << "]" << std::endl;
			
			/*std::cout << "dx = " << vec.dx << std::endl;
			std::cout << "dy = " << vec.dy << std::endl;
			std::cout << "diagonal = " << vec.diagonal << std::endl;
			std::cout << "theta = " << vec.theta << std::endl;*/
			
			
			assert(tempMap.find({vec.dx, vec.dy, (double)vec.initialTheta}) != tempMap.end());

			//std::cout << tempMap[{vec.dx, vec.dy, vec.initialTheta}][vec.finalTheta] << std::endl;
			tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] = true;
            num_of_translations++;
			
			Node temp = close[close_key(next_node.parent)];
            while (!(temp.state == start)) {
                sol_len += 1;
                //solution_add(solution, sol_len, temp);
				
				// fill up the map, what we actually want
				relativeTrajectory vec = calculateRelativeTrajectory(temp.parent, temp.state, orient);
				std::cout << "[" << temp.parent.row << ", " << temp.parent.col << ", " << temp.parent.theta << "] -->";
				std::cout << "[" << temp.state.row << ", " << temp.state.col << ", " << temp.state.theta << "] ";
				std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.finalTheta << ", " << vec.initialTheta << "]" << std::endl;
				/*std::cout << "before fill" << std::endl;
				std::cout << tempMap[{vec.dx, vec.dy, vec.diagonal}][vec.theta] << std::endl;*/

                assert(tempMap.find({vec.dx, vec.dy, (double)vec.initialTheta}) != tempMap.end());

                tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] = true;
				num_of_translations++;
				/*std::cout << "after fill" << std::endl;
				std::cout << tempMap[{vec.dx, vec.dy, vec.diagonal}][vec.theta] << std::endl;*/
                
                absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
                cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "]" << endl;
                temp = close[close_key(temp.parent)];
            }
            absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "]" << endl;
			
			sol_len += 1;
            //solution_add(solution, sol_len, temp);
            //*solution_length = sol_len+1;
			std::cout << "==================================================================" << std::endl;
			std::cout << "Translation map of low-speed Dubins" << std::endl;
			printMap(tempMap, orient);
            return true;
        }
        std::vector<State> neighbours = m_map->get_neighbours(next_node.state, orient, m_numOfVelocity, m_extended_state);
        for (std::vector<State>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			//cout << (*it).row << endl;
            double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
            double cost = (this->*m_cost)(next_node.state, *it, path_from_parent); //cost==-1 means no path
            // check there is a valid path between two states
            if (cost != -1){
                
                double gNew = next_node.g + cost;
//                double hNew = (this->*m_heuristic)(*it);
                double tempPath[4] = {-1, -1, -1, -1};
                m_dubins->vanilaDubinsPath(m_robot, next_node.state, *it, dubinsNoWind, tempPath);
                double hNew = m_dubins->dubinsLength(tempPath);
                double fNew = gNew + hNew;
                
                Node successor_node;
                successor_node.state = *it;
                successor_node.parent = next_node.state;
                successor_node.g = gNew;
                successor_node.f = fNew;
                successor_node.pathFromParent[0] = path_from_parent[0];
                successor_node.pathFromParent[1] = path_from_parent[1];
                successor_node.pathFromParent[2] = path_from_parent[2];
                successor_node.pathFromParent[3] = path_from_parent[3];
                //check if the node is already in open
                bool in_open = false;
                //for (std::set <std::pair<double, Node>>::iterator oit = open.begin(); oit != open.end(); ++oit) {
                //for (std::vector <Node>::iterator oit = open.begin(); oit != open.end(); ++oit) {
				//
				/*for (dArray::iterator oit = open.begin(); oit != open.end(); oit++){
                    //pair<double, Node> temp_pair = *oit;
                    Node temp = *oit;
                    //Node temp = temp_pair.second;
                    if (temp.state == successor_node.state) {
						dArray::handle_type h = dArray::s_handle_from_iterator(oit);
                        in_open = true;
                        if (successor_node.g < temp.g) { // new parent is better
                            //open.erase(oit);
                            (*h).g = gNew;
                            (*h).parent = successor_node.parent;
                            (*h).f = successor_node.f;
                            (*h).pathFromParent[0] = successor_node.pathFromParent[0];
                            (*h).pathFromParent[1] = successor_node.pathFromParent[1]; 
                            (*h).pathFromParent[2] = successor_node.pathFromParent[2];
                            (*h).pathFromParent[3] = successor_node.pathFromParent[3];
                            //open.insert(make_pair(temp.f, temp));
                            open.update(h);
                            //std::sort(open.begin(), open.end(), compareNodes);
                        }
                    }
                }*/
                if (!in_open) { //state not in OPEN maybe in CLOSED
                    //bool in_close = false;
                    std::map<int, Node>::iterator cit = close.find(close_key(successor_node.state));
                    if (cit != close.end()) { //this node exists in CLOSED
                        //in_close = true;
                        Node temp = cit->second;
                        if (successor_node.g < temp.g) { // new parent is better
                            close.erase(cit);
                            temp.g = gNew;
                            temp.parent = successor_node.parent;
                            temp.f = successor_node.f;
                            temp.pathFromParent[0] = successor_node.pathFromParent[0];
                            temp.pathFromParent[1] = successor_node.pathFromParent[1];
                            temp.pathFromParent[2] = successor_node.pathFromParent[2];
                            temp.pathFromParent[3] = successor_node.pathFromParent[3];
                            //open.insert(make_pair(temp.f, temp));
                            open.push(temp);
                            //std::sort(open.begin(), open.end(), compareNodes);
                        }
                    }
                    else { // this is a new state - create a new node = insert new node to OPEN
                        //if (path_from_parent[3] == 95280) {
                        //    std::cout << "95280 is a new state in open for sure! " << std::endl;
                        //    std::cout << "with g,h,f " << gNew << "," << hNew << "," << fNew << std::endl;
                        //    std::cout << successor_node.state.row << "," << successor_node.state.col << "," << successor_node.state.theta << "," << successor_node.state.v << std::endl;
                        //}
                        //if (path_from_parent[3] == 37920) {
                        //    std::cout << "37920 is a new state in open for sure! " << std::endl;
                        //    std::cout << "with g,h,f " << gNew << "," << hNew << "," << fNew << std::endl;
                        //    std::cout << successor_node.state.row << ","<< successor_node.state.col << "," << successor_node.state.theta << "," << successor_node.state.v << std::endl;
                        //}
                        //open.insert(make_pair(successor_node.f, successor_node));
                        open.push(successor_node);
                        //std::sort(open.begin(), open.end(), compareNodes);
                        //if (path_from_parent[3] == 37920 || path_from_parent[3] == 95280) {
                            //for (std::set <std::pair<double, Node>>::iterator it = open.begin(); it != open.end(); ++it) {
                            //for (std::vector <Node>::iterator it = open.begin(); it != open.end(); ++it) {
                                //Node n = it->second;
                            //    Node n = *it;
                            //    std::cout << n.f << "," << n.state.row << "," << n.state.col << "," << n.state.theta << " " << n.state.v << std::endl;
                            //}
                        //}
                    }
                }
            }
        }
		iter++;
	}
    //return 0;
	std::cout << "No solution" << std::endl;
	return false;
}

int TStarImp::AStarEpsilon(double* solution, int* solution_length, std::map<cells, std::vector<bool>> &tempMap, double ocps_time) {
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();
	double first_translations_calc_time = ocps_time; // the time of calculation of the low speed dubins translations 
	
	int** map = m_map->get_map();
    int width = m_map->get_map_width();
    int height = m_map->get_map_height();
    State start = m_map->get_start();
    State goal = m_map->get_goal();

    if (!m_map->isValid(start.row, start.col) || !m_map->isValid(goal.row, goal.col)){
        printf("start or goal states out of range\n");
        return false;
    }
    if (!m_map->collisionFree(start.row, start.col) || !m_map->collisionFree(goal.row, goal.col)){
        printf("start or goal states are not collision free\n");
        return false;
    }
    int orient = m_robot->getNumOfOrientation(); 
    

    //std::set <std::pair<double, Node>> open;
	///
    //std::vector<Node> open;
	binomialHeap timeOptimal;
	binomialHeap non_timeOptimal;
	///
    std::map <int, Node> close;
	
	/////////////////////
	// Astar-epsilon
	//std::vector<Node> focal;
	double omega = m_map->getEpsilon();
	/////////////////////
	

    Node start_node;
    start_node.state = start;
    start_node.g = 0;
    start_node.f = (this->*m_heuristic)(start);
	start_node.pathFromParentType = true;
    //open.insert(make_pair(start_node.f, start_node));
	///
    //open.push_back(start_node);
	timeOptimal.push(start_node);
	///
	int iter = 0;
	
	double findNextNodeTime = 0;
	double sortTime = 0;
	double costTime = 0;

    while (!timeOptimal.empty() || !non_timeOptimal.empty()) {
//        std::cout << "Queue is not empty" << std::endl;
		
		
        bool new_translation = false;
		//Node next_node = find_next_node(open, tempMap, omega, ocps_time, new_translation);
		
		std::chrono::time_point<std::chrono::system_clock> startFind, endFind;
		startFind = std::chrono::system_clock::now();
		///
		//Node next_node = find_next_node(open, tempMap, omega, ocps_time, new_translation);
		Node next_node = find_next_node_heap(timeOptimal, non_timeOptimal, tempMap, omega, ocps_time, new_translation);
		///
		endFind = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = endFind - startFind;
		findNextNodeTime += elapsed_seconds.count();
		
		
		
		
		if (next_node.pathFromParentType == false) {
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "Problem next node with no time optimal translation" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		}
		
		///
		std::map<int, Node>::iterator cit = close.find(close_key(next_node.state));
		if (cit != close.end()) {
			continue;
		}
		///
		
        close[close_key(next_node.state)] = next_node;
		
        if (m_map->isGoal(next_node.state)){
            int num_of_states_in_path = 0;
			// Reconstruct the path
			//cout << "number of iterations: " << iter << std::endl;
			//cout << "size of open list: " << open.size() << endl;
            double absTheta = (double)next_node.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            ///
            cout << "Cost to path: " << next_node.g * 2<< endl; // We multiply by two since the article did it as well
            //cout << "Cost to path: " << next_node.g * 2 * m_robot->getvMin()<< endl; 
			///
			end_time = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end_time - start_time;
			double TStar_eps_time = elapsed_seconds.count() - ocps_time + first_translations_calc_time;
			std::cout << "Time-optimal translations calculate time: " << ocps_time << " sec" << std::endl;
			std::cout << "T*-ε time: " << TStar_eps_time << " sec" << std::endl;
            std::cout << "Total runtime of T*-epsilon: " << ocps_time + TStar_eps_time << std::endl;
            cout << "The path from goal to start:" << endl;
            cout << "["<<next_node.state.row<<", "<< next_node.state.col<<", "<<absTheta<<", "<<next_node.state.v << "] --- g cost = " <<  next_node.g * 2 <<endl;
            num_of_states_in_path++;

            int sol_len = 0;
            solution_add(solution, sol_len, next_node);
			
			relativeTrajectory vec = next_node.parentTranslation;
			
			std::cout << "[" << next_node.parent.row << ", " << next_node.parent.col << ", " << next_node.parent.theta << ", " << next_node.parent.v << "] -->";
			std::cout << "[" << next_node.state.row << ", " << next_node.state.col << ", " << next_node.state.theta << ", " << next_node.state.v << "] ";
			std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.initialTheta << ", " << vec.finalTheta << "]" << std::endl;

            Node temp = close[close_key(next_node.parent)];
			
			//std::cout << "Close Key = " << close_key(temp.state) << std::endl;
			
            while (!(temp.state == start)) {
                sol_len += 1;
                solution_add(solution, sol_len, temp);
                
                absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
                cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "] --- g cost = " <<  temp.g * 2 << endl;
				
				vec = temp.parentTranslation;
				std::cout << "[" << temp.parent.row << ", " << temp.parent.col << ", " << temp.parent.theta << ", " << temp.parent.v <<  "] -->";
				std::cout << "[" << temp.state.row << ", " << temp.state.col << ", " << temp.state.theta << ", " << temp.state.v << "] ";
				std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.initialTheta << ", " << vec.finalTheta << "]" << std::endl;
                num_of_states_in_path++;

                temp = close[close_key(temp.parent)];
				
				//std::cout << "Close Key = " << close_key(temp.state) << std::endl;
				
				if (temp.state.row == 0) {
					std::cout << "Problem state key: " << close_key(temp.state) << std::endl;
					std::cout << "Problem state parent key: " << close_key(temp.parent) << std::endl;
					printMap(tempMap, orient);
					return 1;
				}
            }
            absTheta = (double)temp.state.theta * (2 * M_PI) / m_robot->getNumOfOrientation();
            cout << "[" << temp.state.row << ", " << temp.state.col << ", " << absTheta << ", " << temp.state.v << "] --- g cost = " <<  temp.g * 2 << endl;
            num_of_states_in_path++;
//            std::cout << "Number of states in the path: " << num_of_states_in_path << std::endl;

			sol_len += 1;
            solution_add(solution, sol_len, temp);
            *solution_length = sol_len+1;
			std::cout << "==================================================================" << std::endl;
			std::cout << "Translation map of T*-ε" << std::endl;
			printMap(tempMap, orient);
			end_time = std::chrono::system_clock::now();
			/*std::chrono::duration<double> elapsed_seconds = end_time - start_time;
			double TStar_eps_time = elapsed_seconds.count() - ocps_time + first_translations_calc_time;
			std::cout << "Time-optimal translations claulate time: " << ocps_time << " sec" << std::endl;
			std::cout << "T*-ε time: " << TStar_eps_time << " sec" << std::endl;*/
			std::cout << "Time to find the next node: " << findNextNodeTime << std::endl;
			std::cout << "Sorting time: " << sortTime << std::endl;
			std::cout << "Cost calculation time: " << costTime << std::endl;
            return 1;
        }
        std::vector<State> neighbours = m_map->get_neighbours(next_node.state, orient, m_numOfVelocity, m_extended_state);
        for (std::vector<State>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
			//cout << (*it).row << endl;
			double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
            relativeTrajectory vec = calculateRelativeTrajectory(next_node.state, *it, orient);
			//std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.theta << ", " << vec.diagonal << "]" << std::endl;
			double cost;
			
			/// Cost timing start
			
			std::chrono::time_point<std::chrono::system_clock> startCost, endCost;
			startCost = std::chrono::system_clock::now();
			
			endCost = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = endCost - startCost;
			findNextNodeTime += elapsed_seconds.count();
			
			if (in_map(tempMap, vec)) {
				cost = (this->*m_cost)(next_node.state, *it, path_from_parent); //cost==-1 means no path
            } 
			else {
			    /// Lower-bound for non time-optimal transitions
                /// Obstacle Dubins path
//				m_dubins->obstacleDubinsPath(m_robot, m_map, next_node.state, *it, path_from_parent);
				/// add ground speed
//				double dx = ((double)(*it).col - (double)next_node.state.col);
//				double dy = ((double)(*it).row - (double)next_node.state.row);
//				Wind windDubins = m_map->getWind();
//				double wx = windDubins.x;
//				double wy = windDubins.y;
//				double max_v = m_robot->getvMax();
//				double groundSpeed = m_dubins->getGroundSpeed(dx, dy, wx, wy, max_v);
////				///
//                /// Dubins path without obstacles
//                m_dubins->vanilaDubinsPath(m_robot, next_node.state, *it, m_map->getWind(), path_from_parent);
//				if (path_from_parent[0] == -1) {
//					cost =  -1;
//				}
//				else {
//					cost = m_dubins->dubinsLength(path_from_parent) / groundSpeed;
//                    path_from_parent[0] = cost;
//				}
                /// Euclidean distance
                cost = euclideanDistance(next_node.state, *it);
                path_from_parent[0] = cost;
				/*else {
					//cost = -1;
					cost = euclideanDistance(next_node.state, *it);
					//std::cout << "euclidean cost = " << cost << std::endl;
				}*/
				/*cost = euclideanDistance(next_node.state, *it);
				path_from_parent[0] = cost;
				path_from_parent[1] = cost;
				path_from_parent[2] = cost;
				path_from_parent[3] = cost;*/
				//cost = -1;
			}
			
			endCost = std::chrono::system_clock::now();
			std::chrono::duration<double> costDuration = endCost - startCost;
			costTime += costDuration.count();
			
			/// Cost timing end

			
			// Check there is a valid path between two states
			if (cost != -1){
                
                double gNew = next_node.g + cost;
                double hNew = (this->*m_heuristic)(*it);
                double fNew = gNew + hNew;
                
                Node successor_node;
                successor_node.state = *it;
                successor_node.parent = next_node.state;
                successor_node.g = gNew;
                successor_node.f = fNew;
                successor_node.pathFromParent[0] = path_from_parent[0];
                successor_node.pathFromParent[1] = path_from_parent[1];
                successor_node.pathFromParent[2] = path_from_parent[2];
                successor_node.pathFromParent[3] = path_from_parent[3];
				
				//relativeTrajectory vec = calculateRelativeTrajectory(successor_node.parent, successor_node.state, orient);
				//std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.theta << ", " << vec.diagonal << "]" << std::endl;
				assert(tempMap.find({vec.dx, vec.dy, (double)vec.initialTheta}) != tempMap.end());
				successor_node.parentTranslation.dx = vec.dx;
				successor_node.parentTranslation.dy = vec.dy;
				successor_node.parentTranslation.initialTheta = vec.initialTheta;
				successor_node.parentTranslation.finalTheta = vec.finalTheta;
				successor_node.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
				
                //Check if the node is already in open
                bool in_open = false;
                //for (std::set <std::pair<double, Node>>::iterator oit = open.begin(); oit != open.end(); ++oit) {
                ///
				/*for (std::vector <Node>::iterator oit = open.begin(); oit != open.end(); ++oit) {
                    //pair<double, Node> temp_pair = *oit;
                    Node temp = *oit;
                    //Node temp = temp_pair.second;
                    if (temp.state == successor_node.state) {
                        in_open = true;
                        if ((successor_node.g < temp.g) || ((temp.pathFromParentType == false) && (successor_node.pathFromParentType == true))) { // new parent is better or pathFromParentType was false and now true
                            open.erase(oit);
                            temp.g = gNew;
                            temp.parent = successor_node.parent;
                            temp.f = successor_node.f;
                            temp.pathFromParent[0] = successor_node.pathFromParent[0];
                            temp.pathFromParent[1] = successor_node.pathFromParent[1]; 
                            temp.pathFromParent[2] = successor_node.pathFromParent[2];
                            temp.pathFromParent[3] = successor_node.pathFromParent[3];
							
							relativeTrajectory vec = calculateRelativeTrajectory(temp.parent, temp.state, orient);
							//std::cout << "in open" << std::endl;
							//std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.theta << ", " << vec.diagonal << "]" << std::endl;
							assert(tempMap.find({vec.dx, vec.dy, (double) vec.diagonal}) != tempMap.end());
							temp.parentTranslation.dx = vec.dx;
							temp.parentTranslation.dy = vec.dy;
							temp.parentTranslation.diagonal = vec.diagonal;
							temp.parentTranslation.theta = vec.theta;
							temp.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.diagonal}][vec.theta];
                            //open.insert(make_pair(temp.f, temp));
                            open.push_back(temp);
							//std::sort(open.begin(), open.end(), compareNodes);
							
							std::chrono::time_point<std::chrono::system_clock> startSort, endSort;
							startSort = std::chrono::system_clock::now();
                            std::sort(open.begin(), open.end(), compareNodes);
							endSort = std::chrono::system_clock::now();
							std::chrono::duration<double> sortTimeInSeconds = endSort - startSort;
							sortTime += sortTimeInSeconds.count();
                        }
                    }
                }*/
                if (!in_open) { //State not in OPEN maybe in CLOSED
                    //bool in_close = false;
                    std::map<int, Node>::iterator cit = close.find(close_key(successor_node.state));
                    if (cit != close.end()) { //This node exists in CLOSED
                        //in_close = true;
                        Node temp = cit->second;
						
						relativeTrajectory vec = calculateRelativeTrajectory(successor_node.parent, temp.state, orient);
						
                        if ((successor_node.g < temp.g) && (in_map(tempMap, vec))) { // New parent is better
                            close.erase(cit);
                            temp.g = gNew;
                            temp.parent = successor_node.parent;
                            temp.f = successor_node.f;
                            temp.pathFromParent[0] = successor_node.pathFromParent[0];
                            temp.pathFromParent[1] = successor_node.pathFromParent[1];
                            temp.pathFromParent[2] = successor_node.pathFromParent[2];
                            temp.pathFromParent[3] = successor_node.pathFromParent[3];
							
							//relativeTrajectory vec = calculateRelativeTrajectory(temp.parent, temp.state, orient);
							//std::cout << "in close" << std::endl;
							//std::cout << "relative: [" << vec.dx << ", " << vec.dy << ", " << vec.theta << ", " << vec.diagonal << "]" << std::endl;
							assert(tempMap.find({vec.dx, vec.dy, (double)vec.initialTheta}) != tempMap.end());
							temp.parentTranslation.dx = vec.dx;
							temp.parentTranslation.dy = vec.dy;
							temp.parentTranslation.initialTheta = vec.initialTheta;
							temp.parentTranslation.finalTheta = vec.finalTheta;
							temp.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
							
                            //open.insert(make_pair(temp.f, temp));
							///
                            //open.push_back(temp);
							timeOptimal.push(temp);
							///
                            //std::sort(open.begin(), open.end(), compareNodes);
							
							// Sorting the OPEN
							/*std::chrono::time_point<std::chrono::system_clock> startSort, endSort;
							startSort = std::chrono::system_clock::now();
                            std::sort(open.begin(), open.end(), compareNodes);
							endSort = std::chrono::system_clock::now();
							std::chrono::duration<double> sortTimeInSeconds = endSort - startSort;
							sortTime += sortTimeInSeconds.count();*/
                        }
                    }
                    else { // this is a new state - create a new node = insert new node to OPEN
                        //open.insert(make_pair(successor_node.f, successor_node));
						///
                        //open.push_back(successor_node);
						if (successor_node.pathFromParentType) {
							timeOptimal.push(successor_node);
						}
						else {
							non_timeOptimal.push(successor_node);
						}
						///
                        //std::sort(open.begin(), open.end(), compareNodes);
						
						// Sorting the OPEN
						/*std::chrono::time_point<std::chrono::system_clock> startSort, endSort;
						startSort = std::chrono::system_clock::now();
						std::sort(open.begin(), open.end(), compareNodes);
						endSort = std::chrono::system_clock::now();
						std::chrono::duration<double> sortTimeInSeconds = endSort - startSort;
						sortTime += sortTimeInSeconds.count();*/
                        //if (path_from_parent[3] == 37920 || path_from_parent[3] == 95280) {
                            //for (std::set <std::pair<double, Node>>::iterator it = open.begin(); it != open.end(); ++it) {
                            //for (std::vector <Node>::iterator it = open.begin(); it != open.end(); ++it) {
                                //Node n = it->second;
                            //    Node n = *it;
                            //    std::cout << n.f << "," << n.state.row << "," << n.state.col << "," << n.state.theta << " " << n.state.v << std::endl;
                            //}
                        //}
                    }
                }
            }
        }
		///
		/*if (new_translation == true) {
			// new time optimal translation has calculated during last iteration
			// open list should been updated
			open = update_open_list(open, tempMap);
		}*/
		///
		iter++;
//        std::cout << "==============================================" << std::endl;
//        std::cout << "Number of iteration: " << iter << std::endl;
//        std::cout << "Time optimal size: " << timeOptimal.size() << std::endl;
//        std::cout << "Non time optimal size: " << non_timeOptimal.size() << std::endl;
//        //std::cout << "Open list size: " << open.size() << std::endl;
//        std::cout << "==============================================" << std::endl;
		//std::cout << "Number of iteration: " << iter << std::endl;
		if (iter%150 == 0) {
			std::cout << "==============================================" << std::endl;
			std::cout << "Number of iteration: " << iter << std::endl;
			//std::cout << "Open list size: " << open.size() << std::endl;
			std::cout << "==============================================" << std::endl;
		}
	}
    std::cout << "No solution" << std::endl;
    return 0;
}

int TStarImp::getSegmentNPoints(double* path) {
    if (m_points_sample == g_dubins) {
        return m_dubins->getDubinsSegmentNPoints(m_robot, path);
    }
    else if (m_points_sample == g_varspeed) {
        return m_ocps->getVarSpeedSegmentNPoints((long)path[3]);
    }
    return 0;
}

void TStarImp::getSegmentPoints(State start, double* path, int n_points, double* points_out) {
    if (m_points_sample == g_dubins) {
        m_dubins->getDubinsSegmentPoints(m_robot, start, path, n_points, points_out);
    }
    else if (m_points_sample == g_varspeed) {
        Coordinate coordinate;
        coordinate.row = start.row;
        coordinate.col = start.col;
        int orientation = start.theta;
        m_ocps->getVarSpeedSegmentPoints(coordinate, orientation, (long)path[3], n_points, points_out);
    }
}

void TStarImp::rotationVector(double theta, State vec, double* out) {
    /// returns rotation vector according its rotation matrix
    /// rotation matrix: [cos(theta), sin(theta); -sin(theta), cos(theta)]
    /// State row = y, State col = x
    out[0] = vec.col * cos(theta) - vec.row * sin(theta);
    out[1] = vec.col * sin(theta) + vec.row * cos(theta);
}

void TStarImp::swap(double* arr) {
    double temp = arr[0];
    arr[0] = arr[1];
    arr[1] = temp;
}

relativeTrajectory TStarImp::calculateRelativeTrajectory(State start, State goal, int n_orients) {
    /// In wind condition only dx and dy taking into account
    /// 1. measure the relative degree between start and target orientations
    /// 2. rotate the axis that the start position will be [0, 0, 0] or [0, 0, 1]
    /// 3. get relative translation
    /// 4. mirror if necessary
    relativeTrajectory out;
    if (this->get_no_wind()) {
        out.initialTheta = (int)false;
        double absStartTheta = start.theta * 2 * M_PI / n_orients;
        double absGoalTheta = goal.theta * 2 * M_PI / n_orients;
        double relativeGoalTheta = fmod(absGoalTheta - absStartTheta, 2 * M_PI);
        if (relativeGoalTheta < 0) {
            relativeGoalTheta = 2 * M_PI - fabs(relativeGoalTheta);
        }
        double rotateTheta = 0;
        if ((fabs(sin(absStartTheta)) > 1e-8) && (fabs(cos(absStartTheta)) > 1e-8)) {
            rotateTheta = absStartTheta - M_PI_4;
            out.initialTheta = (int)true;
        } else {
            rotateTheta = absStartTheta;
        }
        double rotateStart[2] = {}; /// [x,y]
        double rotateGoal[2] = {}; /// [x,y]
        rotationVector(-rotateTheta, start, rotateStart);
        rotationVector(-rotateTheta, goal, rotateGoal);
        /// Relative position: [goal.x - start.x, goal.y - start.y]

        /// temp edit
        //double relativePosition[2] = {rotateGoal[0] - rotateStart[0], rotateGoal[1] - rotateStart[1]};
        double relativePosition[2] = {round(rotateGoal[0] - rotateStart[0]), round(rotateGoal[1] - rotateStart[1])};
        /// temp edit

        double dif = 0;
        if (out.initialTheta) {  // start orientation == 1
            if (relativePosition[1] < relativePosition[0]) {  /// y < x
                /// mirror position and orientation
                swap(relativePosition);
                /*dif = fabs(absStartTheta - absGoalTheta);
                relativeGoalTheta = dif + M_PI_4;*/
                relativeGoalTheta = fmod(M_PI_4 + 2 * M_PI - relativeGoalTheta, 2 * M_PI);
            }
            else if ((relativePosition[1] == relativePosition[0]) && (relativeGoalTheta > M_PI)) { // Has to be fixed to == insted of <
                //else if ((relativePosition[1] == relativePosition[0]) && ((relativeGoalTheta > (5 * M_PI / 4 )) || (relativeGoalTheta < M_PI_4))) { // Has to be fixed to == insted of <
                /// mirror orientation
                /*dif = fabs(absStartTheta - absGoalTheta);
                relativeGoalTheta = dif + M_PI_4;*/
                relativeGoalTheta = fmod(M_PI_4 + 2 * M_PI - relativeGoalTheta, 2 * M_PI);
            }
            else {
                relativeGoalTheta = fmod(relativeGoalTheta + M_PI_4, 2 * M_PI);
            }
        }
        else {  // start orientation == 0
            if (relativePosition[1] < 0) { /// y < 0
                /// mirror position and orientation
                relativePosition[1] = fabs(relativePosition[1]);
                relativeGoalTheta = fmod(2 * M_PI - relativeGoalTheta, 2 * M_PI);
            }
            else if ((relativePosition[1] == 0) && (relativeGoalTheta > M_PI)) { // Has to be fixed to == insted of <
                /// mirror orientation
                relativeGoalTheta = fmod(2 * M_PI - relativeGoalTheta, 2 * M_PI);
            }
        }
        double returnedTheta = relativeGoalTheta / (2 * M_PI / n_orients);
        /*if (fabs(relativePosition[0]) < 1e-8) {
            relativePosition[0] = 0;
        }
        if (fabs(relativePosition[1]) < 1e-8) {
            relativePosition[1] = 0;
        }*/
        out.dx = round(relativePosition[0]);
        out.dy = round(relativePosition[1]);
        out.finalTheta = (int) returnedTheta;
        /*std::cout << "Relative trajectory:" << std::endl;
        std::cout << "dx = " << out.dx << std::endl;
        std::cout << "dy = " << out.dy << std::endl;
        std::cout << "theta = " << out.theta << std::endl;
        std::cout << "diagonal = " << out.diagonal << std::endl;*/
    }
    else {
        out.dx = goal.col - start.col;
        out.dy = goal.row - start.row;
        out.finalTheta = goal.theta;
        out.initialTheta = start.theta;
    }
    return out;
}

double TStarImp::euclideanDistance(State s1, State s2) {
	double dist = pow(s1.row - s2.row, 2) + pow(s1.col - s2.col, 2);
	return sqrt(dist);
}

bool TStarImp::in_map(std::map<cells, std::vector<bool>> tempMap, relativeTrajectory vec) {
	//std::cout << "4" << std::endl;
	return tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
}

std::vector<Node> TStarImp::update_open_list(std::vector<Node> openList, std::map<cells, std::vector<bool>> &tempMap) {
	// Update the open list after calculate new time optimal translation
	// update the nodes who now have time optimal translation from its' parnets
	std::cout << "Updating open list" << std::endl;
	std::vector<Node> updated_list;
	for (std::vector <Node>::iterator oit = openList.begin(); oit != openList.end(); ++oit) {
		Node temp = *oit;
		/*Node temp;
		temp.state = (*oit).state;
		temp.parent = (*oit).parent;
		temp.pathFromParent[0] = (*oit).pathFromParent[0];
		temp.pathFromParent[1] = (*oit).pathFromParent[1];
		temp.pathFromParent[2] = (*oit).pathFromParent[2];
		temp.pathFromParent[3] = (*oit).pathFromParent[3];
		temp.g = (*oit).g;
		temp.h = (*oit).h;
		temp.f = (*oit).f;
		temp.pathFromParentType = (*oit).pathFromParentType;
		temp.parentTranslation = (*oit).parentTranslation;*/
		
		relativeTrajectory vec = temp.parentTranslation;
		if ((temp.pathFromParentType == false) && (tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] == true)) {
			double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
            double prev_cost = temp.pathFromParent[0];
			double cost = (this->*m_cost)(temp.parent, temp.state, path_from_parent); //cost==-1 means no path
			if (cost == -1) {
				// Do not insert bad nodes to open list
				continue;
			}
			double gNew = temp.g - prev_cost + cost;
			double fNew = gNew + temp.h;
			
			temp.g = gNew;
			temp.f = fNew;
			temp.pathFromParent[0] = path_from_parent[0];
			temp.pathFromParent[1] = path_from_parent[1];
			temp.pathFromParent[2] = path_from_parent[2];
			temp.pathFromParent[3] = path_from_parent[3];
			
			assert(tempMap.find({vec.dx, vec.dy, (double) vec.initialTheta}) != tempMap.end());
			temp.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
		}
		updated_list.push_back(temp);
		//std::sort(updated_list.begin(), updated_list.end(), compareNodes);
		
		/*Node temp = *oit;
		relativeTrajectory vec = temp.parentTranslation;
		if ((temp.pathFromParentType == false) && (tempMap[{vec.dx, vec.dy, (double)vec.diagonal}][vec.theta] == true)) {
			openList.erase(oit);
			double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
            double prev_cost = temp.pathFromParent[0];
			double cost = (this->*m_cost)(temp.parent, temp.state, path_from_parent); //cost==-1 means no path
			if (cost == -1) {
				// Do not insert bad nodes to open list
				continue;
			}
			double gNew = temp.g - prev_cost + cost;
			double fNew = gNew + temp.h;
			
			temp.g = gNew;
			temp.f = fNew;
			temp.pathFromParent[0] = path_from_parent[0];
			temp.pathFromParent[1] = path_from_parent[1];
			temp.pathFromParent[2] = path_from_parent[2];
			temp.pathFromParent[3] = path_from_parent[3];
			
			assert(tempMap.find({vec.dx, vec.dy, (double) vec.diagonal}) != tempMap.end());
			temp.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.diagonal}][vec.theta];
		}
		openList.push_back(temp);
		std::sort(openList.begin(), openList.end(), compareNodes);
		//updated_list.push_back(temp);*/
	}
	std::sort(updated_list.begin(), updated_list.end(), compareNodes);
	//assert(updated_list != openList);
	return updated_list;
}

Node TStarImp::find_next_node(std::vector<Node> &openList, std::map<cells, std::vector<bool>> &tempMap, double omega, double& ocps_time, bool& new_translation) {
	std::vector<Node> focal;
	//State start = m_map->get_start();

    Node next_node = openList.front();
    double openTopFValue = next_node.f;
    bool foundTimeOptimal = false;

    int iterations = 0;
    for (std::vector<Node>::iterator it = openList.begin(); (it != openList.end() && (*it).f <= omega * openTopFValue); ++it) {

        relativeTrajectory vec = (*it).parentTranslation;
        assert(tempMap.find({vec.dx, vec.dy, (double) vec.initialTheta}) != tempMap.end());

        if ((*it).pathFromParentType == true) {
            Node returnedNode = (*it);
            openList.erase(openList.begin() + iterations);
            return returnedNode;
        }
        iterations++;
    }

    if (foundTimeOptimal == false){
        new_translation = true;
        /// calc time optimal translation for top of the open list node
        std::cout << "Need to calculate new translation" << std::endl;
        std::cout << "New translation: [x, y, final theta, initial theta] = [" << next_node.parentTranslation.dx << ", ";
        std::cout << next_node.parentTranslation.dy << ", " << next_node.parentTranslation.finalTheta;
        std::cout << ", " << next_node.parentTranslation.initialTheta << "]" << std::endl;
        //std::cout << "parent translation type: " << tempMap[{next_node.parentTranslation.dx, next_node.parentTranslation.dy, (double)next_node.parentTranslation.diagonal}][next_node.parentTranslation.theta] << std::endl;

        calc_new_translation(tempMap, next_node.parentTranslation, ocps_time);

        relativeTrajectory vec = next_node.parentTranslation;

        // Update the next node
        double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
        double prev_cost = next_node.pathFromParent[0];
        double cost = (this->*m_cost)(next_node.parent, next_node.state, path_from_parent); //cost==-1 means no path
        std::cout << "Cost of time-optimal path from next_node parent to next_node state = " << cost << std::endl;
        if (cost == -1) {
            std::cout << "*************************************" << std::endl;
            std::cout << "*************************************" << std::endl;
            std::cout << "*************************************" << std::endl;
            std::cout << "No time-optimal translation!!!" << std::endl;
            std::cout << "*************************************" << std::endl;
            std::cout << "*************************************" << std::endl;
            std::cout << "*************************************" << std::endl;
            // Need to find new next node. We update the open list with the new time-optimal translation and run again find_next_node function
            openList = update_open_list(openList, tempMap);
            return find_next_node(openList, tempMap, omega, ocps_time, new_translation);
        }
        else {
            double gNew = next_node.g - prev_cost + cost;
            double fNew = gNew + next_node.h;

            next_node.g = gNew;
            next_node.f = fNew;
            next_node.pathFromParent[0] = path_from_parent[0];
            next_node.pathFromParent[1] = path_from_parent[1];
            next_node.pathFromParent[2] = path_from_parent[2];
            next_node.pathFromParent[3] = path_from_parent[3];

            assert(tempMap.find({vec.dx, vec.dy, (double) vec.initialTheta}) != tempMap.end());
            next_node.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
            return next_node;
        }
    }
	
	/*
	Node next_node = openList.front();
		
	Node topNode = openList.front();
	focal.push_back(topNode);
	openList.erase(openList.begin());
	double openTopFValue = next_node.f;
	bool foundTimeOptimal = false;
	while(topNode.f <= omega * openTopFValue) {
		if (topNode.state == start){ 
			while (focal.size() > 1) {
				openList.push_back(focal.front());
				focal.erase(focal.begin());
			}
			focal.clear();
			//std::sort(openList.begin(), openList.end(), compareNodes);
			return topNode;
		}
		relativeTrajectory vec = topNode.parentTranslation;
		assert(tempMap.find({vec.dx, vec.dy, (double) vec.diagonal}) != tempMap.end());
		
		if (topNode.pathFromParentType == true) {
			// The next node to expend is located in the last index in focal list 
			while (focal.size() > 1) {
				openList.push_back(focal.front());
				focal.erase(focal.begin());
			}
			focal.clear();
			//std::sort(openList.begin(), openList.end(), compareNodes);
			return topNode;
		}
		topNode = openList.front();
		focal.push_back(topNode);
		openList.erase(openList.begin());
		if (tempMap[{vec.dx, vec.dy, vec.diagonal}][vec.theta]) {
			if (topNode.pathFromParentType == false) {
				double temp_path[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
				double temp_cost = (this->*m_cost)(topNode.parent, topNode.state, temp_path); //cost==-1 means no path
				double prev_cost = topNode.pathFromParent[0];
				if (temp_cost == -1){
					std::cout << "There is no time optimal translation between father and child" << std::endl;
					std::cout << "=========" << std::endl;
					std::cout << "=========" << std::endl;
					std::cout << "=========" << std::endl;
					std::cout << "ERROR cost = -1" << std::endl;
					std::cout << "=========" << std::endl;
					std::cout << "=========" << std::endl;
					std::cout << "=========" << std::endl;
				}
				topNode.pathFromParent[0] = temp_path[0];
				topNode.pathFromParent[1] = temp_path[1];
				topNode.pathFromParent[2] = temp_path[2];
				topNode.pathFromParent[3] = temp_path[3];
				topNode.g += temp_cost - prev_cost;
				topNode.f = topNode.g + topNode.h;
				topNode.pathFromParentType = true;
			}
			next_node = topNode;
			std::cout << "Found state with time optimal translation" << std::endl;
			std::cout << "next state: " << next_node.state.row << ", " << next_node.state.col;
			std::cout << ", " << next_node.state.theta << std::endl;
			std::cout << "next node f " << next_node.f << std::endl;
			std:: cout << "=========" << std::endl;
			// next_node is in the last index in the focal list so we 
			// move all of the nodes there exept next_node to the open list
			while (focal.size() > 1) {
				open.push_back(focal.front());
				focal.erase(focal.begin());
			}
			//Node temp_node = focal.front();
			std::cout << "focal top state: " << temp_node.state.row << ", " << temp_node.state.col;
			std::cout << ", " << temp_node.state.theta << std::endl;
			std::cout << "focal top f " << temp_node.f << std::endl;
			std:: cout << "=========" << std::endl;
			focal.clear();
			foundTimeOptimal = true;
			break;
		}*/
		// check if path from parent type is different than the relevant translation in the map
		/*if (topNode.pathFromParentType != tempMap[{vec.dx, vec.dy, (double)vec.diagonal}][vec.theta]) {
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
			std::cout << "Open list did not update well" << std::endl;
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
			std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
		}*/
	//}*/
	
	
	/*
	if (foundTimeOptimal == false){
		std::cout << "2.3\n";
		new_translation = true;
		/// calc time optimal translation for top of the open list node
		std::cout << "Need to calculate new translation" << std::endl;
		std::cout << "New translation: [x, y, theta, diagonal] = [" << next_node.parentTranslation.dx << ", ";
		std::cout << next_node.parentTranslation.dy << ", " << next_node.parentTranslation.theta;
		std::cout << ", " << next_node.parentTranslation.diagonal << "]" << std::endl;
		//std::cout << "parent translation type: " << tempMap[{next_node.parentTranslation.dx, next_node.parentTranslation.dy, (double)next_node.parentTranslation.diagonal}][next_node.parentTranslation.theta] << std::endl;
		
		calc_new_translation(tempMap, next_node.parentTranslation, ocps_time);
		
		// send back all the nodes from focal to open and update open
		while (focal.size() > 1) {
			openList.push_back(focal.back());
			focal.erase(focal.end());
		}
		focal.clear();
		//std::sort(openList.begin(), openList.end(), compareNodes);
		
		relativeTrajectory vec = next_node.parentTranslation;
		
		// Update the next node
		double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
		double prev_cost = next_node.pathFromParent[0];
		double cost = (this->*m_cost)(next_node.parent, next_node.state, path_from_parent); //cost==-1 means no path
		std::cout << "Cost of time-optimal path from next_node parent to next_node state = " << cost << std::endl;
		if (cost == -1) {
			std::cout << "*************************************" << std::endl;
			std::cout << "*************************************" << std::endl;
			std::cout << "*************************************" << std::endl;
			std::cout << "No time-optimal translation!!!" << std::endl;
			std::cout << "*************************************" << std::endl;
			std::cout << "*************************************" << std::endl;
			std::cout << "*************************************" << std::endl;
			// Need to find new next node. We update the open list with the new time-optimal translation and run again find_next_node function
			openList = update_open_list(openList, tempMap);
			return find_next_node(openList, tempMap, omega, ocps_time, new_translation);
		}
		else {
		double gNew = next_node.g - prev_cost + cost;
		double fNew = gNew + next_node.h;
		
		next_node.g = gNew;
		next_node.f = fNew;
		next_node.pathFromParent[0] = path_from_parent[0];
		next_node.pathFromParent[1] = path_from_parent[1];
		next_node.pathFromParent[2] = path_from_parent[2];
		next_node.pathFromParent[3] = path_from_parent[3];
		
		assert(tempMap.find({vec.dx, vec.dy, (double) vec.diagonal}) != tempMap.end());
		next_node.pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.diagonal}][vec.theta];
		return next_node;
		}
	}*/

		
		//Node temp_node = focal.front();
		/*relativeTrajectory temp_vec = temp_node.parentTranslation;
		std:: cout << "==================================" << std::endl;
		std::cout << "focal top translation: [" << temp_vec.dx << ", " << temp_vec.dy;
		std::cout << ", " << temp_vec.theta << ", " << temp_vec.diagonal << "]" << std::endl;
		std:: cout << "==================================" << std::endl;*/
		
		/*openList = update_open_list(openList, tempMap);
		return find_next_node(openList, tempMap, omega, ocps_time, new_translation);*/
		//return next_node;
		
		/*double temp_new_path[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
		double temp_new_cost = (this->*m_cost)(next_node.parent, next_node.state, temp_new_path); //cost==-1 means no path
		double prev_cost_new = next_node.pathFromParent[0];
		next_node.pathFromParent[0] = temp_new_path[0];
		next_node.pathFromParent[1] = temp_new_path[1];
		next_node.pathFromParent[2] = temp_new_path[2];
		next_node.pathFromParent[3] = temp_new_path[3];
		next_node.g += temp_new_cost - prev_cost_new;
		next_node.f = next_node.g + next_node.h;
		next_node.pathFromParentType = true;
		while (focal.size() > 1) {
			openList.push_back(focal.back());
			focal.erase(focal.end());
		}
		/// calc and insert translation from next_node parent to next_node state
		
		focal.clear();*/
	//return next_node;
}

void TStarImp::updateLists(binomialHeap &timeOptimal, binomialHeap &nonTimeOptimal, std::map<cells, std::vector<bool>> &tempMap) {
	std::vector<binomialHeap::handle_type> handlesVector;
    double relative_cost = 0;

	for (binomialHeap::iterator it = nonTimeOptimal.begin(); it != nonTimeOptimal.end(); ++it) {
		Node temp = *it;
		relativeTrajectory vec = temp.parentTranslation;

		if (tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] == true) {
			binomialHeap::handle_type handle = binomialHeap::s_handle_from_iterator(it);
			
			//relativeTrajectory vec = (*it).parentTranslation;
			
			double path_from_parent[4] = { -1, -1, -1, -1 }; // {1-3 means how long in each segment, 4 means dubins segment type/varspeed path key in ocps
			double prev_cost = (*handle).pathFromParent[0];
			double cost = (this->*m_cost)((*handle).parent, (*handle).state, path_from_parent); //cost==-1 means no path
			if (cost != -1) {
//                relative_cost = (prev_cost / cost) * 100;
				double gNew = (*handle).g - prev_cost + cost;
				double fNew = gNew + (*handle).h;
				
				(*handle).g = gNew;
				(*handle).f = fNew;
				(*handle).pathFromParent[0] = path_from_parent[0];
				(*handle).pathFromParent[1] = path_from_parent[1];
				(*handle).pathFromParent[2] = path_from_parent[2];
				(*handle).pathFromParent[3] = path_from_parent[3];
				
				assert(tempMap.find({vec.dx, vec.dy, (double) vec.initialTheta}) != tempMap.end());
				(*handle).pathFromParentType = tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta];
				timeOptimal.push(*handle);
				handlesVector.push_back(handle);
			}
			else {
				handlesVector.push_back(handle);
			}
		}
	}
//	if (relative_cost > 100) {
//	    std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//        std::cout << "Real cost smaller than lower bound" << std::endl;
//        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
//	}
//	std::cout << "Lower bound / time-optimal cost = " << relative_cost << "%" << std::endl;
	// Remove all the bad nodes without time-optimal transition from the queue
	for (std::vector<binomialHeap::handle_type>::iterator hit = handlesVector.begin(); hit != handlesVector.end(); ++hit) {
		nonTimeOptimal.erase(*hit);
	}
}

Node TStarImp::find_next_node_heap(binomialHeap &timeOptimal, binomialHeap &nonTimeOptimal, std::map<cells, std::vector<bool>> &tempMap, double omega, double& ocps_time, bool& new_translation) {
	double openTopFValue;
	//std::cout << "Check if non time-optimal is empty";
	if (nonTimeOptimal.empty()) {
//        std::cout << "non time-optimal is empty" << std::endl;
//        std::cout << "time-optimal size = " << timeOptimal.size() << std::endl;
		Node next_node = timeOptimal.top();
		timeOptimal.pop();
		return next_node;
	}
	else if (timeOptimal.empty()) {
//        std::cout << "time-optimal is empty";
        Node next_node = nonTimeOptimal.top();
        /// calc time optimal translation for top of the open list node
        std::cout << "Time-optimal heap is empty" << std::endl;
        std::cout << "Need to calculate new translation" << std::endl;
        std::cout << "Next node: [x,y,theta] = [" << next_node.state.col << ", " << next_node.state.row ;
        std::cout << ", " << next_node.state.theta << "]\n";
        std::cout << "Next node parent: [x,y,theta] = [" << next_node.parent.col << ", " << next_node.parent.row;
        std::cout << ", " << next_node.parent.theta << "]\n";
        std::cout << "New translation: [x, y, start theta, final theta] = [" << next_node.parentTranslation.dx << ", ";
        std::cout << next_node.parentTranslation.dy << ", " << next_node.parentTranslation.finalTheta;
        std::cout << ", " << next_node.parentTranslation.initialTheta << "]" << std::endl;
		
		/// edit
		relativeTrajectory vec = next_node.parentTranslation;
		tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] = true;
        //calc_new_translation(tempMap, next_node.parentTranslation, ocps_time);
		/// edit
		
        updateLists(timeOptimal, nonTimeOptimal, tempMap);

        return find_next_node_heap(timeOptimal, nonTimeOptimal, tempMap, omega, ocps_time, new_translation);
	}
	else if (timeOptimal.top().f <= omega * nonTimeOptimal.top().f) {
		Node next_node = timeOptimal.top();
		timeOptimal.pop();
		return next_node;
	}
	else {
		Node next_node = nonTimeOptimal.top();
		/// calc time optimal translation for top of the open list node
		std::cout << "Need to calculate new translation" << std::endl;
		std::cout << "New translation: [x, y, start theta, final theta] = [" << next_node.parentTranslation.dx << ", ";
		std::cout << next_node.parentTranslation.dy << ", " << next_node.parentTranslation.finalTheta;
		std::cout << ", " << next_node.parentTranslation.initialTheta << "]" << std::endl;
		
		/// edit
		relativeTrajectory vec = next_node.parentTranslation;
		tempMap[{vec.dx, vec.dy, (double)vec.initialTheta}][vec.finalTheta] = true;
        //calc_new_translation(tempMap, next_node.parentTranslation, ocps_time);
		/// edit
		
		updateLists(timeOptimal, nonTimeOptimal, tempMap);

        return find_next_node_heap(timeOptimal, nonTimeOptimal, tempMap, omega, ocps_time, new_translation);
	}
}
