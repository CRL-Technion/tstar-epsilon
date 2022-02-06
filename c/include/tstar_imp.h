#ifndef TSTAR_IMP_H
#define TSTAR_IMP_H

#include <iostream>
#include <set>
#include <map>
#include <vector>
//#include <bits/stdc++.h>
#include <cassert>
#include <cmath>
#include <chrono>
#include <queue>

#include <boost/heap/d_ary_heap.hpp>

#include "map_handler.h"
#include "Robot.h"
#include "ocps.h"
#include "dubins.h"
#include <boost/heap/binomial_heap.hpp>



typedef struct Node{
    // define the state i,j for x,y coordinate th for theta v for velocity
    // 0 <= theta <= number of orientation -1
    // actual theta calculate by: theta *  number of orientation / 2 PI
    
    State state;
    State parent;
    double pathFromParent[4] = {};

    double g;
    double f;
    double h;
	
	bool pathFromParentType = false;
	relativeTrajectory parentTranslation;
} Node;

struct compareNodes_new {
    bool operator()(const Node & n1, const Node & n2) const
    {
        return n1.f > n2.f;
    }
};


using binomialHeap = boost::heap::binomial_heap<Node,boost::heap::compare<compareNodes_new>>;


typedef std::array<double, 3> cells;



enum h_function { h_dubins=1, h_euclidean};
enum g_function { g_dubins=1, g_euclidean, g_varspeed};

enum algorithm_func {alg_tstar=1, alg_tstareps};

typedef struct Problem {
    double uMax;
    double vMin;
    double vMax;
    int speed; // 0-low , 1-high
    double obstacle_clearance;
    h_function h;
    g_function g;
	double epsilon;
	double wind_x;
	double wind_y;
	algorithm_func alg;
}Problem;

class TStarImp {
	public:
        TStarImp(int* map, int width, int height, State start, State goal, Problem problem);
        ~TStarImp();

		int run(double* solution, int* solution_length);
        
        int getSegmentNPoints(double* path);
        void getSegmentPoints(State start, double* path, int n_points, double* points_out);
		

	private:
        int AStar(double* solution, int* solution_length);
		/// edit
		void initMap(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient);
		void printMap(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient);
		bool firstAStar(std::map<cells, std::vector<bool>> &tempMap);
		int AStarEpsilon(double* solution, int* solution_length, std::map<cells, std::vector<bool>> &tempMap, double ocps_time);
		void rotationVector(double theta, State vec, double* out);
		void swap(double* arr);
		relativeTrajectory calculateRelativeTrajectory(State start, State goal, int n_orients);
		void set_cost(int num);
		void calc_first_time_optimal_trajectories(std::map<cells, std::vector<bool>> &tempMap, int numOfOrient);
		double euclideanDistance(State s1, State s2);
		bool in_map(std::map<cells, std::vector<bool>> tempMap, relativeTrajectory vec);
		void calc_new_translation(std::map<cells, std::vector<bool>> &tempMap, relativeTrajectory vec, double& ocps_time);
		std::vector<Node> update_open_list(std::vector<Node> openList, std::map<cells, std::vector<bool>> &tempMap);
		Node find_next_node(std::vector<Node> &openList, std::map<cells, std::vector<bool>> &tempMap, double omega, double& ocps_time, bool& new_translation);
		Node find_next_node_heap(binomialHeap &timeOptimal, binomialHeap &nonTimeOptimal, std::map<cells, std::vector<bool>> &tempMap, double omega, double& ocps_time, bool& new_translation);
		void updateLists(binomialHeap &timeOptimal, binomialHeap &nonTimeOptimal, std::map<cells, std::vector<bool>> &tempMap);
		
		void calcDubinsPaths(double alpha, double beta, double d, double r, int type, dubinsCandidate cand);
		void calculateDubinsTransitions();
		void dubinsGetBestCandidate(State s, transition trans, dubinsCandidate &bestCandidate);
		
		void set_epsilon_alg(int alg);
		bool get_epsilon_alg();
		
		void set_no_wind(bool wind);
		bool get_no_wind();
		/// edit
		
		
        int close_key(State s);
        void solution_add(double* solution, int curr_pos, Node node);

        double dubins_heuristic(State state);
        double euclidean_heuristic(State state);
        double dubins_cost(State s1, State s2, double* path);
        double euclidean_cost(State s1, State s2, double* path);
        double varspeed_cost(State s1, State s2, double* path);

        double(TStarImp::*m_heuristic)(State);
        double(TStarImp::*m_cost)(State, State, double*);
		

        Map* m_map = 0;
        Robot* m_robot = 0;
        Ocps* m_ocps = 0;
        Dubins* m_dubins = 0;
        bool m_extended_state = false;
		bool m_epsilon_algo;
		bool m_no_wind;
        int m_numOfVelocity = 2;
        int m_points_sample = -1;
		int m_points_sample2 = -1;
		
		std::map<long,dubinsCandidate> m_dubins_table;
        
};

#endif // TSTAR_IMP_H