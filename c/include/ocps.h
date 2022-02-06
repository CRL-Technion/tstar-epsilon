#ifndef OCPS_H
#define OCPS_H

#include <map>
#include <vector>
#include <string>
#include "map_handler.h"
#include<armadillo>

/*#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

// custom
#include<VSDUtils.h>
#include<VSDPath.h>
#include<VSDProblem.h>
#include<VSDSolver.h>
#include<VSDNLP.h>
#include "VarSpeed.h"
#include "VarSpeed_utils.h"*/


typedef struct VS_State {
	int orientation;
	int speed;
	int neighbour_id = -1; //if state is a target than neighbour_id >= 0
} VS_State;

typedef struct Coordinate {
	double row;
	double col;
	int speed; //just for plotting
} Coordinate;

typedef struct intim {
	int row;
	int col;
} intim;

typedef struct Candidate {
	int valid;
	long key;
	double cost;
	Coordinate start;
	int h_initial;
	Coordinate target;
	int h_final;
	std::string path_type;
	std::string path_orientation;
	std::vector<Coordinate>* reduced_path = new std::vector<Coordinate>;
	std::vector<Coordinate>* path_cells = new std::vector<Coordinate>;
} Candidate;

/*bool operator==(const Coordinate& c1, const Coordinate& c2) {
    return (c1.col == c2.col && c1.row == c2.row);
}*/

//void build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, std::map<long, Candidate>& ocps_table);
//void build_ocps_table(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, double buffer, std::map<long, Candidate>& ocps_table);

class Ocps {
	public:
		void load_from_file(Map* map);
		void VarSpeedGetBestCandidate(Map* map, int s1_row, int s1_col, int s1_orientation, int s1_speed, int s2_rel_row, int s2_rel_col, int s2_orientation, int s2_speed, Candidate& best_varspeed);
		int getVarSpeedSegmentNPoints(long ocps_key);
		void getVarSpeedSegmentPoints(Coordinate coordinate, int orientation, long ocps_key, int n_points, double* points_out);
		//void build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation);
		//void build_ocps_tab(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation);
        void build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, double buffer);
		
	private:
		Coordinate rotate(int orientation, Coordinate coordinate);
		int coordinate_to_id(Coordinate coordinate);
		int rotate_neighbours(int n_id);
		long ocps_key(VS_State curr, VS_State target, int path_id);
		bool pathValidityCheck(Map* map, int start_row, int start_col, Candidate& candidate);
		void copy_candidate(Candidate& source, Candidate& target);



//		void build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, std::map<long, Candidate> &ocps_table);



		std::map <long, Candidate> m_ocps_table;
};

#endif // OCPS_H