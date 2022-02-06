#include <iostream>
#include <fstream>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ocps.h"



#define ORIENTATIONS 8
#define NEIGHBOURS 8
#define SPEEDS 2
#define CANDIDATES 34

void build_ocps_table(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, double buffer, std::map<long, Candidate>& ocps_table);

void Ocps::load_from_file(Map* map) {
	std::string line;
	std::ifstream myfile("ocps.txt");
	Coordinate cellCoor;
	cellCoor.row = -1;
	cellCoor.col = -1;
	

	if (myfile.is_open()) {
		long key;
		double cost;
		Candidate candidate;
		std::vector<double> xCords;
		std::vector<double> yCords;
		std::vector<int> speeds;
		std::cout << "ocps file is opened" << std::endl;
		

		while (std::getline(myfile, line)) {
			if (line.find("key") != std::string::npos) {
				//std::cout << "found key: ";
				size_t pos = line.find("=");
				line.erase(0, pos + 1);
				key = std::stol(line);
				candidate.key = key;
				candidate.valid = 1;
				candidate.reduced_path = new std::vector<Coordinate>;
				///
				candidate.path_cells = new std::vector<Coordinate>;
				///
				xCords.clear();
				yCords.clear();
				speeds.clear();
				//std::cout << key << std::endl;
			}
			else if (line.find("cost") != std::string::npos) {
				//std::cout << "found cost: ";
				size_t pos = line.find("=");
				line.erase(0, pos + 1);
				cost = std::stod(line);
				candidate.cost = cost;
				//std::cout << cost << std::endl;
			}
			else if (line.find("speeds=[") != std::string::npos) {
				//std::cout << "found x: ";
				size_t pos = line.find("=[");
				line.erase(0, pos + 2);
				pos = line.find("]");
				line.erase(pos, std::string::npos);

				std::string delimiter = ",";
				pos = 0;
				std::string s;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					s = line.substr(0, pos);
					speeds.push_back(std::stoi(s));
					line.erase(0, pos + delimiter.length());
				}
				std::string end_delimiter = "]";
				pos = line.find(end_delimiter);
				s = line.substr(0, pos);
				speeds.push_back(std::stoi(s));
			}
			else if (line.find("x=[") != std::string::npos) {
				//std::cout << "found x: ";
				size_t pos = line.find("=[");
				line.erase(0, pos + 2);
				pos = line.find("]");
				line.erase(pos, std::string::npos);

				std::string delimiter = ",";
				pos = 0;
				std::string x;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					x = line.substr(0, pos);
					xCords.push_back(std::stod(x));
					line.erase(0, pos + delimiter.length());
				}
				std::string end_delimiter = "]";
				pos = line.find(end_delimiter);
				x = line.substr(0, pos);
				xCords.push_back(std::stod(x));
				
				//for (std::vector<double>::iterator it = xCords.begin(); it < xCords.end(); ++it) {
				//	std::cout << *it << ",";
				//}
				//std::cout << std::endl;
			}
			else if (line.find("y=[") != std::string::npos) {
				//std::cout << "found y: ";
				size_t pos = line.find("=[");
				line.erase(0, pos + 2);
				pos = line.find("]");
				line.erase(pos, std::string::npos);

				std::string delimiter = ",";
				pos = 0;
				std::string y;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					y = line.substr(0, pos);
					yCords.push_back(std::stod(y));
					line.erase(0, pos + delimiter.length());
				}
				std::string end_delimiter = "]";
				pos = line.find(end_delimiter);
				y = line.substr(0, pos);
				yCords.push_back(std::stod(y));

				//for (std::vector<double>::iterator it = yCords.begin(); it < yCords.end(); ++it) {
				//	std::cout << *it << ",";
				//}
				//std::cout << std::endl;

				if (xCords.size() != yCords.size()) {
					std::cout << "parsing error. x and y coords differ in size";
					return;
				}
				else {
					/*std::cout << "++++++++++++++++++++++++++++++" << std::endl;
					std::cout << "Candidate key: " << candidate.key << std::endl;*/
					/*if (!candidate.path_cells->empty()) {
						//std::cout << "Not Empty" << std::endl;
						candidate.path_cells->clear();
					}*/
					/*if (!candidate.path_cells->empty()) {
						std::cout << "Still not empty" << std::endl;
					}*/
					/*if (candidate.key == 17408) {
						std::cout << "reduced path with buffer (row,col) : ";
					}*/
					for (int i = 0; i < xCords.size(); ++i) {
						Coordinate coordinate;
						coordinate.row = yCords[i];
						coordinate.col = xCords[i];
						coordinate.speed = speeds[i];
						candidate.reduced_path->push_back(coordinate);
						/// edit
						double buffer_size = map->getObstacleClearance();
						for (int theta = 0; theta < 8; ++theta) {
							double angle = theta * 2 * M_PI / ORIENTATIONS;
							double x_circle = buffer_size * cos(angle) + coordinate.col;
							double y_circle = buffer_size * sin(angle) + coordinate.row;
							int c_row = (int)round(y_circle);
							int c_col = (int)round(x_circle);
							Coordinate tempCoor;
							tempCoor.row = c_row;
							tempCoor.col = c_col;
							tempCoor.speed = 0;
							
							bool not_in_path_cell = true;
							for (std::vector<Coordinate>::iterator it = candidate.path_cells->begin(); it != candidate.path_cells->end(); ++it){
								if ((*it).row == c_row && (*it).col == c_col) {
									not_in_path_cell = false;
									break;
								}
							}
							if (not_in_path_cell) {
								candidate.path_cells->push_back(tempCoor);
							}
							
							/*std::vector<Coordinate>::iterator it = std::find(candidate.path_cells->begin(),candidate.path_cells->end(),tempCoor);
                            if (it == candidate.path_cells->end()) {
                                candidate.path_cells->push_back(tempCoor);
                            }*/
							/*if (cellCoor.row != tempCoor.row || cellCoor.col != tempCoor.col) {
								cellCoor.row = tempCoor.row;
								cellCoor.col = tempCoor.col;
								candidate.path_cells->push_back(cellCoor);
							}*/
						}
						/// edit
					}
					//if (candidate.key == 17408) {
						/*std::cout << std::endl;
						std::cout << "candidate key: " << candidate.key << std::endl;*/
						/*std::cout << "Round reduced path with buffer (row,col) : ";
						for (std::vector<intim>::iterator intit = vec.begin(); intit != vec.end(); ++intit) {
							std::cout << "(" << (*intit).row << "," << (*intit).col << ") ";
						}
						std::cout << std::endl;*/
						/*std::cout << "Path cells size: "<< candidate.path_cells->size() << std::endl;
						std::cout << "Cell paht (row,col): ";
						for (std::vector<Coordinate>::iterator it = candidate.path_cells->begin(); it != candidate.path_cells->end(); ++it){
							std::cout << "(" << (*it).row << "," << (*it).col << ") ";
						}
						std::cout << std::endl;*/
					//}
					/*std::cout << "Candidate key: " << candidate.key << std::endl;
					if (!candidate.path_cells->empty()) {
						std::cout << "Not Empty" << std::endl;
						candidate.path_cells->clear();
					}
					if (!candidate.path_cells->empty()) {
						std::cout << "Still not empty" << std::endl;
					}*/
					/*std::cout << "Size : " << candidate.path_cells->size() << std::endl;
					for (std::vector<Coordinate>::iterator it = candidate.path_cells->begin(); it != candidate.path_cells->end(); ++it){
						std::cout << "(" << (*it).row << "," << (*it).col << "), ";
					}
					std::cout << std::endl;*/
				}
				/*long temp_key = candidate.key;
				std::map <long, Candidate>::iterator it = m_ocps_table.find(temp_key);
				if (it != m_ocps_table.end()){
					std::cout << "&&&&&&&&&&&&&&&" << std::endl;
					std::cout << "&&&&&&&&&&&&&&&" << std::endl;
					std::cout << "Same key in ocps table" << std::endl;
					std::cout << "Key = " << candidate.key << std::endl;
					std::cout << "&&&&&&&&&&&&&&&" << std::endl;
					std::cout << "&&&&&&&&&&&&&&&" << std::endl;
				}*/
				m_ocps_table[candidate.key] = candidate;
			}
		}
		myfile.close();
		std::cout << "ocps file is closed" << std::endl;
		std::cout << "ocps table size: " << m_ocps_table.size() << std::endl;
		std::cout << "====================================" << std::endl; 
	}

	return;
}

Coordinate Ocps::rotate(int orientation, Coordinate coordinate) {
	double theta = orientation * 2 * M_PI / ORIENTATIONS;
	Coordinate rotated;
	rotated.col = coordinate.col * cos(theta) - coordinate.row * sin(theta);
	rotated.row = coordinate.col * sin(theta) + coordinate.row * cos(theta);
	return rotated;
}

int Ocps::coordinate_to_id(Coordinate coordinate) {
	/*
	Neighbors:
	0 1 2
	3 * 4
	5 6 7
	*/
	
	int row = (int)coordinate.row;
	int col = (int)coordinate.col;

	if (row == 1 && col == -1) return 0;
	else if (row == 1 && col == 0) return 1;
	else if (row == 1 && col == 1) return 2;
	else if (row == 0 && col == -1) return 3;
	else if (row == 0 && col == 1) return 4;
	else if (row == -1 && col == -1) return 5;
	else if (row == -1 && col == 0) return 6;
	else if (row == -1 && col == 1) return 7;
}

int Ocps::rotate_neighbours(int n_id) {
	if (n_id == 0) return 3;
	else if (n_id == 1) return 0;
	else if (n_id == 2) return 1;
	else if (n_id == 3) return 5;
	else if (n_id == 4) return 2;
	else if (n_id == 5) return 6;
	else if (n_id == 6) return 7;
	else if (n_id == 7) return 4;
}

long Ocps::ocps_key(VS_State curr, VS_State target, int path_id) {
	/*
	curr state has 16 options : 2 speeds * 8 orientations
	target state has 128 options : 2 speeds * 8 orientations * 8 neighbours
	path_id has 34 options
	*/
	return (curr.speed + SPEEDS * (curr.orientation + ORIENTATIONS * (target.speed + SPEEDS * (target.orientation + ORIENTATIONS * (target.neighbour_id + NEIGHBOURS * path_id)))));
}

bool Ocps::pathValidityCheck(Map* map, int start_row, int start_col, Candidate& candidate) {
	/*for (std::vector<Coordinate>::iterator it = candidate.reduced_path->begin(); it != candidate.reduced_path->end(); ++it) {
		int row = start_row + (int)round((*it).row);
		int col = start_col + (int)round((*it).col);
		if (!map->isValid(row, col) || !map->collisionFree(row, col)) {
			return false;
		}
		
		double buffer_size = map->getObstacleClearance();
		for (int theta = 0; theta < 8; ++theta) {
			double angle = theta * 2 * M_PI / 8;
			double x_circle = buffer_size * cos(angle) + (*it).col;
			double y_circle = buffer_size * sin(angle) + (*it).row;
			int c_row = start_row + (int)round(y_circle);
			int c_col = start_col + (int)round(x_circle);
			if (!map->isValid(c_row, c_col) || !map->collisionFree(c_row, c_col)) {
				return false;
			}
		}
	}*/
	/*std::cout << "++++++++++++++++++++++++++++" << std::endl;
	std::cout << "Start cell : (" << start_row << "," << start_col << ")" << std::endl;*/
	/*if (candidate.key == 17408) {
		std::cout << "reduced path (row,col) : ";
		for (std::vector<Coordinate>::iterator reit = candidate.reduced_path->begin(); reit != candidate.reduced_path->end(); ++reit) {
			std::cout << "(" << (*reit).row << "," << (*reit).col << ") ";
		}
		std::cout << std::endl;
	}*/
	/*std::cout << "Path cells size: " << candidate.path_cells->size() << std::endl;
	std::cout << "Path cells of candidate " << candidate.key << " (row,col) : ";
	for (std::vector<Coordinate>::iterator it = candidate.path_cells->begin(); it != candidate.path_cells->end(); ++it) {
		int row = (int)(*it).row;
		int col = (int)(*it).col;
		std::cout << "(" << row << "," << col << ") ";
	}
	std::cout << std::endl;*/
	for (std::vector<Coordinate>::iterator pit = candidate.path_cells->begin(); pit != candidate.path_cells->end(); ++pit) {
		int row = start_row + (int)(*pit).row;
		int col = start_col + (int)(*pit).col; 
		if (!map->isValid(row, col) || !map->collisionFree(row, col)) {
			//std::cout << "Cell: (" << row << "," << col << ") occupied by obstacle" << std::endl;
			return false;
		}
	}
	return true;
}

void Ocps::VarSpeedGetBestCandidate(Map* map, int s1_row, int s1_col, int s1_orientation, int s1_speed, int s2_rel_row, int s2_rel_col, int s2_orientation, int s2_speed, Candidate& best_varspeed) {
	best_varspeed.cost = -1;

	VS_State curr;
	curr.orientation = s1_orientation;
	curr.speed = s1_speed;
	
	VS_State target;
	target.orientation = s2_orientation;
	target.speed = s2_speed;
	Coordinate neighbour;
	neighbour.row = (double)s2_rel_row;
	neighbour.col = (double)s2_rel_col;
	int neighbour_id = coordinate_to_id(neighbour);
	target.neighbour_id = neighbour_id;

	
	int best_pathId=-1;
	long best_table_key = -1;
	for (int pathId = 0; pathId < 52; ++pathId) {
		long table_key = ocps_key(curr, target, pathId);
		//if (target.neighbour_id == 4 && target.orientation == 1) {
		//	std::cout << "target id,speed,orientation " << target.neighbour_id << "," << target.speed << "," << target.orientation << std::endl;
		//	std::cout << "table key: " << table_key << std::endl;
		//}
		std::map <long, Candidate>::iterator it = m_ocps_table.find(table_key);
		if (it != m_ocps_table.end()) {
			
			Candidate temp = it->second;
			
			///
			/*if (best_varspeed.cost != -1 && temp.cost > best_varspeed.cost) {
				continue;
			}*/
			///
			
			if (pathValidityCheck(map, s1_row, s1_col, temp)) {
				//if (table_key == 95280) {
				//	std::cout << "found 95280 in table and valid! " << std::endl;
				//	std::cout << "cost " << temp.cost << std::endl;
				//	std::cout << "path len " << temp.reduced_path->size() << std::endl;
				//	for (std::vector<Coordinate>::iterator it = temp.reduced_path->begin(); it != temp.reduced_path->end(); ++it) {
				//		std::cout << "[" << (*it).col << "," << (*it).row << "]" << std::endl;
				//	}
				//}
				if (best_varspeed.cost == -1 || temp.cost < best_varspeed.cost) {
					copy_candidate(temp, best_varspeed);
					best_pathId = pathId;
					best_table_key = table_key;
				}
			}
		}
		
	}
	//std::cout << "bestCandidate  cost,type,orientation "<<bestCandidate.cost << "," << bestCandidate.path_type << "," << bestCandidate.path_orientation << std::endl;
	//std::cout << "pathId,table_key " << best_pathId << "," << best_table_key << std::endl;
	return;
}

int Ocps::getVarSpeedSegmentNPoints(long ocps_key) {
	std::map <long, Candidate>::iterator it = m_ocps_table.find(ocps_key);
	if (it != m_ocps_table.end()) {
		Candidate temp = it->second;
		return temp.reduced_path->size();
	}
	return 0;
}

void Ocps::copy_candidate(Candidate& source, Candidate& target) {
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

void Ocps::getVarSpeedSegmentPoints(Coordinate coordinate, int orientation, long ocps_key, int n_points, double* points_out) {

	std::map <long, Candidate>::iterator it = m_ocps_table.find(ocps_key);
	if (it != m_ocps_table.end()) {
		Candidate temp = it->second;
		int i = 0;
		for (std::vector<Coordinate>::iterator it = temp.reduced_path->begin(); it != temp.reduced_path->end(); ++it) {
			points_out[i * 4] = coordinate.row + (*it).row;
			points_out[i * 4 + 1] = coordinate.col + (*it).col;
			points_out[i * 4 + 2] = 0; // TODO return orientation
			points_out[i * 4 + 3] = (*it).speed; // TODO return speed
			i++;
		}
	}
	return;
}


/*void Ocps::build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation) {
    double r = v_min / u_max;
    double R = v_max / v_max;
    std::map<long, Candidate> ocps_table;
	
	int a = 0;
	std::cout << "id before rotate = " << a << std::endl;
	a = rotate_neighbours(a);
	std::cout << "id after rotate = " << a << std::endl;

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

    std::vector<VS_State> *symmetric_targets = new std::vector<VS_State>;
    std::vector<int> *subset = new std::vector<int>;
    std::vector<double> *robustX = new std::vector<double>;
    std::vector<double> *robustY = new std::vector<double>;

    int count = 0;
    VS_State curr_state;
    curr_state.orientation = startOrientation;
    Coordinate locCoor;
    locCoor.row = yFinalIn;
    locCoor.col = xFinalIn;
    VS_State target_it;
    target_it.neighbour_id = coordinate_to_id(locCoor);
    target_it.orientation = orientFinalIn;
    Coordinate target_coordinate = id_to_coordinate(target_it.neighbour_id, startOrientation);
    VarSpeedDubins::ProblemStatement vsdprob;

    double xFinal = target_coordinate.col;
    double yFinal = target_coordinate.row;
    double hFinal = 0;

    if (startOrientation == 0) {
        hFinal = orientFinalIn * 2 * M_PI / ORIENTATIONS;
    } else if (startOrientation == 1) {
        if (orientFinalIn == 0) {
            hFinal = 7 * 2 * M_PI / ORIENTATIONS;
        } else {
            hFinal = (orientFinalIn - 1) * 2 * M_PI / ORIENTATIONS;
        }
    }
//			double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;

    vsdprob.set_xFinal(xFinal);
    vsdprob.set_yFinal(yFinal);
    vsdprob.set_hFinal(hFinal);
    vsdprob.set_R(R);
    vsdprob.set_r(r);

//	int orientation = 0; // throw out after fix
    for (int curr_speed = 0; curr_speed < SPEEDS; ++curr_speed) {

//        VS_State curr_state;
        curr_state.speed = curr_speed;
//        curr_state.orientation = startOrientation;
//        Coordinate locCoor;
//        locCoor.row = yFinal;
//        locCoor.col = xFinal;
//        VS_State target_it;
//        target_it.neighbour_id = coordinate_to_id(locCoor);
//        target_it.orientation = orientFinal;
        for (int target_speed = 0; target_speed < SPEEDS; ++target_speed) {
//            VS_State target_it;
            target_it.speed = target_speed;
//            Coordinate locCoor;
//            locCoor.row = yFinal;
//            locCoor.col = xFinal;
//            target_it.neighbour_id = coordinate_to_id(locCoor);
//            target_it.orientation = orientFinal;

            //symmetric_targets->clear();
            //get_basic_symmetric_targets(symmetric_targets, orientation);
            //for (std::vector<VS_State>::iterator target_it = symmetric_targets->begin(); target_it != symmetric_targets->end(); ++target_it) {
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
////			double hFinal = orientFinal * 2 * M_PI / ORIENTATIONS;
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
                    }
                }

                if (solveStatus) {
                    // insert candidate to ocps table
                    candidate.key = ocps_key(curr_state, target_it, candidateID);
                    ocps_table[candidate.key] = candidate;

//					// rotate to all other directions and insert to ocps
//					Candidate candidate_rot_arr[3];
//					copy_candidate(candidate, candidate_rot_arr[0]);
//					VS_State curr_state_rot = curr_state;
//					VS_State target_rot = target_it;
//					for (int rot = 1; rot < 4; ++rot) {
//						rotate_target(target_rot);
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
//
//					// mirror the solution
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
//					VS_State target_mirror = target_it;
//					mirror_state(target_it, target_mirror, curr_state.orientation);
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
//					// rotate the mirror to all other directions and insert to ocps
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
//					}
//
                }
                count++;
                std::cout << count << "/2312 Done" << std::endl;
            }
            //}
            //}
        }
//        save_to_file(ocps_table, v_min, v_max, u_max);
    }
    //save_to_file(ocps_table, v_min, v_max, u_max);
}*/

/*void build_ocps_tab(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation){
	void build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, std::map<long, Candidate> &m_ocps_table);
}*/

void Ocps::build_ocps(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, double buffer) {
    build_ocps_table(v_min, v_max, u_max, xFinalIn, yFinalIn, orientFinalIn, startOrientation, windIn, buffer, m_ocps_table);
}