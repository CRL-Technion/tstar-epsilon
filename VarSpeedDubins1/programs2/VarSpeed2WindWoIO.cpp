
#include <map>
#include <vector>
#include <iostream>
#include <fstream>

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

// custom
#include <VSDUtils.h>
#include<VSDPath.h>
#include<VSDProblem.h>
#include<VSDSolver.h>
#include<VSDNLP.h>
#include "VarSpeed.h"
#include "VarSpeed_utils.h"

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
//    std::vector<Coordinate>* path_cells = new std::vector<Coordinate>;
//} Candidate;

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

void build_path_cells(Candidate &cand, double buffer, Coordinate coord) {
    for (int theta = 0; theta < ORIENTATIONS; ++theta) {
        double angle = theta * 2 * M_PI / ORIENTATIONS;
        double x_circle = buffer * cos(angle) + coord.col;
        double y_circle = buffer * sin(angle) + coord.row;
        int c_row = (int)round(y_circle);
        int c_col = (int)round(x_circle);
        Coordinate tempCoor;
        tempCoor.row = c_row;
        tempCoor.col = c_col;
        tempCoor.speed = 0;

        bool not_in_path_cell = true;
        for (std::vector<Coordinate>::iterator pait = cand.path_cells->begin(); pait != cand.path_cells->end(); ++pait){
            if ((*pait).row == c_row && (*pait).col == c_col) {
                not_in_path_cell = false;
                break;
            }
        }
        if (not_in_path_cell) {
            cand.path_cells->push_back(tempCoor);
        }
    }
}

void build_ocps_table(double v_min, double v_max, double u_max, double xFinalIn, double yFinalIn, int orientFinalIn, int startOrientation, Wind windIn, double buffer, std::map<long, Candidate>& ocps_table) {
    double r = v_min / u_max;
    double R = v_max / u_max;
//    std::map<long, Candidate> ocps_table;

    bool no_wind = false;
    if (windIn.x == 0 && windIn.y == 0) {
        no_wind = true;
    }

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

                    for (std::vector<Coordinate>::iterator it = candidate.reduced_path->begin(); it != candidate.reduced_path->end(); ++it) {
//                        Coordinate coordinate;
//                        coordinate.row = (*it).row;
//                        coordinate.col = (*it).col;
//                        coordinate.speed = (*it).speed;
//                        candidate.reduced_path->push_back(coordinate);
                        build_path_cells(candidate, buffer, *it);
                        // TODO Send to function by reference. Do so also for rotation and symmetry if no wind exist.
//                        for (int theta = 0; theta < ORIENTATIONS; ++theta) {
//                            double angle = theta * 2 * M_PI / ORIENTATIONS;
//                            double x_circle = buffer * cos(angle) + coordinate.col;
//                            double y_circle = buffer * sin(angle) + coordinate.row;
//                            int c_row = (int)round(y_circle);
//                            int c_col = (int)round(x_circle);
//                            Coordinate tempCoor;
//                            tempCoor.row = c_row;
//                            tempCoor.col = c_col;
//                            tempCoor.speed = 0;
//
//                            bool not_in_path_cell = true;
//                            for (std::vector<Coordinate>::iterator pait = candidate.path_cells->begin(); pait != candidate.path_cells->end(); ++pait){
//                                if ((*pait).row == c_row && (*pait).col == c_col) {
//                                    not_in_path_cell = false;
//                                    break;
//                                }
//                            }
//                            if (not_in_path_cell) {
//                                candidate.path_cells->push_back(tempCoor);
//                            }
//                        }
                    }

                    candidate.key = ocps_key(curr_state, target_it, candidateID);
                    ocps_table[candidate.key] = candidate;

                    if (no_wind) {
                        /// rotate to all other directions and insert to ocps
                        Candidate candidate_rot_arr[3];
                        copy_candidate(candidate, candidate_rot_arr[0]);
                        State curr_state_rot = curr_state;
                        State target_rot = target_it;
                        for (int rot = 1; rot < 4; ++rot) {
                            rotate_target(target_rot);
                            // Update the path for rotated direction
                            for (std::vector<Coordinate>::iterator it = candidate_rot_arr[rot -1].reduced_path->begin(); it != candidate_rot_arr[rot -1].reduced_path->end(); ++it) {
                                Coordinate rotated = rotate(2, *it);
                                (*it).row = rotated.row;
                                (*it).col = rotated.col;

                                /// edit
                                build_path_cells(candidate_rot_arr[rot-1], buffer, *it);
                                /// edit
                            }

                            curr_state_rot.orientation += 2;
                            Coordinate target_coordinate_rot = id_to_coordinate(target_rot.neighbour_id, 0);
                            candidate_rot_arr[rot - 1].target.row = target_coordinate_rot.row;
                            candidate_rot_arr[rot - 1].target.col = target_coordinate_rot.col;
                            candidate_rot_arr[rot - 1].h_final = target_rot.orientation;
                            candidate_rot_arr[rot - 1].h_initial = curr_state_rot.orientation;

                            candidate_rot_arr[rot - 1].key = ocps_key(curr_state_rot, target_rot, candidateID);
                            ocps_table[candidate_rot_arr[rot - 1].key] = candidate_rot_arr[rot - 1];
                            if (rot < 3){
                                copy_candidate(candidate_rot_arr[rot - 1], candidate_rot_arr[rot]);
                            }
                        }

                        /// mirror the solution
                        Candidate candidate_mirror;
                        copy_candidate(candidate, candidate_mirror);

                        int candidateID_mirror = mirror_candidateID(candidateID);
                        if (candidateID < 10) {
                            flip_path_orientation(candidate_mirror.path_orientation);
                        }
                        else {
                            std::vector<std::string> mirror_cand = candList[candidateID_mirror];
                            candidate_mirror.path_orientation = mirror_cand[2];
                        }
                        State target_mirror = target_it;
                        /// Edited function find in VarSpeed_utils.h
                        mirror_state(target_it, target_mirror, curr_state.orientation);
                        /// Edited function find in VarSpeed_utils.h

                        if (curr_state.orientation == 0) {
                            // mirror path over the x axis
                            for (std::vector<Coordinate>::iterator it = candidate_mirror.reduced_path->begin(); it != candidate_mirror.reduced_path->end(); ++it) {
                                (*it).row *= -1;
                                build_path_cells(candidate_mirror, buffer, *it);
                            }
                        }
                        else if (curr_state.orientation == 1) {
                            // mirror path over y=x line
                            for (std::vector<Coordinate>::iterator it = candidate_mirror.reduced_path->begin(); it != candidate_mirror.reduced_path->end(); ++it) {
                                double temp = (*it).row;
                                (*it).row = (*it).col;
                                (*it).col = temp;
                                build_path_cells(candidate_mirror, buffer, *it);
                            }
                        }

                        Coordinate target_coordinate_mirror = id_to_coordinate(target_mirror.neighbour_id, curr_state.orientation);
                        candidate_mirror.target.row = target_coordinate_mirror.row;
                        candidate_mirror.target.col = target_coordinate_mirror.col;
                        candidate_mirror.h_final = target_mirror.orientation;
                        candidate_mirror.h_initial = curr_state.orientation;
                        candidate_mirror.key = ocps_key(curr_state, target_mirror, candidateID_mirror);
                        ocps_table[candidate_mirror.key] = candidate_mirror;

                        /// rotate the mirror to all other directions and insert to ocps
                        Candidate candidate_mirror_rot_arr[3];
                        copy_candidate(candidate_mirror, candidate_mirror_rot_arr[0]);
                        curr_state_rot = curr_state;
                        target_rot = target_mirror;
                        for (int rot = 1; rot < 4; ++rot) {
                            rotate_target(target_rot);
                            for (std::vector<Coordinate>::iterator it = candidate_mirror_rot_arr[rot - 1].reduced_path->begin(); it != candidate_mirror_rot_arr[rot - 1].reduced_path->end(); ++it) {
                                Coordinate rotated = rotate(2, *it);
                                (*it).row = rotated.row;
                                (*it).col = rotated.col;
                                build_path_cells(candidate_mirror_rot_arr[rot - 1], buffer, *it);
                            }
                            curr_state_rot.orientation += 2;
                            Coordinate target_coordinate_rot = id_to_coordinate(target_rot.neighbour_id, 0);
                            candidate_mirror_rot_arr[rot - 1].target.row = target_coordinate_rot.row;
                            candidate_mirror_rot_arr[rot - 1].target.col = target_coordinate_rot.col;
                            candidate_mirror_rot_arr[rot - 1].h_final = target_rot.orientation;
                            candidate_mirror_rot_arr[rot - 1].h_initial = curr_state_rot.orientation;
                            candidate_mirror_rot_arr[rot - 1].key = ocps_key(curr_state_rot, target_rot, candidateID_mirror);
                            ocps_table[candidate_mirror_rot_arr[rot - 1].key] = candidate_mirror_rot_arr[rot - 1];
                            if (rot < 3) {
                                copy_candidate(candidate_mirror_rot_arr[rot - 1], candidate_mirror_rot_arr[rot]);
                            }
                        }
                    }
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
}
/*
int main() {
	double v_min = 0.5;
	double v_max = 1;
	double u_max = 1;
	double xFinal = 1;
	double yFinal = 1;
	int orientFinal = 0;
	int startOrient = 0;
	Wind wi;
	wi.x = 0.2;
	wi.y = 0;
	build_ocps_table(v_min, v_max, u_max, xFinal, yFinal, orientFinal, startOrient, wi);
	return 0;
}*/