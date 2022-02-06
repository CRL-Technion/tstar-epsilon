// testSolveSinglePath.cpp
// This code generates a VSDpath and attempts passes the endpoint to the solver.
// Last Modified: 02-Feb-2015, Artur Wolek

// c++ standard
#include<string>

// external
#include<armadillo>

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

// custom
#include<VSDUtils.h>
#include<VSDPath.h>
#include<VSDProblem.h>
#include<VSDSolver.h>
#include<VSDNLP.h>
#include<MathTools.h>
#include<MathToolsArma.h>


int main() {
    int numSamplesMax = 1000;
    //std::string samplingAlgorithm = "rejection-sample";
    std::string samplingAlgorithm = "barycentric";    
    double best_cost = 1000;

    VarSpeedDubins::Path bestcand;
    std::string fileName = "vsdpath.m";
    std::string xVarName, yVarName;

    std::cout << " solve the problem " << std::endl;
    // create a problem statement
    VarSpeedDubins::ProblemStatement vsdprob;
    vsdprob.set_xFinal(1.5);
    vsdprob.set_yFinal(-3.2);
    vsdprob.set_hFinal(4.0);

    vsdprob.set_R(1);// vsdpath.get_R());
    vsdprob.set_r(0.5); // vsdpath.get_r());

    vsdprob.set_wind(0.3, 0.2);

    VarSpeedDubins::Solver vsdsolver;
    vsdsolver.set_numSamplesMax(numSamplesMax);
    vsdsolver.set_samplingAlgorithm(samplingAlgorithm);
    vsdsolver.set_problemStatement(vsdprob);
    vsdsolver.solveAll();
    arma::vec solveStatusAll = vsdsolver.get_solveStatusAll();
    // solveStatusAll.print("solveStatusAll");
    vsdsolver.printAll();

    //vsdsolver.plotAll(fileName);

    for (int i = 0; i < vsdsolver.m_numPaths; i++){
        if (solveStatusAll(i) != 0){
            VarSpeedDubins::Path curPath = vsdsolver.m_candPaths.at(i); 
            double costOpt = curPath.get_cost();
            //std::cout << "Cost" << costOpt << std::endl;
            //curPath.print();
            if (costOpt < best_cost) {
                best_cost = costOpt;
                bestcand = curPath;
            }
        }
    }

    RobustDubins::Path bestDubins;
    double dubins_cost = 1e30;
    // show robust dubins cases
    for (int i = 0; i < 6; i++){ // vsdsolver.m_candPathsDubins.size()
        // print if a feasible solution was found
        if (solveStatusAll(vsdsolver.m_numPaths + i) == 1){
            std::string pathClass;
            if (i < 6){
                pathClass = "Dubins-R";    
            }
            else {
                pathClass = "Dubins-r";
            }
            RobustDubins::Path curPath = vsdsolver.m_candPathsDubins.at(i); 
            std::cout << i << ")  " << pathClass 
                        << "," << curPath.get_pathType() << "," 
                        << ",\t" << curPath.get_cost() << " \t," 
                        << curPath.get_aParamUnsigned() << ","  
                        << curPath.get_bParamUnsigned() << ","  
                        << curPath.get_cParamUnsigned() << std::endl;
            //curPath.print(); 
            if(curPath.get_cost() < dubins_cost){
                dubins_cost = curPath.get_cost();
                bestDubins = curPath;
            }
        }
    }

    std::cout << "best dubins : " << dubins_cost << std::endl;
    xVarName = "xxx";
    yVarName = "yyy";
    bestDubins.writePathOctavePlotCommands(fileName, 1, xVarName, yVarName, "g");
    // bestDubins.computePathHistory();

    bestcand.computePathHistory();
    //bestcand.computePathHistoryWithWind();

    bestcand.computeEndpoint();
    bestcand.print();
    std::cout << "best_cost : " << best_cost << std::endl;
    std::cout << "wind : " 
        << bestcand.get_wind_x() << ", " 
        << bestcand.get_wind_y() << std::endl;
    bestcand.writePathPlotCommands(fileName, 1, "x", "y", "b-");
    xVarName = "x";
    yVarName = "y";
    bestcand.writeSwitchingPtPlotCommands(fileName, 1, xVarName, yVarName, "ro");
    MathTools::runOctaveScript(fileName, "--persist");
    
    return 0;
}



