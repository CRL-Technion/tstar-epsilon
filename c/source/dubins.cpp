#include "dubins.h"

//bool operator==(const dubinsCoordinate& c1, const dubinsCoordinate& c2) {
//    return (c1.col == c2.col && c1.row == c2.row);
//}

/// Do not leave function in .h file without implementation in .cpp file, include constructor an destructor

Dubins::Dubins() {

}

Dubins::~Dubins() {

}

void Dubins::dubinsLSL(double alpha, double beta, double d, double r, double* out)
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

void Dubins::dubinsLSR(double alpha, double beta, double d, double r, double* out)
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

void Dubins::dubinsRSL(double alpha, double beta, double d, double r, double* out)
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

void Dubins::dubinsRSR(double alpha, double beta, double d, double r, double* out)
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

void Dubins::dubinsRLR(double alpha, double beta, double d, double r, double* out)
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

void Dubins::dubinsLRL(double alpha, double beta, double d, double r, double* out)
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

double Dubins::dubinsLength(double* path){
    return path[0] + path[1] + path[2];

}

void Dubins::vanilaDubinsPath(Robot* robot, State s, State g, Wind mapWind, double* out)
{
    double r = robot->get_r();
    double modWind = sqrt(mapWind.x * mapWind.x + mapWind.y * mapWind.y);
    r = r * (robot->getvMin() - modWind) / robot->getvMin();
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
    absThStart = (double)s.theta * (2 * M_PI) / robot->getNumOfOrientation();
    absThGoal = (double)g.theta * (2 * M_PI) / robot->getNumOfOrientation();

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

void Dubins::dubinsSegment(double param, double segInit[3], int seg_type, double r, double* out){
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

void Dubins::dubinsPathSample(Robot* robot, State s, double* path, double t, double* out)
{
    double r = robot->get_r();
    if (robot->getSpeed() == 1) {
        r = robot->get_R();
    }
    double tPrime = t;
    double absThStart = (double)s.theta * (2 * M_PI) / robot->getNumOfOrientation();
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

bool Dubins::pathValidityCheck(Robot* robot, Map* map, State s, double* path, double cost)
{
    double x = 0;
    double length = cost;
    int temp = (int)floor(length / robot->getStepSize());
    int pathRows = (int)temp + 1;
    double** pathMid = new double* [pathRows]; //descretizied path in row,col,theta
    for (int i = 0; i < pathRows; ++i) {
        pathMid[i] = new double[3];
    }

    // build all the mid points of the path
    int i = 0;

    while (x <= length) {
        double tempPath[3] = { -1, -1, -1 };
        dubinsPathSample(robot, s, path, x, tempPath);
        for (int j = 0; j < 3; j++) {
            pathMid[i][j] = tempPath[j];
        }
        /*for (int k = 0; k < 3; k++){
            cout<<"["<<tempPath[0]<<", "<<tempPath[1]<<", "<<tempPath[2]<<"]"<<endl;
        }*/
        x += robot->getStepSize();
        i++;
    }

    for (int k = 0; k < i; ++k) {
        int row = (int)round(pathMid[k][0]);
        int col = (int)round(pathMid[k][1]);
        if (!map->isValid(row,col) || !map->collisionFree(row,col)){
            return false;
        }
        double buffer_size = map->getObstacleClearance();
        for (int theta = 0; theta < 8; ++theta) {
            double angle = theta * 2 * M_PI / 8;
            double x_circle = buffer_size * cos(angle) + pathMid[k][1];
            double y_circle = buffer_size * sin(angle) + pathMid[k][0];
            int c_row = (int)round(y_circle);
            int c_col = (int)round(x_circle);
            if (!map->isValid(c_row, c_col) || !map->collisionFree(c_row, c_col)) {
                return false;
            }
        }
    }

    // pathCells[0] = i - 1; // the actual length of the array
    for (i = 0; i < pathRows; ++i) {
        delete[] pathMid[i];
    }
    delete[] pathMid;
    return true;
}

int Dubins::getDubinsSegmentNPoints(Robot* robot, double* path) {
    /* returns the number of points in a dubins path*/
    double length = path[0] + path[1] + path[2]; //TODO SHOULD divide by speed?

    int temp = (int)floor(length / robot->getStepSize());
    return (int)temp + 1;
}

void Dubins::getDubinsSegmentPoints(Robot* robot, State start, double* path, int n_points, double* points_out) {
    double x = 0;

    for (int i=0; i< n_points; ++i){
        double midPoint[3] = { -1, -1, -1 };
        dubinsPathSample(robot, start, path, x, midPoint);

        points_out[i * 4] = midPoint[0];
        points_out[i * 4 + 1] = midPoint[1];
        points_out[i * 4 + 2] = midPoint[2];
        points_out[i * 4 + 3] = start.v;

        x += robot->getStepSize();
    }

}

void Dubins::obstacleDubinsPath(Robot* robot, Map* map, State s, State g, double* out)
{
    double r = robot->get_r();
    if (robot->getSpeed() == 1) {
        r = robot->get_R();
    }
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
        cout<<"r is smaller than 0"<<endl;
        return;
    }

    double absThStart, absThGoal;
    absThStart = (double)s.theta * (2 * M_PI) / robot->getNumOfOrientation();
    absThGoal = (double)g.theta * (2 * M_PI) / robot->getNumOfOrientation();

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
            double cost = wayLength[i][0] + wayLength[i][1] + wayLength[i][2]; //TODO SHOULD divide by speed?
            if ((cost < bestCost) || (bestCost == -1)) {
                double tempPath[4] = { wayLength[i][0], wayLength[i][1], wayLength[i][2], (double)i };
                if (pathValidityCheck(robot, map, s, tempPath, cost)) {
                    bestWay = i;
                    bestCost = cost;
                }
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
    return;

}

long Dubins::dubinsKey(transition trans, int pathId) {
    return (trans.dx * 3 + 5 * (trans.dy + 7 * (trans.endTheta + 11 * ((pathId + 13) + trans.startTheta * 17))));
}

void Dubins::findCellsOfPath(Robot* robot, Map* map, State s, double* path, double cost, std::vector<dubinsCoordinate> &cellsVec) {
    /// find the all cells which the path going throw
    // s is the start state
    double x = 0;
    double length = cost;
    int temp = (int)floor(length / robot->getStepSize());
    int pathRows = (int)temp + 1;
    //std::cout << "3" << std::endl;
    double** pathMid = new double* [pathRows]; //descretizied path in row,col,theta
    for (int i = 0; i < pathRows; ++i) {
        pathMid[i] = new double[3];
    }

    dubinsCoordinate start_cell;
    start_cell.col = s.col;
    start_cell.row = s.row;
    cellsVec.push_back(start_cell);
    dubinsCoordinate tempEnd = cellsVec.back();

    // build all the mid points of the path
    int i = 0;

    while (x <= length) {
        double tempPath[3] = { -1, -1, -1 };
        dubinsPathSample(robot, s, path, x, tempPath);
        for (int j = 0; j < 3; j++) {
            pathMid[i][j] = tempPath[j];
        }
        /*for (int k = 0; k < 3; k++){
            cout<<"["<<tempPath[0]<<", "<<tempPath[1]<<", "<<tempPath[2]<<"]"<<endl;
        }*/
        x += robot->getStepSize();
        i++;
    }

    int orientations = robot->getNumOfOrientation();

    for (int k = 0; k < i; ++k) {
        //std::cout << "3.1" << std::endl;
        double buffer_size = map->getObstacleClearance();
        for (int theta = 0; theta < 8; ++theta) {
            double angle = theta * 2 * M_PI / orientations;
            double x_circle = buffer_size * cos(angle) + pathMid[k][1];
            double y_circle = buffer_size * sin(angle) + pathMid[k][0];
            int c_row = (int)round(y_circle);
            int c_col = (int)round(x_circle);
            dubinsCoordinate tempCoor = {c_row,c_col};
            /*std::vector<dubinsCoordinate>::iterator it = std::find(cellsVec.begin(),cellsVec.end(),tempCoor);
            if (it == cellsVec.end()) {
                cellsVec.push_back(tempCoor);
            }*/
            if (c_row != tempEnd.row || c_col != tempEnd.col) {
                dubinsCoordinate newCoordinate;
                newCoordinate.row = c_row;
                newCoordinate.col = c_col;
                cellsVec.push_back(newCoordinate);
                tempEnd = newCoordinate;
            }
        }
    }

    // pathCells[0] = i - 1; // the actual length of the array
    for (i = 0; i < pathRows; ++i) {
        delete[] pathMid[i];
    }
    delete[] pathMid;
    return;
}

bool Dubins::pathValidityByCells(Map* map,State s, dubinsCandidate cand) {
    std::vector<dubinsCoordinate> vec = cand.reduced_path;
    for (std::vector<dubinsCoordinate>::iterator it = vec.begin(); it != vec.end(); ++it) {
        int row = s.row + (*it).row;
        int col = s.col + (*it).col;
        if (!map->isValid(row, col) || !map->collisionFree(row, col)) {
            return false;
        }
    }
    return true;
}

void Dubins::copyDubinsCandidate(dubinsCandidate &source, dubinsCandidate &target) {
    target.valid = source.valid;
    target.key = source.key;
    target.cost = source.cost;
    target.trans = source.trans;
    target.path[0] = source.path[0];
    target.path[1] = source.path[1];
    target.path[2] = source.path[2];
    target.path[3] = source.path[3];
    target.reduced_path = source.reduced_path;;
}

double Dubins::getGroundSpeed(double dir_x, double dir_y, double wind_x, double wind_y, double speed){
    // normalize dir vector
    double tmp_len = sqrt(dir_x * dir_x + dir_y * dir_y);
    dir_x /= tmp_len;
    dir_y /= tmp_len;

    double b = wind_x * dir_x + wind_y * dir_y;
    double c = wind_x * wind_x + wind_y * wind_y - speed * speed;
    return (b + sqrt(b*b - c));
}

