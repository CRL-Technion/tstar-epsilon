#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include <vector>

using namespace std;

typedef struct State {
	int row;
	int col;
	int theta;
	int v;
} State;

/*typedef struct relativeTrajectory {
    double dx;
    double dy;
    int theta;
    bool diagonal;
} relativeTrajectory;*/

typedef struct relativeTrajectory {
    double dx = -1;
    double dy = -1;
    int finalTheta = -1;
    int initialTheta = -1;
} relativeTrajectory;

typedef struct Wind {
    double x;
    double y;
    // default constructor with zero wind
    Wind(double d=0, double d1=0) : x(0), y(0) {x = d; y = d1;};
} Wind;

typedef struct transition {
    double dx;
    double dy;
    int endTheta;
    int startTheta;
} transition;

class Map {
	public:
		Map(int* map, int width, int height, State start, State goal);
		~Map();

		int** get_map();
		int get_map_width();
		int get_map_height();

		State get_start();
		State get_goal();
		double get_eta();
		
		bool isValid(int row, int col);
		bool collisionFree(int row, int col);
		bool isGoal(State state);
		bool partialIsGoal(State state);
		
		// return false if the state shouldn't be inserted to open list
		bool obstacleBasedPruning(State state, int n_orients);
		//bool speedBasedPruning(State state, int n_orients);
		bool headingBasedPruning(State state, int n_orients);
		std::vector<State> get_neighbours(State state, int n_orients, int n_velocity, bool extended);

		void setObstacleClearance(double clearance);
		double getObstacleClearance();
		
		void setEpsilon (double epsilon);
		double getEpsilon();
		
		void setWind(Wind wind);
		Wind getWind();

        void setVelocities(int v);

	private:
		State m_start;
		State m_goal;
		int m_width;
		int m_height;
		int** m_map = 0;
		double m_eta;
		double m_clearance;
		double m_epsilon;
		
		Wind m_wind;
};

#endif // MAP_HANDLER_H