#define TSTAR_API_EXPORTS

#include "tstar.h"

extern "C" {

TSTAR_API void* tstar_create(int* map, int width, int height, State start, State goal, Problem problem) {
	TStarImp* handler = new TStarImp(map, width, height, start, goal, problem);
	return (void*)handler;
}

TSTAR_API void tstar_destroy(void* handler) {
	TStarImp* tstar = (TStarImp*)handler;
	delete tstar;
}

TSTAR_API int tstar_run(void* handler, double* solution, int* solution_length) {
	TStarImp* tstar = (TStarImp*)handler;
	return tstar->run(solution, solution_length);
}

TSTAR_API int tstar_get_segment_n_points(void* handler, double* path) {
	TStarImp* tstar = (TStarImp*)handler;
	return tstar->getSegmentNPoints(path);
}

TSTAR_API void tstar_get_segment_points(void* handler, State start, double* path, int n_points, double* points_out){
	TStarImp* tstar = (TStarImp*)handler;
	tstar->getSegmentPoints(start, path, n_points, points_out);
}

} // extern "C"