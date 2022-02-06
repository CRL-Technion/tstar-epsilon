#ifndef TSTAR_H
#define TSTAR_H

#include "tstar_imp.h"

#ifdef __cplusplus
extern "C" 
{  
#endif

#ifdef _WIN32
#  ifdef TSTAR_API_EXPORTS
#    define TSTAR_API __declspec(dllexport)
#  else
#    define TSTAR_API __declspec(dllimport)
#  endif
#else
#  define TSTAR_API
#endif

TSTAR_API void* tstar_create(int* map, int width, int height, State start, State goal, Problem problem);
TSTAR_API void tstar_destroy(void* handler);
TSTAR_API int tstar_run(void* handler, double* solution, int* solution_length);

TSTAR_API int tstar_get_segment_n_points(void* handler, double* path);
TSTAR_API void tstar_get_segment_points(void* handler, State start, double* path, int n_points, double* points_out);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TSTAR_H