#ifndef VAR_SPEED_H
#define VAR_SPEED_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WIN32
#  ifdef VAR_SPEED_API_EXPORTS
#    define VAR_SPEED_API __declspec(dllexport)
#  else
#    define VAR_SPEED_API __declspec(dllimport)
#  endif
#else
#  define VAR_SPEED_API
#endif

	VAR_SPEED_API void build_ocps_table(double v_min, double v_max, double u_max);
	
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // VAR_SPEED_H