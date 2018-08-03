// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the DIMFEI_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// DIMFEI_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef _WIN32
#   ifdef DIMFEI_LM_2D_EXPORTS
#       define DIMFEI_LM_2D_API __declspec(dllexport)
#   else
#       define DIMFEI_LM_2D_API __declspec(dllimport)
#   endif
#else
#   define DIMFEI_LM_2D_API
#endif


extern "C" DIMFEI_LM_2D_API int calc_parameters(unsigned char *intensity, int width, int height);
extern "C" DIMFEI_LM_2D_API int read_parameters(float *amp, float *cent_hor, float *cent_ver, float *s_hor, float *s_ver, float *ang, float *offset);
extern "C" DIMFEI_LM_2D_API int set_parameters(float amp, float cent_hor, float cent_ver, float s_hor, float s_ver, float ang, float offset);
extern "C" DIMFEI_LM_2D_API int reset_parameters();

// Wrappers for backward compatibility
extern "C" DIMFEI_LM_2D_API int calcula_parametros(unsigned char *intensity, int width, int height);
extern "C" DIMFEI_LM_2D_API int le_parametros(float *amp, float *cent_hor, float *cent_ver, float *s_hor, float *s_ver, float *ang, float *offset);
extern "C" DIMFEI_LM_2D_API int ajusta_parametros(float amp, float cent_hor, float cent_ver, float s_hor, float s_ver, float ang, float offset);
extern "C" DIMFEI_LM_2D_API int reset_parametros();
