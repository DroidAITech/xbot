#ifndef AIUI_COMMON_HDR_X2342Y3
#define AIUI_COMMON_HDR_X2342Y3

#if defined(_MSC_VER)
typedef long ssize_t;
typedef unsigned long pid_t; 
#   undef AIUIEXPORT
#	if defined(AIUI_LIB_COMPILING)
#	  define AIUIEXPORT __declspec(dllexport)
#     define AIUIAPI    __stdcall
#   else
#     define AIUIEXPORT __declspec(dllimport)
#     define AIUIAPI    __stdcall
#	endif
#else
#   undef AIUIEXPORT
#	define AIUIEXPORT
#   undef AIUIAPI
#   define AIUIAPI
#endif

#endif