// dllmain.cpp : Defines the entry point for the DLL application.
#include "stdafx.h"

#ifdef _WIN32
extern bool are_prms_defined;
extern bool use_last_fit;

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		are_prms_defined = false;
		use_last_fit = false;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
#endif
