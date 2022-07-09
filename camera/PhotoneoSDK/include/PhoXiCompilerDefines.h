#ifndef _PHOXI_COMPILER_DEFINES
#define _PHOXI_COMPILER_DEFINES

#ifndef PHOXI_DLL_API
#   ifdef _WIN32
#       define PHOXI_DLL_API __declspec(dllimport)
#   else
#       define PHOXI_DLL_API
#   endif
#endif

#ifndef PHOXI_API_SERVER
#   define PHOXI_API_CLIENT
#else
#   ifdef PHOXI_API_CLIENT
#       undef PHOXI_API_CLIENT
#   endif
#endif

#ifndef DEPRECATED
#ifdef __GNUC__
#define DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED
#pragma message("DEPRECATED is not defined for this compiler")
#endif
#endif

#endif //_PHOXI_COMPILER_DEFINES