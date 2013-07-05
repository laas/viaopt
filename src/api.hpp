#ifndef __VIAOPT_API__
#define __VIAOPT_API__

#if defined (WIN32)
#  ifdef viaopt_EXPORTS
#    define VIAOPT_EXPORT __declspec(dllexport)
#  else
#    define VIAOPT_EXPORT __declspec(dllimport)
#  endif
#else
#  define VIAOPT_EXPORT
#endif

#endif
