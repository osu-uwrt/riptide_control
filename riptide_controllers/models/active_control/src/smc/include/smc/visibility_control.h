#ifndef SMC__VISIBILITY_CONTROL_H_
#define SMC__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SMC_EXPORT __attribute__ ((dllexport))
    #define SMC_IMPORT __attribute__ ((dllimport))
  #else
    #define SMC_EXPORT __declspec(dllexport)
    #define SMC_IMPORT __declspec(dllimport)
  #endif
  #ifdef SMC_BUILDING_LIBRARY
    #define SMC_PUBLIC SMC_EXPORT
  #else
    #define SMC_PUBLIC SMC_IMPORT
  #endif
  #define SMC_PUBLIC_TYPE SMC_PUBLIC
  #define SMC_LOCAL
#else
  #define SMC_EXPORT __attribute__ ((visibility("default")))
  #define SMC_IMPORT
  #if __GNUC__ >= 4
    #define SMC_PUBLIC __attribute__ ((visibility("default")))
    #define SMC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SMC_PUBLIC
    #define SMC_LOCAL
  #endif
  #define SMC_PUBLIC_TYPE
#endif
#endif  // SMC__VISIBILITY_CONTROL_H_
// Generated 19-Nov-2023 17:55:21
 