#ifndef PID__VISIBILITY_CONTROL_H_
#define PID__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PID_EXPORT __attribute__ ((dllexport))
    #define PID_IMPORT __attribute__ ((dllimport))
  #else
    #define PID_EXPORT __declspec(dllexport)
    #define PID_IMPORT __declspec(dllimport)
  #endif
  #ifdef PID_BUILDING_LIBRARY
    #define PID_PUBLIC PID_EXPORT
  #else
    #define PID_PUBLIC PID_IMPORT
  #endif
  #define PID_PUBLIC_TYPE PID_PUBLIC
  #define PID_LOCAL
#else
  #define PID_EXPORT __attribute__ ((visibility("default")))
  #define PID_IMPORT
  #if __GNUC__ >= 4
    #define PID_PUBLIC __attribute__ ((visibility("default")))
    #define PID_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PID_PUBLIC
    #define PID_LOCAL
  #endif
  #define PID_PUBLIC_TYPE
#endif
#endif  // PID__VISIBILITY_CONTROL_H_
// Generated 21-Dec-2023 11:19:11
 