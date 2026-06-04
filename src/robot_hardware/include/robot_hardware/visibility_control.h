#ifndef ROBOT_HARDWARE__VISIBILITY_CONTROL_H_
#define ROBOT_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOT_HARDWARE_EXPORT __attribute__((dllexport))
#define ROBOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define ROBOT_HARDWARE_EXPORT __declspec(dllexport)
#define ROBOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOT_HARDWARE_BUILDING_DLL
#define ROBOT_HARDWARE_PUBLIC ROBOT_HARDWARE_EXPORT
#else
#define ROBOT_HARDWARE_PUBLIC ROBOT_HARDWARE_IMPORT
#endif
#define ROBOT_HARDWARE_PUBLIC_TYPE ROBOT_HARDWARE_PUBLIC
#define ROBOT_HARDWARE_LOCAL
#else
#define ROBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define ROBOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define ROBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define ROBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOT_HARDWARE_PUBLIC
#define ROBOT_HARDWARE_LOCAL
#endif
#define ROBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // ROBOT_HARDWARE__VISIBILITY_CONTROL_H_
