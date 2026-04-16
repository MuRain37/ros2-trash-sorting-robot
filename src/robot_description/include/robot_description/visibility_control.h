#ifndef ROBOT_DESCRIPTION__VISIBILITY_CONTROL_H_
#define ROBOT_DESCRIPTION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOT_DESCRIPTION_EXPORT __attribute__((dllexport))
#define ROBOT_DESCRIPTION_IMPORT __attribute__((dllimport))
#else
#define ROBOT_DESCRIPTION_EXPORT __declspec(dllexport)
#define ROBOT_DESCRIPTION_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOT_DESCRIPTION_BUILDING_DLL
#define ROBOT_DESCRIPTION_PUBLIC ROBOT_DESCRIPTION_EXPORT
#else
#define ROBOT_DESCRIPTION_PUBLIC ROBOT_DESCRIPTION_IMPORT
#endif
#define ROBOT_DESCRIPTION_PUBLIC_TYPE ROBOT_DESCRIPTION_PUBLIC
#define ROBOT_DESCRIPTION_LOCAL
#else
#define ROBOT_DESCRIPTION_EXPORT __attribute__((visibility("default")))
#define ROBOT_DESCRIPTION_IMPORT
#if __GNUC__ >= 4
#define ROBOT_DESCRIPTION_PUBLIC __attribute__((visibility("default")))
#define ROBOT_DESCRIPTION_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOT_DESCRIPTION_PUBLIC
#define ROBOT_DESCRIPTION_LOCAL
#endif
#define ROBOT_DESCRIPTION_PUBLIC_TYPE
#endif

#endif  // ROBOT_DESCRIPTION__VISIBILITY_CONTROL_H_