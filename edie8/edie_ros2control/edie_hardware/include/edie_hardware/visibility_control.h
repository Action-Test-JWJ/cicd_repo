#ifndef EDIE_HARDWARE__VISIBLITY_CONTROL_H_
#define EDIE_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define EDIE_HARDWARE_EXPORT __attribute__((dllexport))
#define EDIE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define EDIE_HARDWARE_EXPORT __declspec(dllexport)
#define EDIE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef EDIE_HARDWARE_BUILDING_DLL
#define EDIE_HARDWARE_PUBLIC EDIE_HARDWARE_EXPORT
#else
#define EDIE_HARDWARE_PUBLIC EDIE_HARDWARE_IMPORT
#endif
#define EDIE_HARDWARE_PUBLIC_TYPE EDIE_HARDWARE_PUBLIC
#define EDIE_HARDWARE_LOCAL
#else
#define EDIE_HARDWARE_EXPORT __attribute__((visibility("default")))
#define EDIE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define EDIE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define EDIE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define EDIE_HARDWARE_PUBLIC
#define EDIE_HARDWARE_LOCAL
#endif
#define EDIE_HARDWARE_PUBLIC_TYPE
#endif


#endif  // EDIE_HARDWARE__VISIBLITY_CONTROL_H_