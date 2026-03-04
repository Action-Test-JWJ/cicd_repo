#ifndef INWHEEL_MOTOR_HARDWARE__VISIBLITY_CONTROL_H_
#define INWHEEL_MOTOR_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define INWHEEL_MOTOR_HARDWARE_EXPORT __attribute__((dllexport))
#define INWHEEL_MOTOR_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define INWHEEL_MOTOR_HARDWARE_EXPORT __declspec(dllexport)
#define INWHEEL_MOTOR_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef INWHEEL_MOTOR_HARDWARE_BUILDING_DLL
#define INWHEEL_MOTOR_HARDWARE_PUBLIC INWHEEL_MOTOR_HARDWARE_EXPORT
#else
#define INWHEEL_MOTOR_HARDWARE_PUBLIC INWHEEL_MOTOR_HARDWARE_IMPORT
#endif
#define INWHEEL_MOTOR_HARDWARE_PUBLIC_TYPE INWHEEL_MOTOR_HARDWARE_PUBLIC
#define INWHEEL_MOTOR_HARDWARE_LOCAL
#else
#define INWHEEL_MOTOR_HARDWARE_EXPORT __attribute__((visibility("default")))
#define INWHEEL_MOTOR_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define INWHEEL_MOTOR_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define INWHEEL_MOTOR_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define INWHEEL_MOTOR_HARDWARE_PUBLIC
#define INWHEEL_MOTOR_HARDWARE_LOCAL
#endif
#define INWHEEL_MOTOR_HARDWARE_PUBLIC_TYPE
#endif


#endif  // INWHEEL_MOTOR_HARDWARE__VISIBLITY_CONTROL_H_