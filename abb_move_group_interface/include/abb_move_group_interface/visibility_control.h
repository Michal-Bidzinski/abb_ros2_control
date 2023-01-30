#ifndef ABB_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_
#define ABB_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ABB_MOVE_GROUP_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define ABB_MOVE_GROUP_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define ABB_MOVE_GROUP_INTERFACE_EXPORT __declspec(dllexport)
    #define ABB_MOVE_GROUP_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ABB_MOVE_GROUP_INTERFACE_BUILDING_DLL
    #define ABB_MOVE_GROUP_INTERFACE_PUBLIC ABB_MOVE_GROUP_INTERFACE_EXPORT
  #else
    #define ABB_MOVE_GROUP_INTERFACE_PUBLIC ABB_MOVE_GROUP_INTERFACE_IMPORT
  #endif
  #define ABB_MOVE_GROUP_INTERFACE_PUBLIC_TYPE ABB_MOVE_GROUP_INTERFACE_PUBLIC
  #define ABB_MOVE_GROUP_INTERFACE_LOCAL
#else
  #define ABB_MOVE_GROUP_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define ABB_MOVE_GROUP_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define ABB_MOVE_GROUP_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define ABB_MOVE_GROUP_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ABB_MOVE_GROUP_INTERFACE_PUBLIC
    #define ABB_MOVE_GROUP_INTERFACE_LOCAL
  #endif
  #define ABB_MOVE_GROUP_INTERFACE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ABB_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_