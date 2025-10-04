#pragma once

#define APP_NAME            "HALO Strobe Controller"
#define APP_VERSION         "0.4.3"

#define APP_VER_MAJOR       0
#define APP_VER_MINOR       4
#define APP_VER_PATCH       3

#ifndef APP_GIT_HASH
  #define APP_GIT_HASH      "nogit"
#endif
#ifndef APP_BUILD_DATE
  #define APP_BUILD_DATE    __DATE__ " " __TIME__
#endif

static inline const char* app_name()    { return APP_NAME; }
static inline const char* app_version() { return APP_VERSION; }
static inline const char* app_git()     { return APP_GIT_HASH; }
static inline const char* app_build()   { return APP_BUILD_DATE; }
