#ifndef __Version_h__
#define __Version_h__

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : Warning: " #desc)

#define GIT_SHA1 "1512f312ee11cc911776a2c391ddaf4d5851fd58"
#define GIT_REFSPEC "refs/heads/BallJointDev-v3"
#define GIT_LOCAL_STATUS "DIRTY"

#define PBD_VERSION "2.2.0"

#ifdef DL_OUTPUT
#pragma WARNING(Local changes not committed.)
#endif

#endif
