#pragma once
#include "utility/yaml_reader.hpp"
#include "utility/math.h"

#define RSRESET "\033[0m"
#define RSBOLDRED "\033[1m\033[31m"     /* Bold Red */
#define RSBOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define RSBOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define RSBOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define RSBOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define LINFO (std::cout << RSBOLDGREEN)
#define LWARNING (std::cout << RSBOLDYELLOW)
#define LERROR (std::cout << RSBOLDRED)
#define LDEBUG (std::cout << RSBOLDCYAN)
#define LTITLE (std::cout << RSBOLDMAGENTA)
#define END (std::endl)
#define REND "\033[0m" << std::endl
#define GETBIT(X, Y) ((X) >> (Y)&1)
#define NAME(x) (#x)

using namespace std;
using namespace Eigen;
namespace robosense{} // just
using namespace robosense;

typedef Vector3d V3D;
typedef Vector2d V2D;
typedef Vector2i V2I;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

#include "msg/imu_msg.h"
#include "msg/pose_msg.h"
#include "msg/lidar_msg.h"
#include "msg/cloud_msg.h"
#include "utility/cloud_process.h"
