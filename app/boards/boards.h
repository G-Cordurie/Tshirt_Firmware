#ifndef BOARDS_H
#define BOARDS_H

#define HW_TYPE_0705 1
#define HW_TYPE_0706 2

#if (HW_TYPE == HW_TYPE_0705)
#include "0705.h"
#elif (HW_TYPE == HW_TYPE_0706)
#include "0706.h"
#endif

#endif // BOARDS_H