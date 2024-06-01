/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_COMMON_H
#define ROBOCAR_COMMON_H

#include "cycle/cycle.h"
#include "common/messages.h"

namespace robocar
{
    // status
    const int STATUS_OK = 0;
    const int STATUS_WARNING = 1;
    const int STATUS_ERROR = 2;
    const int STATUS_TIMEOUT = 3;
    const int STATUS_ENABLED = 4;
    const int STATUS_DISABLED = 5;
    // planner state
    const int STATE_STANDBY = 0;
    const int STATE_DRIVE = 1;
    const int STATE_KEEP_DIST = 2;
    // object type
    const int OBJ_NONE = 0;
    const int OBJ_OBSTACLE = 1;
    const int OBJ_VIRTUAL = 2;
    const int OBJ_TFL_NONE = 3;
    const int OBJ_TFL_GREEN = 4;
    const int OBJ_TFL_YELLOW = 5;
    const int OBJ_TFL_RED = 6;
}

#endif // ROBOCAR_COMMON_H