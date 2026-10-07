#ifndef TASKS_H
#define TASKS_H

#include "vehicle_state.h"

void *task_stability(void *argument);
void *task_traction(void *argument);
void *task_fuel(void *argument);
void *task_diagnostics(void *argument);

#endif
