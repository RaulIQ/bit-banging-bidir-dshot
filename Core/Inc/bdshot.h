

#ifndef BDSHOT_H_
#define BDSHOT_H_
#include "global_variables.h"

void update_motors();
void preset_bb_Dshot_buffer_single();
void fill_bb_BDshot_buffer(uint16_t m1_value);
int32_t get_motors_rpm();

#endif /*BDSHOT_H_*/