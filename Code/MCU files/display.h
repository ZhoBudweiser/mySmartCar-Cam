#ifndef _display_h
#define _display_h

#include "headfile.h"


void display_init(void);
void car_mode_display(uint8 ips_mode);
void wireless_data_send(car_control_t * car_move, Domain_queue *queue_ptr);
    
#endif