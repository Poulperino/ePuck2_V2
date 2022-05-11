#ifndef DUMB_SPEED_H
#define DUMB_SPEED_H

void go_straight(void);
void go_circle_ccw(void);
void go_circle_cw(void);
void stop_moving(void);
int16_t get_right_speed_mms(void);
int16_t get_left_speed_mms(void);

#endif /* DUMB_SPEED_H */
