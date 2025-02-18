#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>

//40 steps per mm * 7 inches * 25mm/inch
#define MAX_THRUST_LENGTH 40 * 7 * 25 
//lower is faster. This is the closest we can get the pulses without losing steps
//Heavier loads require either more accel lag, or a lower max thrust speed.
//Tighter holes may also require less thrust speed. steppers lose a lot of torque when run fast
#define MAX_THRUST_SPEED 15
//number of motor steps between each acceleration lookup step. Bigger number slows down the maximum thrust rate
// 2 is quite robotic, above 5 gets pretty sinusoidal
#define ACCEL_LAG 4

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define DIR_PORT  PORTC
#define DIR_PLUS		PC5
//This is just OC1A but needs the output definition
#define STEP_PORT  PORTD
#define STEP_DDR   DDRD
#define STEP_PLUS   PD7
#define XYENABLE    PD6
#define THRUST_SPEED_PIN    A1
#define THRUST_LENGTH_PIN   A3
#define THRUST_POSITION_PIN A2
//push this to start/stop the device
#define START_PIN	    PC4
#define START_PORT	    PINC
//#define ZEROSW		18
#define ZERO_PIN  PC2
#define ZERO_PORT PINC
//the direction required to move towards the zero switch
#define ZERODIR		1

enum isr_state{
  state_accel,
  state_decel,
  state_running
};

//TODO: This is currently a sine wave, but should actually be a logarithm because there's a exponentiator - slower steps take longer to execute
// ideally I could write a time-based accelerator (and at one point I had), but predicting steps to hit the same zero every time is hard
// I also tried to read the marlin code but I'm too dumb for that
const uint16_t accelerate_lookup[] = {
100, 100, 100, 100, 100, 99,
99, 99, 98, 98, 98, 97, 96, 96,
95, 95, 94, 93, 92, 91, 90, 90,
89, 88, 86, 85, 84, 83, 82, 81,
79, 78, 77, 75, 74, 73, 71, 70,
68, 67, 65, 64, 62, 61, 59, 58,
56, 55, 53, 52, 50, 48, 47, 45,
44, 42, 41, 39, 38, 36, 35, 33,
32, 30, 29, 27, 26, 25, 23, 22,
21, 19, 18, 17, 16, 15, 14, 12,
11, 10, 10, 9, 8, 7, 6, 5,
5, 4, 4, 3, 2, 2, 2, 1,
1, 1, 0, 0, 0, 0, 0};


#define MIN_THRUST_LENGTH COUNT_OF(accelerate_lookup) * ACCEL_LAG * 2


#endif

