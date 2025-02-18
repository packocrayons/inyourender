#include <stdint.h>
#include "driver.h"

//define this when you goobered something and can't figure it out and you need serial because despite the fact that you said you've solved embedded issues by blinking LEDs in interviews, it's bullshit and this is easier
#define DEBUG_PRINTS

volatile uint8_t currentdir;
uint8_t interrupt_complete = 0;
uint16_t total_steps = 0;
uint16_t steps_from_zero = 0;
uint16_t thrust_length = MAX_THRUST_LENGTH;
uint8_t thrust_speed = 200; //lower is faster - this is the timing between steps
uint16_t thrust_position = 0;
volatile uint8_t double_buffer_lockout = 0; //a "mutex" as it were to lock out the ISR from picking up double buffers halfway through doing math
bool running = true;


void timer_setup();

void calc_global_thrust_params(){
  if (!double_buffer_lockout){ //avr-gcc doesn't have __sync_bool_compare_and_swap and I'm too lazy to figure out how to add c standard libraries to the compiler
    //technically this could still go awry if the ISR runs at _exactly this line_. Of course, that will happen. Where this would break:
    // - the values have changed enough to cause decel overshoot
    // - the ISR that runs is one of the final running steps
    // - the ISR happens right here
    // Insert "some of you may die, but that's a risk I'm willing to take" meme here.
    double_buffer_lockout = 1;
    thrust_position = map(analogRead(THRUST_POSITION_PIN), 0, 1023, 0, MAX_THRUST_LENGTH - 100); //yes, I'm using analogRead. I'm not in an ISR so I get to be lazy here. Please PR this with real analog code
    int temp = map(analogRead(THRUST_LENGTH_PIN), 0, 1023, MIN_THRUST_LENGTH, MAX_THRUST_LENGTH);
    if (temp + thrust_position > MAX_THRUST_LENGTH){
      thrust_length = MAX_THRUST_LENGTH - thrust_position - 1;
    } else {
      thrust_length = temp;
    }
    //thrust_length = ((temp + thrust_position) > MAX_THRUST_LENGTH) ? MAX_THRUST_LENGTH : temp;
    thrust_speed = map(analogRead(THRUST_SPEED_PIN), 0, 1023, MAX_THRUST_SPEED, 255);
    double_buffer_lockout = 0;
  }
}

void setup(){
  #ifdef DEBUG_PRINTS
  Serial.begin(9600);
  #endif
  DDRC &= ~(1 << ZERO_PIN);
  PORTC |= (1 << ZERO_PIN);
  DDRC &= ~(1 << START_PIN);
  STEP_DDR |= (1 << STEP_PLUS); //what are you doing, step data direction register?
  //TODO enable/disable steppers on stop
  DDRD |= 1 << XYENABLE;
  DDRC |= 1 << DIR_PLUS;
	
  if (ZERODIR == LOW){
    PORTC &= ~(1 << DIR_PLUS);
  } else {
    PORTC |= 1 << DIR_PLUS;
  }

  #ifdef DEBUG_PRINTS
  Serial.println("zeroing");
  #endif

  while(!(ZERO_PORT & (1 << ZERO_PIN))){
    STEP_PORT |= 1 << STEP_PLUS;
    delayMicroseconds(500); //please arduino-san. Misuse my timers with a bunch of library bloat
    STEP_PORT &= ~(1 << STEP_PLUS);
    delayMicroseconds(500);
  }

  #ifdef DEBUG_PRINTS
  Serial.println("zero found, commence thrusting");
  #endif

  total_steps = 0;
  currentdir = (~ZERODIR) & 0b1;
  if (currentdir == LOW){
    DIR_PORT &= ~(1 << DIR_PLUS);
  } else {
    DIR_PORT |= (1 << DIR_PLUS);
  }
  //calculate these once so that the ISR can pick up good values on the static init, instead of weird defaults that some nerd picked
  calc_global_thrust_params();
  timer_setup();
}

uint16_t step_time = 255;

void loop(){
  int steps_to_accel = 0;

  calc_global_thrust_params();

#ifdef DEBUG_PRINTS
  Serial.print(" Pos: ");
  Serial.print(thrust_position);
  Serial.print(" Len: ");
  Serial.print(thrust_length);
  Serial.print(" Spd: ");
  Serial.print(thrust_speed);
  Serial.print(" steps");
  Serial.println(total_steps);
#endif

  if ((START_PORT & (1 << START_PIN))){
    running = false;
    TCCR1B &= ~(1 << CS10);//disable the clock
   	TIMSK1 &= ~(1 << OCIE1A); //OCR1A(H/L) interrupt enable
    delay(1000);
  }


//TODO: restarting in the middle of a thrust does not run the accelerator. I _think_ we can just modify the state machine and force it to reaccelerate regardless of where it is.
// We'll need to check for risk of overshoot though
  while(!running){
    if ((START_PORT & (1 << START_PIN))){
      delay(1000);
      running = true;
      TCCR1B |= (1 << CS10);
     	TIMSK1 |= (1 << OCIE1A); //OCR1A(H/L) interrupt enable

    }
  }
  delay(100);//I'm afraid of interrupting the timer ISR with analogRead's ISR too often.

}

void timer_setup(){
  //fast pwm frequency mode
	TCCR1A |= (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS10);
	TCCR1C = 0x00;
	//TIMSK1 = 0x00;
	TIMSK1 |= (1 << OCIE1A); //OCR1A(H/L) interrupt enable
	OCR1A = 0xDEAD; //TODO: this is 16 bit. Does the compiler know that?
}

//NOTE to future developers (probably me). You get maybe 50 instructions in this ISR before you start overrunning things. The main loop has to do analog reads at the same time. Be careful
//Re: the previous warning. This looks like a long ISR already, but it's a big jump table. Most of this code is just figuring out what (simple) math to do
ISR(TIMER1_COMPA_vect){
  static isr_state state = state_accel;
  static uint8_t i = 0;
  static uint8_t j = 0;
  static uint8_t step_state = 0b01;   
  //these values are double buffered, so that they can't be changed unintentionally in the middle of an accel/decel sequence
  static uint16_t tl_db = thrust_length; 
  static uint16_t ts_db = thrust_speed;
  static uint16_t tp_db = thrust_position;
  if (step_state == LOW){
    STEP_PORT &= ~(1 << STEP_PLUS);
  } else {
    STEP_PORT |= (1 << STEP_PLUS);
  }
  step_state = 0b1 & ~(step_state);
  switch (state){
    case state_accel:
      if(i < COUNT_OF(accelerate_lookup)) { //still accelerating
        if (j > (ACCEL_LAG - 1)){
          OCR1A = accelerate_lookup[i++] + ts_db; //the post-increment operator (if it exists on the 328) should save us a clock. Instructions are at a premium in an ISR like this
          j = 0;
        } else {
          j++;
        }
      } else {
        state = state_running;
        i = COUNT_OF(accelerate_lookup) - 1;
      }
    break;
    case state_running:
      if (!double_buffer_lockout){
        double_buffer_lockout = 1;
        //pick up the double buffers when we're not doing math
        tl_db = thrust_length; 
        ts_db = thrust_speed;
        tp_db = thrust_position;
        OCR1A = thrust_speed;
        double_buffer_lockout = 0;
      }
      if (currentdir != ZERODIR){
        if(((tl_db + tp_db) - total_steps) <= (COUNT_OF(accelerate_lookup) * ACCEL_LAG)){
          state = state_decel;
          break;
        }
      } else {
        if ((total_steps) <= ((COUNT_OF(accelerate_lookup) * ACCEL_LAG) + tp_db)){
          state = state_decel;
          break;
        }
      }
      break;
    case state_decel:
      if (i > 0){
        if (j >= (ACCEL_LAG - 1)){
          OCR1A = accelerate_lookup[i--] + ts_db;
          j = 0;
        } else {
          j++;
        }
      }
      if (i <= 1){
        currentdir = (~currentdir) & 0b1;
        if (currentdir == LOW){
          DIR_PORT &= ~(1 << DIR_PLUS);
        } else {
          DIR_PORT |= (1 << DIR_PLUS);
        }
        state = state_accel;
      }
    break;
  }
  total_steps = (currentdir == ZERODIR) ? total_steps - 1: total_steps + 1;
}

