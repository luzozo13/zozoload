#pragma once

// EV Charger Pin Assignments
// #define RX2 16
// #define TX2 17

// Control Pins
#define CP_CTRL 18
#define FLT_CTRL 19
#define REL_CTRL 21
#define MT_CTRL_CLOSE 22
#define MT_CTRL_OPEN 23

// LED Pins
#define L_R 25 // Red LED
#define L_G 26 // Green LED
#define L_B 27 // Blue LED

// Lock Control
#define LOCK_ON 32
#define LOCK_OFF 33

// Input Pins
#define PILOT_READ 34
#define B_R 35 // Button Red
#define B_G 36 // Button Green
#define B_B 39 // Button Blue

// PWM Channel
#define CH_CP_CTRL 0 // PWM channel for CP_CTRL

// State Definitions
#define STATE_INIT 0
#define STATE_A 1
#define STATE_B 2
#define STATE_C 3
#define STATE_FAULT -1

// Thresholds
#define TH_AB 3770
#define TH_BC 3135
#define TH_CD 2400

// CP (Control Pilot) PWM Values
#define CP_AMP_8 221
#define CP_AMP_16 187
#define CP_AMP_BOOST 170
#define CP_12P 0
#define CP_12N 255

int CPP_value;
int CPP_max;
int CPP_min;
int i_nb_pos;
int i_state_meas;