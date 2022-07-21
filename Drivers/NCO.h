#ifndef NCO_H
#define NCO_H

// define some variable types
typedef long T_INT32;
typedef short T_INT16;
typedef unsigned short T_UNT32;
typedef long long T_INT64;

// Define a struct to hold the phasor
typedef struct {
	T_INT32 real;
	T_INT32 imag;
} T_COMPLEX32;


/** function prototypes **/

// function for multiplying complex numbers
T_COMPLEX32 Complex_MUT(T_COMPLEX32 a, T_COMPLEX32 b);
#endif