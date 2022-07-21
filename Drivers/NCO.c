#include "NCO.h"		// contains the structs and variable definitions


// function to compute the 32 bit complex multiplication
T_COMPLEX32 Complex_MUT(T_COMPLEX32 a, T_COMPLEX32 b) {
	T_COMPLEX32 result;

	T_INT64 ar = (T_INT64) a.real;	// declare a variable that's 64 bits long, from the real component of a.
	T_INT64 ai = (T_INT64) a.imag;	// declare a variable that's 64 bits long, from the real component of a.
	T_INT64 br = (T_INT64) b.real;	// declare a variable that's 64 bits long, from the real component of a.
	T_INT64 bi = (T_INT64) b.imag;	// declare a variable that's 64 bits long, from the real component of a.
 
	T_INT64 cr = ar*br - ai*bi;	// compute the real component
	T_INT64 ci = ar*bi + ai*br;	// compute the imag component
 
//	T_INT64 cr = ar*br - ai*bi;	// compute the real component
//	T_INT64 ci = ar*bi - ai*br;	// compute the imag component

	result.real = (T_INT32) (cr >> 31);	// truncate the 64 bit value to a 32 bit, and typedef it
	result.imag = (T_INT32) (ci >> 31);	// truncate the 64 bit value to a 32 bit, and typedef it

	return result;
}

