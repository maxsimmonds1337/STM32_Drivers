#include "stm32f411xe.h"                  // Device header


/*typedef long T_INT32;
typedef short T_INT16;
typedef unsigned short T_UINT16;
typedef long long T_INT64;

typedef struct {
	T_INT32 real;
	T_INT32 imag;
} T_COMPLEX32;
*/

// function prototypes
void init_timer(void);
void start_TIM2(void);
uint16_t read_TIM2();
uint16_t read_flag();
void init_timer_3(void);
void PLL(void);

//T_COMPLEX32 Complex_MUT(T_COMPLEX32 a, T_COMPLEX32 b);

