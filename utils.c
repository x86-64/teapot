#include "common.h"
#include "utils.h"

/*
	utility functions
*/
extern const uint32_t powers[];  // defined in sermsg.c

/// convert a floating point input value into an integer with appropriate scaling.
/// \param *df pointer to floating point structure that holds fp value to convert
/// \param multiplicand multiply by this amount during conversion to integer
///
/// Tested for up to 42'000 mm (accurate), 420'000 mm (precision 10 um) and
/// 4'200'000 mm (precision 100 um).
int32_t decfloat_to_int(decfloat *df, uint16_t multiplicand) {
	uint32_t	r = df->mantissa;
	uint8_t	e = df->exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
	if (e)
		e--;

	// This raises range for mm by factor 1000 and for inches by factor 100.
	// It's a bit expensive, but we should have the time while parsing.
	while (e && multiplicand % 10 == 0) {
		multiplicand /= 10;
		e--;
	}

	r *= multiplicand;
	if (e)
		r = (r + powers[e] / 2) / powers[e];

	return df->sign ? -(int32_t)r : (int32_t)r;
}

void  decfloat_set_int(decfloat *df, int32_t value){
	if(value < 0){
		df->mantissa = -value;
		df->sign     = 1;
	}else{
		df->mantissa = value;
		df->sign     = 0;
	}
	df->exponent = 0;
}
