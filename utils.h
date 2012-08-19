#ifndef UTILS_H
#define UTILS_H

/// this is a very crude decimal-based floating point structure.
/// a real floating point would at least have signed exponent.\n
/// resulting value is \f$ mantissa * 10^{-(exponent - 1)} * ((sign * 2) - 1)\f$
typedef struct {
	uint32_t	mantissa;		///< the actual digits of our floating point number
	uint8_t	exponent	:7;	///< scale mantissa by \f$10^{-exponent}\f$
	uint8_t	sign			:1; ///< positive or negative?
} decfloat;

/*
	decfloat_to_int() is the weakest subject to variable overflow. For evaluation, we assume a build room of +-1000 mm and STEPS_PER_MM_x between 1.000 and 4096. Accordingly for metric units:

		df->mantissa:  +-0..1048075    (20 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  1000            (10 bit)

	imperial units:

		df->mantissa:  +-0..32267      (15 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  25400           (15 bit)
*/
// decfloat_to_int() can handle a bit more:
#define	DECFLOAT_EXP_MAX 3 // more is pointless, as 1 um is our presision
// (2^^32 - 1) / multiplicand - powers[DECFLOAT_EXP_MAX] / 2 =
// 4294967295 / 1000 - 5000 =
#define	DECFLOAT_MANT_MM_MAX 4289967  // = 4290 mm
// 4294967295 / 25400 - 5000 =
#define	DECFLOAT_MANT_IN_MAX 164093   // = 164 inches = 4160 mm

int32_t decfloat_to_int(decfloat *df, uint16_t multiplicand);


#define MIN(a,b)  ((a < b) ? a : b)
#define MAX(a,b)  ((a < b) ? b : a)

#endif
