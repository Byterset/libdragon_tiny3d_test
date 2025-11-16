/**
 * @file mathf.c
 * @brief Implementation of mathematical functions.
 */

#include "mathf.h"
#include <math.h>

unsigned int gRandomSeed = 1; // Seed value for pseudo-random number generation.

#define MAX_INT_VALUE   0x7fff


/**
 * @brief Generates a pseudo-random integer.
 *
 * This function uses a linear congruential generator (LCG) algorithm to 
 * produce a pseudo-random integer. The global variable `gRandomSeed` is 
 * updated with each call to ensure a new random value is generated.
 *
 * @return A pseudo-random integer.
 */
int randomInt() {
    gRandomSeed = gRandomSeed * 22695477 + 1;
    return (gRandomSeed >> 16) & MAX_INT_VALUE;
}

/**
 * @brief Generates a pseudo-random integer within a specified range.
 *
 * This function generates a pseudo-random integer within the specified range
 * by scaling the output of `randomInt()` to the desired range.
 *
 * @param min The minimum value of the range.
 * @param maxPlusOne The maximum value of the range plus one.
 * @return A pseudo-random integer within the specified range.
 */
int randomInRange(int min, int maxPlusOne) {
    return randomInt() * (maxPlusOne - min) / (MAX_INT_VALUE + 1) + min;
}

/**
 * @brief Generates a pseudo-random floating-point number within a specified range.
 *
 * This function generates a pseudo-random floating-point number within the specified range
 * by scaling the output of `randomInt()` to the desired range.
 *
 * @param min The minimum value of the range.
 * @param max The maximum value of the range.
 * @return A pseudo-random floating-point number within the specified range.
 */
float randomInRangef(float min, float max) {
    return randomInt() * (max - min) * (1.0f / MAX_INT_VALUE) + min;
}


/**
 * @brief Linearly interpolates between two float values.
 *
 * This function performs a linear interpolation between the values `from` and `to`
 * based on the interpolation factor `t`. The interpolation factor `t` should be
 * in the range [0, 1], where 0 returns `from` and 1 returns `to`.
 *
 * @param from The starting value.
 * @param to The ending value.
 * @param t The interpolation factor, typically in the range [0, 1].
 * @return The interpolated value.
 */
float mathfLerp(float from, float to, float t) {
    return from * (1.0f - t) + to * t;
}

/**
 * @brief Calculates the linear parameter that produces the interpolant value within the range [from, to].
 *
 * This function computes the inverse linear interpolation of a value within a specified range.
 * It returns a value between 0 and 1 that represents the position of the input value within the range.
 *
 * @param from The start of the range.
 * @param to The end of the range.
 * @param value The value to be interpolated.
 * @return The linear parameter that produces the interpolant value within the range [from, to].
 */
float mathfInvLerp(float from, float to, float value) {
    return (value - from) / (to - from);
}

/**
 * @brief Moves a value towards a target value by a maximum amount.
 *
 * This function calculates a new value that is moved from the starting value
 * (`from`) towards the target value (`to`) by a maximum amount (`maxMove`).
 * If the difference between `from` and `to` is less than or equal to `maxMove`,
 * the function returns the target value (`to`). Otherwise, it returns a value
 * that is `maxMove` units closer to the target value.
 *
 * @param from The starting value.
 * @param to The target value.
 * @param maxMove The maximum amount to move towards the target value.
 * @return The new value moved towards the target value.
 */
float mathfMoveTowards(float from, float to, float maxMove) {
    float offset = to - from;
    if (fabsf(offset) <= maxMove) {
        return to;
    } else {
        return signf(offset) * maxMove + from;
    }
}

/**
 * @brief Computes the modulo of a floating-point number.
 * 
 * @param input 
 * @param divisor 
 * @return float 
 */
float mathfMod(float input, float divisor) {
    float floorDivide = floorf(input / divisor);
    return input - floorDivide * divisor;
}


/**
 * @brief Computes a bounce-back interpolation value.
 *
 * This function calculates a value that simulates a bounce-back effect
 * based on the input parameter t. The result is derived from the equation
 * -t + t^2, which creates a parabolic curve that starts at 0, rises to a
 * peak, and then falls back to 0.
 *
 * @param t The input value, typically in the range [0, 1].
 * @return The interpolated bounce-back value.
 */
float mathfBounceBackLerp(float t) {
    return -t + t * t;
}

/**
 * @brief Computes a pseudo-random floating-point number in the range [0, 1].
 * 
 * @return float 
 */
float mathfRandomFloat() {
    return (float)randomInt() / (float)MAX_INT_VALUE;
}

/**
 * @brief Clamp a float value to a specified range.
 * 
 * @param input 
 * @param min 
 * @param max 
 * @return float clamped value
 */
float clampf(float input, float min, float max) {
    if (input < min) {
        return min;
    }

    if (input > max) {
        return max;
    }

    return input;
}

/**
 * @brief Returns the sign of a floating-point number.
 * 
 * @param input 
 * @return float 
 */
float signf(float input) {
    if (input > 0.0f) {
        return 1.0f;
    } else if (input < 0.0f) {
        return -1.0f;
    } else {
        return 0.0f;
    }
}

/**
 * @brief Returns the sign of an integer.
 * 
 * @param input 
 * @return int Sign of input, 1 if input is positive, -1 if input is negative, 0 if input is zero
 */
int sign(int input) {
    if (input > 0) {
        return 1;
    } else if (input < 0) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief Computes the absolute value of an integer.
 * 
 * @param input 
 * @return int absolute value of input
 */
int abs(int input) {
    if (input < 0) {
        return -input;
    }
    return input;
}

float floorf(float input) {
    int asint = (int)input;
    if (input >= 0 || input == asint) {
        return asint;
    } else {
        return asint - 1;
    }
}

float ceilf(float input) {
    int asint = (int)input;
    if (input <= 0 || input == asint) {
        return asint;
    } else {
        return asint + 1;
    }
}

float minf(float a, float b) {
    return a < b ? a : b;
}

float maxf(float a, float b) {
    return a > b ? a : b;
}

char floatTos8norm(float input) {
    int result = (int)(input * 127.0f);

    if (result > 127) {
        return 127;
    } else if (result < -127) {
        return -127;
    } else {
        return (char)result;
    }
}

float safeInvert(float input) {
    return (input != 0.0f) ? (1.0f / input) : (input > 0.0f ? INFINITY : -INFINITY);
}