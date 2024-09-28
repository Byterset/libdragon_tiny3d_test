#include "math.h"

/**
 * @brief Clamp a int value to a specified range.
 * 
 * @param input 
 * @param min 
 * @param max 
 * @return int clamped value
 */
int clampi(int input, int min, int max) {
    if (input < min) {
        return min;
    }
    if (input > max) {
        return max;
    }
    return input;
}