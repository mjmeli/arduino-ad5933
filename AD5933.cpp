/**
 * @file AD5933.cpp
 * @brief Library code for AD5933
 *
 * Library code for AD5933. Referenced the datasheet and code found at
 * https://github.com/WuMRC/drive
 *
 * @author Michael Meli
 */

#include "AD5933.h"
#include <Math.h>

/**
 * Request to read a byte from the AD5933.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a byte where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success or failure
 */
int AD5933::getByte(byte address, byte *value) {
    // Request to read a byte using the address pointer register
    Wire.beginTransmission(AD5933_ADDR);
    Wire.write(ADDR_PTR);
    Wire.write(address);

    // Ensure transmission worked
    if (byte res = Wire.endTransmission() != I2C_RESULT_SUCCESS) {
        *value = res;
        return false;
    }

    // Read the byte from the written address
    Wire.requestFrom(AD5933_ADDR, 1);
    if (Wire.available()) {
        *value = Wire.read();
        return true;
    } else {
        *value = 0;
        return false;
    }
}

/**
 * Write a byte to a register on the AD5933.
 *
 * @param address The register address to write to
 * @param value The byte to write to the address
 * @return Success or failure of transmission
 */
bool AD5933::sendByte(byte address, byte value) {
    // Send byte to address
    Wire.beginTransmission(AD5933_ADDR);
    Wire.write(address);
    Wire.write(value);

    // Check that transmission completed successfully
    if (byte res = Wire.endTransmission() != I2C_RESULT_SUCCESS) {
        return false;
    } else {
        return true;
    }
}

/**
 * Set the control mode register, CTRL_REG1. This is the register where the
 * current command needs to be written to so this is used a lot.
 *
 * @param mode The control mode to set
 * @return Success or failure
 */
bool AD5933::setControlMode(byte mode) {
    // Get the current value of the control register
    byte val;
    if (!getByte(CTRL_REG1, &val))
        return false;

    // Wipe out the top 4 bits...mode bits are bits 5 through 8.
    val &= 0x0F;

    // Set the top 4 bits appropriately
    val |= mode;

    // Write back to the register
    return sendByte(CTRL_REG1, val);
}

/**
 * Reset the AD5933. This interrupts a sweep if one is running, but the start
 * frequency, number of increments, and frequency increment register contents
 * are not overwritten, but an initialize start frequency command is required
 * to restart a frequency sweep.
 *
 * @return Success or failure
 */
bool AD5933::reset() {
    // Get the current value of the control register
    byte val;
    if (!getByte(CTRL_REG2, &val))
        return false;

    // Set bit D4 for restart
    val |= CTRL_RESET;

    // Send byte back
    return sendByte(CTRL_REG2, val);
}

/**
 * Set enable temperature measurement. This interferes with frequency sweep
 * operation, of course.
 *
 * @param enable Option to enable to disable temperature measurement.
 * @return Success or failure
 */
bool AD5933::enableTemperature(byte enable) {
    // If enable, set temp measure bits. If disable, reset to no operation.
    if (enable == TEMP_MEASURE) {
        return setControlMode(CTRL_TEMP_MEASURE);
    } else {
        return setControlMode(CTRL_NO_OPERATION);
    }
}

/**
 * Get the temperature reading from the AD5933. Waits until a temperature is
 * ready. Also ensures temperature measurement mode is active.
 *
 * @return The temperature in celcius, or -1 if fail.
 */
double AD5933::getTemperature() {
    // Set temperature mode
    if (enableTemperature(TEMP_MEASURE)) {
        // Wait for a valid temperature to be ready
        while((readStatusRegister() & STATUS_TEMP_VALID) != STATUS_TEMP_VALID) ;

        // Read raw temperature from temperature registers
        byte rawTemp[2];
        if (getByte(TEMP_DATA_1, &rawTemp[0]) &&
            getByte(TEMP_DATA_2, &rawTemp[1]))
        {
            // Combine raw temperature bytes into an interger. The ADC
            // returns a 14-bit 2's C value where the 14th bit is a sign
            // bit. As such, we only need to keep the bottom 13 bits.
            int rawTempVal = (rawTemp[0] << 8 | rawTemp[1]) & 0x1FFF;

            // Convert into celcius using the formula given in the
            // datasheet. There is a different formula depending on the sign
            // bit, which is the 5th bit of the byte in TEMP_DATA_1.
            if ((rawTemp[0] & (1<<5)) == 0) {
                return rawTempVal / 32.0;
            } else {
                return (rawTempVal - 16384) / 32.0;
            }
        }
    }
    return -1;
}


/**
 * Set the color source. Choices are between internal and external.
 *
 * @param source Internal or External clock
 * @return Success or failure
 */
bool AD5933::setClockSource(byte source) {
    // Determine what source was selected and set it appropriately
    switch (source) {
        case CLOCK_EXTERNAL:
            return sendByte(CTRL_REG2, CTRL_CLOCK_EXTERNAL);
        case CLOCK_INTERNAL:
            return sendByte(CTRL_REG2, CTRL_CLOCK_INTERNAL);
        default:
            return false;
    }
}

/**
 * Set the color source to internal or not.
 *
 * @param internal Whether or not to set the clock source as internal.
 * @return Success or failure
 */
bool AD5933::setInternalClock(bool internal) {
    // This function is mainly a wrapper for setClockSource()
    if (internal)
        return setClockSource(CLOCK_INTERNAL);
    else
        return setClockSource(CLOCK_EXTERNAL);
}

/**
 * Set the start frequency for a frequency sweep.
 *
 * @param start The initial frequency.
 * @return Success or failure
 */
bool AD5933::setStartFrequency(unsigned long start) {
    // Page 24 of the Datasheet gives the following formula to represent the
    // start frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (start / (clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    // Attempt sending all three bytes
    return sendByte(START_FREQ_1, highByte) &&
           sendByte(START_FREQ_2, midByte) &&
           sendByte(START_FREQ_3, lowByte);
}

/**
 * Set the increment frequency for a frequency sweep.
 *
 * @param start The frequency to increment by. Max of 0xFFFFFF.
 * @return Success or failure
 */
bool AD5933::setIncrementFrequency(unsigned long increment) {
    // Page 25 of the Datasheet gives the following formula to represent the
    // increment frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (increment / (clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    // Attempt sending all three bytes
    return sendByte(INC_FREQ_1, highByte) &&
           sendByte(INC_FREQ_2, midByte) &&
           sendByte(INC_FREQ_3, lowByte);
}

/**
 * Set the number of frequency increments for a frequency sweep.
 *
 * @param start The number of increments to use. Max 511.
 * @return Success or failure
 */
bool AD5933::setNumberIncrements(unsigned int num) {
    // Check that the number sent in is valid.
    if (num > 511) {
        return false;
    }

    // Divide the 9-bit integer into 2 bytes.
    byte highByte = (num >> 8) & 0xFF;
    byte lowByte = num & 0xFF;

    // Write to register.
    return sendByte(NUM_INC_1, highByte) &&
           sendByte(NUM_INC_2, lowByte);
}

/**
 * Set the PGA gain factor.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
bool AD5933::setPGAGain(byte gain) {
    // Get the current value of the control register
    byte val;
    if (!getByte(CTRL_REG1, &val))
        return false;

    // Clear out the bottom bit, D8, which is the PGA gain set bit
    val &= 0xFE;

    // Determine what gain factor was selected
    if (gain == PGA_GAIN_X1 || gain == 1) {
        // Set PGA gain to x1 in CTRL_REG1
        val |= PGA_GAIN_X1;
        return sendByte(CTRL_REG1, val);
    } else if (gain == PGA_GAIN_X5 || gain == 5) {
        // Set PGA gain to x5 in CTRL_REG1
        val |= PGA_GAIN_X5;
        return sendByte(CTRL_REG1, val);
    } else {
        return false;
    }
}

/**
 * Read the value of a register.
 *
 * @param reg The address of the register to read.
 * @return The value of the register. Returns 0xFF if can't read it.
 */
byte AD5933::readRegister(byte reg) {
    // Read status register and return it's value. If fail, return 0xFF.
    byte val;
    if (getByte(reg, &val)) {
        return val;
    } else {
        return STATUS_ERROR;
    }
}

/**
 * Read the value of the status register.
 *
 * @return The value of the status register. Returns 0xFF if can't read it.
 */
byte AD5933::readStatusRegister() {
    return readRegister(STATUS_REG);
}

/**
 * Read the value of the control register.
 *
 * @return The value of the control register. Returns 0xFFFF if can't read it.
 */
int AD5933::readControlRegister() {
    return ((readRegister(CTRL_REG1) << 8) | readRegister(CTRL_REG2)) & 0xFFFF;
}

/**
 * Get a raw complex number for a specific frequency measurement.
 *
 * @param real Pointer to an int that will contain the real component.
 * @param imag Pointer to an int that will contain the imaginary component.
 * @return Success or failure
 */
bool AD5933::getComplexData(int *real, int *imag) {
    // Wait for a measurement to be available
    while ((readStatusRegister() & STATUS_DATA_VALID) != STATUS_DATA_VALID);

    // Read the four data registers.
    // TODO: Do this faster with a block read
    byte realComp[2];
    byte imagComp[2];
    if (getByte(REAL_DATA_1, &realComp[0]) &&
        getByte(REAL_DATA_2, &realComp[1]) &&
        getByte(IMAG_DATA_1, &imagComp[0]) &&
        getByte(IMAG_DATA_2, &imagComp[1]))
    {
        // Combine the two separate bytes into a single 16-bit value and store
        // them at the locations specified.
        *real = (int16_t)(((realComp[0] << 8) | realComp[1]) & 0xFFFF);
        *imag = (int16_t)(((imagComp[0] << 8) | imagComp[1]) & 0xFFFF);

        return true;
    } else {
        *real = -1;
        *imag = -1;
        return false;
    }
}

/**
 * Set the power level of the AD5933.
 *
 * @param level The power level to choose. Can be on, standby, or down.
 * @return Success or failure
 */
bool AD5933::setPowerMode(byte level) {
    // Make the appropriate switch. TODO: Does no operation even do anything?
    switch (level) {
        case POWER_ON:
            return setControlMode(CTRL_NO_OPERATION);
        case POWER_STANDBY:
            return setControlMode(CTRL_STANDBY_MODE);
        case POWER_DOWN:
            return setControlMode(CTRL_POWER_DOWN_MODE);
        default:
            return false;
    }
}

/**
 * Perform a complete frequency sweep.
 *
 * @param real An array of appropriate size to hold the real data.
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5933::frequencySweep(int real[], int imag[], int n) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    if (!(setPowerMode(POWER_STANDBY) &&         // place in standby
         setControlMode(CTRL_INIT_START_FREQ) && // init start freq
         setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             return false;
         }

    // Perform the sweep. Make sure we don't exceed n.
    int i = 0;
    while ((readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Make sure we aren't exceeding the bounds of our buffer
        if (i >= n) {
            return false;
        }

        // Get the data for this frequency point and store it in the array
        if (!getComplexData(&real[i], &imag[i])) {
            return false;
        }

        // Increment the frequency and our index.
        i++;
        setControlMode(CTRL_INCREMENT_FREQ);
    }

    // Put into standby
    return setPowerMode(POWER_STANDBY);
}

/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold phase data.
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5933::calibrate(double gain[], int phase[], int ref, int n) {
    // We need arrays to hold the real and imaginary values temporarily
    int *real = new int[n];
    int *imag = new int[n];

    // Perform the frequency sweep
    if (!frequencySweep(real, imag, n)) {
        delete [] real;
        delete [] imag;
        return false;
    }

    // For each point in the sweep, calculate the gain factor and phase
    for (int i = 0; i < n; i++) {
        gain[i] = (double)(1.0/ref)/sqrt(pow(real[i], 2) + pow(imag[i], 2));
        // TODO: phase
    }

    delete [] real;
    delete [] imag;
    return true;
}

/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 * Also provides the caller with the real and imaginary data.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold the phase data
 * @param real An array of appropriate size to hold the real data
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5933::calibrate(double gain[], int phase[], int real[], int imag[],
                       int ref, int n) {
    // Perform the frequency sweep
    if (!frequencySweep(real, imag, n)) {
        return false;
    }

    // For each point in the sweep, calculate the gain factor and phase
    for (int i = 0; i < n; i++) {
        gain[i] = (double)(1.0/ref)/sqrt(pow(real[i], 2) + pow(imag[i], 2));
        // TODO: phase
    }

    return true;
}
