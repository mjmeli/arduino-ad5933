# AD5933 Arduino Library

This is a simple library for using the AD5933 impedance convert system with an Arduino compatible device.

## AD5933
The AD5933 is developed by Analog Devices. [From the AD5933 page](http://www.analog.com/en/products/rf-microwave/direct-digital-synthesis-modulators/ad5933.html#product-overview):

>The AD5933 is a high precision impedance converter system solution that combines an on-board frequency generator with a 12-bit, 1 MSPS, analog-to-digital converter (ADC). The frequency generator allows an external complex impedance to be excited with a known frequency. The response signal from the impedance is sampled by the on-board ADC and a discrete Fourier transform (DFT) is processed by an on-board DSP engine. The DFT algorithm returns a real (R) and imaginary (I) data-word at each output frequency.

>Once calibrated, the magnitude of the impedance and relative phase of the impedance at each frequency point along the sweep is easily calculated. This is done off chip using the real and imaginary register contents, which can be read from the serial I2C interface.

In other words, the AD5933 lets you measure the complex impedance of something.

### Hardware Setup
Proper setup of the AD5933 is essential for obtaining a good measurement. The excitation voltage, unknown impedance, feedback resistor, and gain should result in the greatest possible voltage at the ADC *without saturating the ADC*. The saturation voltage should be equal to your VDD input, normally 3.0 or 3.3V. The voltage at the ADC is found by: (Voltage at ADC)=(Excitation voltage)(Feedback resistor)(PGA gain)/(unknown impedance).

### Limitations
Some users have had trouble with the low frequency (sub 8kHz) performance of the AD5933. A possible cause of this was explained in a paper published by L. Matsiev in Electronics:
https://www.mdpi.com/2079-9292/4/1/1 

In short, the mathematical assumptions of the approach used by the AD5933 are only met when an integer number of waveforms are sampled. At higher frequencies the error introduced disappears in the noise, but at lower frequencies it can be a problem. L. Matsiev models this error and mathematically removes it. Another approach is to attach a lower frequency external clock, which will change the sample rate.

Further information can be found in chapter 4 of the thesis "Adaptation of Electro-Mechanical Impedance Structural Health Monitoring for Use in Spacecraft" by David C. Hunter. This thesis is supposed to be made available on Proquest and through the New Mexico Tech library. But, as of this writing (Sep 2018), there is no sign of it.

## Library

### Compatibility
This library *should* be compatible with any Arduino compatible device, albeit perhaps with some changes. It was developed and tested with an RFduino, but I do not see a reason why it would not work with a regular Arduino. Futher development was done on an Adafruit Feather Adalogger M0. To allow improve memory management, and allow for increased compatibility, getComplexData16(...) and frequencySweep16(...) functions were added to control word size across platforms.

### Missing Features
While the library is enough to get impedance readings, I must admit that it is somewhat incomplete. The following features are yet to be implemented (and likely will not be implemented by me):

* Phase support in the calibration functions. The phase can be correctly calculating by implementing the phase correction from the raw data as describe in the AD5933 datasheet, but it is not currently supported in this library.
* A floating point frequency code converter; freqToCode is currently integer only.

### Installation
Simply move the entire folder `arduino-ad5933` to your `Arduino/libraries` folder, usually in your home directory or documents folder.

### Usage

#### Example
Perhaps the easiest way to see how to use the library is to look at the example code in the `examples` directory. Once you install the library, you can easily open this code in the Arduino editor by going to `File > Examples > arduino-ad5933 > ad5933-test`.

This example will show you how to initially setup the AD5933 and run frequency sweeps. There are two methods for doing a frequency sweep. The first is by using the `frequencySweep()` function. This function is very easy to use, but you have to wait until the entire sweep is complete before handling data in bulk and this requires more memory, especially for large sweeps. The other method is by manipulating the AD5933 at the I2C level directly, which is slightly more complex, but allows you to handle data immediately after a reading and has significantly lower memory overhead.

#### Brief Overview
There are an assortment of functions in `AD5933.h` that can be used with numerous constants. Each one of the functions are static, so be sure to include `AD5933::` in front. Here I cover a few of the main ones.

NOTE: Many of these functions return booleans indicating their success. This may be useful for debugging.

To reset the board: `AD5933::reset();`

To enable on-board temperature measurement: `AD5933::enableTemperature()`

To get a temperature reading: `double temp = AD5933::getTemperature()`

To set the clock source to internal: `AD5933::setInternalClock(true)` OR `AD5933::setClockSource(CTRL_CLOCK_INTERNAL)`

To set the clock source to external: `AD5933::setClockSource(CTRL_CLOCK_EXTERNAL)`

To set the frequency sweep start frequency: `AD5933::setStartFrequency(#)`

To set the frequency sweep increment frequency: `AD5933::setIncrementFrequency(#)`

To set the frequency sweep number of increments: `AD5933::setNumberIncrements(#)`

To set the PGA gain: `AD5933::setPGAGain(PGA_GAIN_X1/PGA_GAIN_X5)`

To set the power mode: `AD5933:setPowerMode(POWER_ON/POWER_DOWN/POWER_STANDBY)`

To perform a calibration sweep, which computes gain factor for each frequency step based on a known reference resistance (the results are stored in the gainFactor array, so make sure this is large enough): `AD5933::calibration(double[] gainFactor, double[] phase, int referenceResistor, int numIncrements)`

To perform an entire frequency sweep (the results are stored in the real and imaginary arrays): `AD5933::frequencySweep(int[] real, int[] imag, int numIncrements)`

To read a register: `byte res = AD5933::readRegister(/* register address */)`

That is sufficient for most things. Check out the header file for more functions and constants. Also check out the example code for better illustrations of usage.

## Reference
A lot of code was reference from [this GitHub repo](https://github.com/WuMRC/drive). I made a fair amount of modifications, but the repo can also probably be used for additional sample code if needed.
