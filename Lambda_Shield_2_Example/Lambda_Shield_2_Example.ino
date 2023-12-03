/*
    Bosch CJ125 based wideband oxygen sensor controller.

    Originally forked from https://github.com/Bylund/Lambda-Shield-2-Example

    Version history:
    2023-01-28        v1.0.6        Added button and rolling average battery voltage. Hardware v1.3 compatability
    2022-04-23        v1.0.5        Added 4x7 digit LED display in place of status LEDs. Compatibility with hardware v1.1.
    2022-03-18        v1.0.4        Corrected analogue output gradient and intercept terms.
    2021-02-09        v1.0.3        Removed unused oxygen lookup table and function. Updated analog output function. Added calibration output voltages during warm-up phase.
    2021-02-07        v1.0.2        Corrected heater control routines during setup. Updated status LED sequences.
    2020-08-16        v1.0.1        Updated error status handling.
    2020-08-14        v1.0.0        Changed pinout to suit ATTin1614. Removed serial output debugging & changed analog output signal to suit Speeduino. Removed power loss reset due to single supply.
*/

// Define included headers.
#include <SPI.h>
#include <elapsedMillis.h>
#include <TM1637TinyDisplay.h>
#include <Bounce2.h>

// Display module connection pins (Digital Pins)
#define CLK 4
#define DIO 5

TM1637TinyDisplay LEDdisplay(CLK, DIO);

// Define CJ125 registers used.
#define CJ125_IDENT_REG_REQUEST 0x4800         /* Identify request, gives revision of the chip. */
#define CJ125_DIAG_REG_REQUEST 0x7800          /* Dignostic request, gives the current status. */
#define CJ125_INIT_REG1_REQUEST 0x6C00         /* Requests the first init register. */
#define CJ125_INIT_REG2_REQUEST 0x7E00         /* Requests the second init register. */
#define CJ125_INIT_REG1_MODE_CALIBRATE 0x569D  /* Sets the first init register in calibration mode. */
#define CJ125_INIT_REG1_MODE_NORMAL_V8 0x5688  /* Sets the first init register in operation mode. V=8 amplification. */
#define CJ125_INIT_REG1_MODE_NORMAL_V17 0x5689 /* Sets the first init register in operation mode. V=17 amplification. */
#define CJ125_DIAG_REG_STATUS_OK 0x28FF        /* The response of the diagnostic register when everything is ok. */
#define CJ125_DIAG_REG_STATUS_NOPOWER 0x2855   /* The response of the diagnostic register when power is low. */
#define CJ125_DIAG_REG_STATUS_NOSENSOR 0x287F  /* The response of the diagnostic register when no sensor is connected. */
#define CJ125_INIT_REG1_STATUS_0 0x2888        /* The response of the init register when V=8 amplification is in use. */
#define CJ125_INIT_REG1_STATUS_1 0x2889        /* The response of the init register when V=17 amplification is in use. */

// Define pin assignments (ATTiny1614)
#define CJ125_NSS_PIN 0       /* Pin used for chip select in SPI communication. */
//#define LED_STATUS_POWER 4    /* Pin used for power the status LED, indicating we have power. */
//#define LED_STATUS_HEATER 5   /* Pin used for the heater status LED, indicating heater activity. */
#define HEATER_OUTPUT_PIN 1  /* Pin used for the PWM output to the heater circuit. */
#define ANALOG_OUTPUT_PIN 2   /* Pin used for DAC output 0 to 4.3V. */
#define UB_ANALOG_INPUT_PIN 3 /* Analog input for power supply.*/
#define UR_ANALOG_INPUT_PIN 7 /* Analog input for temperature.*/
#define UA_ANALOG_INPUT_PIN 6 /* Analog input for lambda.*/
#define BUTTON_INPUT_PIN 11 /* Button input pin */

// Define adjustable parameters.
#define UBAT_MIN 150 /* Minimum voltage (ADC value) on Ubat to operate */
elapsedMillis statusUpdate;
Bounce b = Bounce(); // Instantiate a Bounce object

// Global variables.
int adcValue_UA = 0;                                /* ADC value read from the CJ125 UA output pin */
int adcValue_UR = 0;                                /* ADC value read from the CJ125 UR output pin */
int adcValue_UB = 0;                                /* ADC value read from the voltage divider caluclating Ubat */
int adcValue_UA_Optimal = 0;                        /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */
int adcValue_UR_Optimal = 0;                        /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
int HeaterOutput = 0;                               /* Current PWM output value (0-255) of the heater output pin */
int CJ125_Status = 0;                               /* Latest stored DIAG registry response from the CJ125 */
int brightness = 0;                                 /* Heater/status LED brighteness */
int fadeAmount = 5;                                 /* Heater/status LED fade ammount */
int updateInterval = 50;                            /* Update interval for status LED timing */
bool toggle = false;                                /* Status update toggle */
bool faulted = false;                               /* Fault was raised */
int disp = 0;                                       /* Display counter */

// PID regulation variables.
int dState;              /* Last position input. */
int iState;              /* Integrator state. */
const int iMax = 250;    /* Maximum allowable integrator state. */
const int iMin = -250;   /* Minimum allowable integrator state. */
const float pGain = 120; /* Proportional gain. Default = 120*/
const float iGain = 0.8; /* Integral gain. Default = 0.8*/
const float dGain = 10;  /* Derivative gain. Default = 10*/

// Lambda Conversion Lookup Table. (ADC 39-791).
const PROGMEM float Lambda_Conversion[753] {
  0.750, 0.751, 0.752, 0.752, 0.753, 0.754, 0.755, 0.755, 0.756, 0.757, 0.758, 0.758, 0.759, 0.760, 0.761, 0.761, 0.762, 0.763, 0.764, 0.764,
  0.765, 0.766, 0.766, 0.767, 0.768, 0.769, 0.769, 0.770, 0.771, 0.772, 0.772, 0.773, 0.774, 0.774, 0.775, 0.776, 0.777, 0.777, 0.778, 0.779,
  0.780, 0.780, 0.781, 0.782, 0.782, 0.783, 0.784, 0.785, 0.785, 0.786, 0.787, 0.787, 0.788, 0.789, 0.790, 0.790, 0.791, 0.792, 0.793, 0.793,
  0.794, 0.795, 0.796, 0.796, 0.797, 0.798, 0.799, 0.799, 0.800, 0.801, 0.802, 0.802, 0.803, 0.804, 0.805, 0.805, 0.806, 0.807, 0.808, 0.808,
  0.809, 0.810, 0.811, 0.811, 0.812, 0.813, 0.814, 0.815, 0.815, 0.816, 0.817, 0.818, 0.819, 0.820, 0.820, 0.821, 0.822, 0.823, 0.824, 0.825,
  0.825, 0.826, 0.827, 0.828, 0.829, 0.830, 0.830, 0.831, 0.832, 0.833, 0.834, 0.835, 0.836, 0.837, 0.837, 0.838, 0.839, 0.840, 0.841, 0.842,
  0.843, 0.844, 0.845, 0.846, 0.846, 0.847, 0.848, 0.849, 0.850, 0.851, 0.852, 0.853, 0.854, 0.855, 0.855, 0.856, 0.857, 0.858, 0.859, 0.860,
  0.861, 0.862, 0.863, 0.864, 0.865, 0.865, 0.866, 0.867, 0.868, 0.869, 0.870, 0.871, 0.872, 0.873, 0.874, 0.875, 0.876, 0.877, 0.878, 0.878,
  0.879, 0.880, 0.881, 0.882, 0.883, 0.884, 0.885, 0.886, 0.887, 0.888, 0.889, 0.890, 0.891, 0.892, 0.893, 0.894, 0.895, 0.896, 0.897, 0.898,
  0.899, 0.900, 0.901, 0.902, 0.903, 0.904, 0.905, 0.906, 0.907, 0.908, 0.909, 0.910, 0.911, 0.912, 0.913, 0.915, 0.916, 0.917, 0.918, 0.919,
  0.920, 0.921, 0.922, 0.923, 0.924, 0.925, 0.926, 0.927, 0.928, 0.929, 0.931, 0.932, 0.933, 0.934, 0.935, 0.936, 0.937, 0.938, 0.939, 0.940,
  0.941, 0.942, 0.944, 0.945, 0.946, 0.947, 0.948, 0.949, 0.950, 0.951, 0.952, 0.953, 0.954, 0.955, 0.957, 0.958, 0.959, 0.960, 0.961, 0.962,
  0.963, 0.965, 0.966, 0.967, 0.969, 0.970, 0.971, 0.973, 0.974, 0.976, 0.977, 0.979, 0.980, 0.982, 0.983, 0.985, 0.986, 0.987, 0.989, 0.990,
  0.991, 0.992, 0.994, 0.995, 0.996, 0.998, 0.999, 1.001, 1.003, 1.005, 1.008, 1.010, 1.012, 1.015, 1.017, 1.019, 1.022, 1.024, 1.026, 1.028,
  1.030, 1.032, 1.035, 1.037, 1.039, 1.041, 1.043, 1.045, 1.048, 1.050, 1.052, 1.055, 1.057, 1.060, 1.062, 1.064, 1.067, 1.069, 1.072, 1.075,
  1.077, 1.080, 1.082, 1.085, 1.087, 1.090, 1.092, 1.095, 1.098, 1.100, 1.102, 1.105, 1.107, 1.110, 1.112, 1.115, 1.117, 1.120, 1.122, 1.124,
  1.127, 1.129, 1.132, 1.135, 1.137, 1.140, 1.142, 1.145, 1.148, 1.151, 1.153, 1.156, 1.159, 1.162, 1.165, 1.167, 1.170, 1.173, 1.176, 1.179,
  1.182, 1.185, 1.188, 1.191, 1.194, 1.197, 1.200, 1.203, 1.206, 1.209, 1.212, 1.215, 1.218, 1.221, 1.224, 1.227, 1.230, 1.234, 1.237, 1.240,
  1.243, 1.246, 1.250, 1.253, 1.256, 1.259, 1.262, 1.266, 1.269, 1.272, 1.276, 1.279, 1.282, 1.286, 1.289, 1.292, 1.296, 1.299, 1.303, 1.306,
  1.310, 1.313, 1.317, 1.320, 1.324, 1.327, 1.331, 1.334, 1.338, 1.342, 1.345, 1.349, 1.352, 1.356, 1.360, 1.364, 1.367, 1.371, 1.375, 1.379,
  1.382, 1.386, 1.390, 1.394, 1.398, 1.401, 1.405, 1.409, 1.413, 1.417, 1.421, 1.425, 1.429, 1.433, 1.437, 1.441, 1.445, 1.449, 1.453, 1.457,
  1.462, 1.466, 1.470, 1.474, 1.478, 1.483, 1.487, 1.491, 1.495, 1.500, 1.504, 1.508, 1.513, 1.517, 1.522, 1.526, 1.531, 1.535, 1.540, 1.544,
  1.549, 1.554, 1.558, 1.563, 1.568, 1.572, 1.577, 1.582, 1.587, 1.592, 1.597, 1.601, 1.606, 1.611, 1.616, 1.621, 1.627, 1.632, 1.637, 1.642,
  1.647, 1.652, 1.658, 1.663, 1.668, 1.674, 1.679, 1.684, 1.690, 1.695, 1.701, 1.707, 1.712, 1.718, 1.724, 1.729, 1.735, 1.741, 1.747, 1.753,
  1.759, 1.764, 1.770, 1.776, 1.783, 1.789, 1.795, 1.801, 1.807, 1.813, 1.820, 1.826, 1.832, 1.839, 1.845, 1.852, 1.858, 1.865, 1.872, 1.878,
  1.885, 1.892, 1.898, 1.905, 1.912, 1.919, 1.926, 1.933, 1.940, 1.947, 1.954, 1.961, 1.968, 1.975, 1.983, 1.990, 1.997, 2.005, 2.012, 2.020,
  2.027, 2.035, 2.042, 2.050, 2.058, 2.065, 2.073, 2.081, 2.089, 2.097, 2.105, 2.113, 2.121, 2.129, 2.137, 2.145, 2.154, 2.162, 2.171, 2.179,
  2.188, 2.196, 2.205, 2.214, 2.222, 2.231, 2.240, 2.249, 2.258, 2.268, 2.277, 2.286, 2.295, 2.305, 2.314, 2.324, 2.333, 2.343, 2.353, 2.363,
  2.373, 2.383, 2.393, 2.403, 2.413, 2.424, 2.434, 2.444, 2.455, 2.466, 2.476, 2.487, 2.498, 2.509, 2.520, 2.532, 2.543, 2.554, 2.566, 2.577,
  2.589, 2.601, 2.613, 2.625, 2.637, 2.649, 2.662, 2.674, 2.687, 2.699, 2.712, 2.725, 2.738, 2.751, 2.764, 2.778, 2.791, 2.805, 2.819, 2.833,
  2.847, 2.861, 2.875, 2.890, 2.904, 2.919, 2.934, 2.949, 2.964, 2.979, 2.995, 3.010, 3.026, 3.042, 3.058, 3.074, 3.091, 3.107, 3.124, 3.141,
  3.158, 3.175, 3.192, 3.209, 3.227, 3.245, 3.263, 3.281, 3.299, 3.318, 3.337, 3.355, 3.374, 3.394, 3.413, 3.433, 3.452, 3.472, 3.492, 3.513,
  3.533, 3.554, 3.575, 3.597, 3.618, 3.640, 3.662, 3.684, 3.707, 3.730, 3.753, 3.776, 3.800, 3.824, 3.849, 3.873, 3.898, 3.924, 3.950, 3.976,
  4.002, 4.029, 4.056, 4.084, 4.112, 4.140, 4.169, 4.198, 4.228, 4.258, 4.288, 4.319, 4.350, 4.382, 4.414, 4.447, 4.480, 4.514, 4.548, 4.583,
  4.618, 4.654, 4.690, 4.726, 4.764, 4.801, 4.840, 4.879, 4.918, 4.958, 4.999, 5.040, 5.082, 5.124, 5.167, 5.211, 5.255, 5.299, 5.345, 5.391,
  5.438, 5.485, 5.533, 5.582, 5.632, 5.683, 5.735, 5.788, 5.841, 5.896, 5.953, 6.010, 6.069, 6.129, 6.190, 6.253, 6.318, 6.384, 6.452, 6.521,
  6.592, 6.665, 6.740, 6.817, 6.896, 6.976, 7.059, 7.144, 7.231, 7.320, 7.412, 7.506, 7.602, 7.701, 7.803, 7.906, 8.013, 8.122, 8.234, 8.349,
  8.466, 8.587, 8.710, 8.837, 8.966, 9.099, 9.235, 9.374, 9.516, 9.662, 9.811, 9.963, 10.119
};

const PROGMEM float Oxygen_Conversion[256] {
  11.03, 11.07, 11.10, 11.14, 11.18, 11.22, 11.25, 11.29, 11.33, 11.37, 11.40, 11.44, 11.48, 11.52, 11.55, 11.59, 11.63, 11.67, 11.70, 11.74,
  11.78, 11.82, 11.85, 11.89, 11.93, 11.97, 12.00, 12.04, 12.08, 12.12, 12.15, 12.19, 12.23, 12.27, 12.30, 12.34, 12.38, 12.42, 12.45, 12.49,
  12.53, 12.57, 12.60, 12.64, 12.68, 12.72, 12.75, 12.79, 12.83, 12.87, 12.90, 12.94, 12.98, 13.01, 13.05, 13.09, 13.13, 13.16, 13.20, 13.24,
  13.28, 13.31, 13.35, 13.39, 13.43, 13.46, 13.50, 13.54, 13.58, 13.61, 13.65, 13.69, 13.73, 13.76, 13.80, 13.84, 13.88, 13.91, 13.95, 13.99,
  14.03, 14.06, 14.10, 14.14, 14.18, 14.21, 14.25, 14.29, 14.33, 14.36, 14.40, 14.44, 14.48, 14.51, 14.55, 14.59, 14.63, 14.66, 14.70, 14.74,
  14.78, 14.81, 14.85, 14.89, 14.92, 14.96, 15.00, 15.04, 15.07, 15.11, 15.15, 15.19, 15.22, 15.26, 15.30, 15.34, 15.37, 15.41, 15.45, 15.49,
  15.52, 15.56, 15.60, 15.64, 15.67, 15.71, 15.75, 15.79, 15.82, 15.86, 15.90, 15.94, 15.97, 16.01, 16.05, 16.09, 16.12, 16.16, 16.20, 16.24,
  16.27, 16.31, 16.35, 16.39, 16.42, 16.46, 16.50, 16.54, 16.57, 16.61, 16.65, 16.69, 16.72, 16.76, 16.80, 16.83, 16.87, 16.91, 16.95, 16.98,
  17.02, 17.06, 17.10, 17.13, 17.17, 17.21, 17.25, 17.28, 17.32, 17.36, 17.40, 17.43, 17.47, 17.51, 17.55, 17.58, 17.62, 17.66, 17.70, 17.73,
  17.77, 17.81, 17.85, 17.88, 17.92, 17.96, 18.00, 18.03, 18.07, 18.11, 18.15, 18.18, 18.22, 18.26, 18.30, 18.33, 18.37, 18.41, 18.45, 18.48,
  18.52, 18.56, 18.60, 18.63, 18.67, 18.71, 18.74, 18.78, 18.82, 18.86, 18.89, 18.93, 18.97, 19.01, 19.04, 19.08, 19.12, 19.16, 19.19, 19.23,
  19.27, 19.31, 19.34, 19.38, 19.42, 19.46, 19.49, 19.53, 19.57, 19.61, 19.64, 19.68, 19.72, 19.76, 19.79, 19.83, 19.87, 19.91, 19.94, 19.98,
  20.02, 20.06, 20.09, 20.13, 20.17, 20.21, 20.24, 20.28, 20.32, 20.36, 20.39, 20.43, 20.47, 20.51, 20.54, 20.58
};

const uint8_t PWR[1][4] = {
  { 0x1c, 0x86, 0xbf, 0x7d }
};

const uint8_t CAL_1[3][4] = {
  { 0x39, 0x5c, 0x54, 0x5e },  // Frame 0
  { 0x77, 0x71, 0x50, 0x48 },  // Frame 1
  { 0x06, 0xe6, 0x07, 0x00 },  // Frame 2
};

const uint8_t CAL_2[3][4] = {
  { 0x76, 0x79, 0x77, 0x78 },  // Frame 0
  { 0x77, 0x71, 0x50, 0x48 },  // Frame 1
  { 0x5b, 0xbf, 0x6d, 0x7f },  // Frame 2
};

void UpdateLEDStatus()
{
  // CJ125 error status handling.
  if (statusUpdate > updateInterval)
  {
    statusUpdate = 0;
    switch (CJ125_Status)
    {
      case CJ125_DIAG_REG_STATUS_OK:
        // Clear fault flag. Clear display
        if (faulted)
        {
          faulted = false;
          LEDdisplay.clear();
        }
        updateInterval = 200;
        break;

      case CJ125_DIAG_REG_STATUS_NOPOWER:
        // No power.
        faulted = true;
        LEDdisplay.showString("No PWR");
        updateInterval = 200;
        break;

      case CJ125_DIAG_REG_STATUS_NOSENSOR:
        // No sensor.
        faulted = true;
        LEDdisplay.showString("No sensor");
        updateInterval = 200;
        break;

      default:
        // Generic error.
        faulted = true;
        LEDdisplay.showString("Fault");
        updateInterval = 200;
        break;
    }
  }
}

// Function for transfering SPI data to the CJ125.
uint16_t COM_SPI(uint16_t TX_data)
{
  // Configure SPI for CJ125 controller.
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  // Set chip select pin low, chip in use.
  digitalWrite(CJ125_NSS_PIN, LOW);

  // Transmit request.
  uint16_t Response = SPI.transfer16(TX_data);

  // Set chip select pin high, chip not in use.
  digitalWrite(CJ125_NSS_PIN, HIGH);

  return Response;
}

// Temperature regulating software (PID).
int Heater_PID_Control(int input)
{
  // Calculate error term.
  int error = adcValue_UR_Optimal - input;

  // Set current position.
  int position = input;

  // Calculate proportional term.
  float pTerm = -pGain * error;

  // Calculate the integral state with appropriate limiting.
  iState += error;

  if (iState > iMax)
  {
    iState = iMax;
  }

  if (iState < iMin)
  {
    iState = iMin;
  }

  // Calculate the integral term.
  float iTerm = -iGain * iState;

  // Calculate the derivative term.
  float dTerm = -dGain * (dState - position);
  dState = position;

  // Calculate regulation (PI).
  int RegulationOutput = pTerm + iTerm + dTerm;

  // Set maximum heater output (full power).
  if (RegulationOutput > 255)
  {
    RegulationOutput = 255;
  }

  // Set minimum heater value (cooling).
  if (RegulationOutput < 0.0)
  {
    RegulationOutput = 0;
  }

  //Return calculated PWM output.
  return RegulationOutput;
}

// 0-4.3V analog output
void UpdateAnalogOutput()
{
  // Local constants.
  const int maximumOutput = 255; /* 4.3V */
  const int minimumOutput = 0;   /* 0V */
  const float m = 27.217;  /* Gradient term */
  const float c = 20.2413; /* Intercept term */

  // Local variables.
  uint8_t analogOutput = 0;

  // Calculate ADC value from Lambda
  float lambdaADC = (m * Lookup_Lambda(adcValue_UA)) - c;
  analogOutput = lambdaADC + 0.5; /* Float to integer is truncated, adding 0.5 effectively rounds the float */

  // Limit check output value
  if (analogOutput > maximumOutput)
  {
    analogOutput = maximumOutput;
  }
  if (analogOutput < minimumOutput)
  {
    analogOutput = minimumOutput;
  }

  // Update the display - using AFR
  if (!faulted)
  {
    switch (disp) {
      case 0:
        float afr = pgm_read_float_near(Oxygen_Conversion + analogOutput);
        int afri = (afr * 100);

        LEDdisplay.showNumberDec(afri, 64, false, 4, 0);
        break;
      case 1:
        LEDdisplay.showNumberDec(Lookup_Lambda(adcValue_UA) * 100, 64, false, 4, 0);
        break;
    }
  }

  // Set analog output.
  analogWrite(ANALOG_OUTPUT_PIN, analogOutput);
}

// Lookup Lambda Value.
float Lookup_Lambda(int Input_ADC)
{
  // Declare and set default return value.
  float LAMBDA_VALUE = 0;

  // Validate ADC range for lookup table.
  if (Input_ADC >= 39 && Input_ADC <= 791)
  {
    LAMBDA_VALUE = pgm_read_float_near(Lambda_Conversion + (Input_ADC - 39));
  }

  if (Input_ADC > 791)
  {
    LAMBDA_VALUE = 10.119;
  }

  if (Input_ADC < 39)
  {
    LAMBDA_VALUE = 0.750;
  }

  // Return value.
  return LAMBDA_VALUE;
}

// Function to set up device for operation.
void setup()
{
  // Set up SPI.
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  // Set up digital output pins.
  pinMode(CJ125_NSS_PIN, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);

  // Set initial values.
  digitalWrite(CJ125_NSS_PIN, HIGH);
  analogWrite(HEATER_OUTPUT_PIN, 0); /* PWM is initially off. */
  analogWrite(ANALOG_OUTPUT_PIN, 0); /* PWM is initially off. */
  DACReference(0x3); /* DAC reference set to 4.3V (maximum for the ATTiny1614) */

  // Start of operation. (Power LED on).
  LEDdisplay.setBrightness(0x0f);
  LEDdisplay.setScrolldelay(300);
  LEDdisplay.showAnimation_P(PWR, FRAMES(PWR), TIME_MS(2750));

  // Set up button
  b.attach(BUTTON_INPUT_PIN, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  b.interval(25); // Use a debounce interval of 25 milliseconds

  // Start main function.
  start();
}

void start()
{
  // Wait until everything is ready.
  while (adcValue_UB < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK)
  {
    // Read CJ125 diagnostic register from SPI.
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

    // Read input voltage.
    adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

    UpdateLEDStatus();
  }

  // Start of operation. (Power LED on, heater LED off).

  // Set CJ125 in calibration mode.
  COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);

  // Let values settle.
  delay(500);

  // Store optimal values before leaving calibration mode.
  adcValue_UA_Optimal = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR_Optimal = analogRead(UR_ANALOG_INPUT_PIN);

  // Update analog output, display the optimal value.
  adcValue_UA = adcValue_UA_Optimal;
  UpdateAnalogOutput();

  // Set CJ125 in normal operation mode.
  //COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  /* V=0 */
  COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17); /* V=1 */

  /* Heat up sensor. This is described in detail in the datasheet of the LSU 4.9 sensor with a
     condensation phase and a ramp up phase before going in to PID control. */
  int ledBlink = 255;

  // Calculate supply voltage.
  float SupplyVoltage = map(adcValue_UB, 0, 1023, 0, 15);

  // Condensation phase, 2V for 5s.
  int CondensationPWM = (2 / SupplyVoltage) * 255;
  analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

  // First calibration voltage. ADC 98, 1.65V, 1.0 Lambda
  int firstCal = 98;
  analogWrite(ANALOG_OUTPUT_PIN, firstCal);

  int t = 0;
  while (t < 5 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN)
  {
    t += 1;

    // Condensation phase
    //LEDdisplay.showString("COND CAL=1.92V");
    LEDdisplay.showAnimation_P(CAL_1, FRAMES(CAL_1), TIME_MS(750));
    delay(1000); // 5 seconds total
  }

  // Second calibration voltage. ADC 255, 5.0V, 1.4 Lambda
  int secondCal = 255;
  analogWrite(ANALOG_OUTPUT_PIN, secondCal);

  // Ramp up phase, +0.4V/s until 100% PWM from 8.5V.
  float UHeater = 8.5;
  while (UHeater < 13.0 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN)
  {
    // Set heater output during ramp up.
    CondensationPWM = (UHeater / SupplyVoltage) * 255;

    if (CondensationPWM > 255)
    {
      CondensationPWM = 255; /* If supply voltage is less than 13V, maximum is 100% PWM */
    }

    analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

    // Increment Voltage.
    UHeater += 0.4;

    // Heat-up phase
    LEDdisplay.showAnimation_P(CAL_2, FRAMES(CAL_2), TIME_MS(750));
    //LEDdisplay.showString("HEAT CAL=L1.4");

    delay(1000); //0.4V/s
  }

  // Heat until temperature optimum is reached or exceeded (lower value is warmer).
  while (analogRead(UR_ANALOG_INPUT_PIN) > adcValue_UR_Optimal && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN)
  {
    // Display heating phase
    LEDdisplay.showString("PID");
  }

  // Heating phase finished, hand over to PID-control. Turn on LEDs, turn off heater.
  analogWrite(HEATER_OUTPUT_PIN, 0);
  LEDdisplay.showString("Ready");
}

// Infinite loop.
void loop()
{
  // Update CJ125 diagnostic register from SPI.
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

  // Proceed if CJ125 status is OK, otherwise turn off the heater
  if (CJ125_Status == CJ125_DIAG_REG_STATUS_OK)
  {
    // Update analog inputs.
    adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
    adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
    adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

    // Adjust PWM output by calculated PID regulation.
    if (adcValue_UR < 500 || adcValue_UR_Optimal != 0 || adcValue_UB > UBAT_MIN)
    {
      // Calculate and set new heater output.
      HeaterOutput = Heater_PID_Control(adcValue_UR);
      analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    }
    else
    {
      // Turn off heater if we are not in PID control.
      HeaterOutput = 0;
      analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
    }

    // Update analog output.
    UpdateAnalogOutput();
  }
  else
  {
    analogWrite(HEATER_OUTPUT_PIN, 0);
  }

  // Update LED status - ignore on first loop
  if (toggle)
  {
    UpdateLEDStatus();
  }
  toggle = true;

  b.update(); // Update the Bounce instance

  if ( b.fell() ) {  // Call code if button transitions from HIGH to LOW
    disp += 1;
    LEDdisplay.clear();
  }
  if (disp > 1)
  {
    disp = 0;
  }
}

float movingAverage(float value) {
  const byte nvalues = 128;             // Moving average window size

  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static float sum = 0;               // Rolling sum
  static float values[nvalues];

  sum += value;

  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    sum -= values[current];

  values[current] = value;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum / cvalues;
}
