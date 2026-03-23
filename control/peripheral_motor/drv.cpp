#include "drv.h"

// Motor instance (90KV, pero se recomienda un 50 a 70% más)
BLDCMotor motor = BLDCMotor(NUM_POLES, MOTOR_R, MOTOR_KV);

BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  chip_address         - I2C chip address
//  bit_resolution       - resolution of the sensor
//  angle_register_msb   - angle read register msb
//  bits_used_msb        - number of used bits in msb register
MagneticSensorI2C sensor = MagneticSensorI2C(SENSOR_ADD, 12, 0x0E, 4);

void drv_setup(void)
{
  pinMode(EN_LED, OUTPUT);
  digitalWrite(EN_LED, LOW);
  pinMode(ALARM_OC_LED, OUTPUT);
  digitalWrite(ALARM_OC_LED, LOW);
  pinMode(TEMP_LED, OUTPUT);
  digitalWrite(TEMP_LED, LOW);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, LOW);
  // Alerts
  pinMode(N_FAULT, INPUT);
  pinMode(N_OCTW, INPUT);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = PWM_FREQUENCY;

  Serial.println("Driver init ");
  driver.init();
  // init driver
  // if (driver.init()) Serial.println("success!");
  //   else {
  //   Serial.println("failed!");
  //     return;
  // }
  // enable driver
  // driver.enable();
  // link the motor and the driver
  Wire.begin();
  motor.linkDriver(&driver);
  // detect I2C device (0x36 for AS5600)
  Wire.beginTransmission(SENSOR_ADD);
  byte error = Wire.endTransmission();
  Serial.println(error);
  while (error != 0)
  {
    digitalWrite(TEMP_LED, HIGH);
    delay(500);
    digitalWrite(TEMP_LED, LOW);
    delay(500);
    Wire.beginTransmission(SENSOR_ADD);
    error = Wire.endTransmission();
  }
  // This will improve the velocity estimation but it will add some lag to the velocity response.
  sensor.min_elapsed_time = 0.001; // seconds - default 0.0001s - 100us
  // initialise magnetic sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // check for I2C connection
  //  motor.velocity_index_search = 3; // rad/s
  //  Limits voltage (and therefore current) during motor alignment. Value in Volts.
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
  // choose FOC modulation
  motor.foc_modulation = FOC_MODULATION;
  // set motion control loop to be used
  motor.controller = MOTION_CONTROL;
  // velocity PI controller parameters
  motor.PID_velocity.P = PID_VEL_P;
  motor.PID_velocity.I = PID_VEL_I;
  motor.PID_velocity.D = PID_VEL_D;
  // motor.PID_velocity.output_ramp = PI_VEL_OUTPUT_RAMP;
  //  velocity low pass filtering time constant
  motor.LPF_velocity.Tf = LPF_VEL_TF;
  motor.LPF_angle.Tf = LPF_ANGLE_TF;
  // angle PID controller
  motor.P_angle.P = ANGLE_PID_P;
  motor.P_angle.I = ANGLE_PID_I; // usually only P controller is enough
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = ANGLE_PID_P_OUTPUT_RAMP; // default 1e6 rad/s^2
  // maximal voltage to be set to the motor
  motor.voltage_limit = VOLTAGE_LIMIT;
  // maximal velocity of the position control
  motor.velocity_limit = VELOCITY_LIMIT;
  // motor.PID_velocity.limit = 100;
  motor.current_limit = CURRENT_LIMIT;
  // encoder offset
  motor.sensor_offset = SENSOR_OFFSET;
  // initialize motor
  motor.init();

  // init FOC
  while (!motor.initFOC())
  {
    digitalWrite(EN_LED, HIGH);
    delay(1000);
    digitalWrite(EN_LED, LOW);
    delay(1000);
  }
  digitalWrite(EN_LED, HIGH);
  _delay(1000);
  while (!digitalRead(N_FAULT))
  {
    digitalWrite(ALARM_OC_LED, HIGH);
    delay(500);
    digitalWrite(ALARM_OC_LED, LOW);
    delay(500);
  }
}

void motor_loop(void)
{
  motor.loopFOC();
}

void motor_move(void)
{
  motor.move();
}

void update_target(float target)
{
  motor.target = target;
}