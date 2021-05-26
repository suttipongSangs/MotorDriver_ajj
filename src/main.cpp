#include <Arduino.h>
#include <Wire.h>

uint16_t DriveSpeed = 50;

#define INA1 0
#define INB1 1
#define PWM1 5

#define INA2 4
#define INB2 7
#define PWM2 6

#define MODE0 A1
#define MODE1 A2
#define MODE2 A3

uint8_t current_pwm1 = 0;
uint8_t current_pwm2 = 0;

#define terminatGAP 1
inline bool inRange(int t, int v, int g)
{
  return t - g <= v && v <= t + g;
}

#define COUNTER_PIN1 2
#define COUNTER_PIN2 3

enum FN
{
  stop,
  move,
  setTarget1,
  setTarget2,
  setDir1,
  setDir2,
  setSpeed1,
  setSpeed2,
  HALL = 255,
};

volatile int counter1 = 0;
volatile int counter2 = 0;

typedef struct OpData
{
  bool motorActive1 = false;
  bool motorActive2 = false;
  int duration;
  int target1;
  int target2;
  bool dir1;
  bool dir2;
  int power;
  uint16_t speed1;
  uint16_t speed2;
  uint8_t current_excute;
} OperateData;

OperateData opData;

typedef struct I2C_handle
{
  uint8_t buffer[10];
  uint8_t size = 0;
  uint8_t address;
  uint8_t Excute_num = HALL;
  uint16_t unDecodeParam;

} I2C_data;

I2C_data iicData;

void setDir1()
{
  digitalWrite(INA1, opData.dir1);
  digitalWrite(INB1, !opData.dir1);
}

void setDir2()
{
  digitalWrite(INA2, opData.dir2);
  digitalWrite(INB2, !opData.dir2);
}
void stopMotor1()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
}
void stopMotor2()
{
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
}

int PD(int target, int current)
{
  return 1;
}

uint16_t setMinMax(uint16_t min, uint16_t max, uint16_t val)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

// void unControlDrive()
// {

//   if (opData.dir)
//   {
//     //drive
//   }
//   else
//   {
//     //drive
//   }
// }

void setPwm(int val1, int val2)
{
  analogWrite(PWM1, val1);
  analogWrite(PWM2, val2);
}

void controlSpeed(int sp1, int sp2)
{
  // do pd(pid) to set the pwm
  uint8_t update1 = PD(opData.speed1, sp1);
  uint8_t update2 = PD(opData.speed2, sp2);

  uint16_t new_val1 = update1 + current_pwm1;
  uint16_t new_val2 = update2 + current_pwm2;

  current_pwm1 = setMinMax(0, 255, new_val1);
  current_pwm2 = setMinMax(0, 255, new_val2);

  setPwm(current_pwm1, current_pwm2);
}

float cal_speed(int prev, int now, int timeDiv)
{
  float time_ = (float)timeDiv / 1000.f;
  return (float)(now - prev) / time_;
}

void targetDrive()
{
  opData.speed1 = DriveSpeed;
  opData.speed2 = DriveSpeed;
  // uint8_t dar = opData.dir;
  int now_counter[2] = {counter1, counter2};
  int prev_counter[2] = {counter1, counter2};

  int prev_time = millis();
  while (!(opData.speed1 == 0 && opData.speed2 == 0))
  {

    if (iicData.Excute_num == FN::stop)
    {
      opData.current_excute = FN::HALL;
      return;
    }

    int now = millis();
    // get sample into local var to avoid counter change while compute
    now_counter[0] = counter1;
    now_counter[1] = counter2;

    float speed1 = cal_speed(prev_counter[0], now_counter[0], now - prev_time);
    float speed2 = cal_speed(prev_counter[1], now_counter[1], now - prev_time);

    // try to match speed if not in range of the target
    if (!inRange(opData.target1, now_counter[0], 5))
    {
      opData.speed1 = 0;
    }
    if (!inRange(opData.target2, now_counter[1], 5))
    {
      opData.speed2 = 0;
    }

    controlSpeed(speed1, speed2);

    prev_time = now;
    prev_counter[0] = now_counter[0];
    prev_counter[1] = now_counter[1];
  }

  stopMotor1();
  stopMotor2();

  iicData.Excute_num = FN::HALL;
}

void counter_func1()
{
  if (opData.dir1)
    counter1 += 1;
  else
    counter1 -= 1;
}

void counter_func2()
{
  if (opData.dir2)
    counter2 += 1;
  else
    counter2 -= 1;
}

void receiveEvent(int howMany)
{

  String tmp = Wire.readStringUntil((char)0);
  uint8_t com = (uint8_t)tmp[0];

  iicData.Excute_num = com;

  tmp = tmp.substring(1);

  iicData.unDecodeParam = atoi(tmp.c_str());

  switch (iicData.Excute_num)
  {
  case FN::setTarget1:
    opData.target1 = iicData.unDecodeParam;
    break;
  case FN::setTarget2:
    opData.target2 = iicData.unDecodeParam;
    break;
  case FN::setDir1:
    opData.dir1 = iicData.unDecodeParam;
    break;
  case FN::setDir2:
    opData.dir2 = iicData.unDecodeParam;
    break;
  case FN::setSpeed1:
    opData.speed1 = iicData.unDecodeParam;
    break;
  case FN::setSpeed2:
    opData.speed2 = iicData.unDecodeParam;
    break;
  case FN::stop:
    opData.current_excute = FN::stop;
    break;
  case FN::move:
    opData.current_excute = FN::move;
    break;
  default:
    break;
  }
}

uint8_t getMode()
{
  uint8_t mode = 0;
  mode |= digitalRead(MODE0);
  mode |= digitalRead(MODE1) << 1;
  mode |= digitalRead(MODE2) << 2;

  return mode;
}

void setup()
{
  pinMode(MODE0, INPUT_PULLUP);
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);

  pinMode(COUNTER_PIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN1), counter_func1, RISING);

  pinMode(COUNTER_PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN2), counter_func1, RISING);

  iicData.address = getMode();

  Wire.begin(iicData.address);
  Wire.onReceive(receiveEvent);
}

void loop()
{
  if(opData.current_excute != FN::HALL){
    if(opData.current_excute == FN::move){
      targetDrive();
    }
  }

}