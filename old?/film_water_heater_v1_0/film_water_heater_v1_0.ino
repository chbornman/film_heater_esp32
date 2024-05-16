#include <max6675.h>

const float desired_temp = 102;
const float deadband = 0.3;
const float bias = 1.5;
const int coil_pin = 2;

//tc_L
const int thermoDO_L = 4;
const int thermoCS_L = 5;
const int thermoCLK_L = 6;
MAX6675 tc_L(thermoCLK_L, thermoCS_L, thermoDO_L);

//tc_R
const int thermoDO_R = 8;
const int thermoCS_R = 9;
const int thermoCLK_R = 10;
MAX6675 tc_R(thermoCLK_R, thermoCS_R, thermoDO_R);

////tc_U
//const int thermoDO_U = 11;
//const int thermoCS_U = 12;
//const int thermoCLK_U = 13;
//MAX6675 tc_U(thermoCLK_U, thermoCS_U, thermoDO_U);
//
////tc_D
//const int thermoDO_D = 7;
//const int thermoCS_D = 2;
//const int thermoCLK_D = 3;
//MAX6675 tc_D(thermoCLK_D, thermoCS_D, thermoDO_D);

float temp_L = 0;
float temp_R = 0;
//float temp_U = 0;
//float temp_D = 0;

bool inRange(float temp)
{
  bool l_bool = true;
  
  if (temp <= 32 or
      temp >= 212)
      {
        l_bool = false;
      }

  return(l_bool);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Testing thermocouple");
  delay(1000);
  pinMode(coil_pin, OUTPUT);
}

void loop() {

  float temp_average = 0;
  
  temp_L = tc_L.readFahrenheit();
  temp_R = tc_R.readFahrenheit();
//  temp_U = tc_U.readCelsius();
//  temp_D = tc_D.readCelsius();

  if (!inRange(temp_L) or
      !inRange(temp_R) /*or
      !inRange(temp_U) or
      !inRange(temp_D)*/)
  {
    Serial.println("ERROR");
  }
  else
  {
    temp_average = temp_L + temp_R /*+ temp_U + temp_D*/;
    temp_average = temp_average / /*4*/ 2;

    if (temp_average < (desired_temp - deadband))
    {
      digitalWrite(coil_pin, HIGH);
      Serial.print("coil on");
    }
    else if (temp_average >= (desired_temp + deadband - bias))
    {
      digitalWrite(coil_pin, LOW);
      Serial.print("coil off");
    }
    else
    {
      Serial.print("coil in deadband");
    }

  Serial.print("\t temp_L = ");
  Serial.print(temp_L);
  Serial.print(" degF");
  Serial.print("\t temp_R = ");
  Serial.print(temp_R);
  Serial.print(" degF");
//  Serial.print("\t temp_U = ");
//  Serial.print(temp_U);
//  Serial.print(" degC");
//  Serial.print("\t temp_D = ");
//  Serial.print(temp_D);
//  Serial.print(" degC");
  Serial.print("\t AVG = ");
  Serial.print(temp_average);
  Serial.print(" degF");
  Serial.println("");

  }

  delay(2000);
}
