int sensorPin = A8;
int mPin1 = 4;
int mPin2 = 2;
int sensorValue = 0;
int target = 0;
float kp = 4;
float ki = 0.002;
float kd = 0.005;
float integral = 0;
float prev_error = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  target = analogRead(sensorPin);
  if (target >=512){target -=512;}
  else {target +=512;}
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);

  int error = target-sensorValue;

//  if (error < 6) {kd = 0.5;}
  
  integral += ki*error;
  float derivative = kd*(error-prev_error);
  float motor_control = kp*error + integral + derivative;

  if (motor_control > 254) { motor_control = 254;}
  if (motor_control < -254){motor_control = -254;}

  float time_ms = millis();
  
  if (time_ms > 2000) {analogWrite(mPin1, 0); analogWrite(mPin2, 0);}
  else if (motor_control >=0) {analogWrite(mPin2, motor_control);}
  else {analogWrite(mPin1, -motor_control);}

  Serial.println(sensorValue);
  
  prev_error = error;
  delay(1);
}
