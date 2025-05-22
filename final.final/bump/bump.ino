#define pwm 12
#define dir 13

#define bump_up 18

bool bump = false;

void IRAM_ATTR buttonISR() {
  bump = true; 
}

void setup() {

  Serial.begin(9600);

  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT);
  
  pinMode(bump_up, INPUT_PULLDOWN);  
  attachInterrupt(digitalPinToInterrupt(bump_up), buttonISR, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  int pin = digitalRead(bump_up);
  Serial.print(pin);
  Serial.println(bump);

  if(!bump){
    digitalWrite(pwm,HIGH);
  }
  else{
    digitalWrite(pwm,LOW);
  }

}
