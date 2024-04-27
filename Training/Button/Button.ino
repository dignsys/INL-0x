#define buttonPin 0
int buttonState = 0;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  Serial.print("Button pin : "); Serial.println(buttonPin); 
}

void loop() {
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
  if (buttonState == HIGH) {      
    Serial.println("Button input value High");
  } else {    
    Serial.println("Button input value Low");
  }
  delay(500);
}
