#define LED_PIN 13
void setup() {
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if(Serial.available()){
    int x=Serial.readString().toInt();
    if(x==1){
      digitalWrite(LED_PIN,HIGH);
    }
    else{
      digitalWrite(LED_PIN,LOW);
    }
  }
  delay(0.1);
}
