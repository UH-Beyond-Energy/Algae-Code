int relayFan = 8;
int relayHeater = 9;
int relayAgitator= 10;

void setup() 
{
  pinMode(relayFan, OUTPUT);
  digitalWrite(relayFan, HIGH);
  
    pinMode(relayHeater, OUTPUT);
  digitalWrite(relayHeater, HIGH);

    pinMode(relayAgitator, OUTPUT);
  digitalWrite(relayAgitator, HIGH);
}
void loop() {
  digitalWrite(relayFan, HIGH);
  Serial.print("AGITATOR ON FOR 30 MIN");
  delay(5000);
  digitalWrite(relayFan, LOW);
  Serial.print("AGITATOR OFF FOR 30 MIN");
  delay(5000);

    digitalWrite(relayHeater, HIGH);
  Serial.print("AGITATOR ON FOR 30 MIN");
  delay(5000);
  digitalWrite(relayHeater, LOW);
  Serial.print("AGITATOR OFF FOR 30 MIN");
  delay(5000);

   digitalWrite(relayAgitator, HIGH);
  Serial.print("AGITATOR ON FOR 30 MIN");
  delay(5000);
  digitalWrite(relayAgitator, LOW);
  Serial.print("AGITATOR OFF FOR 30 MIN");
  delay(5000);
}
