const int Sensor = 11;
void setup()
{
    Serial.begin(9600);
    pinMode(Sensor, INPUT);
}
void loop()
{
    Serial.print("Sensor: ");
    Serial.println(digitalRead(Sensor));
    delay(1000);
}
