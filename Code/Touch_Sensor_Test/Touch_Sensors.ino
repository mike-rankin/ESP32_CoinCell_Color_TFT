// ESP32 Touch Test
// Touch Pad1 = GPIO27. T7
// Touch Pad2 = GPIO12, T5 
// Touch Pad3 = GPIO15, T3
// Touch Pad4 = GPIO2 , T2

void setup()
{
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  Serial.println("ESP32 Touch Test");
}

void loop()
{
  Serial.println(touchRead(T2));  // get value using T?
  delay(1000);
}
