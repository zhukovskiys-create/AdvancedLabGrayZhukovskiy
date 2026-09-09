First, we explore our ability to send voltage signals out from the Arduino by making an LED blink. We did this through a basic loop telling the Arduino to send out 5V or 0V at different intervals.

Second, we wanted to read the voltage output across a potentiometer (int sensorValue = analogRead(A0);); this gave us a relative 10-bit integer.

Third, we averaged the value to reduce noise (by averaging our data over the course of several cycles, we get a more consistent output), and then mapped it from a 10-bit integer (0,1024) to an 8-bit one (0,255) and assigned relative lengths for how long our LED stays on for within a cycle.
Letting LED stay on longer relative to how long it stays off within a cycle makes it appear brighter; Doing the opposite (shortening the 5V input to the LED within the cycle) makes the LED appear dimmer.
while (micros() - sTime < time) {
    int sensorValue = analogRead(A0);
    readCount++;
  }
  
  double eTime = (micros());
  double tTime = (eTime-sTime);
  float reads = (float)readCount*100000/tTime;
  float tRead = (float)tTime/readCount;
  voltAvg = voltAvg/numAvg;
  int brightness = map(voltAvg, 0, 1023, 0, 255);
  voltAvg = voltAvg*5/(pow(2,10));
