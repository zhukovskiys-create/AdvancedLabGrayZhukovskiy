void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  double voltAvg = 0;
  int numAvg =1;
  double sTime = (micros());
  int time = 100000;
  int readCount=0;
  int dLay=500;
  
  for (int i = 0; i < numAvg; i++){
    int sensorValue = analogRead(A0);
    voltAvg += sensorValue;
    readCount++;

  }
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
  
  Serial.println(brightness);
  //Serial.println(voltAvg);
  //Serial.println(tRead/dLay);
  analogWrite(10, brightness);
  delay(dLay);


}
