// Code for current measurement by using a ACS712 (5A) hall effect current sensor
// by deba168 on 17/03/15


  int temp=0;
  float sum =0;
  float AMPS_SCALE =0;
  float amps=0;
  void setup()
  {

  Serial.begin(9600);
  }

  void loop() 
  {
    for(int i = 0; i < 100; i++)  // loop through reading raw adc values 100 number of times 
  {
    temp=analogRead(A1);     // read the input pin   
    sum += temp;             // store sum for averaging
    delayMicroseconds(50);           
  }
   sum=sum/100;              // divide sum by 100 to get average 
   
  // Calibration  for current
  
   AMPS_SCALE= 0.00488/ 0.185;    //5/1024 = 0.00488  // Sensitivity = 185mV
   amps = AMPS_SCALE* sum - 13.51;   // 2.5/0.185 = 13.51 
  
  Serial.print(amps);
  Serial.println("A");
  delay(500);
  } 
  
  
