 
 // Code for dc voltage measurement by using a voltage divider circuit
 // By deba168 dated 17/03/15
  
  int temp=0;
  float sum =0;
  float VOLTS_SCALE =0;
  float volt=0;
  void setup()
  {

  Serial.begin(9600);
  }

  void loop() 
  {
    for(int i = 0; i < 100; i++)  // loop through reading raw adc values 100 number of times 
  {
    temp=analogRead(A0);     // read the input pin   
    sum += temp;             // store sum for averaging
    delay(2);           
  }
   sum=sum/100;              // divide sum by 100 to get average 
   
  // Calibration  for Voltage
  
   VOLTS_SCALE = 0.00488 * (120/20); // The voltage divider resistors are R1=100k and R2=20k // 5/1024 =0.00488
   volt = VOLTS_SCALE * sum ;    
  
  Serial.print(volt);
  Serial.println("V");
  delay(500);
  } 
  
  
