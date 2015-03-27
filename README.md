# MPPT_Master
ARDUINO MPPT SOLAR CHARGE CONTROLLER (Version-3.0)



This is a project build for a Arduino based Solar MPPT charge controller.It has features like: LCD display,Led Indication,
Wi Fi data logging and provision for charging different USB devices.It is equipped with various protections to protect the 
circuitry from abnormal condition.

The microcontroller used is in this controller is Arduino Nano. This design is suitable for a 50W solar panel to charge 
a commonly used 12V lead acid battery. You can also use other Arduino board like Pro Mini,Micro and UNO.

Now a days the most advance solar charge controller available in the market is Maximum Power Point Tracking (MPPT).
The MPPT controller is more sophisticated and more expensive.It has several advantages over the earlier charge controller.
It is 30 to 40 % more efficient at low temperature.But making a MPPT charge controller is little bit complex in compare to 
PWM charge controller.It require some basic knowledge of power electronics.

I put a lot of effort to make it simple, so that any one can understand it easily.If you are aware about the basics of MPPT 
charge controller then skip the first few steps.

The Maximum Power Point Tracker (MPPT) circuit is based around a synchronous buck converter circuit..It steps the higher solar
panel voltage down to the charging voltage of the battery. The Arduino tries to maximize the watts input from the solar panel 
by controlling the duty cycle to keep the solar panel operating at its Maximum Power Point.


Specification of version-3 charge controller :

1.Based on MPPT algorithm

2. LED indication for the state of charge

3. 20x4 character LCD display for displaying voltages,current,power etc

4. Overvoltage / Lightning protection

5. Reverse power flow protection

6. Short Circuit and Over load protection

7. Wi Fi data logging

8.USB port for Charging Smart Phone /Gadgets

Electrical specifications :

1.Rated Voltage= 12V

2.Maximum current = 5A

3.Maximum load current =10A

4. In put Voltage = Solar panel with Open circuit voltage from 12 to 25V

5.Solar panel power = 50W

