/**
  **************************************************************************
  * @file     readme.txt
  * @brief    readme
  **************************************************************************
  */

  this demo is based on the AT-MOTOR-EVB board with AT32F455_456_457 MCU to develop a foc control mode project for controlling a PMSM motor with or without sensor.
  in this demo, shows how to setup the MCU peripheral and make a control program with Artery motor control library and comm. functions.
  the test devices and main peripheral pin definitions are shown as below.


/*******************************************AT-MOTOR-EVB V2.x**********************************************/
  Board: AT-MOTOR-EVB V2.x
  Motor: JK42BLS01-X038ED
  Motor connection:
 ¢z¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢{
 ¢x Phase U -> CN4 OUT1¢x  (PWM_U_H: PA8, PWM_V_H: PA9, PWM_W_H: PA10)
 ¢x Phase V -> CN4 OUT2¢x  (PWM_U_L: PB13, PWM_V_L: PB14, PWM_W_L: PB15)
 ¢x Phase W -> CN4 OUT3¢x
 ¢|¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢}
  If the sensor is required, use the following pin definitions to connect the corresponding sensor and the Jumpers need to be changed as follows.
 ¢z¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢{
 ¢x hall sensor -> J3¢x  ( Hall_A: PB4, Hall_B: PB5, Hall_C: PB0) and (JP4: OPEN, JP5: OPEN, JP6: OEPN) 
 ¢|¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢}
 ¢z¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢{
 ¢x encoder sensor  -> J4¢x  ( A+: PH2, B+: PH3, Z+: PD2) and (JP4: OPEN, JP5: OPEN, JP6: OPEN) 
 ¢|¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢}

  If user interface(UI) is needed, use a AT-LINK to connect PC USB port and CN2 of AT32-MOTOR-Sub board as follow:
 ¢z¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢{
 ¢x PC -> AT-LINK -> CN2(AT32-MOTOR-Sub board)¢x  
 ¢|¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢w¢}
  for more detailed information. please refer to the quick start guide document AN0218.
/*******************************************AT-MOTOR-EVB V2.x**********************************************/