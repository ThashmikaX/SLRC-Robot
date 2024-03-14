/* This code works with GY-31 TCS3200 TCS230 color sensor module
 *  It select a photodiode set and read its value (Red Set/Blue set/Green set) and displays  it on the Serial monitor
 * and identify if possible the color
 * Refer to  www.surtrtech.com for more details
 */
#include <color.h>
void SetColorSensor()
{
    pinMode(s1_0,OUTPUT);    //pin modes
    pinMode(s1_1,OUTPUT);
    pinMode(s1_2,OUTPUT);
    pinMode(s1_3,OUTPUT);
    pinMode(out1,INPUT);
   
    digitalWrite(s1_0,HIGH); //Putting S0/S1 on HIGH/HIGH levels  means the output frequency scalling is at 100% (recommended)
    digitalWrite(s1_1,HIGH);  //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%

    pinMode(s2_0,OUTPUT);    //pin modes
    pinMode(s2_1,OUTPUT);
    pinMode(s2_2,OUTPUT);
    pinMode(s2_3,OUTPUT);
    pinMode(out2,INPUT);
   
    digitalWrite(s2_0,HIGH); //Putting S0/S1 on HIGH/HIGH levels  means the output frequency scalling is at 100% (recommended)
    digitalWrite(s2_1,HIGH);  //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%

}

void setupFrontColorSensor(){
   pinMode(s2_0,OUTPUT);    //pin modes
    pinMode(s2_1,OUTPUT);
    pinMode(s2_2,OUTPUT);
    pinMode(s2_3,OUTPUT);
    pinMode(out2,INPUT);
   
    digitalWrite(s2_0,HIGH); //Putting S0/S1 on HIGH/HIGH levels  means the output frequency scalling is at 100% (recommended)
    digitalWrite(s2_1,HIGH);   //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
}

void setupFloorColorSensor(){
   pinMode(s1_0,OUTPUT);    //pin modes
    pinMode(s1_1,OUTPUT);
    pinMode(s1_2,OUTPUT);
    pinMode(s1_3,OUTPUT);
    pinMode(out1,INPUT);
   
    digitalWrite(s1_0,HIGH); //Putting S0/S1 on HIGH/HIGH levels  means the output frequency scalling is at 100% (recommended)
    digitalWrite(s1_1,HIGH);  //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
}

uint8_t GetColorsFloor()  
{
  int  Red=0, Blue=0, Green=0;  //RGB values
  uint8_t color = 4;
  digitalWrite(s1_2,  LOW);                                           //S2/S3 levels define which set  of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH  is for green 
  digitalWrite(s1_3, LOW);                                           
  Red = pulseIn(out1, digitalRead(out1) == HIGH ? LOW : HIGH);       //here we wait  until "out1" go LOW, we start measuring the duration and stops when "out1" is  HIGH again, if you have trouble with this expression check the bottom of the code
  delay(20);  
  digitalWrite(s1_3, HIGH);                                         //Here  we select the other color (set of photodiodes) and measure the other colors value  using the same techinque
  Blue = pulseIn(out1, digitalRead(out1) == HIGH ? LOW  : HIGH);
  delay(20);  
  digitalWrite(s1_2, HIGH);  
  Green = pulseIn(out1,  digitalRead(out1) == HIGH ? LOW : HIGH);
  delay(20);  

  if (Red <=15 && Green <=15 && Blue <=15)         //If the values  are low it's likely the white color (all the colors are present)
             color = 0; //white

  else if (Red<Blue && Red<=Green && Red<23)      //if  Red value is the lowest one and smaller thant 23 it's likely Red
             color = 1;//red

  else if (Blue<Green && Blue<Red && Blue<20)    //Same thing for Blue
             color = 2; //blue

  else if (Green<Red && Green-Blue<= 8)           //Green it was a little tricky,  you can do it using the same method as above (the lowest), but here I used a reflective  object
             color = 3;//green                    //which means the  blue value is very low too, so I decided to check the difference between green and  blue and see if it's acceptable
  else
            color = 4;//unknown                  //if the color is not recognized, you can add as many as you want

  return color;
}

//Here's an  example how to understand that expression above,
//""digitalRead(out1) == HIGH  ? LOW : HIGH"" this whole expression is either HIGH or LOW as pulseIn function  requires a HIGH/LOW value on the second argument

//led_Status = led_Status  == HIGH ? LOW : HIGH;  
//
//if(led_Status == HIGH) 
//{ 
//led_Status  =LOW; 
//} 
//else 
//{ 
//led_Status = HIGH; 
//}
uint8_t GetColorsForward()  
{
  int  Red=0, Blue=0, Green=0;  //RGB values
  uint8_t color = 4;
  digitalWrite(s2_2,  LOW);                                           //S2/S3 levels define which set  of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH  is for green 
  digitalWrite(s2_3, LOW);                                           
  Red = pulseIn(out2, digitalRead(out2) == HIGH ? LOW : HIGH);       //here we wait  until "out1" go LOW, we start measuring the duration and stops when "out1" is  HIGH again, if you have trouble with this expression check the bottom of the code
  delay(20);  
  digitalWrite(s2_3, HIGH);                                         //Here  we select the other color (set of photodiodes) and measure the other colors value  using the same techinque
  Blue = pulseIn(out2, digitalRead(out2) == HIGH ? LOW  : HIGH);
  delay(20);  
  digitalWrite(s2_2, HIGH);  
  Green = pulseIn(out2,  digitalRead(out2) == HIGH ? LOW : HIGH);
  delay(20);  

  if (Red <=15 && Green <=15 && Blue <=15)         //If the values  are low it's likely the white color (all the colors are present)
             color = 0; //white

  else if (Red<Blue && Red<=Green && Red<23)      //if  Red value is the lowest one and smaller thant 23 it's likely Red
             color = 1;//red

  else if (Blue<Green && Blue<Red && Blue<20)    //Same thing for Blue
             color = 2; //blue

  else if (Green<Red && Green-Blue<= 8)           //Green it was a little tricky,  you can do it using the same method as above (the lowest), but here I used a reflective  object
             color = 3;//green                    //which means the  blue value is very low too, so I decided to check the difference between green and  blue and see if it's acceptable

  else
            color = 4;//unknown                  //if the color is not recognized, you can add as many as you want

  return color;
}