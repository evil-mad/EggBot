/**
 * EggBot Simple Write. 
 * 
 * Example code for talking to the EggBot from Processing.
 */


import processing.serial.*;

Serial myPort;  // Create object from Serial class
int val;        // Data received from the serial port
boolean PenDown;


void liftPen() 
{
  myPort.write("SP,0\r");           
  PenDown = false;
}

void lowerPen() 
{
  myPort.write("SP,1\r");           
  PenDown = true;
}



 
 void Up() 
{
   myPort.write("SM,100,10,0\r");           
  //"SM,<duration>,<penmotor steps>,<eggmotor steps><CR>"
} 

 void Down() 
{
   myPort.write("SM,100,-10,0\r");           
  //"SM,<duration>,<penmotor steps>,<eggmotor steps><CR>"
}   
  
  
 void Left() 
{
   myPort.write("SM,100,0,10\r");           
  //"SM,<duration>,<penmotor steps>,<eggmotor steps><CR>"
} 

 void Right() 
{
   myPort.write("SM,100,0,-10\r");           
  //"SM,<duration>,<penmotor steps>,<eggmotor steps><CR>"
}   
  

void setup() 
{
  size(800, 250);
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1];
  myPort = new Serial(this, portName, 38400);

  liftPen();
   background(255);
}

//String str1 = "SP,1\r";


void draw() {

//   myPort.write("SM,25,0,50\r");           
  //"SM,<duration>,<penmotor steps>,<eggmotor steps><CR>"
  
 // delay(500);
   //  myPort.write("SM,25,0,0\r");           
  
  
}
 


void keyPressed()
{
  // if the key is between 'A'(65) and 'z'(122)
  if( key == ' ') 
  {

    if (PenDown)
      liftPen();
    else
      lowerPen();
  }
  
    if( key == 'u') 
  {
    Up();
  }
  
    if( key == 'd') 
  {
    Down();
  }
 
   
    if( key == 'l') 
  {
    Left();
  }
  
    if( key == 'r') 
  {
    Right();
  } 
    
}

