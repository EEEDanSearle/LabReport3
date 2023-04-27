
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <stdio.h>
#include <MPU6050_tockn.h>
#define I2C_SLAVE_ADDR 0x04
#define MPU_6050_ADDR 0x68
#define DISTANCE 128  //travelled in clicks per fwd or bwd 

MPU6050 mpu6050(Wire);
uint16_t Enc1,Enc2;

const byte ROWS = 4; //four rows
const byte COLS = 3; //four columns

const int rs = 27, en = 26, d4 = 25, d5 = 33, d6 = 32, d7 = 14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//define the symbols on the buttons of the keypads
char Keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {15,2,0,4}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {16,17,5}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(Keys), rowPins, colPins, ROWS, COLS); 

void setup(){
  Serial.begin(9600);
  
  lcd.begin(16,2);//Initialise LCD With columns and rows
  lcd.setCursor(0,0);
  lcd.print("Initializing MPU");

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Maze Navigation");
  lcd.setCursor(0,1);
  lcd.print("Press any key");
}

long CalculateDistance(uint16_t clicks)  //Define Function Calculate Distance
{
  float diameter = 6,distance;   //Define variable diameter and distance
  distance = (clicks*((M_PI*diameter)/24));   //Calculates distance by piD and dividing by clicks per revolution of the wheel
  return(distance);  // Returns calculated value
}

uint16_t rotaryCount(int EncNum)
{
  Wire.requestFrom(I2C_SLAVE_ADDR,4); //Requests Data from Arduino encoders
  uint16_t Enc1, Enc2;  //Encoder Speed
  uint8_t Enc1_169 = Wire.read();  //Receving encoder bits 16-9
  uint8_t Enc1_81 = Wire.read();  //Receiving encoder bits 8-1
  uint8_t Enc2_169 = Wire.read();
  uint8_t Enc2_81 = Wire.read();
  Enc1 = (Enc1_169 <<8) | Enc1_81;  //Combines two 8 bit numbers into a 16 bit
  Enc2 = (Enc2_169 <<8) | Enc2_81;

  switch(EncNum)
  {
    case 1:
    return(Enc1);
    case 2:
    return(Enc2);
  }
}

void ChangeDirection(int leftMotor, int rightMotor, int SteeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  
  //send leftMotorSpeed
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Send rightMotorSpeed
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  //send steering angle to arduino
  Wire.write((byte)((SteeringAngle & 0x0000FF00) >> 8));    
  Wire.write((byte)(SteeringAngle & 0x000000FF));
 
  Wire.endTransmission();   // stop transmitting
}

void TurnAntiClockwise(int Angle)
{
  int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (StartAngle - findAngle());

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,120); //turn left (anticlockwise)
   AngleTurned = (StartAngle - findAngle());
   // Serial.println("AngleTurned: ");
   // Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,150); //Stop car
}

void TurnClockwise(int Angle)
{

 int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (findAngle() - StartAngle);

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,200); //turn Right (clockwise)
   AngleTurned = (findAngle() - StartAngle);
    Serial.println("AngleTurned: ");
    Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,150); //Stop car
}

int findAngle()
{
  mpu6050.update();//get data from mpu
  int ZAngle = mpu6050.getAngleZ();  //Find Angle Z
  return(ZAngle);
}

int process_input(char keypad, char* command)
{
  switch(keypad)
  {
    case '2':
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("FWD");
      strcpy(command, "FWD"); // copy command name to buffer
      break;
    
    case '4':
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("LEFT");
      strcpy(command, "LEFT"); // copy command name to buffer
      break;

    case '6':
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("RIGHT");
      strcpy(command, "RIGHT"); // copy command name to buffer
      break;

    case '8':
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("BWD");
      strcpy(command, "BWD"); // copy command name to buffer
      break;
      
    case '*':
      //lcd.print("CLEAR");
      break;

    case '#':
      //lcd.print("GO");
      break;
      
    default:
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("Not a Valid Command");
      return (0);
  }
  command[strlen(command)] = '\0'; // terminate the string with a null character
  return 1;
}

void MoveBot(char instructions)
{
  int startCount = rotaryCount(1); //take intitial count
  int currentCount = startCount;
  int StartAngle = findAngle();

  switch (instructions)
  {
    case '2':  //FWD 10cm
      while ((currentCount - startCount) <= DISTANCE) 
      {
        ChangeDirection(150,150,150);
        int actualAngle = findAngle();
       
        if (actualAngle != StartAngle)
       {
         int Turn = StartAngle - actualAngle;
         ChangeDirection(150,150,150+Turn);
       }
    
        currentCount = rotaryCount(1);
        delay(100);
      }
      ChangeDirection(0,0,150);
      break;
    
    case '4': //Left 90
    /*
      ChangeDirection(150,150,110);
      delay(2200); */
      TurnAntiClockwise(90);
      break;

    case '6':  //Right 90
   /* ChangeDirection(150,150,170);
    delay(2200);*/
    TurnClockwise(90);
      break;

    case '8':  //BWD 10cm
      while ((startCount - currentCount) <= DISTANCE) 
      {
        ChangeDirection(-150,-150,150);
        currentCount = rotaryCount(1);
        delay(100);
      }
      ChangeDirection(0,0,150);
      break;
      
    case '*':
      //lcd.print("CLEAR");
      break;

    case '#':
      //lcd.print("GO");
      break;
      
  }
}
  
void loop(){

  ChangeDirection(0,0,150);
  
  char customKey = customKeypad.getKey();

  static int instructionCount;
  static char currentInstructions[30];
  char command[5];

  if (customKey){

    if (instructionCount == 0) lcd.clear(); //If there are no instructions within the array, clear the screen
    
    int result = process_input(customKey,command); 

    if (result == 0) //If input is not a valid command
    {
     lcd.print("Not a Valid Input");
     return;
    }
    currentInstructions[instructionCount] = customKey;
    currentInstructions[instructionCount + 1] = '\0'; // null-terminate the string
    
    
    lcd.setCursor(0,0);
    lcd.print(currentInstructions);
    lcd.setCursor(0,1);
    instructionCount += 1;
    

    //lcd.print(String("Count = ") + String(instructionCount));
    
    int array_size = sizeof(currentInstructions) / sizeof(char); //size of array of instructions

    

    if (customKey == '*')  //clear command
    {
      lcd.print("CLEAR");
      instructionCount = 0; //set count to 0
      for(int i = 0; i < array_size ; i++) 
      {
      currentInstructions[i] = 0; // set each element to 0
      }
      currentInstructions[0] = '\0';
    }

    if (customKey == '#') //GO Command
{
  lcd.clear();
  lcd.print("GO");
  lcd.setCursor(0, 1);
  
  char command[10]; // buffer to store command name
  for (int i = 0; i < instructionCount; i++)
  {
   process_input(currentInstructions[i], command);
   delay(500);
  }

  lcd.clear();
  lcd.print("EXECUTING");image.png

  for (int i =0;i<instructionCount;i++)
  {
    MoveBot(currentInstructions[i]);
  }
  ChangeDirection(0,0,145);
  lcd.clear();
  lcd.print("Finished");


  instructionCount = 0; //set count to 0
      for(int i = 0; i < array_size ; i++) 
      {
      currentInstructions[i] = 0; // set each element to 0
      }
      currentInstructions[0] = '\0';

}

    else 
    {
      lcd.setCursor(0, 0);
      lcd.print(currentInstructions);
    }
  }
}
