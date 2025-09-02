const int buttonPin = 2;      //Safetyoff-d2
const int buttonPin11 = A6;   //safetyon-a6

const int buttonPin1 = 7;     //Arm -d7
const int buttonPin2 = 3;     //Disarm -d3

const int buttonPin3 = 8;     //Althold -d8
const int buttonPin4 = 4;     //Loiter -d4
const int buttonPin5 = 9;     //Auto -d9
const int buttonPin6 = 5;     //RTL -d5
const int buttonPin7 = 6;     //Land -d6

const int buttonPin8 = 10;    //Cameraautocenter -d10

const int Switch = A3;   //flip switch
const int buttonPin9 = 11;    //payload Arm and open- d11
const int buttonPin10 = 12;   //payload disarm and close -d12


float pot1Value; // Declare variable to store value from potentiometer 1
float pot2Value; // Declare variable to store value from potentiometer 2
float pot3Value; // Declare variable to store value from potentiometer 3
const int pot1 = A0; // Define the pin connected to potentiometer 1
const int pot2 = A1; // Define the pin connected to potentiometer 2
const int pot3 = A2; // Define the pin connected to potentiometer 3

float prevPot1Value = 0; // To store the previous value of pot1
float prevPot2Value = 0; // To store the previous value of pot2
float prevPot3Value = 0; // To store the previous value of pot2

int buttonState9;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin5, INPUT_PULLUP);
  pinMode(buttonPin6, INPUT_PULLUP);
  pinMode(buttonPin7, INPUT_PULLUP);
  pinMode(buttonPin8, INPUT_PULLUP);
  pinMode(buttonPin9, INPUT_PULLUP);
  pinMode(buttonPin10, INPUT_PULLUP);
  pinMode(buttonPin11, INPUT_PULLUP);
  pinMode(Switch, INPUT_PULLUP);
}

void loop() 
  {
  delay(100);
  // Read potentiometer values
  pot1Value = analogRead(pot1); // Read from potentiometer connected to A0 (for tilt)
  pot2Value = analogRead(pot2); // Read from potentiometer connected to A1 (for pan)
  pot3Value = analogRead(pot3); // Read from potentiometer connected to A1 (for pan)
  
  // Map the potentiometer values to -32767 to 32767 range
  pot1Value = map(pot1Value, 0, 1023, 60, -60);
  pot2Value = map(pot2Value, 0, 1023, 60, -60);
  pot3Value = map(pot3Value, 0, 1023, 0, 100);

  int deadZone = 11; 
  if (abs(pot1Value) <= deadZone) {
    pot1Value = 0;
  }

  if (abs(pot2Value) <= deadZone) {
    pot2Value = 0;
  }

  if (abs(pot3Value) <= 0.2) {
    pot3Value = 0;
  }

  int buttonState = digitalRead(buttonPin);                      
  int buttonState1 = digitalRead(buttonPin1);                    
  int buttonState2 = digitalRead(buttonPin2);                 
  int buttonState3 = digitalRead(buttonPin3);                   
  int buttonState4 = digitalRead(buttonPin4);                    
  int buttonState5 = digitalRead(buttonPin5);                   
  int buttonState6 = digitalRead(buttonPin6);                   
  int buttonState7 = digitalRead(buttonPin7);                   
  int buttonState8 = digitalRead(buttonPin8);                   
  int buttonState11 = digitalRead(buttonPin11);
  int switchstate = digitalRead(Switch);
  //int buttonState9 = digitalRead(buttonPin9);                   
  int buttonState10 = digitalRead(buttonPin10);

  if (switchstate == 0) { // If switch is ON 
    Serial.println(String(buttonState)+","+String(buttonState1)+","+String(buttonState2)+","+String(buttonState3)+","+String(buttonState4)+","+String(buttonState5)+","+String(buttonState6)+","+String(buttonState7)+","+String(buttonState8)+","+"1"+","+String(buttonState10)+","+String(buttonState11)+","+String(pot1Value)+","+String(pot2Value)+","+String(pot3Value)+","+String(switchstate));               
  }
   if (switchstate == 1) { // If switch is ON 
    int buttonState9 = digitalRead(buttonPin9);  
    Serial.println(String(buttonState)+","+String(buttonState1)+","+String(buttonState2)+","+String(buttonState3)+","+String(buttonState4)+","+String(buttonState5)+","+String(buttonState6)+","+String(buttonState7)+","+String(buttonState8)+","+String(buttonState9)+","+String(buttonState10)+","+String(buttonState11)+","+String(pot1Value)+","+String(pot2Value)+","+String(pot3Value)+","+String(switchstate));  
                 
  }

}




