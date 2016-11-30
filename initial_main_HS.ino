#include <Arduino.h>
#include "hs_general.h"

int nothing = 5;
// If the state read from the digital pin equals trippedState then
// the pin will be considered activated.
struct digitalPinMonitor
{
  char type[TYPE_SIZE];
  char name[NAME_SIZE];
  int pin;
  int trippedState;
};

void SetDigitalPinMonitor(struct digitalPinMonitor & dpm, const char * type, const char * name, int p, int tS)
{
  strcpy(dpm.type, type);
  strcpy(dpm.name, name);
  dpm.pin = p;
  dpm.trippedState = tS; 
};

// The value read on the Analogue pin will be multiplied by the valueModifier
// then compared to the threshold and if the modified value is greater than
// the threshold then the pin will be considered activated.
struct analogePinMonitor
{
  char type[TYPE_SIZE];
  char name[NAME_SIZE];
  int pin;
  float valueModifier;
  int threshold;
};

void SetAnalogePinMonitor(struct analogePinMonitor & apm, const char * type,
                          const char * name, int p, float vm, int t)
{
  strcpy(apm.type, type);
  strcpy(apm.name, name);
  apm.pin = p;
  apm.valueModifier = vm;
  apm.threshold = t;
};

static const int NUM_OF_DIG_MON_PINS = 3;
digitalPinMonitor digMonPins[NUM_OF_DIG_MON_PINS];

void CheckDigitalPins()
{
  const char * state;
  for(int i = 0; i<NUM_OF_DIG_MON_PINS; i++)
  {
    if(digMonPins[i].trippedState == digitalRead(digMonPins[i].pin))
    {
      state = STATE_TRIPPED;
    }
    else
    {
      state = STATE_CLEARED;
    }
    SEND_SERIAL(Serial, digMonPins[i].type, digMonPins[i].name, state, NULL);
  }
};

static const int NUM_OF_ANL_MON_PINS = 2;
analogePinMonitor anlMonPins[NUM_OF_ANL_MON_PINS];

void CheckAnalogPins()
{
  float mod = 0;
  int pin = 0;
  int thresh = 0;
  int value = 0;
  const char * state;
  for(int j = 0; j<NUM_OF_ANL_MON_PINS; j++)
  {
    pin = anlMonPins[j].pin;
    mod = anlMonPins[j].valueModifier;
    thresh = anlMonPins[j].threshold;
    value = analogRead(pin);

    if(mod*value>thresh)
    {
      state = STATE_TRIPPED;
    }
    else
    {
      state = STATE_CLEARED;
    }
    
    SEND_SERIAL(Serial, anlMonPins[j].type, anlMonPins[j].name, state, value);
  }
};

//One for each of the serial lines (including upstream)
SerialInput serialData[4];

void ReadInSerial()
{
  READ_SERIAL(Serial, serialData[0]);
  READ_SERIAL(Serial1, serialData[1]);
  READ_SERIAL(Serial2, serialData[2]);
  READ_SERIAL(Serial3, serialData[3]);
};

static const int ALARM_PIN = 2;

void CheckFromRoot()
{
  int ipos = -1;
  Message temp;
  
  FIND_END(serialData[0], ipos);
  
  if(ipos != -1)
  {
    if(ParseMessage(serialData[0].data, temp))
    {
      if((temp.source[0] == '0')
        && (0 == strncmp(temp.msgType, TYPE_STATUS, MSG_INT_SIZE)))
      {
        if(0 == strncmp(temp.info, STATUS_ALARMED, strlen(STATUS_ALARMED)))
        {
          digitalWrite(ALARM_PIN, HIGH);
        }
        else
        {
          digitalWrite(ALARM_PIN, LOW);
        }
      }
    }
  }
};

void ForwardSerialFromRoot()
{
  // If received anything from the root/source then echo it too all
  // devices further down the chain
  int ipos = -1;

  FIND_END(serialData[0], ipos);

  if(ipos != -1)
  {
    //Send the data if an end was found plus 1 to include the newline
    Serial1.write((uint8_t*)(serialData[0].data), ipos+1);
    Serial2.write((uint8_t*)(serialData[0].data), ipos+1);
    Serial3.write((uint8_t*)(serialData[0].data), ipos+1);

    //Clear out the data that was just sent               
    CLEAR_DATA(serialData[0], ipos+1);
  }
};

void ForwardToSource()
{
  int ipos;
  Message unusedMsg;

  // For each serial stream
  for(int i=1; i < 4; i++)
  {
    ipos = -1;
    FIND_END(serialData[i], ipos);
    if(ipos != -1)
    {
      if (ParseMessage(serialData[i].data, unusedMsg))
        Serial.write((uint8_t*)(serialData[i].data), ipos+1);
      CLEAR_DATA(serialData[i], ipos+1);
    }
    
    //If no end has been found and the max amount of data is in the
    //buffer then clear it out since no messages are that large.
    if(serialData[i].dataLen == MAX_SERIAL_IN)
    {
      CLEAR_DATA(serialData[i], MAX_SERIAL_IN);
    }
  }
};

int msSinceLastMonitor = 0;
static const int MS_BETWEEN_MONITORS = 500;
static const int LOOP_DELAY = 20;

//====The setup() method runs once, when the sketch starts
void setup()   
{
  // Set the id that is sent over the serial as the source
  SRC_ID = 1;
  
  SetDigitalPinMonitor(digMonPins[0], TYPE_DOOR, "Back Door", 22, LOW);
  SetDigitalPinMonitor(digMonPins[1], TYPE_DOOR, "Front Door", 23, LOW);
  SetDigitalPinMonitor(digMonPins[2], TYPE_DOOR, "Garage Door", 24, LOW);

  SetAnalogePinMonitor(anlMonPins[0], TYPE_MOTION, "Main Floor Mtn", 0, 1, 768);
  SetAnalogePinMonitor(anlMonPins[1], TYPE_MOTION, "Basement Mtn", 1, 1, 768);

  for (int i = 0; i < NUM_OF_DIG_MON_PINS; i++)
  {
    pinMode(digMonPins[i].pin, INPUT);
  }

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  serialData[0].dataLen = 0;
  serialData[1].dataLen = 0;
  serialData[2].dataLen = 0;
  serialData[3].dataLen = 0;
  
  pinMode(ALARM_PIN, OUTPUT);
};

//====the loop() method runs over and over again,
// as long as the Arduino has power
void loop()
{
  int tripped = false;
  
  if(MS_BETWEEN_MONITORS < msSinceLastMonitor)
  {
    msSinceLastMonitor = 0;
    
    CheckDigitalPins();
    
    CheckAnalogPins();
  }
  else
  {
    msSinceLastMonitor += LOOP_DELAY;
    delay(LOOP_DELAY);
  }

  ReadInSerial();
  
  CheckFromRoot();
  
  ForwardSerialFromRoot();

  ForwardToSource();
};

