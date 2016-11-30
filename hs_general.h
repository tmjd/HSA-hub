static int SRC_ID;
static const int NAME_SIZE = 30;
static const int TYPE_SIZE = 10;
static const int MAX_SERIAL_IN = 100;
static const int MSG_INT_SIZE = 4;

const char TYPE_INPUT[] = "0";
const char TYPE_DOOR[] = "1";
const char TYPE_MOTION[] = "2";
const char TYPE_STATUS[] = "100";
const char TYPE_LINE[] = "101";

const char STATUS[] = "Status";
const char STATUS_ALARMED[] = "Alarmed";
const char STATUS_DELAY_ALARM[] = "Delayed";
const char STATUS_ARMED[] = "Armed";
const char STATUS_STANDBY[] = "Standby";
const char STATUS_ARMING[] = "Arming";

const char STATE_TRIPPED[] = "trp";
const char STATE_CLEARED[] = "clr";

const char USER_INPUT[] = "Input";

typedef struct
{ 
  char data[MAX_SERIAL_IN];
  int dataLen;
}
SerialInput;

void READ_SERIAL(HardwareSerial & ser, SerialInput & serIn)
{
  while(ser.available() && serIn.dataLen < MAX_SERIAL_IN)
  {
    serIn.data[serIn.dataLen] = ser.read();
    serIn.dataLen++;
  }
};

void SEND_SERIAL(HardwareSerial & ser, const char * type, const char * name, 
                 const char * state, const char * msg)
{
  ser.print(SRC_ID);
  ser.print(",");
  ser.print(type);
  ser.print(",");
  ser.print(name);
  ser.print(",");
  if(msg == NULL)
    ser.println(state);
  else
  {
    ser.print(state);
    ser.print(",");
    ser.println(msg);
  }
};

void SEND_SERIAL(HardwareSerial & ser, const char * type, const char * name, 
                 const char * state, int iMsg)
{
  char strMsg[MSG_INT_SIZE];
  snprintf(strMsg, MSG_INT_SIZE, "%d", iMsg);
  SEND_SERIAL(ser, type, name, state, strMsg);
};

// Returns the position of the end character (the number of characters
// is one pluse the position)
void FIND_END(SerialInput & serIn, int & ipos)
{
  ipos = -1;
  int index = 0;
  // Search thru the data already read in and see if we have
  // a newline character indicating the end of a message.
  while(serIn.dataLen > index)
  {
    if(serIn.data[index] == '\n' || serIn.data[index] == '\r')
    {
      ipos = index;
      // Set index to the max to exit the loop
      index = MAX_SERIAL_IN;
    }
    index++;
  }
};

// Clears out the number of characters specified by len from the input data.
// Part of this is shifting any remaining data to the beginning of the data
// and updating the length.
void CLEAR_DATA(SerialInput & serIn, int len = MAX_SERIAL_IN)
{
  int tLen = serIn.dataLen;
  serIn.dataLen = 0;

  while(len < tLen)
  {
    serIn.data[serIn.dataLen] = serIn.data[len];
    serIn.dataLen++;
    len++;
  }
};

typedef struct
{
  char source[MSG_INT_SIZE];
  char msgType[MSG_INT_SIZE];
  char name[NAME_SIZE];
  char info[MAX_SERIAL_IN];
  char extra[MAX_SERIAL_IN];
}
Message;

void ClearMessage(Message & msg)
{
  memset(msg.source, '\0', MSG_INT_SIZE);
  memset(msg.msgType, '\0', MSG_INT_SIZE);
  memset(msg.name, '\0', NAME_SIZE);
  memset(msg.info, '\0', MAX_SERIAL_IN);
  memset(msg.extra, '\0', MAX_SERIAL_IN);
};

void CleanString(char * msgString)
{
  int len = strlen(msgString);
  char tC;
  for(int i = 0; i < len; i++)
  {
    tC = msgString[i];
    if((tC == '\n') || (tC == '\r'))
    {
      msgString[i] = ' ';
    }
  }
};

static const char DELIMITERS[] = ",";
bool ParseMessage(char * msgString, Message & parsedMsg)
{
  bool retVal = true;
  char * temp = NULL;
  char tempMsg[MAX_SERIAL_IN];
  if(msgString != NULL)
  {
    strncpy(tempMsg, msgString, MAX_SERIAL_IN);
//    CleanString(msgString);
    ClearMessage(parsedMsg);
    temp = strtok(tempMsg, DELIMITERS);
    if((temp != NULL) && (strlen(temp) < MSG_INT_SIZE))
    {
      strcpy(parsedMsg.source, temp);
      temp = strtok(NULL, DELIMITERS);
      if((temp != NULL) && (strlen(temp) < MSG_INT_SIZE))
      {
        strcpy(parsedMsg.msgType, temp);
        temp = strtok(NULL, DELIMITERS);
        if((temp != NULL) && (strlen(temp) < NAME_SIZE))
        {
          strcpy(parsedMsg.name, temp);
          temp = strtok(NULL, DELIMITERS);
          if((temp != NULL) && (strlen(temp) < MAX_SERIAL_IN))
          {
            strcpy(parsedMsg.info, temp);
            temp = strtok(NULL, DELIMITERS);
            if((temp != NULL) && (strlen(temp) < MAX_SERIAL_IN))
            {
              strcpy(parsedMsg.extra, temp);
            }
            //No else because extra is not required
          }
          else
          {
            retVal = false;
          }
        }
        else
        {
          retVal = false;
        }
      }
      else
      {
        retVal = false;
      }
    }
    else
    {
      retVal = false;
    }    
  }
  else
  {
    retVal = false;
  }

  return(retVal);
};


