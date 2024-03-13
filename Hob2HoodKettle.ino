/* 
* Hob2Hood Kettle
* 1Pats March 2024
* Code is developed according to KISS principle
*
* Linking electric kettle with kitchen hood. 
* While kettle is switched on - ventilation switches on to pick up steam.
* As hob and hood support Hob2Hood interface, hood is commanded by infrared signals bradcasted by arduino uno / KY-005 module, 
* which emmits infrared signals. 
* Detection of kettle is done on a basis of current's sensor. In my case ZMCT103C.
* Power wire wired through induction coil,  sensor module glued with hot glue at opposite part of power socket, where kettle is plugged-in.
* If sensor detects power consumption in appropriate range, it assumes -  this is a kettle and UNO sends IR commands to hood. 
* My kettle boils completely in about 4 minutes and 45 seconds and then switches off (if filled fully)
* As fan has 4 levels, fan speed rises proportionly boiling time. Boost level switches on shortly before expected end, but, 
* if the lid is unattentivelly left open and kettle is not switching off, it might help to prevent kitchen cabinets to spoil due to steam.
* During kettle's usage hood light is on.
* Code was developed with significant help from Arminjo/Armin Joachimsmeyer, one of IRremote library authors, who presented example how to communicate with Hob2Hood devices
* Without him I would not be able to finish this setup as receiving IR signal and sending them out significantly differs, 
* but Hob2Hood protcol is nor supported protocols list.
* For debugging purposes and alternate communication way, remote support is added.
* As my TV box is Sony and TV Remote always is ready to hand, I decided to apply  rare used buttons from teletext row
* (but as bluetooth is alternate comuunication channell for Sony remote - it should be switched off)
* Teletext, seems, now is in past...
* RED button - cancel all immediatelly
* GREEN - looping: "Light on", "fan speed 1","fan speed 2","fan speed 3","fan speed 4", "all off"
*
* Code might be reused without any restrictions.
* Code contains debugging features - all that is related to receiving Hov2Hood infrared signals might be commented out. 
* For final version all lines related to Serial monitor might be commented out.
*
* I am widely use bitwise operations as it is very convinient. If you are not familiar with it, browse internet. One of examples: 
* https://www.geeksforgeeks.org/bitwise-operators-in-c-cpp/
*/

#define DECODE_HASH                                                     // per Arminjo
//#define DECODE_SONY                                                   // seems smth is not working if I try to recognize Sony protocol. RAW codes used instead
#include "PinDefinitionsAndMore.h"                                      // define macros for input and output pin etc.
#include <IRremote.hpp>  

#define ZMCT103C_PIN                        A0                          // GND,VCC-5V, A0
//define  pins used for infrared communicayion
//#define IR_SEND_PIN                       3                           // GND,VCC-5V, 3   - this is defined in PinDefinitionsAndMore kY-005
//#define IR_RECEIVE_PIN                    2                           // GND,VCC-5V, 2   - this is defined in PinDefinitionsAndMore KY-022
#define ACTION_TIMEOUT                      (5L*1000L)                  // do next activity only in ACTION_TIMEOUT miliseconds
#define BOILING_TIME                        (5L*60L*1000L)              // 5 mimutes 
#define FAN2_TIME                           (BOILING_TIME/2L)           // time when second level for fans  is switched on    
#define FAN3_TIME                           (FAN2_TIME + BOILING_TIME/4L) // third
#define FAN4_TIME                           (BOILING_TIME- 30L*1000L)   // Boost level - at the same end, but might be uefull 

// Current's levels for my kettle (no measurement units, just some value)
// ~42 is  a level of unplugged socket
#define A_START                             50                          // smth plugged-in, light should be switched on
#define A_MIN                               85                          // in my case - electric kettle
#define A_MAX                               100                         // these are experimental values, and need to be adjusted for each particular device

#define ARRAY_SIZE                          64                          // number of instances for averaging sensor readings
  
// Status bits
#define LIGHT_IS_ON                         0x0001                      // set this bit, if light is on
#define FAN_1                               0x0002                      // first fan level                 
#define FAN_2                               0x0004                      // second fan level
#define FAN_3                               0x0008                      // third fan level
#define FAN_4                               0x0010                      // fourth fan level
#define MANUAL_MODE                         0x0020                      // if hood is commanded by SONY remote

// IR commands (code received) from Hob2Hood device or SONY remote
#define HOB_TO_HOOD_HASH_CODE_FAN_1         0xE3C01BE2
#define HOB_TO_HOOD_HASH_CODE_FAN_2         0xD051C301
#define HOB_TO_HOOD_HASH_CODE_FAN_3         0xC22FFFD7
#define HOB_TO_HOOD_HASH_CODE_FAN_4         0xB9121B29
#define HOB_TO_HOOD_HASH_CODE_FAN_OFF       0x55303A3
#define HOB_TO_HOOD_HASH_CODE_LIGHT_ON      0xE208293C
#define HOB_TO_HOOD_HASH_CODE_LIGHT_OFF     0x24ACF947
#define SONY_RED                            0xE23B4151                  // Sony remote buttons
#define SONY_GREEN                          0xCE5541E4                  // Red and Green buttons from teletext row                    

//  This part completelly is taken from IRremote example ReceiceAndSendHob2Hood
#define HOB_TO_HOOD_UNIT_MICROS     725
#define H2H_1   HOB_TO_HOOD_UNIT_MICROS     // 725
#define H2H_2   (HOB_TO_HOOD_UNIT_MICROS*2) // 1450
#define H2H_3   (HOB_TO_HOOD_UNIT_MICROS*3) // 2175
#define H2H_4   (HOB_TO_HOOD_UNIT_MICROS*4) // 2900
#define H2H_5   (HOB_TO_HOOD_UNIT_MICROS*5) // 3625

// Each infrared command has specific sequence
// First entry is the length of the raw command
const uint16_t Fan1[]     PROGMEM { 15, H2H_2, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1, H2H_2, H2H_1, H2H_3, H2H_1 };
const uint16_t Fan2[]     PROGMEM { 9,  H2H_2, H2H_2, H2H_1, H2H_4, H2H_1, H2H_3, H2H_5, H2H_3, H2H_3 };
const uint16_t Fan3[]     PROGMEM { 9,  H2H_1, H2H_3, H2H_4, H2H_4, H2H_3, H2H_1, H2H_1, H2H_3, H2H_3 };
const uint16_t Fan4[]     PROGMEM { 13, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_2, H2H_1, H2H_3, H2H_1, H2H_1, H2H_2 };
const uint16_t FanOff[]   PROGMEM { 15, H2H_1, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_2, H2H_3, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1 };
const uint16_t LightOn[]  PROGMEM { 17, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_4, H2H_1, H2H_1, H2H_1, H2H_1, H2H_2 };
const uint16_t LightOff[] PROGMEM { 17, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1, H2H_1, H2H_1, H2H_3, H2H_1, H2H_1, H2H_1, H2H_2, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1 };

// global variables
int aiLastReadings[ARRAY_SIZE];                                       // array where to accumulate readings
int iLastReadingsCounter       = 0;                                   // count of fulfilled values in the array
int iLastReadingsIndex         = 0;                                   // actual index to replace actual value
long lTotal                    = 0L;                                  // sum of values 
unsigned int uiAverage         = 0;                                   // average from recent readings
unsigned int uiStatus          = 0;                                   // system status
unsigned long ulPrevActionTime = 0L;                                  // mark time, if command to hood is sent
unsigned long ulKettleStartTime;                                      // time, when kettle is switched on

void setup() {
  pinMode(ZMCT103C_PIN,INPUT);                                        // current's sensor pin
  pinMode(LED_BUILTIN,OUTPUT);                                        // use, just to do ilustration
  Serial.begin(115200);                                               // for debugging
  Serial.println(F("\n\n\n\n"));                                      // print blank lines
  memset((void *)aiLastReadings,0,ARRAY_SIZE);                        // set all cells to 0 
   // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing IRremote library version " VERSION_IRREMOTE));
  digitalWrite(LED_BUILTIN, LOW);                                     // switch led off 
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive Hob2Hood IR signals at pin " STR(IR_RECEIVE_PIN)));
  IrSender.begin();                                                    // Start with IR_SEND_PIN as send pin and enable feedback LED at default feedback LED pin
  Serial.println(F(" Send Hob2Hood IR signals at pin " STR(IR_SEND_PIN)));
  Serial.println(F("\n"));
}

/* 
 *  Read current's sensor value.  While it fits in kettle power consumption runs fan.
 *  Switch light on, if smth plugged-in and switched on
 *  To avoid fluctuations, use delta between max/min, not abolute value, use average from recent readings
 */
void loop() {    
  vGetIRCommand();                                                      // check infrared input 
  if (uiStatus & MANUAL_MODE) return;                                   // manipulation with Sony remote. No automation                                            
  unsigned int uiVal = uiReadZMCT103C();                                // read current's sensor 
  uiGetAverageValue(uiVal);                                             // get average from recent ARRAY_SIZE readings 
  if (millis() - ulPrevActionTime < ACTION_TIMEOUT) return;             // ignore this, if it happens too fast
  if (uiAverage > A_START) vSwitchLightOn();                            // first thing - switch light on
  else vSwitchLightOff();                                               // and last - switch it off 
  if ((uiAverage >=  A_MIN) && (uiAverage <= A_MAX))  vSwitchHoodOn();  // switch hood on if cuurent's level fits in kettle interval or increase speed
  else  vSwitchHoodOff();                                               // switch hood off, 
}

/*
 * read current's sensor
 * return delta between min and max instead of value, as it less suffer from fluctuations
 * approach was suggested in arduino forum by Wawa Brattain
 */
unsigned int uiReadZMCT103C(){
  unsigned int uiMaxPeak = 0;                                            // reset
  unsigned int uiMinPeak = 1023;                                         // reset
  unsigned long ulStartMillis = millis();                                // mark
  while (millis() - ulStartMillis < 16) {                                // 15ms is ~3/4 of 50Hz  
    unsigned int uiRawValue = analogRead(ZMCT103C_PIN);                  // sample continuously
    if (uiRawValue > uiMaxPeak) uiMaxPeak = uiRawValue;                  // getting max value
    if (uiRawValue < uiMinPeak) uiMinPeak = uiRawValue;                  // getting min value
  }
  return (uiMaxPeak - uiMinPeak);                                        // return delta
}

/*
* Switch hood light on
*/
void vSwitchLightOn(){
    if (uiStatus & LIGHT_IS_ON) return;                                  // if lights is already on- nothing to do
    uiStatus |= LIGHT_IS_ON;                                             // set bit on - as a sign lights are switched on
    Serial.println(F("Switch Light on"));
    vSendRemoteCommand(LightOn);                                         // send infrared command to hood
    ulPrevActionTime = millis();                                         // memorize eactivity time (this is used to prevent flickering)
}

/*
* switch hood lights off
*/
void vSwitchLightOff(){
    if (!(uiStatus & LIGHT_IS_ON)) return;                               // if lights is alraedy off- nothing to do
    uiStatus = 0;                                                        // remove bit - as a sign this is already done. No more manual mode, if such was 
    Serial.println(F("Switch Light off"));
    vSendRemoteCommand(LightOff);                                        // send infrared command
    ulPrevActionTime = millis();                                         // memorize activity time                          
}

/* 
 * Switch Hood on
 * Increase fan speed y time  by sending appropriate infrared command to hood
 */
void vSwitchHoodOn(){
   unsigned long ulNow = millis();                                        // what time is now
   if (!(uiStatus & FAN_1)){                                              // Fan is not running
      uiStatus |= FAN_1;                                                  // this is a first time, set status bit on
      ulKettleStartTime = ulPrevActionTime = ulNow;                       // mark last activity time
      Serial.println(F("Switch Fan Spped 1"));
      vSendRemoteCommand(Fan1);                                           // send command to hood to switch fan at level 1 
      digitalWrite(LED_BUILTIN, HIGH);    
      return;
   }
   if ((ulNow - ulKettleStartTime >= FAN2_TIME) && !(uiStatus & FAN_2)) { // time for level 2?
      uiStatus |= FAN_2;                                                  // mark it as done 
      Serial.println(F("Switch Fan Spped 2"));
      vSendRemoteCommand(Fan2);                                           // and send command to hood
      return;
   }
  if ((ulNow - ulKettleStartTime >= FAN3_TIME) && !(uiStatus & FAN_3)) {  // time to switch to level 3?
      uiStatus |= FAN_3;
      Serial.println(F("Switch Fan Spped 3"));
      vSendRemoteCommand(Fan3);
      return;
  }
  if ((ulNow - ulKettleStartTime >= FAN4_TIME) && !(uiStatus & FAN_4)) {   // max levelof speed. switch to few seconds before expected end  
      uiStatus |= FAN_4;                                                   // but it might help, steam continues to come 
      Serial.println(F("Switch Fan Spped 4"));
      vSendRemoteCommand(Fan4);                                            // send this command to hood
      return;
  }
}

/*
 * swith fan off independently of running speed
 * reset status
 */
void vSwitchHoodOff(){
     if (!(uiStatus & FAN_1)) return;                                         // if it is in waiting state
     digitalWrite(LED_BUILTIN, LOW);                                          // swith built-in led off                                      
     uiStatus &= ~(FAN_1|FAN_2|FAN_3|FAN_4);                                  // reset all status bits except light amd maunual mode (if it switched on) 
     Serial.println(F("Switch all Fans off"));
     vSendRemoteCommand(FanOff);                                              // Send infrared command to hood to switch all off
     ulPrevActionTime = millis();                                             // memorize activity time  to prevent flickering 
}

/* 
 * accumulate values and calculate the average from recent readings, 
 * but no more than recent ARRAY_SIZE instances 
 * returns average value (also stored in a global variable uiAverage)
 */
unsigned int uiGetAverageValue(unsigned int uiValue){
     lTotal -= aiLastReadings[iLastReadingsIndex];                             // previous value has to be replaced (0 initially)
     aiLastReadings[iLastReadingsIndex] = uiValue;                             // store new value 
     lTotal += long(uiValue);                                                  // add new value instead of previous one
     if (++iLastReadingsIndex >= ARRAY_SIZE) iLastReadingsIndex = 0;           // if end of array, start filling from beginning
     if (iLastReadingsCounter < ARRAY_SIZE) iLastReadingsCounter++;            // next one 
     uiAverage = (unsigned int) (lTotal/long(iLastReadingsCounter));           // calculate average value
     return uiAverage;                                                         // return aerage value
}

/* 
 * Send command to hood. Hob2Hood protocol
 * It is based on ReceiveAndSendHob2Hood example by Armin Joachimsmeyer
 */ 
void vSendRemoteCommand(const uint16_t *pCommand){
     uint16_t tLengthOfRawCommand = pgm_read_word(pCommand);                      // length is the 1. word in array
     const uint16_t *tAddressOfRawCommandSequence = pCommand + 1;                 // Raw sequence starts at the 2. word of array
     Serial.println(" Send remote command");
     IrSender.sendRaw_P(tAddressOfRawCommandSequence, tLengthOfRawCommand, 38);   // send it out
}

/*
 * receive IR commands  
 * As Infraed sygnal receiver module is attached to pin 2, it receives just broadcest signal
 * it might come from 2 sources;  Arduino infrared Sender or Sony TV remote
 * Sony TV remote has typically unused buttons: red and green in teletext buttons row
 * RED - cancel/switch all off independently of status
 * GREEN - loop:  LIGHT ON->FAN1->FAN2->FAN3->FAN4->ALL OFF->starts from beginning
 * Hob2Hood commnds - confirms sending through printing to serial.
 */ 
void vGetIRCommand(){
   if (IrReceiver.decode()) {
        IrReceiver.resume();                                   // Early enable receiving of the next IR frame
        IrReceiver.printIRResultShort(&Serial);                // Print it out
        switch (IrReceiver.decodedIRData.decodedRawData) {     // Finally, check the received data and perform actions according to the received command
          case HOB_TO_HOOD_HASH_CODE_FAN_OFF:                  // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation 
            Serial.println(F("Confirm:FANS OFF"));
            break;
          case HOB_TO_HOOD_HASH_CODE_FAN_1:                    // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation
            Serial.println(F("Confirm:FAN 1 ON"));
            break;
          case HOB_TO_HOOD_HASH_CODE_FAN_2:                    // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation
            Serial.println(F("Confirm:FAN 2 ON"));
            break;
          case HOB_TO_HOOD_HASH_CODE_FAN_3:                    // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation
            Serial.println(F("Confirm:FAN 3 ON"));
            break;
          case HOB_TO_HOOD_HASH_CODE_FAN_4:                    // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation     
            Serial.println(F("Confirm:FAN 4 ON"));
            break;
         case HOB_TO_HOOD_HASH_CODE_LIGHT_ON:                  // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation
            Serial.println(F("Confirm:LIGHT ON"));
            break;
         case HOB_TO_HOOD_HASH_CODE_LIGHT_OFF:                 // HOB TO HOOD  signal come as just broadcasred signal by Arduino.  Just for confirmation
            Serial.println(F("Confirm:LIGHT OFF"));
            break;
         case SONY_RED:                                        // red button (teletext row) on Sony TV remote is pressed. switch all off
            if (millis() - ulPrevActionTime < 1000) break;     // ignore long press
            ulPrevActionTime = millis();                       // just to prevent repetitions
            Serial.println(F("Remote:SONY RED"));                                         
            vSwitchHoodOff();                                  // swith fan and 
            vSwitchLightOff();                                 // light off
            break;
         case SONY_GREEN:                                      // green button (teletext row) on Sony TV remote is pressed. Loop: light, fan1,2,3,4, all off,...
            if (millis() - ulPrevActionTime < 1000) break;     // one press in one scond
            Serial.println(F("Remote:SONY GREEN"));
            ulPrevActionTime = millis();                       // memorize activity time (this is used to prevent flickering)
            if (!(uiStatus & LIGHT_IS_ON)) {                   // start with light
               uiStatus |= (LIGHT_IS_ON|MANUAL_MODE);          // set bit on, set manual mode on 
               Serial.println(F("Ask:Light on"));
               vSendRemoteCommand(LightOn);                    // Send infrared commnd to hood
               break;
            }
            if (!(uiStatus & FAN_1)) {                         // fan1 is not switched on. do it now
               uiStatus |= FAN_1;                              // set bit on 
               Serial.println(F("Ask:FAN1 on"));
               vSendRemoteCommand(Fan1);                       // Send infrared commnd to hood
               break;
            }
            if (!(uiStatus & FAN_2)) {
               uiStatus |= FAN_2;                              // set bit on 
               Serial.println(F("Ask:FAN2 on"));
               vSendRemoteCommand(Fan2);                       // Send infrared commnd to hood
               break;
            }
            if (!(uiStatus & FAN_3)) {
               uiStatus |= FAN_3;                              // set bit on 
               Serial.println(F("Ask:FAN3 on"));
               vSendRemoteCommand(Fan3);                       // Send infrared commnd to hood
               break;
            }  
            if (!(uiStatus & FAN_4)) {
               uiStatus |= FAN_4;                              // set bit on 
               Serial.println(F("Ask:FAN4 on "));
               vSendRemoteCommand(Fan4);                       // Send infrared commnd to hood
               break;
            }
            else {
               uiStatus = 0;
               Serial.println(F("Ask:All off "));
               vSendRemoteCommand(FanOff);                      // Send infrared commnd to hood 
               vSendRemoteCommand(LightOff);                    // Send infrared commnd to hood
               break;
            }
            break;
         default:
            Serial.print(F(" *** unknown infrared command ***"));
            break;
        }
   }
}

