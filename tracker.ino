/* Example for the ANT+ Library @ https://github.com/brodykenrick/ANTPlus_Arduino
Copyright 2013 Brody Kenrick.

Developed for http://retrorunnerreadout.blogspot.com

Interfacing of Garmin ANT+ device (via a cheap Nordic nRF24AP UART module) to an Arduino.

Opens an ANT+ channel listening for HRM. Prints out computed hear rate.

Hardware
An Arduino Pro Mini 3v3 connected to this nRF24AP2 module : http://www.goodluckbuy.com/nrf24ap2-networking-module-zigbee-module-with-ant-transceiver-.html

The connector on nRF24AP2 board is (looking from the front, pin 1 is marked []):
[]GND(=VSS) | VDD(=3.3 volts)
UART_TX   | UART_RX
!(SUSP)   | SLEEP
RTS       | !(RESET)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'antplus' below.
*/

#define __SW012__


#define __GPS_GSM_Shield__
#undef  __TIMER1__


#if defined(__GPS_GSM_Shield__)
#include <TinyGPS++.h> 
#include <GSM_Shield.h>
#endif

#include <Arduino.h>
// watchdog
#include <avr/wdt.h>

// TimerOne
#include <TimerOne.h>


#undef AVERAGE_VALUES 
//#undef NDEBUG
#define __ASSERT_USE_STDERR
#include <assert.h>

#define HW_UART_MEGA2650 //!< H/w UART (i.e. Serial) instead of software serial. NOTE: There seems to be issues in not getting as many broadcast packets when using hardware serial.........
#define LCD_I2C

#if defined(LCD_I2C)
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
void              DisplayLCD();
#endif

#if !defined(HW_UART_MEGA2650)
#include <SoftwareSerial.h>
#endif

#include <ANTPlus.h>




//Logging macros
//********************************************************************
#define   USE_SERIAL_CONSOLE 
#define   SERIAL_DEBUG
#define   INFO_DEBUG

#if !defined(USE_SERIAL_CONSOLE)
//Disable logging under these circumstances
#undef SERIAL_DEBUG
#undef INFO_DEBUG
#endif

//F() stores static strings that come into existence here in flash (makes things a bit more stable)
#ifdef SERIAL_DEBUG

#define SERIAL_DEBUG_PRINT(x)  	        (Serial.print(x))
#define SERIAL_DEBUG_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_DEBUG_PRINT_F(x)  	      (Serial.print(F(x)))
#define SERIAL_DEBUG_PRINTLN_F(x)	      (Serial.println(F(x)))
#define SERIAL_DEBUG_PRINT2(x,y)      	(Serial.print(x,y))
#define SERIAL_DEBUG_PRINTLN2(x,y)	    (Serial.println(x,y))

#else

#define SERIAL_DEBUG_PRINT(x)           
#define SERIAL_DEBUG_PRINTLN(x)         
#define SERIAL_DEBUG_PRINT_F(x)         
#define SERIAL_DEBUG_PRINTLN_F(x)
#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)

#endif


#ifdef INFO_DEBUG

#define SERIAL_INFO_PRINT(x)            (Serial.print(x))
#define SERIAL_INFO_PRINTLN(x)          (Serial.println(x))
#define SERIAL_INFO_PRINT_F(x)          (Serial.print(F(x)))
#define SERIAL_INFO_PRINTLN_F(x)        (Serial.println(F(x)))
#define SERIAL_INFO_PRINT2(x,y)         (Serial.print(x,y))
#define SERIAL_INFO_PRINTLN2(x,y)       (Serial.println(x,y))

#else

#define SERIAL_INFO_PRINT(x)           
#define SERIAL_INFO_PRINTLN(x)         
#define SERIAL_INFO_PRINT_F(x)         
#define SERIAL_INFO_PRINTLN_F(x)
#define SERIAL_INFO_PRINT2(x,y)
#define SERIAL_INFO_PRINTLN2(x,y)

#endif


//********************************************************************

#define ANTPLUS_BAUD_RATE (19200) //!< The moduloe I am using is hardcoded to this baud rate.


//The ANT+ network keys are not allowed to be published so they are stripped from here.
//They are available in the ANT+ docs at thisisant.com

//The ANT+ network key is 8 hex values and is shown below:
//0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45

//The ANT-FS key is 8 hex values and is shown below:
//0xA8, 0xA4, 0x23, 0xB9, 0xF5, 0x5E, 0x63, 0xC1

//The Public network key is 8 hex values and is shown below:
//0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1

// Suunto Sensor Network Key (HRM,....)
//#define NETWORK_KEY      0xB9, 0xAD, 0x32 0x28 0x75 0x7E 0xC7 0x4D 


#define ANT_SENSOR_NETWORK_KEY {0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45}
#define ANT_GPS_NETWORK_KEY    {0xa8, 0xa4, 0x23, 0xb9, 0xf5, 0x5e, 0x63, 0xc1}


#if !defined( ANT_SENSOR_NETWORK_KEY ) || !defined(ANT_GPS_NETWORK_KEY)
#error "The Network Keys are missing. Better go find them by signing up at thisisant.com"
#endif

// ****************************************************************************
// ******************************  GLOBALS  ***********************************
// ****************************************************************************


PROGMEM static const int RTS_PIN      = 13; // 13; //!< RTS on the nRF24AP2 module
PROGMEM static const int RTS_PIN_INT  = 0; //!< The interrupt equivalent of the RTS_PIN


#if !defined(HW_UART_MEGA2650)
PROGMEM static const int RX_PIN       = 2; //Ditto
PROGMEM static const int TX_PIN       = 3; //Using software serial for the UART
static SoftwareSerial ant_serial(RX_PIN, TX_PIN); // RXArd, TXArd -- Arduino is opposite to nRF24AP2 module
#else
//Using Hardware Serial (0,1) instead
#endif


static  ANTPlus        antplus   = ANTPlus( RTS_PIN, 10/*SUSPEND*/, 11/*SLEEP*/, 12/*RESET*/ );


//ANT Channel config for HRM
static ANT_Channel hrm_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,            /* 0 */
  DEVCE_TIMEOUT,             /* N * 2.5 : 12 > 30 seconds */   
  DEVCE_TYPE_HRM,            /* 120 */
  DEVCE_SENSOR_FREQ,         /* 2400 + 57 = 2457 */
  DEVCE_HRM_LOWEST_RATE,     /* 32280 Message period */
  ANT_SENSOR_NETWORK_KEY,    /* //0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 */
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};

//ANT Channel config for Bike Power Meter sensor
static ANT_Channel powermeter_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,           /* 0 */
  DEVCE_TIMEOUT,            /* N * 2.5 : 12 > 30 seconds */ 
  DEVCE_TYPE_POWERM,        /* 11 or (0x0B) for the Power Meter */
  DEVCE_SENSOR_FREQ,        /* 2400 + 57 = 2457 */
  DEVCE_POWERM_LOWEST_RATE, /* Message period, decimal 8182 (4.0049 Hz).  */
  ANT_SENSOR_NETWORK_KEY,   /* 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 */
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};

//ANT Channel config for Bike Speed and Cadence sensor
static ANT_Channel bsc_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,           /* 0 */
  DEVCE_TIMEOUT,            /* N * 2.5 : 12 > 30 seconds */ 
  DEVCE_TYPE_BSC,           /* 121 or (0x79) for the COMBO */
  DEVCE_SENSOR_FREQ,        /* 2400 + 57 = 2457 */
  DEVCE_BSC_LOWEST_RATE,    /* 8086 (4.05hz) Message Period */
  ANT_SENSOR_NETWORK_KEY,   /* 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 */
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};

// Garmin Footpod
static ANT_Channel sdm_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,           /* 0 */
  DEVCE_TIMEOUT,            /* N * 2.5 : 12 > 30 seconds */ 
  DEVCE_TYPE_SDM,           /* 124 or (0x7C)  */
  DEVCE_SENSOR_FREQ,        /* 2400 + 57 = 2457 */
  DEVCE_SDM_LOWEST_RATE,    /* 16268 (2.01hz) Message Period */
  ANT_SENSOR_NETWORK_KEY,   /* 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 */
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};


// For future developments, can use Forerunner GPS
static ANT_Channel forerunner_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_GPS,
  DEVCE_GPS_FREQ,
  DEVCE_GPS_RATE,
  ANT_GPS_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};


#if defined(__SW012__)

#define _BIKE_WKO_   
#undef _RUNNING_WKO_  
 

#if defined(_BIKE_WKO_)

//This is how we tell the system which channels to establish
static ANT_Channel * channels_to_setup[ANT_DEVICE_NUMBER_CHANNELS] =
{
  &hrm_channel,
  &powermeter_channel,
  &bsc_channel,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

#endif /* _BIKE_WKO_ */

#if defined(_RUNNING_WKO_)

//This is how we tell the system which channels to establish
static ANT_Channel * channels_to_setup[ANT_DEVICE_NUMBER_CHANNELS] =
{
  &hrm_channel,
  &sdm_channel, 
  NULL, 
  NULL, 
  NULL,
  NULL,
  NULL,
  NULL
};

#endif /* _RUNNING_WKO_ */

uint8_t channels_to_configure = 0;
uint8_t channels_configured   = 0;


#endif /* __SW012__ */


uint8_t   first_time =1;
uint8_t   count =0;
uint32_t  tmp=0, dock=0;

uint16_t cadence_event_time, cadence_event_time0, Tc;
uint16_t cadence_revolution_count, cadence_revolution_count0;
uint16_t speed_event_time, speed_event_time0,   Ts;
uint16_t speed_revolution_count, speed_revolution_count0;
float    RPM, SPD;  
uint16_t Pow, HRM;
uint8_t  V,C, rpm_timeout, spd_timeout;

// Bike string for LCD
String   HRM_text;
String   PWR_text;
String   RPM_text;
String   SPD_text;

// Running string for LCD
String   FP_text;
String   ALT_text;

static int last_inst_speed_int_mps       = -1;
static int last_inst_speed_frac_mps_d256 = -1;
static int last_inst_speed_mps           = -1;
float      footpod_speed;

static unsigned long int cumulative_distance      = 0;
static int               prev_msg_distance        = -1;

static unsigned long int cumulative_stride_count  = 0;
static int               prev_msg_stride_count    = -1;


#if defined(__GPS_GSM_Shield__)
String    SMS_text;
uint32_t  incrementer=0;
#define   SMS_PERIOD   1000   /* 1000 x delay(50) = 50s */
#endif

#define RPM_TIMEOUT 10
#define SPD_TIMEOUT 10


uint8_t flagHRM=false;
uint8_t flagPOW=false;
uint8_t flagBSC=false;
uint8_t flagSDM=false;

uint8_t wdt_hrm=false ;
uint8_t wdt_pow=false ;
uint8_t wdt_bsc=false ;
uint8_t wdt_sdm=false ;


#if defined(__GPS_GSM_Shield__)
// **************************************************************************************************
// *********************************  GPS Setting  **************************************************
// **************************************************************************************************

PROGMEM static const uint32_t GPSBaud = 38400;

// The TinyGPS++ object
TinyGPSPlus gps;
PROGMEM static const int      GPS_RX_DIGITAL_OUT_PIN = 6, GPS_TX_DIGITAL_OUT_PIN = 7;

#if !defined(HW_UART_MEGA2650)
SoftwareSerial                ss(GPS_RX_DIGITAL_OUT_PIN, GPS_TX_DIGITAL_OUT_PIN);
#else
//Using Hardware Mega2560 Serial2 instead 
#endif

uint8_t   location_gps      = 0; // 0 - False
uint8_t   date_gps          = 0; // 0 - False
uint8_t   time_gps          = 0; // 0 - False
uint8_t   altitude_gps      = 0; // 0 - False

float      gps_lat   ;
float      gps_lng   ;
float      gps_alt = 0; 
float      gps_alt_prev = 0;

uint8_t    gps_month ;
uint8_t    gps_day   ;
uint16_t   gps_year  ;
uint8_t    gps_hour  ;
uint8_t    gps_minute;
uint8_t    gps_second;
uint8_t    gps_centisecond;

// **************************************************************************************************
// *********************************  GSM Setting  **************************************************
// **************************************************************************************************

char number[]="06xxxxxxxx";  //  TWILIO number
GSM gsm;
PROGMEM static const uint32_t GSMBaud = 19200;
PROGMEM static const int      GSM_RX_DIGITAL_OUT_PIN = 4, GSM_TX_DIGITAL_OUT_PIN = 5;


#endif /* __GPS_GSM_Shield__ */





// **************************************************************************************************
// *********************************  Average Value  ************************************************
// **************************************************************************************************


#if defined(AVERAGE_VALUES)

/*
 *  Define some function in order to get an average value based 
 *  on MAX_ITEMS elements
 *  
 *  ie: Pow = (pow1 + pow2 ..... + powMAX_ITEMS) / MAX_ITEMS
 * 
 */

#define MAX_ITEMS    10

typedef struct circularQueue_s
{
    uint8_t     first;
    uint16_t     data[MAX_ITEMS];
} circularQueue_t;

void  initializeQueue (circularQueue_t *theQueue);
int   putItem         (circularQueue_t *theQueue, int theItemValue);

void initializeQueue(circularQueue_t *theQueue)
{
    int i;

    theQueue->first       =  0;
    for(i=0; i<MAX_ITEMS; i++)
    {
        theQueue->data[i] = 0;
    }        
    return;
}

int putItem(circularQueue_t *theQueue, int theItemValue)
{
    if(theQueue->first < MAX_ITEMS) {
      theQueue->data[theQueue->first] = theItemValue;
      theQueue->first += 1;
    } else {
        theQueue->first = 0;
        theQueue->data[theQueue->first] = theItemValue;
        theQueue->first = 1;
    }
}

circularQueue_t   power, heartrate, cadence, vitesse;
uint16_t readP=0, readH=0, readC=0, readV=0;

#endif /* AVERAGE_VALUES */

// **************************************************************************************************
// *********************************  SDM4 FootPOD  *************************************************
// **************************************************************************************************


// SDM -- 6.2.2
//Distance, time and stride count
int update_sdm_rollover( int MessageValue, unsigned long int * Cumulative, int * PreviousMessageValue )
{
  //Initialize CumulativeDistance to 0
  //Above is external to this function
  //PreviousMessageDistance is set to -1 to indicate no previous message -- external to this function
  //initialize PreviousMessageDistance to the distance in the first SDM data message.
  if((*PreviousMessageValue) == -1)
  {
    (*PreviousMessageValue) = MessageValue;
    //This assumes that the first measurement we get from device is at 'point 0' -- any first measurement is therefore ignored in the cumulative
  }
  else
  {
    //For each subsequent SDM sensor Data message
    //a. CumulativeDistance += MessageDistance – PreviousMessageDistance
    (*Cumulative) += (MessageValue - (*PreviousMessageValue));
  
    //b. If PreviousMessageDistance > MessageDistance, CumulativeDistance += 256m
    if ((*PreviousMessageValue) > MessageValue)
    {
      (*Cumulative) += 256; //All fields rollover on this amount
    }
    //c. PreviousMessageDistance = MessageDistance
    (*PreviousMessageValue) = MessageValue;
  }
  return (*Cumulative);
}


// **************************************************************************************************
// *********************************  ISRs  *********************************************************
// **************************************************************************************************


volatile int rts_ant_received = 0; //!< ANT RTS interrupt flag see isr_rts_ant()


//! Interrupt service routine to get RTS from ANT messages

#if defined(__AVR_ATmega2560__)

ISR(PCINT0_vect) 
{
  rts_ant_received = 1;
}

#else  /* Arduino UNO */


void isr_rts_ant()
{
  rts_ant_received = 1;
}

#endif


// **************************************************************************************************
// ***********************************  ANT+  Process Packet ****************************************
// **************************************************************************************************

void process_packet( ANT_Packet * packet )
{
#if defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
    //This function internally uses Serial.println
    //Only use it if the console is available and if the ANTPLUS library is in debug mode
    antplus.printPacket( packet, false );
#endif //defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
   
   
  switch ( packet->msg_id )
  {

    case MESG_RESPONSE_EVENT_ID:
    {

      const ANT_Broadcast * broadcast = (const ANT_Broadcast *) packet->data;
      const ANT_DataPage * dp = (const ANT_DataPage *) broadcast->data;

      if (packet->length == 3)
      {
        
        
        // A  receive channel has timed out on searching (ant.c re-open on this EVENT_RX_SEARCH_TIMEOUT)
        // Re-open without waiting for EVENT_CHANNEL_CLOSED
        
        if (packet->data[2] == EVENT_RX_SEARCH_TIMEOUT /* 0x1 */ ) {
            
                SERIAL_INFO_PRINT_F(" EVENT_RX_SEARCH_TIMEOUT(0x1) : CHAN "); SERIAL_INFO_PRINTLN(broadcast->channel_number);  

                // HRM re-opne

                if (broadcast->channel_number == hrm_channel.channel_number) 
                {
                    // re-open channel
                    antplus.reopen_channel( &hrm_channel );
                    if (hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE) {
                        wdt_hrm = false ;  
                        SERIAL_INFO_PRINT(" RE-OPEN CHAN "); SERIAL_INFO_PRINTLN(broadcast->channel_number);
                    } else 
                        wdt_hrm = true ;
                }

                // POW re-open
                
                if (broadcast->channel_number == powermeter_channel.channel_number)
                {
                    // re-open channel
                    antplus.reopen_channel( &powermeter_channel );
                    if (powermeter_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE) {
                      wdt_pow = false ;  
                      SERIAL_INFO_PRINT(" RE-OPEN CHAN "); SERIAL_INFO_PRINTLN(broadcast->channel_number);
                    } else
                      wdt_pow = true ;
                }

                // BSC re-open
      
                if (broadcast->channel_number == bsc_channel.channel_number)
                {
                    // re-open channel
                    antplus.reopen_channel( &bsc_channel );
                    if (bsc_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE) {
                      wdt_bsc = false ;  
                      SERIAL_INFO_PRINT(" RE-OPEN CHAN "); SERIAL_INFO_PRINTLN(broadcast->channel_number);
                      // reset timing
                      first_time = 1;
                    } else
                      wdt_bsc = true ;
                }  

                // FootPOD re-open
      
                if (broadcast->channel_number == sdm_channel.channel_number)
                {
                    // re-open channel
                    antplus.reopen_channel( &sdm_channel );
                    if (sdm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE) {
                      wdt_sdm = false ;  
                      SERIAL_INFO_PRINT(" RE-OPEN CHAN "); SERIAL_INFO_PRINTLN(broadcast->channel_number);
                      
                    } else
                      wdt_sdm = true ;
                }  
                
        }  

        // A receive channel missed a message which it was expecting. Not great, but not the end of the world
      
        if (packet->data[2] == EVENT_RX_FAIL /*0x2*/ ) {
           SERIAL_DEBUG_PRINT(" EVENT_RX_FAIL(0x2) : CHAN "); SERIAL_DEBUG_PRINTLN(broadcast->channel_number);  
        }  

        // The channel has been successfully closed. 
        
        if (packet->data[2] ==  EVENT_CHANNEL_CLOSED /* 0x7 */) {
            SERIAL_DEBUG_PRINT(" EVENT_CHANNEL_CLOSED(0x7) : CHAN  "); SERIAL_DEBUG_PRINTLN(broadcast->channel_number);  
        }

        // The channel has dropped to search mode after missing too many messages.
        
        if (packet->data[2] == EVENT_RX_FAIL_GO_TO_SEARCH /*0x8*/ ) {
            SERIAL_DEBUG_PRINT(" EVENT_RX_FAIL_GO_TO_SEARCH(0x8) :  CHAN  "); SERIAL_DEBUG_PRINTLN(broadcast->channel_number);  
        }   

        // Two channels have drifted into each other and overlapped in time on the 
        // device causing one channel to be blocked.
        
        if (packet->data[2] == EVENT_CHANNEL_COLLISION /*0x9*/ ) {
            SERIAL_DEBUG_PRINT(" EVENT_CHANNEL_COLLISION(0x9) : CHAN "); SERIAL_DEBUG_PRINTLN(broadcast->channel_number);  
        }   

        if (packet->data[2] > 0x9 ) {
            
            SERIAL_DEBUG_PRINT(" MESG_RESPONSE_EVENT_ID val= "); SERIAL_DEBUG_PRINT(packet->data[2]);  
            SERIAL_DEBUG_PRINT(" on CHAN "); SERIAL_DEBUG_PRINTLN(broadcast->channel_number);  
        }   
      }
    }
    break;
     
    case MESG_BROADCAST_DATA_ID: /* 0x4E hex, 78 dec */
    {

      const ANT_Broadcast * broadcast = (const ANT_Broadcast *) packet->data;
      const ANT_DataPage * dp = (const ANT_DataPage *) broadcast->data;



      //Update received data from HRM
      if( broadcast->channel_number == hrm_channel.channel_number )
      {
        hrm_channel.data_rx = true;
        //To determine the device type -- and the data pages -- check channel setups
        if(hrm_channel.device_type == DEVCE_TYPE_HRM /* 120 */)
        {

            switch(dp->data_page_number)
            {
              case DATA_PAGE_HEART_RATE_0:
              case DATA_PAGE_HEART_RATE_0ALT:
              case DATA_PAGE_HEART_RATE_1:
              case DATA_PAGE_HEART_RATE_1ALT:
              case DATA_PAGE_HEART_RATE_2:
              case DATA_PAGE_HEART_RATE_2ALT:
              case DATA_PAGE_HEART_RATE_3:
              case DATA_PAGE_HEART_RATE_3ALT:
              case DATA_PAGE_HEART_RATE_4:
              case DATA_PAGE_HEART_RATE_4ALT:
              
              {
                //As we only care about the computed heart rate
                // we use a same struct for all HRM pages
                const ANT_HRMDataPage * hrm_dp = (const ANT_HRMDataPage *) dp;   // CAST TO HRM DATA PAGE structure
                
                
                HRM = hrm_dp->computed_heart_rate ;
 
#if defined(AVERAGE_VALUES)
                putItem(&heartrate, HRM);
#endif

              }
              break;
  
              default:
                  //SERIAL_DEBUG_PRINT_F(" HRM DP# ");
                  //SERIAL_DEBUG_PRINTLN( dp->data_page_number );
                break;
            }
        }
      }



      //Update received data from Power Meter
      if( broadcast->channel_number == powermeter_channel.channel_number )
      {
  
          powermeter_channel.data_rx = true;
  
          //To determine the device type -- and the data pages -- check channel setups
          if(powermeter_channel.device_type == DEVCE_TYPE_POWERM /* 0x0B for the PowerMeter */)
          {
  
  
                if (dp->data_page_number == DATA_PAGE_POWER_METER_16 ) 
                {
                        const ANT_PMDataPage * pm_dp = (const ANT_PMDataPage *) dp;  
        
                        //SERIAL_DEBUG_PRINT_F( " Pow[w] = ");
                        Pow = ((pm_dp->inst_power_lsb ) | ((pm_dp->inst_power_msb & 0xFF) << 8));
  
  #if defined(AVERAGE_VALUES)
                        putItem(&power, Pow);
  #endif
  
               } else {
                    //SERIAL_DEBUG_PRINT_F(" PWR DP# ");
                    //SERIAL_DEBUG_PRINTLN( dp->data_page_number );
               }     
  
          }
      }



      //Update received data from Bike Speed and Cadence combo (BSC)
      if( broadcast->channel_number == bsc_channel.channel_number )
      {
          bsc_channel.data_rx = true;
  
         //To determine the device type -- and the data pages -- check channel setups
          if(bsc_channel.device_type == DEVCE_TYPE_BSC /* 121 for the COMBO */)
          {
  
                  const ANT_BSCDataPage * bsc_dp = (const ANT_BSCDataPage *) dp;   
  
                  // Cadence
                  cadence_event_time        = ((bsc_dp->cadence_event_time_lsb & 0xFF) | ((bsc_dp->cadence_event_time_msb & 0xFF) << 8)); 
                  cadence_revolution_count  = ((bsc_dp->cadence_rev_count_lsb  & 0xFF) | ((bsc_dp->cadence_rev_count_msb & 0xFF)  << 8));
  
                  // Speed
  
                  speed_event_time          = ((bsc_dp->speed_event_time_lsb   & 0xFF) | ((bsc_dp->speed_event_time_msb & 0xFF)   << 8));
                  speed_revolution_count    = ((bsc_dp->speed_rev_count_lsb    & 0xFF) | ((bsc_dp->speed_rev_count_msb & 0xFF)    << 8));
  
   
                  if(first_time) {
  
                      // Cadence (T0)
                      cadence_event_time0        = cadence_event_time;
                      cadence_revolution_count0  = cadence_revolution_count;
  
                      // Speed (T0)
                      speed_event_time0          = speed_event_time;   
                      speed_revolution_count0    = speed_revolution_count;
  
  
                      first_time = 0;
                    
                  }  else {
  
                      // RPM Algorithm Calculation
                      // =========================
  
                      // Until the next pulse arrives (C = 0) or the timeout expires (RPM_TIMEOUT), show the last 
                      // speed value. When the pulse arrives (C != 0), calculate the value. 
                      
                      Tc    = (cadence_event_time - cadence_event_time0); /* time of 1 crank revolution */
                      C     = (cadence_revolution_count - cadence_revolution_count0); 
  
                       // Bugfix: 
                      // C is supposed to be always 1, but observed that some time its value is
                      // greater then 1 and as a consequence Tc is doubled (at cadence constant)
                      // or tripled if V= 2,3....
  
  
                      if ( C ) { // we had C revolution on the crank, with C=1,2,3
  
                           // Bugfix
                          Tc =  (float) Tc/C;
                        
                          RPM =  ((float) 172.5 * ((float)360/Tc));  // RPM as angular speed x Lenght(crank)
  #if defined(AVERAGE_VALUES)
                          putItem(&cadence, RPM);
  #endif
                          rpm_timeout = 0;
                      
                      } else {
  
                          rpm_timeout++;
                          // At 120RPM we have a pulse each 500 ms
                          // At 90RPM  we have a pulse each 667 ms
                          // At 60RPM, we have a pulse each 1000 ms
                          // At 30RPM, we have a pulse each 2000 ms
                          // RPM_TIMEOUT need to take in account the slowest RPM (2000 ms)
                          if (rpm_timeout > RPM_TIMEOUT )
                          {
                              RPM=0;
                              rpm_timeout =0;
                          }
                      
                      }  
  
                      // Speed Algorithm Calculation
                      // ===========================
  
                      // Until the next pulse arrives (V = 0) or the timeout expires (SPD_TIMEOUT), show the last 
                      // speed value. When the pulse arrives (V != 0), calculate the value. 
                      
  
                      Ts    = (speed_event_time - speed_event_time0) ;    /* time of 1 wheel revolution */
                      V     = (speed_revolution_count - speed_revolution_count0); 
  
                      // Bugfix: 
                      // V is supposed to be always 1, but observed that some time its value is
                      // greater then 1 and as a consequence Ts is doubled (at speed constant)
                      // or tripled if V= 2,3....
  
                      if ( V ) { // we had V revolution on the wheel (2.11 metre) ... V=1,2,....
  
                          // Bugfix
                          Ts =  (float) Ts/V;
                        
                          SPD = (float) 2.11*((float)1/Ts)*3600;
                          
                          
  #if defined(AVERAGE_VALUES)
                          putItem(&vitesse, SPD);  
  #endif
                          spd_timeout = 0;
                      
                      } else {
  
                          spd_timeout++;
                          if (spd_timeout > SPD_TIMEOUT )
                          {
                              SPD=0;
                              spd_timeout =0;
                          }
                      
                      }  
  
  
                      cadence_event_time0         = cadence_event_time;
                      cadence_revolution_count0   = cadence_revolution_count;
  
                      speed_event_time0           = speed_event_time;   
                      speed_revolution_count0     = speed_revolution_count;
  
                  }
          }
      } 
   


      //Update received data from FootPOD (SDM4)
      if( broadcast->channel_number == sdm_channel.channel_number )
      {

          sdm_channel.data_rx = true;
          
          //To determine the device type -- and the data pages -- check channel setups
          if(sdm_channel.device_type == DEVCE_TYPE_SDM /* 124 */)
          {

              switch(dp->data_page_number)
              {
                  case DATA_PAGE_SPEED_DISTANCE_1:
                  {


                        const ANT_SDMDataPage1 * sdm_dp = (const ANT_SDMDataPage1 *) dp;   // CAST TO SDM DATA PAGE 1 structure

                        //  byte last_time_frac; //  1/200 of a second
                        //  byte last_time_int;
                        //  byte distance_int;
                        //  byte inst_speed_int:4;
                        //  byte distance_frac:4;  //  1/16 of metre
                        //  byte inst_speed_frac;
                        //  byte stride_count;
                        //  byte update_latency;

                        SERIAL_INFO_PRINT_F( "SD[1] : ");
                        //Time

                        // Distance
                        SERIAL_INFO_PRINT_F( "Distance = ");
                        SERIAL_INFO_PRINT( sdm_dp->distance_int );
                        SERIAL_INFO_PRINT_F( " : ");
                        SERIAL_INFO_PRINT( sdm_dp->distance_frac );
                  
                        //Speed 
                        SERIAL_INFO_PRINT_F( " | Inst Speed = ");
                        SERIAL_INFO_PRINT( sdm_dp->inst_speed_int );
                        SERIAL_INFO_PRINT_F( " : ");
                        SERIAL_INFO_PRINT( sdm_dp->inst_speed_frac );
                  
                  
                        last_inst_speed_mps = ((float)sdm_dp->inst_speed_int) + (sdm_dp->inst_speed_frac/256.0);
                        last_inst_speed_int_mps       = sdm_dp->inst_speed_int;
                        last_inst_speed_frac_mps_d256 = sdm_dp->inst_speed_frac;

                        footpod_speed = (float) last_inst_speed_mps * ((float)1/1000)*3600;

       
                        // Stride
                        SERIAL_INFO_PRINT_F(" | Stride count = ");
                        SERIAL_INFO_PRINT( sdm_dp->stride_count );
                  
                        //Latency
                  
                        //Processed
                        SERIAL_INFO_PRINT_F( " | ");
                        SERIAL_INFO_PRINT_F( " CumStrides = ");
                        
                        update_sdm_rollover( sdm_dp->stride_count, &cumulative_stride_count, &prev_msg_stride_count );
                  
                        SERIAL_INFO_PRINT( cumulative_stride_count );
                        SERIAL_INFO_PRINT_F( " | CumDistance = ");
                  
                        update_sdm_rollover( sdm_dp->distance_int, &cumulative_distance, &prev_msg_distance );
                  
                        SERIAL_INFO_PRINT( cumulative_distance );
                        SERIAL_INFO_PRINT_F( " | ");
                        SERIAL_INFO_PRINT_F( " Speed = ");
                        SERIAL_INFO_PRINT( footpod_speed );
                        SERIAL_INFO_PRINTLN( );
                  }                    
                  break;

                  case DATA_PAGE_SPEED_DISTANCE_2:
                  {

                        const ANT_SDMDataPage2 * sdm_dp = (const ANT_SDMDataPage2 *) dp;   // CAST TO SDM DATA PAGE 2 structure

                        // byte data_page_number;
                        // byte reserved1;
                        // byte reserved2;
                        // byte cadence_int;
                        // byte inst_speed_int:4;
                        // byte cadence_frac:4;
                        // byte inst_speed_frac;
                        // byte reserved6;
                        // byte status;

                        SERIAL_INFO_PRINT_F( "SD[2] : ");
                        //Reserved1
                        //Reserved2
                        
                        // Cadence
                        SERIAL_INFO_PRINT_F( "Cadence = ");
                        SERIAL_INFO_PRINT( sdm_dp->cadence_int );
                        SERIAL_INFO_PRINT_F( " : ");
                        SERIAL_INFO_PRINT( sdm_dp->cadence_frac );
                        
                        //Speed for DP1
                        SERIAL_INFO_PRINT_F( " | Inst Speed = ");
                        SERIAL_INFO_PRINT( sdm_dp->inst_speed_int );
                        SERIAL_INFO_PRINT_F( " : ");
                        SERIAL_INFO_PRINT( sdm_dp->inst_speed_frac );
                        
                        
                        last_inst_speed_mps = ((float)sdm_dp->inst_speed_int) + (sdm_dp->inst_speed_frac/256.0);
                        last_inst_speed_int_mps       = sdm_dp->inst_speed_int;
                        last_inst_speed_frac_mps_d256 = sdm_dp->inst_speed_frac;
                        
                        footpod_speed = (float) last_inst_speed_mps * ((float)1/1000)*3600;
                        
                        //Reserved6
                        //Status
                        //Processed
                        SERIAL_INFO_PRINT_F( " | ");
                        SERIAL_INFO_PRINT_F( "Speed = ");
                        SERIAL_INFO_PRINT( footpod_speed );
                        SERIAL_INFO_PRINTLN( );
                
                  }
                  break;

                  default:
                    break;
                  
              }

            
            
          }
    
        
      }

    
    }
    break;
    
    default:
     /* RX 0x6F (111 dec) - MESG_START_UP       */
     /* RX 0x54 (84 dec) - MESG_CAPABILITIES_ID */
     SERIAL_DEBUG_PRINT("Non-broadcast data received: ");
     SERIAL_DEBUG_PRINTLN(packet->msg_id);
      break;
      
  }
}


// **************************************************************************************************
// ************************************  Watchdog  **************************************************
// **************************************************************************************************


unsigned long resetTime = 0;
#define TIMEOUTPERIOD 10000                     // You can make this time as long as you want,
                                                // it's not limited to 8 seconds like the normal
                                                // watchdog
#define doggieTickle() resetTime = millis();    // This macro will reset the timer
void(* resetFunc) (void) = 0;                   //declare reset function @ address 0

void watchdogSetup()
{
    cli();                    // disable all interrupts
    wdt_reset();              // reset the WDT timer
    MCUSR &= ~(1<<WDRF);      // because the data sheet said to
    /*
    WDTCSR configuration:
    WDIE = 1 :Interrupt Enable
    WDE = 1  :Reset Enable - I won't be using this on the 2560
    WDP3 = 0 :For 1000ms Time-out
    WDP2 = 1 :bit pattern is 
    WDP1 = 1 :0110  change this for a different
    WDP0 = 0 :timeout period.
    */
    // Enter Watchdog Configuration mode:
    WDTCSR = (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings: interrupte enable, 0110 for timer
    WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
    sei();
    SERIAL_DEBUG_PRINTLN("finished watchdog setup");  // just here for testing
}

ISR(WDT_vect) // Watchdog timer interrupt, it is called each 1000ms (1s)
{ 

  // if TIMEOUTPERIOD (10s) expired, then Reboot
  if(millis() - resetTime > TIMEOUTPERIOD) {
    
    SERIAL_INFO_PRINTLN("This is where it would have rebooted");  // just here for testing
    //doggieTickle();                                          // take these lines out  
    //resetFunc();     // This will call location zero and cause a reboot.

    // if wdt_hrm == true => reopen_channel(hrm) FAILED, then Reboot
    if (wdt_hrm == true) resetFunc();
    // if wdt_pow == true => reopen_channel(pow) FAILED, then Reboot
    if (wdt_pow == true) resetFunc();
    // if wdt_bsc == true => reopen_channel(bsc) FAILED, then Reboot
    if (wdt_bsc == true) resetFunc();
    // if wdt_sdm == true => reopen_channel(sdm) FAILED, then Reboot
    if (wdt_sdm == true) resetFunc();
    
  }
  //else                                                       // these lines should
  //  Serial.println("Howdy");                                 // be removed also
}


#if defined(__TIMER1__)
// **************************************************************************************************
// ************************************  TimerOne ISR  **********************************************
// **************************************************************************************************

#define TIMER1PERIOD 20000000    

void timerIsr()
{
    // Toggle LED WILL USE A LED TO INDICATE SMS DELIVERY !!!!!
    //digitalWrite( 13, digitalRead( 13 ) ^ 1 );
    SERIAL_DEBUG_PRINT("timerIsr ms > "); SERIAL_DEBUG_PRINTLN( millis() );
}
#endif /* __TIMER1__ */


// **************************************************************************************************
// ************************************  Setup  *****************************************************
// **************************************************************************************************




void setup()
{
#if defined(USE_SERIAL_CONSOLE)
  Serial.begin(19200);
#endif //defined(USE_SERIAL_CONSOLE)

  SERIAL_DEBUG_PRINTLN("ANTPlus HRM Test!");
  SERIAL_DEBUG_PRINTLN_F("Setup.");

#if defined(LCD_I2C)
  // initialize the lcd 
  lcd.init();       
 // Print a message to the LCD.
  lcd.backlight();
  lcd.clear();
#endif    

#if defined(__GPS_GSM_Shield__)

#if defined(LCD_I2C) 
  lcd.setCursor(0, 0); lcd.print("> Search GSM ...");
#endif


  // prevent controller pins 4 and 5 from interfering with the comms from GSM
  pinMode(GSM_TX_DIGITAL_OUT_PIN, INPUT);
  pinMode(GSM_RX_DIGITAL_OUT_PIN, INPUT);

  // GSM setup
  gsm.TurnOn(pgm_read_word(&GSMBaud));          //module power on
  gsm.InitParam(PARAM_SET_1);                   //configure the module  
  gsm.Echo(0);                                  //enable AT echo 

#if defined(LCD_I2C) 
  lcd.setCursor(13, 0); lcd.print("ok!!");
  delay(1000);
  lcd.setCursor(0, 1); lcd.print("> Search GPS ...");
#endif 
  
  
  // GPS setup (need to be 38400)
  Serial2.begin(pgm_read_word(&GPSBaud)); 
  // prevent controller pins 6 and 7 from interfering with the comms from GPS
  pinMode(GPS_TX_DIGITAL_OUT_PIN, INPUT);
  pinMode(GPS_RX_DIGITAL_OUT_PIN, INPUT);
  connectGPS();


#if defined(LCD_I2C) 
  lcd.setCursor(13, 1); lcd.print("ok!!");
  delay(2000);
  lcd.clear();
#endif    

#endif /* __GPS_GSM_Shield__ */

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config.");

#if defined(AVERAGE_VALUES)
  initializeQueue(&power);
  initializeQueue(&heartrate);
  initializeQueue(&cadence);
  initializeQueue(&vitesse);
#endif 

#if defined(__AVR_ATmega2560__)

  // PIN CHANGE INTERRUPTS ON MEGA2560 FOR ANT+ RTS PIN 13 
  
  //The Pin change interrupt PCI2 will trigger if any enabled PCINT23:16 pin toggles, 
  //The Pin change interrupt PCI1 will trigger if any enabled PCINT15:8  pin  toggles,
  //The Pin change interrupt PCI0 will trigger if any enabled PCINT7:0 pin toggles. 
  
  //PCMSK2, PCMSK1 and PCMSK0 Registers control which pins contribute to the pin change interrupts.
  
  //PCI0:                          PCI1:                     PCI2:
    //PCINT7         PIN 13         PCINT15        N/A          PCINT23        PIN ANALOG15
    //PCINT6         PIN 12         PCINT14        N/A          PCINT22        PIN ANALOG14
    //PCINT5         PIN 11         PCINT13        N/A          PCINT21        PIN ANALOG13
    //PCINT4         PIN 10         PCINT12        N/A          PCINT20        PIN ANALOG12
    //PCINT3         PIN 50         PCINT11        N/A          PCINT19        PIN ANALOG11
    //PCINT2         PIN 51         PCINT10        PIN 14       PCINT18        PIN ANALOG10
    //PCINT1         PIN 52         PCINT9         PIN 15       PCINT17        PIN ANALOG9
    //PCINT0         PIN 53         PCINT8         PIN 0        PCINT16        PIN ANALOG8

  // The external interrupts (used as PCInt) and the pins they are wired to:
  //INT7:0
    //INT7             N/A
    //INT6             N/A
    //INT5             PIN 3  
    //INT4             PIN 2
    //INT3             PIN 18
    //INT2             PIN 19
    //INT1             PIN 20
    //INT0             PIN 21

     // Lets try to use PIN 13 as Pin Change Interrupt; it is mapped on PCINT7
  
     PCICR |= 0x01;              // PCIE0 - Pin Change Interrupt Enable 0
     PCMSK0 |= 0x80;             // Enable PCINT7 (PIN 13) on  Pin Change Mask Register 0


    // If we want to use PIN 2, it is mapped on INT4
    //EICRB |= 0x03;          //INT4, triggered on rising edge
    //EIMSK |= 0x10;          //Enable only INT4
    



    // EXTERNAL INTERRUPTS

    //The External Interrupts are connected to the following Pins when using attachInterrupt():
    //              Digital Pin
    //INT5 :        18
    //INT4 :        19
    //INT3 :        20
    //INT2 :        21
    //INT1 :        3
    //INT0 :        2

   // For Pin 2, need to pass 0
   // attachInterrupt(0, isr_rts_ant,  RISING);


#else /* Arduino UNO */

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each valid ANT message received from the host at the chip
  // RTS_PIN 13
  // RTS_PIN_INT 0
  attachInterrupt(RTS_PIN_INT, isr_rts_ant, RISING);
  

#endif


#if defined(HW_UART_MEGA2650)
  //Using hardware UART
  Serial1.begin(ANTPLUS_BAUD_RATE); 
  antplus.begin( Serial1 );
#else
  //Using soft serial
  ant_serial.begin( ANTPLUS_BAUD_RATE ); 
  antplus.begin( ant_serial );
#endif

#if defined(LCD_I2C)
  lcd.print("> GPS/GSM ready");
  delay(1000);
  lcd.clear();

  lcd.print("> Test ANT  ..");
#endif    

  testANTchip();  

#if defined(LCD_I2C)
  lcd.setCursor(12, 0); lcd.print("ok!!");
  delay(2000);
  lcd.clear();
#endif  
  

  //initialize watchdog
  watchdogSetup();



#if defined(__TIMER1__)
  //initialise TimerOne
  Timer1.initialize(TIMER1PERIOD);        // set a timer 15sec
  Timer1.attachInterrupt( timerIsr );     // attach the service routine here
#endif /* __TIMER1__ */

#if defined(__SW012__)
  for(int i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
  {
    if(channels_to_setup[i])
    {
      //For now we do a simple direct mapping
      channels_to_setup[i]->channel_number = i;
      SERIAL_INFO_PRINT_F("Configured to establish ANT channel #");
      SERIAL_INFO_PRINTLN( channels_to_setup[i]->channel_number );
      channels_to_configure += 1;
    }
  }

#endif

#if defined(LCD_I2C)
#if defined( _BIKE_WKO_ )  
  lcd.print("> Bike ON");
  delay(1000);
  lcd.clear();
#endif 
#if defined( _RUNNING_WKO_)  
  lcd.print("> Running ON");
  delay(1000);
  lcd.clear();
#endif  
#endif 

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config Finished. ");
  SERIAL_DEBUG_PRINTLN_F("Setup Finished.");
}

// **************************************************************************************************
// ************************************  Loop *******************************************************
// **************************************************************************************************

void loop()
{
  byte packet_buffer[ANT_MAX_PACKET_LEN];    /* 80 byte */
  ANT_Packet * packet = (ANT_Packet *) packet_buffer;
  MESSAGE_READ ret_val = MESSAGE_READ_NONE;
  int error;
  int reg;

  
  if(rts_ant_received == 1)
  {

    //tmp = millis();
    //lcd.setCursor(0, 0);
    //lcd.print("RTS > "); lcd.print(tmp);
 
    //SERIAL_DEBUG_PRINTLN_F("Received RTS Interrupt. ");
    antplus.rTSHighAssertion();
    //Clear the ISR flag
    rts_ant_received = 0;

    // Reset WD timer; we are receiving Ints
    doggieTickle(); 
    
  } 

#if !defined(HW_UART_MEGA2650)
    ant_serial.listen();
#endif  

  
  //Read messages until we get a none
  while( (ret_val = antplus.readPacket(packet, ANT_MAX_PACKET_LEN, 0 )) != MESSAGE_READ_NONE )
  {
    if((ret_val == MESSAGE_READ_EXPECTED) || (ret_val == MESSAGE_READ_OTHER))
    {
      //SERIAL_DEBUG_PRINT_F( "ReadPacket success = " );
      if( (ret_val == MESSAGE_READ_EXPECTED) )
      {
        //SERIAL_DEBUG_PRINTLN_F( "Expected packet" );
      }
      else
      if( (ret_val == MESSAGE_READ_OTHER) )
      {
        //SERIAL_DEBUG_PRINTLN_F( "Other packet" );
      }
      
      process_packet(packet);
      
#if defined(AVERAGE_VALUES)    
      if((count % MAX_ITEMS) == 0) {
          for(int i=0; i<MAX_ITEMS; i++) {
              readP += power.data[i];
              readH += heartrate.data[i];
              readC += cadence.data[i];
              readV += vitesse.data[i];
          }
          readP = readP/MAX_ITEMS ;
          readH = readH/MAX_ITEMS  ;
          readC = readC/MAX_ITEMS  ;
          readV = readV/MAX_ITEMS  ;
          
          SERIAL_DEBUG_PRINT_F( "AV(pow) = " );
          SERIAL_DEBUG_PRINT( readP );
          SERIAL_DEBUG_PRINT( " " );
          SERIAL_DEBUG_PRINT( Pow );
          SERIAL_DEBUG_PRINT_F( " AV(hr) = " );
          SERIAL_DEBUG_PRINT( readH );
          SERIAL_DEBUG_PRINT( " " );
          SERIAL_DEBUG_PRINT( HRM );
          SERIAL_DEBUG_PRINT_F( " AV(cad) = " );
          SERIAL_DEBUG_PRINT( readC );
          SERIAL_DEBUG_PRINT( " " );
          SERIAL_DEBUG_PRINT( RPM );
          SERIAL_DEBUG_PRINT_F( " AV(speed) = " );
          SERIAL_DEBUG_PRINT( readV );
          SERIAL_DEBUG_PRINT( " " );
          SERIAL_DEBUG_PRINTLN( SPD );

          readP = 0 ;
          readH = 0 ;
          readC = 0 ;
          readV = 0 ;
          count = 0 ;
      }
#endif /* AVERAGE_VALUES */



#if defined(LCD_I2C)
                DisplayLCD();
#endif  /* defined(LCD_I2C) */

                if (channels_configured == channels_to_configure)
                {

#if defined(_BIKE_WKO_)

                    SERIAL_INFO_PRINT_F( "Mode _BIKE_WKO_ > " );
                    SERIAL_INFO_PRINT_F( " HR = " );
                    SERIAL_INFO_PRINT( HRM );
                    SERIAL_INFO_PRINT_F( "  Pow = " );
                    SERIAL_INFO_PRINT( Pow );
                    SERIAL_INFO_PRINT_F( " RPM = " );
                    SERIAL_INFO_PRINT( (uint16_t)RPM );
                    SERIAL_INFO_PRINT_F( " SPD = " );
                    SERIAL_INFO_PRINTLN( SPD );
#endif                

#if defined(_RUNNING_WKO_)
    
                    SERIAL_INFO_PRINT_F( "Mode _RUNNING_WKO_ > " );  
                    SERIAL_INFO_PRINT_F( " HR = " );
                    SERIAL_INFO_PRINT( HRM );
                    SERIAL_INFO_PRINT_F( " SPD = " );
                    SERIAL_INFO_PRINTLN( footpod_speed );
#endif  
                }
    


      
    }
    else
    {
      if(ret_val == MESSAGE_READ_ERROR_MISSING_SYNC)
      {
        //Nothing -- allow a re-read to get back in sync
        SERIAL_DEBUG_PRINTLN_F("  MESSAGE_READ_ERROR_MISSING_SYNC "); 
      }
      else
      if(ret_val == MESSAGE_READ_ERROR_BAD_CHECKSUM)
      {
        //Nothing -- fully formed package just bit errors
        SERIAL_DEBUG_PRINTLN_F("  MESSAGE_READ_ERROR_BAD_CHECKSUM "); 
      }
      else
      {
        break;
      }
    }
 }


#if defined(__SW012__)

  // Must be called with the same channel until an error or established 
  //(i.e. don't start with a different channel in the middle -- one channel at a time) 

  
  for(int i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
  {
    if(channels_to_setup[i])
    {
      if(channels_to_setup[i]->channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
      {
        antplus.progress_setup_channel( channels_to_setup[i] );
        if(channels_to_setup[i]->channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
        {
          SERIAL_INFO_PRINT( channels_to_setup[i]->channel_number );
          SERIAL_INFO_PRINTLN_F( " - Established." );
          channels_configured += 1;
          delay(100);
          lcd.clear();
        }
        else
        if(channels_to_setup[i]->channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
        {
          SERIAL_INFO_PRINT( channels_to_setup[i]->channel_number );
          SERIAL_INFO_PRINTLN_F( " - Progressing." );
          lcd.setCursor(0, 0);  lcd.print("ch "); lcd.print(channels_to_setup[i]->channel_number); lcd.print(" Progressing");
        }
        else
        {
          SERIAL_DEBUG_PRINT( channels_to_setup[i]->channel_number );
          SERIAL_DEBUG_PRINTLN_F( " - ERROR!" );
        }

        // Assure that next channel is not called if channel "i" is still progressing
        break;
      }
    }
  }

  

#else

  // HRM

  if((hrm_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE) && (flagHRM==false) && (flagPOW==false) && (flagBSC==false) && (flagSDM==false) )
  {
    antplus.progress_setup_channel( &hrm_channel );
    if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT( hrm_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - HRM Established." );
      flagHRM = true;
    }
    else
    if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      SERIAL_INFO_PRINT( hrm_channel.channel_number );
      SERIAL_INFO_PRINTLN_F( " - HRM Progressing." );
    }
    else
    {
      //SERIAL_DEBUG_PRINT( hrm_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - HRM ERROR!" );
    }
  } 

  // POWER METER  (to execute only after HRM is established)

  if((powermeter_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE) && (flagHRM==true) && (flagPOW==false) && (flagBSC==false) && (flagSDM==false))
  {
    antplus.progress_setup_channel( &powermeter_channel );
    if(powermeter_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT( powermeter_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - POW Established." );
      flagPOW=true;
    }
    else
    if(powermeter_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      //SERIAL_DEBUG_PRINT( powermeter_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - POW Progressing." );
    }
    else
    {
      //SERIAL_DEBUG_PRINT( powermeter_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - POW ERROR!" );
    }
  } 



  // BSC (to execute only after HRM & POW established)

  if((bsc_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE) && (flagHRM==true) && (flagPOW==true) && (flagBSC==false) && (flagSDM==false))
  {
    antplus.progress_setup_channel( &bsc_channel );
    if(bsc_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT( bsc_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - BSC Established." );
      flagBSC=true;
    }
    else
    if(bsc_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      //SERIAL_DEBUG_PRINT( bsc_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - BSC Progressing." );
    }
    else
    {
      //SERIAL_DEBUG_PRINT( bsc_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - BSC ERROR!" );
    }
  } 


    // FootPOD (to execute only after HRM & POW & BSC established)

  if((sdm_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE) && (flagHRM==true) && (flagPOW==true) && (flagBSC==true) && (flagSDM==false))
  {
    antplus.progress_setup_channel( &sdm_channel );
    if(sdm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT( sdm_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - FootPOD Established." );
      flagSDM=true;
    }
    else
    if(sdm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      //SERIAL_DEBUG_PRINT( sdm_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - FootPOD Progressing." );
    }
    else
    {
      //SERIAL_DEBUG_PRINT( sdm_channel.channel_number );
      //SERIAL_DEBUG_PRINTLN_F( " - FootPOD ERROR!" );
    }
  } 

#endif
  

#if defined(__GPS_GSM_Shield__)
  incrementer ++;


#if defined(__SW012__)
  if ( (channels_configured == channels_to_configure) /* All connections Established */
        && ((incrementer % SMS_PERIOD) == 0) )    /* 5000 x delay(50) */
#else

  if ( (flagHRM==true) && (flagPOW==true) && (flagBSC==true) && (flagSDM==true) /* All connections Established */
        && ((incrementer % SMS_PERIOD) == 0) )    /* 300 x delay(100) */
#endif  
  {

      // GPS
      getGPS();

      SERIAL_INFO_PRINT_F("incrementer:  ");      
      SERIAL_INFO_PRINT(incrementer);
      SERIAL_INFO_PRINT_F(" Time  ");      
      SERIAL_INFO_PRINTLN(millis());

      if (location_gps && altitude_gps && date_gps && time_gps) {

          incrementer = 0;

          // Lat -90.123456 to +90.123456     (10 characters included sign et point)
          // Lon  -180.000000 to +180.000000  (11 characters included sign et point)
          // Alt  0.00 to 9999.99             (8 characters included sign et point)
          //gps_lat = -89.123456;
          //gps_lng = -180.123456;
          //gps_alt = -9999.1234;

          // Format SMS text message
          char Lat[12],Lon[12], Alt[10];
          dtostrf(gps_lat,1,6,Lat); 
          dtostrf(gps_lng,1,6,Lon);
          dtostrf(gps_alt,1,2,Alt);
         
          
          String text0 = ""; 

#if defined(_BIKE_WKO_)          
          String SMS_text  = text0 + Lat + ":" + Lon + ":" + Alt + ":" + gps_month + ":" + gps_day + ":" + gps_year + ":" 
          + gps_hour + ":" + gps_minute + ":" + gps_second + ":" + HRM + ":" + (uint8_t) RPM + ":" + (uint8_t)SPD + ":" + Pow + ":" + "B";
#endif
#if defined(_RUNNING_WKO_)          
          String SMS_text  = text0 + Lat + ":" + Lon + ":" + Alt + ":" + gps_month + ":" + gps_day + ":" + gps_year + ":" 
          + gps_hour + ":" + gps_minute + ":" + gps_second + ":" + HRM + ":" + 0 + ":" + footpod_speed + ":" + 0 + ":" + "R";
#endif

          // SMS_test = Lat:Lon:Alt:month:day:year:hours:minute:seconds:HRM:RPM:SPD:Pow:B/R
      
          SERIAL_INFO_PRINT_F("SMS_text:  ");      
          SERIAL_INFO_PRINT(SMS_text);
          SERIAL_INFO_PRINT_F(" Time  ");      
          SERIAL_INFO_PRINTLN(millis());

          reg=gsm.CheckRegistration();
          switch (reg){    

            case REG_NOT_REGISTERED:
                SERIAL_DEBUG_PRINTLN_F("GSM not registered");
              break;

            case REG_REGISTERED:
            
                SERIAL_DEBUG_PRINTLN_F("GSM module is registered");     
 
                // Send SMS
                SERIAL_INFO_PRINT_F("Send SMS to ");
                SERIAL_INFO_PRINTLN(number);
                      
                //error = gsm.SendSMS(number,(char *)&SMS_text[0]);  
                error = 1;
                if (error==0)  //Check status
                {
                  SERIAL_DEBUG_PRINTLN_F("SMS ERROR");
                }
                else
                {
                  SERIAL_INFO_PRINTLN_F("SMS OK");
                        
                  location_gps = 0; // 0 - False
                  date_gps = 0;     // 0 - False
                  time_gps = 0;     // 0 - False
                  altitude_gps = 0; // 0 - False
                }      
                break;

                case REG_NO_RESPONSE:
                  SERIAL_DEBUG_PRINTLN_F("GSM doesn't response");
                break;
                
                case REG_COMM_LINE_BUSY:
                  SERIAL_DEBUG_PRINTLN_F("GSM line is not free");
                break;

          }  /* end switch */
          
          //delay(2000);
        
          reg=gsm.IsRegistered();
          SERIAL_DEBUG_PRINT_F("GSM Registration ");
          SERIAL_DEBUG_PRINTLN(reg);
          
      } /* location_gps && altitude_gps && date_gps && time_gps */

    } /* All channel established & imcrementer = SMS_PERIOD */
    
#endif /* __GPS_GSM_Shield__ */
  

    // This is fundamental for correct behaviour, we will see what happen when integrating
    // GPS/GSM (they need delay too)
    delay(50);

    //if (dock > 0xFFFFF) dock=0;

#if defined(AVERAGE_VALUES) 
  count++;
#endif 

  
}

#if defined(LCD_I2C)
void DisplayLCD()
{

    if (channels_configured == channels_to_configure)
    {

#if defined(_BIKE_WKO_)

                  if (wdt_hrm) { lcd.setCursor(0, 0);  lcd.print("_?_"); }
                  if (wdt_pow) { lcd.setCursor(4, 0);  lcd.print("_?_"); }
                  if (wdt_bsc) { lcd.setCursor(8, 0);  lcd.print("_?_"); 
                               lcd.setCursor(12, 0); lcd.print("_?_"); }


                
                  lcd.setCursor(0, 0);  lcd.print("HRM");
                  lcd.setCursor(4, 0);  lcd.print("PWR");
                  lcd.setCursor(8, 0);  lcd.print("RPM");
                  lcd.setCursor(12, 0); lcd.print("SPD");
                 
                  if(HRM <10)                   { HRM_text = "  ";}
                  if((HRM >=10) && (HRM < 100)) { HRM_text = " ";}
                  if((HRM >=100) )              { HRM_text = "";}
                  
                  
                  if(Pow <10)                   { PWR_text = "  ";}
                  if((Pow >=10) && (Pow < 100)) { PWR_text = " ";}
                  if((Pow >=100) )              { PWR_text = "";}
                
               
               
                  if(RPM <10)                   { RPM_text = "  ";}
                  if((RPM >=10) && (RPM < 100)) { RPM_text = " ";}
                  if((RPM >=100) )              { RPM_text = "";}
 
                  if(SPD <10)                   { SPD_text = " ";}
                  if((SPD >=10) && (SPD < 100)) { SPD_text = "";}
                  if((SPD >=100) )              { SPD_text = "";}
                  
       
                  String LCD_text0 = ""; 
                  String LCD_text  = LCD_text0 + HRM_text + HRM + " " + PWR_text + Pow + " " + 
                                                 RPM_text + (uint16_t)RPM + " " + SPD_text + SPD;
          
                  lcd.setCursor(0,  1);  lcd.print(LCD_text);
#endif /* _BIKE_WKO_ */

#if defined(_RUNNING_WKO_)

                  char FP_speed[10], gps_a[10];
 
                  if (wdt_hrm) { lcd.setCursor(0, 0);  lcd.print("_?_"); }
                  if (wdt_sdm) { lcd.setCursor(4, 0);  lcd.print("_?_"); }

                
                  lcd.setCursor(0, 0);    lcd.print("HRM");
                  lcd.setCursor(5, 0);    lcd.print("SPD");
                  lcd.setCursor(12, 0);   lcd.print("ALT");
                 
                  if(HRM <10)                   { HRM_text = "  ";}
                  if((HRM >=10) && (HRM < 100)) { HRM_text = " ";}
                  if((HRM >=100) )              { HRM_text = "";}
                  
                  
                  if(footpod_speed <10)                             { FP_text = "  ";}
                  if((footpod_speed >=10) && (footpod_speed < 100)) { FP_text = " ";}
                  if((footpod_speed >=100) )                        { FP_text = "";}
                  dtostrf(footpod_speed,1,1,FP_speed);



                  if(gps_alt <10)                             { ALT_text = "  ";}
                  if((gps_alt >=10) && (gps_alt < 100))       { ALT_text = " ";}
                  if((gps_alt >=100) )                        { ALT_text = "";}
                  dtostrf(gps_alt,1,1,gps_a);  
                
                  String LCD_text0 = ""; 
                  String LCD_text  = LCD_text0 + HRM_text + HRM + " " + FP_text + FP_speed + " " + ALT_text + gps_a;
          
                  lcd.setCursor(0,  1);  lcd.print(LCD_text);


#endif /* _RUNNING_WKO_ */

    }
  
}
#endif /* LCD_I2C */


#if defined(__GPS_GSM_Shield__)

/*
 * Return only when GPS is found (ret = 3 )
 */

void connectGPS()
{

    int ret = 0;


#if !defined(HW_UART_MEGA2650)        
    ss.listen();
#endif    

   SERIAL_DEBUG_PRINT_F("Searching GPS ...");

#if defined(LCD_I2C)
    //lcd.print("> GPS ..");
#endif

    while(ret != 4) {

#if !defined(HW_UART_MEGA2650)
      while (ss.available() > 0) {
          if (gps.encode(ss.read())) 
              ret = displayInfo();
      }   
#else
      while (Serial2.available() > 0) {
          if (gps.encode(Serial2.read())) 
              ret = displayInfo();
      }   

#endif /* HW_UART_MEGA2650 */      

    } /* end while ret!=3 */

    SERIAL_DEBUG_PRINTLN_F("done!");    

#if defined(LCD_I2C)
    //lcd.print("ok!");
#endif    

}

/*
 * Return when some GPS data are found (ret = 1 ) but do not lock in order
 * to leave GSM to check for registration.
 */


void getGPS()
{
    int ret = 0;
#if !defined(HW_UART_MEGA2650)   
    ss.listen();
#endif

    while(!ret) {

#if !defined(HW_UART_MEGA2650)
      while (ss.available() > 0) {
          if (gps.encode(ss.read())) 
              ret = displayInfo();
      }   
#else
      while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) 
              ret = displayInfo();
      }   
 
#endif /* HW_UART_MEGA2650 */     

      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
            SERIAL_DEBUG_PRINTLN_F("No GPS detected: check wiring.");
           // while(true);
      }
    }
}


int displayInfo()
{
            int ret =0;

            smartDelay(0);

            SERIAL_INFO_PRINT_F(" Altitude [meters]: ");   
            if (gps.altitude.isValid())
            {
                gps_alt =  gps.altitude.meters();    
                SERIAL_INFO_PRINT( gps_alt );
                altitude_gps = 1;
                ret+=1;

            } 
            else {
                SERIAL_INFO_PRINT_F("INVALID");
                altitude_gps = 0;
                ret=0;
            }
  
            SERIAL_INFO_PRINT_F(" Location: ");   
            if (gps.location.isValid())
            {
              // lat, lon
              gps_lat = gps.location.lat();
              gps_lng = gps.location.lng();

              SERIAL_INFO_PRINT2(gps_lat, 6);
              SERIAL_INFO_PRINT_F(",");
              SERIAL_INFO_PRINT2(gps_lng, 6);

              location_gps = 1;
              ret+=1;
            }
            else
            {
           
              SERIAL_INFO_PRINT_F("INVALID");
              location_gps = 0;
              ret=0;
            }

            SERIAL_INFO_PRINT_F("  Date/Time: ");   

            if (gps.date.isValid())
            {
              gps_month = gps.date.month();
              gps_day   = gps.date.day();
              gps_year  = gps.date.year();

              SERIAL_INFO_PRINT(gps_month);
              SERIAL_INFO_PRINT_F("/");
              SERIAL_INFO_PRINT(gps_day);
              SERIAL_INFO_PRINT_F("/");
              SERIAL_INFO_PRINT(gps_year);
              date_gps = 1;
              ret+=1;
            }
            else
            {
              SERIAL_INFO_PRINT_F("INVALID");       
              date_gps = 0;
              ret=0;
            }


            SERIAL_INFO_PRINT_F(" "); 

            if (gps.time.isValid())
            {
 
              gps_hour        = gps.time.hour();
              gps_minute      = gps.time.minute();
              gps_second      = gps.time.second();
              gps_centisecond = gps.time.centisecond();


              if (gps_hour < 10) SERIAL_INFO_PRINT_F("0");
              SERIAL_INFO_PRINT(gps_hour);
              SERIAL_INFO_PRINT_F(":");
              if (gps_minute < 10) SERIAL_INFO_PRINT_F("0");
              SERIAL_INFO_PRINT(gps_minute);
              SERIAL_INFO_PRINT_F(":");
              if (gps_second < 10) SERIAL_INFO_PRINT_F("0");
              SERIAL_INFO_PRINT(gps_second);
              SERIAL_INFO_PRINT_F(".");
              if (gps_centisecond < 10) SERIAL_INFO_PRINT_F("0");
              SERIAL_INFO_PRINT(gps_centisecond);
              
              time_gps = 1;
              ret+=1;
            }
            else
            {
            
                SERIAL_INFO_PRINT_F("INVALID");       
                time_gps = 0;
                ret=0;
            }
            SERIAL_INFO_PRINTLN("");

            return ret;
}

// This custom version of delay() ensures that the gps object is being "fed".
 void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

#endif /* __GPS_GSM_Shield__ */

int testANTchip() 
{

  int state_counter = 0;
  boolean clear_to_send = false;


  while(1) // run over and over
  {
      //Print out everything received from the ANT chip
      if (Serial1.available())
      {
        Serial.print("RECV: 0x");
        while(Serial1.available())
        {
          Serial.print( Serial1.read(), HEX );
        }
        Serial.println(".");
      }


      if(rts_ant_received == 1)
      {
        Serial.println("Received RTS Interrupts");  
        //Clear the ISR
        rts_ant_received = 0;
        
        if( digitalRead(RTS_PIN) == LOW )
        {
          Serial.println("Host CTS (ANT is ready to receive again).");  
          clear_to_send = true;
        }
        else
        {
          Serial.println("Waiting for ANT to let us send again.");  
          //Need to make sure it is low again
          while( digitalRead(RTS_PIN) != LOW )
          {
                Serial.print(".");
                delay(50);
          }
          clear_to_send = true;
        }
      }
  
    if( clear_to_send )
    {
      if((state_counter%2) == 0)
      {
        Serial.println("Wait...");  
        delay(500);
      }
      else
      if(state_counter == 1)
      {
        Serial.println("Hardware RESET");  
        // ANTPlus( RTS_PIN, 10/*SUSPEND*/, 11/*SLEEP*/, 12/*RESET*/ );
        digitalWrite(12/*RESET*/ ,   LOW);
        delay(100);
        digitalWrite(12/*RESET*/ ,   HIGH);
        clear_to_send = false;
      }
      else
      if(state_counter == 3)
      {
        Serial.println("RESET_SYSTEM()");  
        Serial1.write((uint8_t)0xA4);
        Serial1.write((uint8_t)0x01);
        Serial1.write((uint8_t)0x4A);
        Serial1.write((uint8_t)0x00);
        Serial1.write((uint8_t)0xEF);
        //Padding
        Serial1.write((uint8_t)0x00);
        clear_to_send = false;
      }
      else
      if(state_counter == 5)
      {
        Serial.println("REQUEST( CAPS )");  
        Serial1.write((uint8_t)0xA4);
        Serial1.write((uint8_t)0x02);
        Serial1.write((uint8_t)0x4D);
        Serial1.write((uint8_t)0x00);
        Serial1.write((uint8_t)0x54);
        Serial1.write((uint8_t)0xBF);
        //Padding
        Serial1.write((uint8_t)0x00);
        clear_to_send = false;
      }
      else
      if(state_counter == 7)
      {
        Serial.println("Test ANT+ successfull !!!");  
        Serial.flush();
        break;
      }
      
      state_counter++;
    }
    
  } /* while (1) */

  return 1;
  
}




