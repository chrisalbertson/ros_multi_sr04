
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <Ultrasonic.h>
#include <limits.h>


//Set to either "1" or "0" to include experimental features
#define INCLUDE_EXPERIMENTAL  0


#define LED_BUILTIN PC13
#define LED_ON      LOW
#define LED_OFF     HIGH

//#define SPEEDOFSOUND   343.0  // meters per second (at about 20C tempurature)
#define US_PER_METER_RT 5830.9  // uSec per meter for round trip 


#define SONAR_NUM  4    // Number of sensors.
#define MAXRANGE   1.0  // Maximum distance (in meters) to ping.
#define MINRANGE   0.1  // 
#define FOV       30.0  // Sensor field of view in DEGREES
#define QUIET     50    // number of milliseconds to be "quiet" between pings

// Data is send from a sensor only if the sensed distance changes or the maximum reporting
// period expires.  The default maxium period on one second.   This ensures the at least
// 1 Hz reporting.
//
// There is also a minimum reporting period.  The isthe fastest rate that data will be
// sent from a given sensor.   But the actual rate is determined by the laws of logic
// and physics and can be longer then the specificed minumium period.  For example we
// have to wait for the ultrasonic pulse to travel to the target and return and then wait
// a littl longer for the room echoes to dies down before we can pulse the next sensor.
// the actual rate also depends on the number active sensors and when their minimu rates
// are.   This softwre make best effort to pulse each sensor at the specified minimum
// periodic rate.    Note that the time to measure distance is less for objects that
// are closer so the rate of measurement can depend onthe target distance.  This software
// make a "best effort" to schedule measurements but exact timing depends on
// run time condidtions

// The default minimum "ping period" is 100 milliseconds or 10Hz
// the sensor is turned off if the period is set to zero.
short min_period[SONAR_NUM] = {100, 100, 100, 100};

// The default maximum "ping period" is 1000 milliseconds or 1Hz
const short max_period[SONAR_NUM] = {1000, 1000, 1000, 1000};

// Array of times when we last sent a distance.
// Eachelement is the time in "millis" when the next "ping" is to be sent
unsigned long next_ping[SONAR_NUM] = {0, 0, 0, 0};

// Array of "last sent" distances.   We intilize with "impossible" values
// to force a send on any value the first time
unsigned int last_value[SONAR_NUM] = {9999, 9999, 9999, 9999};

// This array records the time in millis when data from each sensor
// was last publised.   This array is used along with max_period[]
// to enforce a minimum publsh rate
unsigned long last_published[SONAR_NUM] = {0, 0, 0, 0};


// Compute sensor time out in uSec rounded to integer based on maximum
// range in meters
const unsigned long timeout = long((US_PER_METER_RT * MAXRANGE) + 0.5);


// Frame ID is used to denote which physical sensor
char frameid[SONAR_NUM][4] = {"s0", "s1", "s2", "s3"};



ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub( "/multisonar", &range_msg);


// Sensor object array. Each sensor's trigger pin, echo pin,
// and timeout in uSec to ping.
//
// Below is for Arduino Uno platform
//                                           T   E  microseconds
//Ultrasonic sonar[SONAR_NUM] = {Ultrasonic( 4,  5, timeout),
//                               Ultrasonic( 6,  7, timeout),
//                               Ultrasonic( 8,  9, timeout),
//                               Ultrasonic(10, 11, timeout)
//                              };
//
// Below is for "Blue Pill" platform.  It uses only 5 volt tolerent pins
// and avids using pins that are shared with USB cable
Ultrasonic sonar[SONAR_NUM] = {Ultrasonic(PB12, PB13, timeout),
                               Ultrasonic(PB14, PB15, timeout),

                               Ultrasonic(PB6,  PB7,  timeout),
                               Ultrasonic(PB8,  PB9,  timeout)
                              };

// find the index of the next sensor to ping.
short nextSendor() {
  unsigned long smallest;
  short         index;

  index    = 0; 
  smallest = ULONG_MAX; 
  
  for (int i = 0; i <= SONAR_NUM; i++) {
    
    if (next_ping[i] < smallest) {
      smallest = next_ping[i];
      index = i;
    }
  }
  return index;
}


void setup()
{
  nh.initNode();
  nh.advertise(pub);

  range_msg.radiation_type  = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view   = FOV * DEG_TO_RAD;
  range_msg.min_range       = MINRANGE;
  range_msg.max_range       = MAXRANGE;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

#if INCLUDE_EXPERIMENTAL 

 //>>>>>>>>>>>>>>>>>>>>>>  Parameter processing not 100% debuged <<<<<<<<<<<<<<<<<<<
 //>>>>>>>>>>>>>>>>>>>>>>  Parameter processing not 100% debuged <<<<<<<<<<<<<<<<<<<
 //>>>>>>>>>>>>>>>>>>>>>>  Parameter processing not 100% debuged <<<<<<<<<<<<<<<<<<<
 
  while(!nh.connected()) nh.spinOnce();

  int sr_min_period_parm[SONAR_NUM];
  if (nh.getParam("~sp_min_period", sr_min_period_parm,SONAR_NUM)){ 
       
       // if we were able to connect with parmater server then over write defaults
       // after checking that the value is valid
       for (int i=0; i < SONAR_NUM; i++) {
           int val = sr_min_period_parm[i];
           
           if ( val > 0 && val < 16000) {
               min_period[i] = sr_min_period_parm[i];
               nh.loginfo("parm ok");
           }
           else {
               nh.loginfo("parm invalid");
           }
       }
  }
  else {
      nh.loginfo("sp_min_period parm not found, using defaults");
  }
  
  nh.loginfo("Startup complete");
#endif /*INCLUDE_EXPERIMENTAL */
}


// TODO  Optionally round the distance.
// TODO  Only publish the message if the value changes but maybe also send at low rate
//       Sat mauybe one per second unless the data changes.   So if we round the data
//       then it changes a lot less frequently.


#define LOOPDELAY 1

void loop()
{

  unsigned int  distance_cm;
  short         sonar_index = 0;
  unsigned long last_ping;
  int           wait_time;

  digitalWrite(LED_BUILTIN, LED_OFF);   // only turn the LED on if we actually publish.

  // Look at the timer that will expire first and determin if we need to do anything at all
  sonar_index = nextSendor();
  if (next_ping[sonar_index] <=  millis()) {

    // Record the next time this sensor should ping
    // Note that we don't publish each ping so this may not be the actual published rate
    next_ping[sonar_index] = millis() + min_period[sonar_index];


    // TODO get range in microseconds then use atmospheric temperatue/humidity sensor
    //      data to compute the speed of sound.  But for now use standard conditions.

    range_msg.header.frame_id = &frameid[sonar_index][0];

    // ping the sensor get record distance and the time when this finished
    distance_cm = sonar[sonar_index].read();
    last_ping = millis();

    range_msg.header.stamp = nh.now();

    // Check if we need to publish this distance.
    //     1. We do not publish the distqnce if the distnce has not changed from the
    //        last time it was published but,
    //     2. We will do publish the distance after a specified minium time interval
    //        even it is has not changed. and
    //     3. We always publish the first measurement.
    //
    if ( (last_value[sonar_index] != distance_cm) ||
         (millis()  >= last_published[sonar_index] + max_period[sonar_index])   ) {

      digitalWrite(LED_BUILTIN, LED_ON);  // Blink for every publication

      // do the conversion to float and divide here, inside the if statment only
      // if we need to publish.  Floating point is expensive with no FPU.
      range_msg.range = float(distance_cm) / 100.0;
      pub.publish(&range_msg);

      last_published[sonar_index] = millis();
      last_value[sonar_index]  = distance_cm;
    }

    // we just completed a ping and maybe published the measurment.  Now we must
    // wait for any echos to die out before continueing.  Publishing the data may
    // taken some unknown about of time. We will waitfor "QUIET" milliseconds
    // before doing the next ping
    wait_time =  QUIET ;                  // milliseconds between pings
    wait_time -= LOOPDELAY;               // we always do the LOOPDAY, so take it out.
    wait_time -= (millis() - last_ping);  // reduce the wait time by the the elapsed time.
    if (wait_time > 0 ) {
      delay(wait_time);
    }
  }

  nh.spinOnce();
  delay(LOOPDELAY);
}
