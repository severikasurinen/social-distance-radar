#include <Stepper.h> // include steooer motor library
#include <Servo.h> // include servo library

const int trig = 6, echo = 7, buzzer = 8, servoPins[] = { 9, 10, 11, 12 }; // ultrasonic sensor trigger and echo pins, buzzer pin, and servo pins
const int revSteps = 2048, motorSpeed = 13, radarPortions = 16; // steps per revolution (set according to datasheet), motor rpm, and radar scans per rotation (minimum is 4)
const int scanDelay = 65, minDistAccepted = 30, maxDistAccepted = 450; // minimum time in ms between radar scans (datasheet suggests a minimum of 60), and minimum and maximum distances accepted in selected units (hat's radius is about 30cm and according to the datasheet, the ultrasonic sensor's range is about 2cm-450cm)
const int alarmDist = 250, coverDist = 100; // distance in selected units for activating alarm and deploying cover
const int alarmScanAngle = 120; // approx. angle to scan when alarm is activated
const float startDelay = 10; // delay for radar to activate in seconds
const bool metric = true; // true = cm, false = inches
Stepper stepper(revSteps, 2, 4, 3, 5); // revSteps and stepper motor pins
Servo servos[4];

bool clockWise = true; // is radar turning clockwise?
bool coverDeployed = false; // has cover been deployed?
int turns = radarPortions/2, alarmDir = -1; // current radar direction, and direction of closest object (-1 = null)
float dist[radarPortions]; // distances at each radar direction
unsigned long lastScan = 0; // time of last scan

void setup()
{
  // initialize components
  stepper.setSpeed(motorSpeed); // set stepper motor rpm
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(buzzer, OUTPUT);
  for(int i=0; i<4; i++)
  {
    servos[i].attach(servoPins[i]);
  }
  
  // default to max distance
  for(int i=0; i<radarPortions; i++)
  {
    dist[i] = maxDistAccepted;
  }
  
  Serial.begin(9600);

  // turn servos to default position
  for(int i=0; i<4; i++)
  {
    servos[i].write(0);
  }
  
  delay(startDelay * 1000); // wait for selected amount of time
}

void loop()
{
  if(!coverDeployed) // check if cover is deployed
  {
    // mark down index number of current direction
    int o = turns;
    if(o == radarPortions)
    {
      o = 0;
    }
    
    dist[o] = Scan(); // request a scan

    // print data in serial monitor
    Serial.print("Distance[");
    Serial.print(o);
    Serial.print("]: ");
    Serial.print(dist[o]);
    if(metric)
    {
      Serial.print(" cm --- ");
    }
    else
    {
      Serial.print(" inches --- ");
    }

    // find direction with shortest distance to an object
    float minDist = -1; // -1 = null
    for(int i=0; i<radarPortions; i++)
    {
      if(dist[i] < minDist || minDist == -1)
      {
        minDist = dist[i];
        if(minDist <= alarmDist) // check if object is in alarm range
        {
          // set direction of closest object
          alarmDir = i;
          if(i == 0 && turns > radarPortions/2)
          {
            alarmDir = radarPortions;
          }
        }
      }
    }

    // print data in serial monitor
    Serial.print("Min Distance: ");
    Serial.print(minDist);
    if(metric)
    {
      Serial.println(" cm");
    }
    else
    {
      Serial.println(" inches");
    }
    
    if(minDist <= alarmDist) // check if closest object is in alarm range
    {
      tone(buzzer, 300, 100); // sound alarm
      
      if(minDist <= coverDist) // check if closest object is in cover deployment range
      {
        alarmDir = -1; // set direction of closest object to null

        // turn servos to cover deployment position
        for(int i=0; i<4; i++)
        {
          servos[i].write(180);
        }
        
        coverDeployed = true; // mark down that cover has been deployed
      }
    }
    else
    {
      alarmDir = -1; // if there's no objects in alarm range, set direction of closest object to null
    }
    
    Turn(); // rotate radar
  }
  else if(turns != radarPortions/2) // cover has been deployed but radar is not in default position
  {
    Turn(); // rotate radar
  }
  else // cover has been deployed and radar is in default position
  {
    delay(10000); // idle
  }
}

void Turn() // rotate radar
{
  if(clockWise) // check if radar is turning clockwise
    {
      stepper.step(revSteps/radarPortions); // turn clockwise to next scan position
      turns++;
      if(turns == radarPortions || (alarmDir != -1 && turns >= alarmDir + round((alarmScanAngle/(360.0/radarPortions))/2-1))) // check if reached full rotation or if scanning certain direction
      {
        clockWise = false;
      }
    }
    else
    {
      stepper.step(-revSteps/radarPortions); // turn anticlockwise to next scan position
      turns--;
      if(turns == 0 || (alarmDir != -1 && turns <= alarmDir - round((alarmScanAngle/(360.0/radarPortions))/2-1))) // check if reached full rotation or if scanning certain direction
      {
        clockWise = true;
      }
    }
}

float Scan() // scan distance
{
  unsigned int lTime = millis() - lastScan;
  if(lTime < scanDelay) // check if enough time has passed since last scan
  {
    delay(scanDelay - lTime); // if not, wait for minimum time
  }

  // make sure trigger pin is off
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  
  // send 10μs pulse to ultrasonic sensor
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  float lDist = (0.0343*pulseIn(echo, HIGH)/2); // read signal travel time and convert to distance in cm
  if(!metric)
  {
    lDist *= 2.54; // convert to inches if selected unit
  }
  lastScan = millis(); // mark down the time of the scan
  
  if(lDist < 0)
  {
    lDist = -lDist; // sometimes value received is the opposite value of what it's supposed to be
  }
  else if(lDist < minDistAccepted || lDist > maxDistAccepted) // check if value is in accepter range
  {
    lDist = maxDistAccepted; // if not, default to max value
  }
  
  return lDist;
}
