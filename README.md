This is a project with an Arduino Mega 2560 board with GPS/GSM & ANT+ devices for tracking outdoor activities (Run, Bike, ..) 
and show everything on a web site (not all code is mine)

On the Arduino, it works like this:</br>
  1) get GPS position
  2) get HRM/SPD (if running) or HRM/Speed/cadence/Power (if byking)
  3) send all data via SMS on Twilio account
  
on the web site, it retrive (via Twilio API) the SMS messages and show position on a map and telemetry data
in gauge.


