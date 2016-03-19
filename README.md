An Arduino Mega 2560 with GPS/GSM & ANT+ chip for tracking outdoor activities (Run, Bike, ..) on a web site

On the Arduino, it works like this:
  1) get GPS position
  2) get HRM/SPD (if running) or HRM/Speed/cadence/Power (if byking)
  3) send all data via SMS on Twilio account
  
on the web site, it retrive (via Twilio API) the SMS messages and show position on a map and telemetry data
in gauge.
