# hackAthon-Nomad
This is the fw for the 2021 tsi hackathon event. It utlizes a ESP32 Dev kit, neo 6m gps, and sps30 sensor. The certs are currently updated to work with google iot core as of 11/6/2021. You can use this <https://www.survivingwithandroid.com/cloud-iot-core-esp32/> tutorial to get started and learn how to create the cloud package, pub/sub, etc. Here is the fix <https://github.com/GoogleCloudPlatform/google-cloud-iot-arduino/issues/221> for the certs and other IOT core cpp files that needs to be implemented in your own projects. This project has these fixes.

The arduino sketch lives in the Esp32-lwmqtt folder.
