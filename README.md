Kalman filter to estimate altitude and climbrate(sinkrate) by fusing altitude and acceleration sensor data.

Altitude can come from a pressure sensor. Acceleration is assumed to be vertical acceleration.

The filter also estimates a third parameter, acceleration bias, as this is assumed to be unknown and can drift unpredictably.

For details on the algorithm, see the document imu_kalman_filter_notes.pdf

Example usage of the Kalman Filter API is in the file imuvario.cpp. This is the main initialization and loop code for an altimeter-vario, the project is on pataga.net/imukalmanvario.html

Update : You can find the most recent algorithm implementation code in my ESP8266_BLUETOOTH_AUDIO_VARIO and ESP32_IMU_BARO_GPS_VARIO github projects. I also have algorithm comparison test results on real data in the ESP32_IMU_BARO_GPS_LOGGER/docs folder.
