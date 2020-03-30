Kalman filter to estimate altitude and climbrate(sinkrate) by fusing altitude and acceleration sensor data for lag-and-overshoot-free output.

Altitude can come from a pressure sensor. Acceleration is assumed to be vertical acceleration.

The filter also estimates a third parameter, acceleration bias, as this is assumed to be unknown and can drift unpredictably.

For details on the algorithm, see the document imu_kalman_filter_notes.pdf

Example usage of the Kalman Filter API is in the file imuvario.cpp. This is the main initialization and loop code for an altimeter-vario. The project history is at http://pataga.net/imukalmanvario.html

For algorithm comparisons on real data, see http://github.com/har-in-air/ESP32_IMU_BARO_GPS_LOGGER. 

The /docs directory has an example binary data log, and code used to extract the data from the log, and process it. The code compares two Kalman filters that output filtered altitude data. The first filter KFZ only uses the pressure sensor derived altitude, and the second filter KFZA fuses altitude and acceleration data for lag-and-overshoot free output.

The spreadsheet http://github.com/har-in-air/ESP32_IMU_BARO_GPS_LOGGER/blob/master/docs/FilterResults.ods has the results.

You can find the most recent algorithm implementations at http://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO and http://github.com/har-in-air/ESP8266_BLUETOOTH_AUDIO_VARIO. 
