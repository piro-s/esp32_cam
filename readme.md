### ESP32_cam stream & temperature server

ESP32_cam AI Thinker bought on Ali, to which the sensor DS18B20 is connected to  
pin SensorDataPin = 13.  
Local IP camera: 192.168.0.221,  
stream port: 80,  
temperature data port: 81.  
To access a stream: http://192.168.0.221:80  
To access the current temperature value: http://192.168.0.221:81/temper  
Data save in json, value is updated every 30 seconds.
  
To install in Arduino IDE:  
1. In File->Preferences->Additional Boards Managers URLS add:  
	'https://dl.espressif.com/dl/package_esp32_index.json'  
2. Go to Tools->Board->Boards Manager.  
3. Search for ESP32 and press install button for the “ESP32 by Espressif Systems“.  
4. After install select board in Board->ESP32->AI Thinker ESP32-Cam.  
5. Done.  