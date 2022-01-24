#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"    //disable brownout problems
#include "esp_http_server.h"
#include <OneWire.h>
#include <DallasTemperature.h>

///--- WiFi
//Replace with your network credentials
const char* ssid = "ssid";
const char* password = "password";

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 221);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);


///--- Temperature
const int SensorDataPin = 13; // DS18B20 pin

OneWire oneWire(SensorDataPin); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature
DeviceAddress deviceAddress; // Device address

String temperature_Celsius = "";

//AsyncWebServer server_temper(80);

unsigned long lastTime = 0;
uint8_t flag_temperReq = 0x00; // flag request temperature
unsigned long timerDelay_temperReq = 1000; // send request temper
unsigned long timerDelay_temper = 30000; // send readings timer

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
    for(uint8_t it = 0; it < 8; it++)
    {
        if(deviceAddress[it] < 16)
            Serial.print("0");
        Serial.print(deviceAddress[it], HEX);
    } // for(uint8_t it = 0; it < 8; it++)
} //   void printAddress(DeviceAddress deviceAddress)

String readDSTemperatureC() 
{
    // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
    float tempC = sensors.getTempC(deviceAddress);

    if(tempC == -127.00) {
        Serial.println("Failed to read from DS18B20 sensor");
        return "--";
    } else {
        Serial.print("Temperature Celsius: ");
        Serial.println(tempC); 
    }
    return String(tempC);
} // String readDSTemperatureC() 



///--- ESP32 camera
#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK)
        return res;

    while(true)
    {
        fb = esp_camera_fb_get();
        if(!fb)
        {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } // if(!fb)
        else
        {
            if(fb->width > 400)
            {
                if(fb->format != PIXFORMAT_JPEG)
                {
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted)
                    {
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    } // if(!jpeg_converted)
                } // if(fb->format != PIXFORMAT_JPEG)
                else 
                {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                } // else
            } // if(fb->width > 400)
        } // else
        if(res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        } // if(res == ESP_OK)
        if(res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        if(res == ESP_OK)
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if(fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } // if(fb)
        else if(_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        } // else if(_jpg_buf)
        if(res != ESP_OK)
            break;
    } // while(true)

    return res;
} // static esp_err_t stream_handler(httpd_req_t *req)

static esp_err_t temper_handler(httpd_req_t *req)
{
    static char json_response[32];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"temperature\":%s", temperature_Celsius);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
} // static esp_err_t temper_handler(httpd_req_t *req)

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t temper_uri = {
        .uri       = "/temper",
        .method    = HTTP_GET,
        .handler   = temper_handler,
        .user_ctx  = NULL
    };


    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK)
        httpd_register_uri_handler(stream_httpd, &index_uri);

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
        httpd_register_uri_handler(camera_httpd, &temper_uri);
} // void startCameraServer()

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
    Serial.begin(115200);
    Serial.setDebugOutput(false);


    // DS18B20
    // locate devices on the bus
    Serial.print("Locating devices...");
    sensors.begin();
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    if(!sensors.getAddress(deviceAddress, 0)) Serial.println("Unable to find address for Device 0"); 
    Serial.print("Device 0 Address: ");
    printAddress(deviceAddress);
    Serial.println();

    sensors.setResolution(deviceAddress, 12); // Resolution 12 bit
    Serial.print("Device 0 Resolution: ");
    Serial.print(sensors.getResolution(deviceAddress), DEC); 
    Serial.println();


    sensors.requestTemperatures(); // Send the command to get temperatures
    delay(1500); // Waiting
    float tempC = sensors.getTempC(deviceAddress);
    if(tempC == DEVICE_DISCONNECTED_C) 
    {
        Serial.println("Error: Could not read temperature data");
        return;
    } // if(tempC == DEVICE_DISCONNECTED_C) 
    Serial.print("Temp C: ");
    Serial.print(tempC);


    // ESP32 cam    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG; 
    
    if(psramFound())
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } // if(psramFound())
    else
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 10;
        config.fb_count = 1;
    } // else
    
    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if(err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    } // if(err != ESP_OK)
    

  
    // Set device as a Wi-Fi Station
    if(!WiFi.config(local_IP, gateway, subnet))
        Serial.println("STA Failed to configure");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Setting as a Wi-Fi Station..");
    } // while (WiFi.status() != WL_CONNECTED)

    Serial.print("Station IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println();


    // Start streaming web server
    startCameraServer();
} // void setup()


void loop()
{
    if((millis() - lastTime) > timerDelay_temper)
    {
        if(flag_temperReq == 0x00) // Wait 1 sec to get temper
        {
            sensors.requestTemperatures();
            flag_temperReq = 0xFF;
        } // if(flag_temperReq == False)
        if((millis() - lastTime) > timerDelay_temper + timerDelay_temperReq)
        {
            temperature_Celsius = readDSTemperatureC();
            flag_temperReq = 0x00;
            lastTime = millis();
        } // if((millis() - lastTime) > timerDelay_temper + timerDelay_temperReq)
    } // if((millis() - lastTime) > timerDelay_temper)

    delay(1);
} // void loop()
