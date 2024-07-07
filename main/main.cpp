
#include <iostream>
#include <sstream>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "stdlib.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "qrcode.h"
#include "core_mqtt.h"
#include "network_transport.h"
#include "sdkconfig.h"
#include "clock.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"
#include "bmx280.h"
#include "json.hpp"
#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21
#define WIFI_CONNECT_RETRIES (3)
/*******************************************************************/
static const char* TAG = "AWS_main";
uint8_t wifi_connect_retries = 0;
bmx280_t* bmx280;
bool led_state = 0;
bool ses_present = 0;
TaskHandle_t mqtt_task_handle; //mqtt task
 const char* topic = "sensor/reading";
 const char* thing_name = CONFIG_THING_NAME;
 const char* user_name = CONFIG_USER_NAME;
const char* endpoint = CONFIG_AWS_ENDPOINT;
bool connected_to_aws = false;
uint8_t buffer[1024];
float temperature, pressure, humidity;
NetworkContext_t network_context;
TransportInterface_t transport;
MQTTConnectInfo_t connect_info;
MQTTFixedBuffer_t network_buffer;
MQTTPublishInfo_t publish_info;
MQTTContext_t mqtt_context;
QueueHandle_t mqtt_aws_queue;
QueueHandle_t sensor_queue;
extern  "C"
{
 extern const char root_cert_auth_start[] asm(
        "_binary_root_cert_auth_crt_start");
    extern const char root_cert_auth_end[] asm(
        "_binary_root_cert_auth_crt_end");
    extern const char client_cert_start[] asm(
        "_binary_client_crt_start");
    extern const char client_cert_end[] asm(
        "_binary_client_crt_end");
    extern const char client_key_start[] asm(
        "_binary_client_key_start");
    extern const char client_key_end[] asm(
        "_binary_client_key_end");
}

/*******************************************************************/
MQTTStatus_t mqtt_subscribe_to(MQTTContext_t* mqtt_context,const char* topic,MQTTQoS_t qos_level)
{
    uint16_t pkt = MQTT_GetPacketId(mqtt_context);
    MQTTSubscribeInfo_t subscribe_topic ;
    subscribe_topic.qos = qos_level;
    subscribe_topic.pTopicFilter = topic;
    subscribe_topic.topicFilterLength = 5;
    return MQTT_Subscribe(mqtt_context,&subscribe_topic,1,pkt);
}
static void print_qr_code(void)
{
     nlohmann::json qr_wifi = 
    {
        {"ver", "v1"},
        {"name", "PROV_ESP32"},
        {"pop", "abcd1234"},
        {"transport", "softap"}
    };
    esp_qrcode_config_t qr_cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate( &qr_cfg,qr_wifi.dump().c_str());
    ESP_LOGI(TAG, "QR code generated: %s", qr_wifi.dump().c_str());
}
static void wifi_prov_handler(void *user_data, wifi_prov_cb_event_t event_base, void *event_data)
{
    switch(event_base)
    {
        case WIFI_PROV_START :
            ESP_LOGI(TAG, "--HANDLER ----Starting provisioning");
            // print_qr_code();
             break;
        case WIFI_PROV_CRED_RECV :
            ESP_LOGI(TAG, "Received credentials : \n SSID : %s \n Password : %s",((wifi_sta_config_t*)event_data)->ssid,((wifi_sta_config_t*)event_data)->password);
            break;
        case WIFI_PROV_CRED_SUCCESS :
            ESP_LOGI(TAG,"Credentials accepted");
            wifi_prov_mgr_stop_provisioning();
            break;
        case WIFI_PROV_CRED_FAIL :
            ESP_LOGE(TAG,"Credentials rejected, press RST and try again");
            wifi_prov_mgr_reset_sm_state_on_failure();
            break;
        case WIFI_PROV_END :
            ESP_LOGI(TAG,"Provisioning ended, deinitializing manager");
            wifi_prov_mgr_deinit();
            break;
        default : break;
    }
}
static void softap_prov_init(void)
{
    ESP_LOGI(TAG, "Initializing provisioning");
    wifi_prov_mgr_config_t mgr= 
    {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = wifi_prov_handler//WIFI_PROV_EVENT_HANDLER_NONE,
    };
    wifi_prov_mgr_init(mgr);
    bool is_prov = false;
    wifi_prov_mgr_is_provisioned(&is_prov);
    if(is_prov)
    {
        ESP_LOGI(TAG, "Already provisioned");
        wifi_prov_mgr_deinit();
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
      
    }
    else
    {
        ESP_LOGI(TAG,"Starting provisioning");
        esp_netif_create_default_wifi_ap();
        wifi_prov_mgr_disable_auto_stop(1000);
        wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0,NULL,"prov_esp","password");
        print_qr_code();

    }
}

void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = 
        {
            .clk_speed = 100000
        },
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}
void sensor_read(void* pvArgs)
{
    float sensor_readings[3];
    float temperature,pressure,humidity;
    uint32_t tick_count;
    while(1)
    {
        tick_count = xTaskGetTickCount();
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));

        while((bmx280_isSampling(bmx280) == 1) && ((xTaskGetTickCount() - tick_count) < pdMS_TO_TICKS(1000)))
        {
            //Wait for incoming sensor readings
        }
        bmx280_readoutFloat(bmx280,&temperature,&pressure,&humidity);
        sensor_readings[0] = temperature;
        sensor_readings[1] = pressure;
        sensor_readings[2] = humidity;
        xQueueSend(sensor_queue,sensor_readings,100);
    }
}
void sensor_init(void)
{
    ESP_LOGI(TAG,"Initializing sensor");
    i2c_init();
    bmx280 = bmx280_create(I2C_NUM_0);
    if(bmx280 == 0)
    {
        ESP_LOGE(TAG,"Could not initialize bmx280");
    }
    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280,&bmx_cfg));
    ESP_LOGI(TAG,"Sensor initialized");
}
IRAM_ATTR void gpio_interrupt_handler(void* arg)
{
    
    xQueueSendFromISR(mqtt_aws_queue,&(led_state = !led_state),NULL);
}
void led_poll_state(void* arg)
{
    bool _led_state;
    while(1)
    {
        xQueueReceive(mqtt_aws_queue,&(_led_state),portMAX_DELAY);
        ESP_LOGI(TAG,"led_state = %d",_led_state);
        gpio_set_level(GPIO_NUM_2,_led_state);
    }
}
void input_interrupt_init(void)
{
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0,GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0,GPIO_PULLDOWN_ONLY);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0,gpio_interrupt_handler,NULL);
    gpio_set_intr_type(GPIO_NUM_0,GPIO_INTR_NEGEDGE);
    gpio_intr_enable(GPIO_NUM_0);
}
static void mqtt_event_cb(MQTTContext_t* pMQTTContext,MQTTPacketInfo_t* pPacketInfo, MQTTDeserializedInfo_t* pDeserializedInfo)
{
   switch(pPacketInfo->type)
   {
      case MQTT_PACKET_TYPE_PUBLISH : //Message received
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_PUBLISH");
        ESP_LOGI(TAG,"received :\n%s",(const char*)(pDeserializedInfo->pPublishInfo->pPayload));        break;
     case MQTT_PACKET_TYPE_SUBSCRIBE :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_SUBSCRIBE");
        break;
    case MQTT_PACKET_TYPE_CONNECT  :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_CONNECT");
        break;  
    case MQTT_PACKET_TYPE_CONNACK :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_CONNACK");
        break;
        break;
    default:break;
   }

}
void mqtt_process_task(void* arg)
{
    float sensor_readings[3];
    ESP_LOGI(TAG,"mqtt_process_task started");
    mqtt_subscribe_to(&mqtt_context,"/read",MQTTQoS0);
    
    while(1)
    {
        xQueueReceive(sensor_queue,sensor_readings,portMAX_DELAY);// wait for sensor readings
        MQTT_ProcessLoop(&mqtt_context);
        std::stringstream ss_mqtt;
        // ESP_LOGI(TAG,"t: %f",sensor_readings[0]);
        // ESP_LOGI(TAG,"p: %f",sensor_readings[1]);
        // ESP_LOGI(TAG,"h: %f",sensor_readings[2]);

        ss_mqtt << "{\"temperature\":" << sensor_readings[0] 
                << ",\"pressure\":"    << sensor_readings[1] 
                << ",\"humidity\":"    << sensor_readings[2] << "}";
        std::string mqtt_string = ss_mqtt.str();
        publish_info.pPayload = mqtt_string.c_str();
        publish_info.payloadLength = mqtt_string.length();
        uint16_t packet_id = MQTT_GetPacketId(&mqtt_context);
        MQTT_Publish(&mqtt_context,&publish_info,packet_id);
    }
}
void set_wifi_status(bool connected)
{
    ESP_LOGI(TAG,"tls err : %d",xTlsConnect(&network_context));
    ESP_LOGI(TAG,"MQTT Connect err : %d",MQTT_Connect(&mqtt_context,&connect_info,NULL,1000,&ses_present));
    xTaskCreate(mqtt_process_task,"mqtt_process_task", 2048 * 2,NULL,5,&mqtt_task_handle);
   // connected_to_aws = MQTT_Connect(&mqtt_context,&connect_info,NULL,2000,&ses_present);
    
}
static void event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
     {
        esp_wifi_connect();
     } 
     else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
     {
         ESP_LOGI(TAG,"STA CONNECTED");
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        set_wifi_status(true);
     
    }
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGE(TAG,"STA DISCONNECTED");
        wifi_prov_mgr_reset_provisioning();
        //set_wifi_status(false);
    }
}
void wifi_init(void)
{
     esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
 

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &event_handler,
                                                    NULL,
                                                    &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                    IP_EVENT_STA_GOT_IP,
                                                    &event_handler,
                                                    NULL,
                                                    &instance_got_ip));
    esp_netif_create_default_wifi_sta();
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     wifi_config_t wfcfg;
//     memset(&wfcfg, 0, sizeof(wfcfg));
//     strcpy(reinterpret_cast<char*>(wfcfg.sta.ssid), CONFIG_SSID);
//     strcpy(reinterpret_cast<char*>(wfcfg.sta.password),CONFIG_PASSWORD);
//     wfcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wfcfg) );
//   ESP_ERROR_CHECK(esp_wifi_start());
}
void user_main_task(void* arg)
{
    ESP_LOGI(TAG,"user_main_task started");
    while(1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void aws_init(void)
{
    network_context.pcHostname = endpoint;
    network_context.xPort = 8883;
    network_context.pxTls = NULL;
    network_context.xTlsContextSemaphore = xSemaphoreCreateMutex();
    network_context.disableSni = false;
    network_context.pcServerRootCA = root_cert_auth_start;
    network_context.pcServerRootCASize = root_cert_auth_end - root_cert_auth_start;
    network_context.pcClientCert = client_cert_start;
    network_context.pcClientCertSize = client_cert_end - client_cert_start;
    network_context.pcClientKey = client_key_start;
    network_context.pcClientKeySize = client_key_end - client_key_start;
    network_context.pAlpnProtos = NULL;
    transport.pNetworkContext = &network_context;
    transport.recv = espTlsTransportRecv;
    transport.send = espTlsTransportSend;
    transport.writev = NULL;
    network_buffer.pBuffer = buffer;
    network_buffer.size = sizeof(buffer);
    connect_info.cleanSession = true;
    connect_info.pClientIdentifier = thing_name;
    connect_info.clientIdentifierLength = strlen(thing_name);
    connect_info.keepAliveSeconds = 60;
    connect_info.pUserName = user_name;
    connect_info.userNameLength = strlen(user_name);
    publish_info.qos = MQTTQoS0;
    publish_info.pTopicName = topic;
    publish_info.topicNameLength = strlen(topic);
    MQTT_Init(&mqtt_context,&transport,Clock_GetTimeMs,&mqtt_event_cb,&network_buffer);
}

void led_init(void)
{
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2,GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2,0);
}

extern "C" void app_main(void)
{
    esp_log_level_set(TAG,ESP_LOG_VERBOSE);
    mqtt_aws_queue = xQueueCreate(10,sizeof(uint32_t));
    sensor_queue = xQueueCreate(10,sizeof(float[3]));
    sensor_init();
    wifi_init();
    softap_prov_init(); 
    aws_init(); 
   // set_wifi_status(1);
  //  xTaskCreate(user_main_task,"user_main_task",2048,NULL,5,NULL);
    xTaskCreate(led_poll_state,"led_poll_state",2048,NULL,5,NULL);
    xTaskCreate(sensor_read,"sensor_read",2048,NULL,5,NULL);
    led_init();
    input_interrupt_init();
    
}
