/*  (c) 2020-2021 HomeAccessoryKid
 *  This is a heater booster concept. It measures temperatures and drives multiple PWM computer fans...
 *  It uses any ESP8266 with as little as 1MB flash. 
 *  GPIO-2 is used as a bus with one-wire DS18B20 sensors to measure various temperatures
 *  GPIO-3 is used for I2S to generate the pwm signal
 *  GPIO-4 is used to control the first  bank of 8 fans (because fans are class A and do not switch off by themselves)
 *  GPIO-5 is used to control the second bank of 7 fans (because fans are class A and do not switch off by themselves)
 *  GPIO-0 is reading a manual override switch
 *  UDPlogger is used to have remote logging
 *  LCM is enabled in case you want remote updates
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_system.h> //for timestamp report only
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <string.h>
#include "lwip/api.h"
#include <udplogger.h>
#include "ds18b20/ds18b20.h"
#include "i2s_dma/i2s_dma.h"
#include "math.h"
#include <lwip/apps/sntp.h>
#include <espressif/esp8266/eagle_soc.h>

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif

#ifndef SENSOR_PIN
 #error SENSOR_PIN is not specified
#endif
#ifndef BANK1_PIN
 #error BANK1_PIN is not specified
#endif
#ifndef BANK2_PIN
 #error BANK2_PIN is not specified
#endif
#ifndef SWITCH_PIN
 #error SWITCH_PIN is not specified
#endif
#ifndef LED_PIN
 #error LED_PIN is not specified
#endif

/* ============== BEGIN HOMEKIT CHARACTERISTIC DECLARATIONS =============================================================== */
// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include "ota-api.h"
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");

// next use these two lines before calling homekit_server_init(&config);
//    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
//                                      &model.value.string_value,&revision.value.string_value);
//    config.accessories[0]->config_number=c_hash;
// end of OTA add-in instructions

homekit_characteristic_t tgt_heat1 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 );
homekit_characteristic_t cur_heat1 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 3 );
homekit_characteristic_t tgt_temp1 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         21.5 );
homekit_characteristic_t cur_temp1 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         1.0 );
homekit_characteristic_t dis_temp1 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );
homekit_characteristic_t cur_temp2 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         2.0 );
homekit_characteristic_t cur_temp3 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         3.0 );

// void identify_task(void *_args) {
//     vTaskDelete(NULL);
// }

void identify(homekit_value_t _value) {
    UDPLUS("Identify\n");
//    xTaskCreate(identify_task, "identify", 256, NULL, 2, NULL);
}

/* ============== END HOMEKIT CHARACTERISTIC DECLARATIONS ================================================================= */

static   dma_descriptor_t dma_block;
uint32_t dma_buf[1];

#define TEMP2HK(n)  do {old_t##n=cur_temp##n.value.float_value; \
                        cur_temp##n.value.float_value=isnan(temp[S##n])?S##n##avg:(float)(int)(temp[S##n]*10+0.5)/10; \
                        if (old_t##n!=cur_temp##n.value.float_value) \
                            homekit_characteristic_notify(&cur_temp##n,HOMEKIT_FLOAT(cur_temp##n.value.float_value)); \
                    } while (0) //TODO: do we need to test for changed values or is that embedded in notify routine?
#define BEAT 10 //in seconds
#define SENSORS 3
#define S1 7 //       room temp sensor
#define S2 2 // water high temp sensor
#define S3 3 // water low  temp sensor
float temp[16]={85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85}; //using id as a single hex digit, then hardcode which sensor gets which meaning
float S1temp[6],S2temp[6],S3temp[6],S1avg,S2avg,S3avg;
void temp_task(void *argv) {
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS];
    float old_t1,old_t2,old_t3;
    int sensor_count=0,id;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        UDPLUS("Only found %d sensors\n",sensor_count);
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    }

    while(1) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        ds18b20_measure_and_read_multi(SENSOR_PIN, addrs, SENSORS, temps);
        for (int j = 0; j < SENSORS; j++) {
            // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
            // I have manually selected that I have unique ids using the second hex digit of CRC
            id = (addrs[j]>>56)&0xF;
            temp[id] = temps[j];
        } // ds18b20_measure_and_read_multi operation takes about 800ms to run, 3ms start, 750ms wait, 11ms/sensor to read
        if (isnan(temp[S1])||temp[S1]==85||isnan(temp[S2])||temp[S2]==85||isnan(temp[S3])||temp[S3]==85) {
            gpio_write(SENSOR_PIN,0);vTaskDelay(150/portTICK_PERIOD_MS);gpio_write(SENSOR_PIN,1); //reset one-wire bus
        }
        TEMP2HK(1);
        TEMP2HK(2);
        TEMP2HK(3);
    }
}

#define RTC_ADDR    0x600013B0
#define RTC_MAGIC   0xaabecede
int  time_set=0;
void init_task(void *argv) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    printf("RTC: "); for (int i=0;i<7;i++) printf("%08x ",READ_PERI_REG(RTC_ADDR+i*4)); printf("\n");
    uint32_t *dp;
	if (READ_PERI_REG(RTC_ADDR)==RTC_MAGIC) {
        dp=(void*)&(tgt_temp1.value.float_value);*dp=READ_PERI_REG(RTC_ADDR+4);
    }
    printf("INITIAL setpoint=%4.1f\n", tgt_temp1.value.float_value);
    S1temp[0]=22;
    
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.pool.ntp.org");
    sntp_setservername(1, "1.pool.ntp.org");
    sntp_setservername(2, "2.pool.ntp.org");
    while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) vTaskDelay(200/portTICK_PERIOD_MS); //Check if we have an IP every 200ms
    sntp_init();
    time_t ts;
    do {ts = time(NULL);
        if (ts == ((time_t)-1)) printf("ts=-1 ");
        vTaskDelay(100/portTICK_PERIOD_MS);
    } while (!(ts>1609876543)); //Tue Jan  5 20:55:43 CET 2021
    printf("TIME SET: %u=%s", (unsigned int) ts, ctime(&ts));
    time_set=1;
    vTaskDelete(NULL);
}

#define CalcAvg(Sx) do {            Sx##temp[5]=Sx##temp[4];Sx##temp[4]=Sx##temp[3]; \
            Sx##temp[3]=Sx##temp[2];Sx##temp[2]=Sx##temp[1];Sx##temp[1]=Sx##temp[0]; \
            if ( !isnan(temp[Sx]) && temp[Sx]!=85 )         Sx##temp[0]=temp[Sx];    \
            Sx##avg=(Sx##temp[0]+Sx##temp[1]+Sx##temp[2]+Sx##temp[3]+Sx##temp[4]+Sx##temp[5])/6.0; \
        } while(0)
#define HYSTERESIS 0.07 // considering the smallest step of a DS18B20 is 0.06125
static TaskHandle_t tempTask = NULL;
float deltaT,S1anchor=0;
int timeIndex=0,switch_state=0,pwm=0,dir=1;
TimerHandle_t xTimer;
void vTimerCallback( TimerHandle_t xTimer ) {
    struct timeval tv;
    uint32_t seconds = ( uint32_t ) pvTimerGetTimerID( xTimer );
    vTimerSetTimerID( xTimer, (void*)seconds+1); //136 year to loop
    int switch_on=0;
    if (gpio_read(SWITCH_PIN)) switch_state--; else switch_state++; //pin is low when switch is on
    if (switch_state<0) switch_state=0;
    if (switch_state>3) switch_state=3;
    switch_on=switch_state>>1;

//     gpio_write(BANK1_PIN,tgt_heat1.value.int_value/2);
//     gpio_write(BANK2_PIN,tgt_heat1.value.int_value%2);
//     printf("Bank1: %d  Bank2: %d\n",gpio_read(BANK1_PIN),gpio_read(BANK2_PIN));
    
    switch (timeIndex) { //send commands
        case 0: //measure temperature
            xTaskNotifyGive( tempTask ); //temperature measurement start
            break;
        case 1: // calc avg temperatures and display values
            CalcAvg(S1); CalcAvg(S2); CalcAvg(S3);
            gettimeofday(&tv, NULL);
            time_t now=tv.tv_sec;
            printf("@%d Sw%d PWM=%2d dir=%2d S1anchor=%7.4f S1avg=%7.4f S1=%7.4f S2=%7.4f S3=%7.4f bank1=%d bank2=%d @%s" \
                    ,seconds,switch_on,pwm,dir,S1anchor,S1avg,temp[S1],temp[S2],temp[S3],gpio_read(BANK1_PIN),gpio_read(BANK2_PIN),ctime(&now));
            break;
        default: break;
    }

    if (seconds%60==51) {
        if (dir*(S1anchor-S1avg)>HYSTERESIS) {
            dir*=-1; S1anchor=S1avg;
        } else {
            if (dir*(S1anchor-S1avg)<0) S1anchor=S1avg;
        }
        deltaT=S1anchor-tgt_temp1.value.float_value;
        pwm=0;
        switch (tgt_heat1.value.int_value) {
            case 0: //off
                gpio_write(BANK1_PIN,0);
                gpio_write(BANK2_PIN,0);
                break;
            case 1: //heat
                if (deltaT<0) pwm=(int)(deltaT*-10);
                // no break on purpose
            case 3: //auto
                if (S2avg>32) {
                    if (deltaT<-0.2) gpio_write(BANK1_PIN,1); else gpio_write(BANK1_PIN,0);
                    if (deltaT<-0.5) gpio_write(BANK2_PIN,1); else gpio_write(BANK2_PIN,0);
                    if (pwm<32) dma_buf[0]=0xffffffff<<pwm; else dma_buf[0]=0;
                } else if (S2avg<28) {
                    gpio_write(BANK1_PIN,0);
                    gpio_write(BANK2_PIN,0);                    
                }
                break;
            case 2: //cool
                if (deltaT>0) {
                    dma_buf[0]=0; // full speed on both banks
                    gpio_write(BANK1_PIN,1);
                    gpio_write(BANK2_PIN,1);
                } else {
                    dma_buf[0]=0xffffffff; // both banks off (at lowest speed (for the record))
                    gpio_write(BANK1_PIN,0);
                    gpio_write(BANK2_PIN,0);
                }
                break;
            default: break;
        }

        //save state to RTC memory
        uint32_t *dp;
        dp=(void*)&tgt_temp1.value.float_value; WRITE_PERI_REG(RTC_ADDR+4,*dp      ); //float
                                                WRITE_PERI_REG(RTC_ADDR   ,RTC_MAGIC);
    }

    timeIndex++; if (timeIndex==BEAT) timeIndex=0;
} //this is a timer that restarts every 1 second

homekit_server_config_t config;
void device_init() {
  if (homekit_is_paired()) {
    config.on_event=NULL;
    udplog_init(3);
    UDPLUS("\n\n\nHeater-Booster " VERSION "\n");
    
//     gpio_enable(LED_PIN, GPIO_OUTPUT); gpio_write(LED_PIN, 0);
    gpio_set_pullup(SENSOR_PIN, true, true);
    gpio_enable(SWITCH_PIN, GPIO_INPUT);
    gpio_enable(BANK1_PIN, GPIO_OUTPUT); gpio_write(BANK1_PIN, 0);
    gpio_enable(BANK2_PIN, GPIO_OUTPUT); gpio_write(BANK2_PIN, 0);
    //PWM_PIN is GPIO3 = RX0 because hardcoded in i2s
    i2s_pins_t i2s_pins = {.data = true, .clock = false, .ws = false};
    i2s_clock_div_t clock_div = i2s_get_clock_div(800000); //32bits in 40microseconds is 800kHz
    i2s_dma_init(NULL, NULL, clock_div, i2s_pins);
    dma_block.owner = 1; dma_block.sub_sof = 0; dma_block.unused = 0;
    dma_block.next_link_ptr = &dma_block; dma_block.eof = 0; //only one block which loops so no EOF ???
    dma_block.datalen = 4; dma_block.blocksize = 4; // 32 bits = 4byte data
    dma_block.buf_ptr = dma_buf; //uint32_t buffer type is 4byte data
    dma_buf[0]=0xffffffff; //initial value = 0%
    i2s_dma_start(&dma_block); //transmit the dma_buf in a loop at 25kHz

    xTaskCreate(temp_task,"Temp", 512, NULL, 1, &tempTask);
    xTaskCreate(init_task,"Time", 512, NULL, 6, NULL);
    xTimer=xTimerCreate( "Timer", 1000/portTICK_PERIOD_MS, pdTRUE, (void*)0, vTimerCallback);
    xTimerStart(xTimer, 0);
  }
}

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Booster"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Water T-high"),
                    &cur_temp2,
                    &ota_trigger,
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Water T-low"),
                    &cur_temp3,
                    NULL
                }),
            NULL
        }),
    HOMEKIT_ACCESSORY(
        .id=2,
        .category=homekit_accessory_category_thermostat,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
                    &tgt_heat1,
                    &cur_heat1,
                    &tgt_temp1,
                    &cur_temp1,
                    &dis_temp1,
                    NULL
                }),
            NULL
        }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .on_event=device_init,
    .password = "111-11-111"
};


void user_init(void) {
    uart_set_baud(0, 115200);
    device_init();
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);
}
