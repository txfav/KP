#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
//ble
#include "nimble/nimble_npl.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "blecent.h"
#include "esp_central.h"
//sensor
#include "driver/adc.h"
#include "driver/gpio.h"
#include "malloc.h"
#include "sdkconfig.h"
#include "esp_sleep.h"
#include "esp_timer.h"

/*hall sensor linear*/
//#define ADC_HALL_SENSOR ADC1_CHANNEL_0
//#define tHold 3300
//#define dOpen raw_adc_value < tHold
//#define dClosed raw_adc_value > tHold

/*Hall Sensor Unipolar*/
#define HALL_DIGITAL GPIO_NUM_5
/* 16 Bit SPP Service UUID */
#define GATT_SPP_SVC_UUID   0xF5, 0x7C, 0x0A, 0x09, 0xA4, 0x2F, 0x40, 0xBB, 0xD8, 0x45, 0xE7, 0x5F, 0x9A, 0x16, 0x4B, 0x3D

/* 16 Bit SPP Service Characteristic UUID */
#define GATT_SPP_CHR_UUID   0x12, 0x14, 0xD5, 0xF9, 0xEE, 0xFB, 0xFB, 0xBB, 0xF0, 0x45, 0xF4, 0xA8, 0xC9, 0x19, 0xF7, 0x41

QueueHandle_t queue;
int state;
int raw_adc_value;
bool status ;
int pintu;
uint16_t attr_handle;
uint16_t conn_handle;
uint8_t ble_addr_type;
char *SERVER_NAME = "Banana";
// estatus = (status != true);

struct os_mbuf *om;


uint16_t attribute_handle[3];
void ble_store_config_init(void);
void discover_chr(uint16_t conn_handle);
void connect_ble(void);
static void ble_spp_client_set_handle(const struct peer *peer);
void send (void *pvParameters);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);


/*TEST*/
/*
void button_task(void *pvParameter) {
    gpio_set_direction(HALL_DIGITAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HALL_DIGITAL, GPIO_PULLUP_ONLY);
    bool last_state = true;
    bool state = true;
    while (true) {
        state = gpio_get_level(HALL_DIGITAL) == 0;
        if (state != last_state) {
            printf("Button state changed: %d\n", state);
            last_state = state;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
*/
//test sleep mode
/*
void sensor_hall(void*pVparameters){
    
    gpio_config_t io_cfg ={
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << HALL_DIGITAL),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_cfg);
    gpio_set_direction(HALL_DIGITAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HALL_DIGITAL, GPIO_PULLUP_ONLY);
    int cState = gpio_get_level(HALL_DIGITAL);
    int nState = !cState;
    for(;;){
        gpio_set_direction(HALL_DIGITAL, GPIO_MODE_INPUT);
        gpio_set_pull_mode(HALL_DIGITAL, GPIO_PULLUP_ONLY);
        if(nState != cState){
            cState = nState;
            nState = gpio_get_level(HALL_DIGITAL);
            if(nState != cState){
                
                if (nState == 0){
                    gpio_wakeup_enable(HALL_DIGITAL, GPIO_INTR_HIGH_LEVEL);
                }else{
                    gpio_wakeup_enable(HALL_DIGITAL, GPIO_INTR_LOW_LEVEL);
                }
                
                //esp_sleep_enable_gpio_wakeup();
                printf("Hall sensor state changed: %d\n", nState);
                //vTaskDelay(pdMS_TO_TICKS(200));
                //esp_light_sleep_start();
                printf("bangunn\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
*/

/* Hall Sensor Unipolar */

void sensor_hall(void*pVparameters){
    uint32_t value;
    gpio_config_t io_cfg ={
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << HALL_DIGITAL),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_cfg);
    //gpio_set_direction(HALL_DIGITAL, GPIO_MODE_INPUT);
    //gpio_set_pull_mode(HALL_DIGITAL, GPIO_PULLUP_ONLY);
    int cState = gpio_get_level(HALL_DIGITAL);
    int nState = !cState;
    for(;;){
        if(nState != cState){
            cState = nState;
            nState = gpio_get_level(HALL_DIGITAL);
            if(nState != cState){
                printf("Hall sensor state changed: %d\n", nState);
                value = nState;
                if(xQueueSendToBack(queue, &value, 1) != pdTRUE){
                    printf("Failed to send value to queue\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


/* Hall Sensor Linear */
/*
static void sensor (void *pvParameters){
    uint32_t value;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_HALL_SENSOR, ADC_ATTEN_DB_0);
    for (;;) {
        printf("Waiting For DOOR Closed :\n");
        while(dOpen){
            raw_adc_value = adc1_get_raw(ADC_HALL_SENSOR);
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
        status = false;
        value = 1;
        if(xQueueSendToBack(queue, &value, 0) != pdTRUE){
            printf("Failed to send value to queue\n");
        }
        printf("Waiting for door Open\n");
        while(dClosed){
            raw_adc_value = adc1_get_raw(ADC_HALL_SENSOR);
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
        value = 0;
        if(xQueueSendToBack(queue, &value, 0) != pdTRUE){
            printf("Failed to send value to queue\n");
        }
        status = true;
        vTaskDelay(pdMS_TO_TICKS(100));
        
    }
}
*/


static void ble_spp_client_set_handle(const struct peer *peer)
{
    //int rc;
    const struct peer_chr *chr;
    chr = peer_chr_find_uuid(peer,
                             BLE_UUID128_DECLARE(GATT_SPP_SVC_UUID),
                             BLE_UUID128_DECLARE(GATT_SPP_CHR_UUID));
    attribute_handle[peer->conn_handle - 1] = chr->chr.val_handle;
    attr_handle = chr->chr.val_handle;
    conn_handle = peer->conn_handle;

    xTaskCreate(sensor_hall, "hall sensor", 2048, NULL, 5, NULL );    
    xTaskCreate(send, "send data", 4096,NULL, 5, NULL);    
}

static void disc_on_complete_cb(const struct peer *peer, int status, void *arg){
    if(status != 0){
        MODLOG_DFLT(ERROR, "Error: Service discovery failed; status=%d "
                    "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    MODLOG_DFLT(INFO, "Service discovery complete; status=%d "
                "conn_handle=%d\n", status, peer->conn_handle);
    
    ble_spp_client_set_handle(peer);
    //kirim_data(peer);

}

void ble_app_scan(void)
{
    int rc;
    printf("Start scanning ...\n");

    struct ble_gap_disc_params disc_params;
    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
    
}
static int blecent_should_conn(const struct ble_gap_disc_desc *disc){
    struct ble_hs_adv_fields fields;
    int rc;
    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0){
        MODLOG_DFLT(WARN, "Cant Parse Field");
        return rc;
    }

    if(fields.name_len > 0 ){
        if(memcmp(fields.name, SERVER_NAME, fields.name_len) == 0)
            return 1;
    }
    return 0;
}

static void ble_try_connect(const struct ble_gap_disc_desc *disc){
    uint8_t own_addr_type;
    int rc;
    ble_addr_t *addr;

    if (!blecent_should_conn(disc)){
        MODLOG_DFLT(INFO, "shouldnt connect");
        return;
    }

    rc = ble_gap_disc_cancel();
    if (rc != 0){
        MODLOG_DFLT(DEBUG, "Failed to Cancel Scan, rc : %d\n",rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_gap_connect(own_addr_type, &disc->addr, 30000, NULL, ble_gap_event_cb,NULL);
        if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error : Failed to Connect to : addr_type = %d , addr : %s: rc = %d\n", disc->addr.type, addr_str(disc->addr.val), rc);
        return;
    }

}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    
    int rc;
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                            event->disc.length_data);
            if (rc != 0) {
                return 0;
            }

            /* An advertisment report was received during GAP discovery. */
            print_adv_fields(&fields);

            /* Try to connect to the advertiser if it looks interesting. */
            ble_try_connect(&event->disc);
            return 0;
        case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            MODLOG_DFLT(INFO, "Connection established , status=%d\n", event->connect.status);
            print_conn_desc(&desc);
            MODLOG_DFLT(INFO, "\n");
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }
            rc = peer_disc_all(event->connect.conn_handle, disc_on_complete_cb, NULL);
            if (rc != 0 ){
                MODLOG_DFLT(ERROR, "Failed to discovery rc: %d\n", rc);
                return 0;
            }
        }
        else{
            MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n",
            event->connect.status);
            //connect_ble();
            ble_app_scan();
            
        }
        break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED %s", event->connect.status == 1 ? "OK!" : "DISCONNECTED!");
            peer_delete(event->disconnect.conn.conn_handle);
            //connect_ble();
            ble_app_scan();
            break;
        case BLE_GAP_EVENT_CONN_UPDATE:
            printf("Connection updated");
            break;
        default:
            printf("Unknown gap event: %d", event->type);
            break;
    }

    return 0;
}

/*
void connect_ble(void){ 
    struct ble_gap_conn_params c_param;
    ble_addr_t addr = {
        .type = BLE_OWN_ADDR_PUBLIC
    };
    addr.val[0] = 0xA2;  //A2
    addr.val[1] = 0x79;  //79
    addr.val[2] = 0x75;  //75
    addr.val[3] = 0x60;  //60
    addr.val[4] = 0x62;  //62
    addr.val[5] = 0xEC;  //EC

    memset(&c_param, 0, sizeof c_param);
    c_param.scan_itvl = 0x0100;
    c_param.scan_window = 0x0100;
    c_param.itvl_min = 0x0100;
    c_param.itvl_max = 0x0100;
    c_param.supervision_timeout = 0x0100;
    c_param.min_ce_len = 0x0100;
    c_param.max_ce_len = 0x0100;
    ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &addr, 30000 , &c_param, ble_gap_event_cb, NULL);

}
*/
void send (void *pvParameters){
    uint32_t value;
    int i;
    for(;;){
        if(xQueueReceive(queue, &value, portMAX_DELAY)==pdTRUE){
            uint8_t *mybuffer;
            mybuffer = malloc(sizeof(value));
            memset(mybuffer, 0, sizeof(value));
            memcpy(mybuffer, &value, sizeof(value));
            //printf("Attr handle = %d\n", attribute_handle[i]);
            int rc = ble_gattc_write_flat(conn_handle, attr_handle, mybuffer, sizeof(mybuffer), NULL, NULL);
            if (rc == 0){
                printf("Write succes\n");
            }
            else{
                printf("no\n");
            }
            free(mybuffer);                
            printf("Menerima data : %ld\n", value);
            int pin = gpio_get_level(HALL_DIGITAL);
            if (pin == 0){
                gpio_wakeup_enable(HALL_DIGITAL, GPIO_INTR_HIGH_LEVEL);
            }
            else{
                gpio_wakeup_enable(HALL_DIGITAL, GPIO_INTR_LOW_LEVEL);
            }
            esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO,ESP_PD_OPTION_ON);
            esp_sleep_enable_gpio_wakeup();
            esp_sleep_config_gpio_isolate();
            vTaskDelay(pdMS_TO_TICKS(200));
            //esp_deep_sleep_start();
            esp_light_sleep_start();
            printf("esp bangun\n");
            //ble_app_scan();

        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

}

static void
ble_spp_client_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}


void ble_app_on_sync(void)
{
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    //connect_ble();
    ble_app_scan();
}
void host_task(void *param)
{
   // connect_ble();
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
}
void app_main(void)
{
    
    int rc;
    queue = xQueueCreate(6000,sizeof(uint32_t));
    nvs_flash_init();
    nimble_port_init();
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);
    ble_svc_gap_device_name_set("Halo-hi"); // 4 - Set device name characteristic
    ble_svc_gap_init();                             // 4 - Initialize GAP service
    ble_store_config_init();

    ble_hs_cfg.reset_cb = ble_spp_client_on_reset;
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(host_task);
    
}