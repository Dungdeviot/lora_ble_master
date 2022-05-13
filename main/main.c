#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include <string.h>
#include"stdlib.h"
#include"driver/adc.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "kc225_gatts_server.h"
#include "esp_gatt_common_api.h"

#include "lora.h"
#include "esp32_lcd.c"
#include "esp32_lcd.h"


#define ESP_INTR_FLAG_DEFAULT 0
SemaphoreHandle_t xSemaphore = NULL;
bool flag = true;
int soil, soil_control, brightness_read, brightness_write;
float DIST; int DIST_read;
bool status_motor;

// LCD
#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2

#define GATTS_TABLE_TAG "GATT Server"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "Smart Agriculture"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t kc225_handle_table[KC225_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

//#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
		0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','E', 'M', 'O'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst kc225_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* 2: Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x3333;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0x003A;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0x003B;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0x003C;
static const uint16_t GATTS_CHAR_UUID_TEST_D       = 0x003D;
// dinh nghia them NEU bo sung CHAR

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[KC225_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

	/* Characteristic Declaration */
	[IDX_CHAR_D]      =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

	/* Characteristic Value */
	[IDX_CHAR_VAL_D]  =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_D, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};


// Khai báo prototype
void IRAM_ATTR button_isr_handler(void* arg);
void button_task(void* arg);
void task_rx(void *p);
void INT_Init();
void display(char* s, int len_status, int len_soil, int len_ultra, int len_pwm);
int char_to_num(char c);
float str_to_float(char *s);

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void app_main() {
	INT_Init();
	lora_init();
	lora_set_frequency(433e6);
	lora_enable_crc();
	LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
	gpio_set_direction(4, GPIO_MODE_OUTPUT);    //Led connect BLE

	esp_err_t ret;

	/* Initialize NVS. */
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(ESP_APP_ID);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret){
		ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}

	// Tạo cờ binary semaphore
	xSemaphore = xSemaphoreCreateBinary();

	xTaskCreatePinnedToCore(task_rx,"task RX",10000,NULL,1,NULL,0);
	xTaskCreatePinnedToCore(button_task,"Task TX",10000,NULL,1,NULL,1);

//	xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
//	xTaskCreate(task_rx, "task_rx", 4096, NULL, 10, NULL);
}

void button_task(void* arg){
	printf("button_task is running on Core: %d\n",xPortGetCoreID());
	for( ; ;) {// lập vô định
		// chờ semaphore từ ISR
		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE){
			//Gui lenh ON motor
			if(flag) lora_send_packet((uint8_t*)"ONX", sizeof("ONX"));
			//Gui lenh OFF motor
			if(!flag) lora_send_packet((uint8_t*)"OFX", sizeof("OFX"));

			printf("Packet sent...\n");
			// xử lý dội phím ví dụ
			gpio_intr_disable(GPIO_NUM_0);
			while(!gpio_get_level(GPIO_NUM_0));
			vTaskDelay(10/portTICK_PERIOD_MS);
			gpio_intr_enable(GPIO_NUM_0);
		}
	}
}

void task_rx(void *p) {
	printf("task_rx is running on Core: %d\n",xPortGetCoreID());
	lora_receive();    // put into receive mode
   	for(;;) {
   		lora_receive();    // put into receive mode
   		if(lora_received()) {
			uint8_t buff[16];
			lora_receive_packet(buff, sizeof(buff));	//Nhan data

			//Nhan gia tri do dai cua cac chuoi data
			size_t len_status = char_to_num(buff[0]);
			size_t len_soil = char_to_num(buff[1]);
			size_t len_ultra = char_to_num(buff[2]);
			size_t len_pwm = char_to_num(buff[3]);

			//Hien thi data len terminal va LCD
			printf("Received: %s\n", buff);
			display((char*)buff,len_status,len_soil,len_ultra,len_pwm);

			lora_receive();
   		}
   		vTaskDelay(pdMS_TO_TICKS(10));
   	}
}

//Chuyen ki tu thanh so
int char_to_num(char c) {
	return c - 48;
}

//Chuyen chuoi thanh so thuc
float str_to_float(char *s)
{
	int i = 0;
	int mode = 0;
	float num = 0.0;
	float phan_nguyen = 0.0;
	float phan_thap_phan = 0.0;
	int minus = 0;			//Khi co dau '-'
	while(s[i] != '\0') {
		if(s[i] == '-' ) {
			minus = 1 ;
			i++;
			continue;
		}
		if (s[i] == '.') {
			mode = 1;
			++i;
			continue;
		}
		if (mode == 0) { //Truoc '.' / lay phan nguyen
			phan_nguyen = phan_nguyen*10 + char_to_num(s[i]);
		}
		if (mode == 1) { //Sau '.' / lay phan thap phan
			phan_thap_phan = phan_thap_phan*10 + char_to_num(s[i]);
		}
		++i;
	}
	num = phan_nguyen + phan_thap_phan / 100;
	num = minus == 1 ? -num : num;
	return num;
}

void display(char* s, int len_status, int len_soil, int len_ultra, int len_pwm) {
	//LCD_clearScreen();	//Clear full screen
	LCD_setCursor(14, 0);	//Clear status
	LCD_writeStr("  ");

	LCD_setCursor(5, 0);	//Clear soil
	LCD_writeStr("   ");
	LCD_setCursor(9, 0);
	LCD_writeStr(" ");

	LCD_setCursor(5, 1);	//Clear distance
	LCD_writeStr("         ");

	//Distance value
	LCD_setCursor(0, 1);
	LCD_writeStr("DIST:");
	LCD_setCursor(14, 1);
	LCD_writeStr("cm");

	//Soil value
	LCD_setCursor(0, 0);
	LCD_writeStr("Soil:");
	LCD_setCursor(8, 0);
	LCD_writeStr("%");

	//Status motor
	LCD_setCursor(10, 0);
	LCD_writeStr("Stt:");

	int offset = 4; //Length of 3 string
	//-------------status----------------------
	// backup
	char backup = s[offset + len_status];
	s[offset + len_status] = '\0';
	LCD_setCursor(14, 0);
	LCD_writeStr(s + offset);	//Xuat trang thai ON/OF ra LCD
	if(!strcmp((const char*)(s + offset),"ON")){
		status_motor = 1;	//Read status motor
		flag = false;
	}
	if(!strcmp((const char*)(s + offset),"OF")){
		status_motor = 0;
		flag = true;
	}

	// restore
	s[offset + len_status] = backup;
	offset += len_status;

	//-------------soil----------------------
	// backup
	backup = s[offset + len_soil];
	s[offset + len_soil] = '\0';
	LCD_setCursor(5, 0);
	LCD_writeStr(s + offset);
	soil = str_to_float(s + offset); // read soil

	//-------------distance----------------------
	// restore
	s[offset + len_soil] = backup;
	offset += len_soil;
	LCD_setCursor(7, 1);
	backup = s[offset + len_ultra];
	s[offset + len_ultra] = '\0';
	LCD_writeStr(s + offset);		//Xuat gia tri muc nuoc ra LCD
	DIST = str_to_float(s + offset); // read distance

	DIST_read = DIST*100;		//Chuyen thanh gia tri int de hien thi len App
	if(DIST_read <= 0) DIST_read = 0;	//Chi nhan so +

	//-------------pwm----------------------
	s[offset + len_ultra] = backup;
	offset += len_ultra;
	brightness_read = str_to_float(s + offset);
}

void INT_Init(){
	// Cấu hình GPIO
	gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
	// Cho phép ngắt tại cạnh xuống
	gpio_set_intr_type(GPIO_NUM_0, GPIO_INTR_NEGEDGE);
	// Cài đặt dịch vụ ngắt mặc định
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	// Cấu hình handler
	gpio_isr_handler_add(GPIO_NUM_0, button_isr_handler, NULL);
}

void IRAM_ATTR button_isr_handler(void* arg){
	// Tạo semaphore, kích hoạt buttonTask
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, KC225_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT, handle = %d, :", kc225_handle_table[IDX_CHAR_VAL_A]);
            // Read ADC value

            if (kc225_handle_table[IDX_CHAR_VAL_A]==param->read.handle){
            	esp_gatt_rsp_t rsp;
				memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

				rsp.attr_value.handle = param->read.handle;
				rsp.attr_value.len = 5;
				rsp.attr_value.value[0] = status_motor;	//Doc trang thai motor
				rsp.attr_value.value[1] = soil;			//Doc gia tri do am
				rsp.attr_value.value[2] = DIST_read;	//Doc gia tri muc nuoc
				rsp.attr_value.value[3] = DIST_read>>8;	//16bit
				rsp.attr_value.value[4] = brightness_read;	//Doc gia tri do sang LED
				esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
											 ESP_GATT_OK, &rsp);
            }
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

            	//PWM

            	if (kc225_handle_table[IDX_CHAR_VAL_B]==param->write.handle){  //nhan thong tin tu app, chinh do sang led
            		brightness_write = param->write.value[0];

            		//Ghi gia tri do sang LED
            		char buff[5];
            		snprintf(buff, sizeof(buff), "%d", brightness_write);
            		size_t len = strlen(buff);
            		buff[len] = 'A';	//address
            		buff[len+1] = '\0';
            		lora_send_packet((uint8_t*)buff, strlen(buff)+1);
            	}
            	if (kc225_handle_table[IDX_CHAR_VAL_C]==param->write.handle){
            		int button0 = param->write.value[0];

            		//Bat/tat motor khi nhan phim tren App
            		if(button0 && flag) lora_send_packet((uint8_t*)"ONX", sizeof("ONX"));
            		if(!button0 && !flag) lora_send_packet((uint8_t*)"OFX", sizeof("OFX"));
            	}

            	if (kc225_handle_table[IDX_CHAR_VAL_D]==param->write.handle){
            	    soil_control = param->write.value[0];

            	    //Gui gia tri do am dat so sanh
					char buff[5];
					snprintf(buff, sizeof(buff), "%d", soil_control);
					size_t len = strlen(buff);
					buff[len] = 'B';		//address
					buff[len+1] = '\0';
					lora_send_packet((uint8_t*)buff, strlen(buff)+1);
            	}

            	/* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            gpio_set_level(4, 1);		//Bat LED bao ket noi
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            gpio_set_level(4, 0);		//Tat LED bao ket noi
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != KC225_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, KC225_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(kc225_handle_table, param->add_attr_tab.handles, sizeof(kc225_handle_table));
                esp_ble_gatts_start_service(kc225_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            kc225_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == kc225_profile_tab[idx].gatts_if) {
                if (kc225_profile_tab[idx].gatts_cb) {
                    kc225_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
