#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

esp_bd_addr_t sensor_list[3] = {{0xA4, 0xC1, 0x38, 0x71, 0x86, 0xE4},{0xA4, 0xC1, 0x38, 0xA4, 0xFA, 0x78},{0xA4, 0xC1, 0x38, 0x07, 0x43, 0x7B}};

typedef struct sensor_struct{
    //char name[20];
    esp_bd_addr_t addres;
    float temperature;
    int humidity;
    int battery;
} sensor;

sensor sypialnia = {{0xA4, 0xC1, 0x38, 0x71, 0x86, 0xE4}, 0, 0, 0};
sensor balkon = {{0xA4, 0xC1, 0x38, 0xA4, 0xFA, 0x78}, 0, 0, 0};
sensor lazienka = {{0xA4, 0xC1, 0x38, 0x07, 0x43, 0x7B}, 0, 0, 0};



// array of found devices
#define SAVED_DEVICES 3
//esp_bd_addr_t saved_devices[SAVED_DEVICES][6] = {sensor_1, sensor_2, sensor_3};
//int discovered_devices_num = 0;

// scan parameters
static esp_ble_scan_params_t ble_scan_params = {
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval          = 0x50,
		.scan_window            = 0x30
	};

bool serchForDevice(esp_bd_addr_t f_address){
    bool found = false;
    int match = 0;
	for (uint8_t j=0; j < sizeof(sensor_list) / sizeof(sensor_list[0]); j++){	
		for (uint8_t i = 0; i < ESP_BD_ADDR_LEN / 2; i++){
				if (f_address[i] == sensor_list[j][i]){
					match++;
				}
		}
		if (match == ESP_BD_ADDR_LEN / 2){
		found = true;
		}
	break;
    }
    return found;
}

void printToConsole(){
	return;
}

// GAP callback
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
		
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
				
			printf("ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT\n");
			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan parameters set, start scanning for 10 seconds\n\n");
				esp_ble_gap_start_scanning(10);
			}
			else printf("Unable to set scan parameters, error code %d\n\n", param->scan_param_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			
			printf("ESP_GAP_BLE_SCAN_START_COMPLETE_EVT\n");
			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan started\n\n");
			}
			else printf("Unable to start scan process, error code %d\n\n", param->scan_start_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			
			if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
				if(serchForDevice(param->scan_rst.bda)) {
					
					printf("ESP_GAP_BLE_SCAN_RESULT_EVT\n");
					printf("Device found: ADDR=");
					for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
						printf("%X", param->scan_rst.bda[i]);
						if(i != ESP_BD_ADDR_LEN -1) printf(":");
					}
                    printf("\nDevice data : ");
                    for(int i = 0; i < param->scan_rst.adv_data_len; i++){
                        printf("%02X", param->scan_rst.ble_adv[i]);
                        if(i != param->scan_rst.adv_data_len -1) printf(":");
                    }
                    uint16_t temp = 0;
                    temp = param->scan_rst.ble_adv[10] << 8;
                    temp = temp |  param->scan_rst.ble_adv[11];
                    float f_temp = (float)temp / 10;
                    printf("\nBattery: %d", param->scan_rst.ble_adv[13]);
                    printf("\nHumidity: %d", param->scan_rst.ble_adv[12]);
                    printf("\nTemperature: %d", temp); //temperature is rempresented as follow 23.5*C in DEC 235 HEX 0xEB
                    printf("\nTemp_f: %0.1f", f_temp);
					
					printf("\n\n");
					
				}
				
			}
			else if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
				printf("Scan complete\n\n");
			break;
		
		default:
		
			printf("Event %d unhandled\n\n", event);
			break;
	}
}


void app_main() {

	printf("BT scan\n\n");
	
	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);
	
	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	printf("- NVS init ok\n");
	
	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	printf("- Memory for classic BT released\n");
	
	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	printf("- BT controller init ok\n");
	
	// enable the BT controller in BLE mode
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
	printf("- BT controller enabled in BLE mode\n");
	
	// initialize Bluedroid library
	ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
	printf("- Bluedroid initialized and enabled\n");
	
	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	printf("- GAP callback registered\n\n");
	
	
	
	while(true){
		// configure scan parameters
		esp_ble_gap_set_scan_params(&ble_scan_params);
		vTaskDelay(30000 / portTICK_PERIOD_MS);
	}
}