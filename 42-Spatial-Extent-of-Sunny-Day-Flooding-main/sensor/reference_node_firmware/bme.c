// Copyright 2022 Blues Inc.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#include <math.h>
#include "main.h"
#include "bme280/bme280.h"
#include "appdefs.h"

//Application Defines **IMPORTANT**
#define POLLINGPERIOD				10	//how often a measurement is attempted
#define ACTIVATIONPERIOD			10	//how often a measurement is sent to gateway (300s = 5min)

// Special request IDs
#define REQUESTID_TEMPLATE          1

// The dynamic filename of the application specific queue.
// NOTE: The Gateway will replace `*` with the originating node's ID.
#define SENSORDATA_NOTEFILE         "*#air.qo"

// TRUE if we've successfully registered the template
static bool templateRegistered = false;


extern UART_HandleTypeDef huart1;
#define CHANDLER_TX_EN_Pin GPIO_PIN_2
#define CHANDLER_TX_EN_GPIO_Port GPIOB
#define CHANDLER_RX_EN_Pin GPIO_PIN_3
#define CHANDLER_RX_EN_GPIO_Port GPIOB

#define MAXNUMPOLLS		10
int numPolls = 0;

uint8_t TxData[16] = {0x04,0x03,0x00,0x08,0x00,0x02,0x45,0x9C};
uint8_t RxData[32];
int indx = 0;
static double salinity = 0;

// CRC-16/MODBUS parameters
#define CRC_POLYNOMIAL 0xA001
#define CRC_INITIAL_VALUE 0xFFFF

// Maximum size of the received message
#define MAX_RX_DATA_SIZE 9

// Maximum size of the buffer for storing valid messages
#define MAX_STORED_MSGS 100

// Stored messages buffer
uint8_t StoredMsgs[MAX_STORED_MSGS][MAX_RX_DATA_SIZE];

// Number of stored messages
uint16_t NumStoredMsgs = 0;

float hex_to_ieee754(uint8_t *hex_data) {
    uint32_t uint_value = ((uint32_t)hex_data[0]) |
                          ((uint32_t)hex_data[1] << 8) |
                          ((uint32_t)hex_data[2] << 16) |
                          ((uint32_t)hex_data[3] << 24);

    float *p_float = (float*)&uint_value;
    return *p_float;
}

void hex_to_bin(uint8_t hex, uint8_t *binary) {
    uint8_t i, j;

    for (i = 0; i < 8; i++) {
        j = (hex >> i) & 0x01;
        binary[7 - i] = j;
    }
}




double convertHexToDecimal(int hex[]) {
    int sign=0, exponent=0, significand=0;
    double decimal=0.0;
    int i;

    // combine the hex values to get the full binary representation
    int binary = 0;
    for (i = 0; i < 4; i++) {
        binary = (binary << 8) | hex[i];
    }

    // determine the sign, exponent, and significand bits
    sign = (binary >> 31) & 1;
    exponent = (binary >> 23) & 0xff;
    significand = binary & 0x7fffff;

    // convert to decimal using the formula
    decimal = (sign ? -1 : 1) * (1 + (double)significand / pow(2, 23)) * pow(2, exponent - 127);
    return decimal;
}

// CRC-16/MODBUS calculation function
uint16_t crc16_modbus(uint8_t *data, uint16_t length, uint16_t initial_value)
{
    uint16_t crc = initial_value;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ CRC_POLYNOMIAL;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

//data transmission functions
bool sendData (uint8_t *data)
{
	//increase numPolls in order to keep track of how many times we requested a measurement
	//this number is reset when a successful measurement is sent to the gateway
	numPolls++;
	int Rx4Byte[4];
	uint8_t TxData[16] = {0x04,0x03,0x00,0x08,0x00,0x02,0x45,0x9C};
	//uint8_t TxData[16] = {0x04,0x03,0x04,0x42,0xB2,0xF7,0xA8,0x5C,0xE2};
	// Pull DE high to enable TX operation
	//PA15 is connected to both Receive enable and transmit enable
	HAL_GPIO_WritePin(CHANDLER_TX_EN_GPIO_Port, CHANDLER_TX_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, TxData, 8 , 1000);//changed strlen (data) to 9
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(CHANDLER_TX_EN_GPIO_Port, CHANDLER_TX_EN_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_DMA (&huart1, RxData, 32);

	uint16_t crc = crc16_modbus(RxData, MAX_RX_DATA_SIZE - 2, CRC_INITIAL_VALUE);
	//printf("Calculated CRC-16/MODBUS: 0x%04X\n", crc);

	// Extract the CRC bytes from the received message
	uint16_t received_crc = (RxData[MAX_RX_DATA_SIZE - 1] << 8) | RxData[MAX_RX_DATA_SIZE - 2];

	//printf("Received CRC-16/MODBUS: 0x%04X\n", received_crc);

	if (crc == received_crc) {
			// If they match, store the received message
		for (int i = 3; i < 7; i++) {
			Rx4Byte[i-3] = RxData[i];
		}
		salinity = convertHexToDecimal(Rx4Byte);
		if(salinity==0)
			salinity = 0.00000001;//flag for sensor being dry (zero conductivity)
		//JSON doesn't display body's with a value of zero so we are setting it to almost zero
		return 1;
	} else {
		printf("Invalid message received!\n");
		//need to reset salinity measurement
		return 0;
	}
	salinity = convertHexToDecimal(Rx4Byte);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 16);
}

// An instance of an env sample
//not using this
typedef struct {
    double temperature;
    double pressure;
    double humidity;
} envSample;
static envSample envSamples[5];
static envSample lastBME = {0};

// Which I2C device we are using
extern I2C_HandleTypeDef hi2c2;

// Device address
static uint8_t bme_dev_addr = 0;

// Whether or not the next note should sync
static bool syncNow = false;

// Our scheduled app's ID
static int appID = -1;

// Forwards
static bool bme280_read(struct bme280_dev *dev, struct bme280_data *comp_data);
static void bme280_delay_us(uint32_t period, void *intf_ptr);
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static bool addNote(void);
static bool registerNotefileTemplate(void);
static bool bmeUpdate(void);
static void bmePoll(int appID, int state, void *appContext);
static void bmeResponse(int appID, J *rsp, void *appContext);

// Scheduled App One-Time Init
bool bmeInit()
{
	// Power on the sensor to see if it's here
	    GPIO_InitTypeDef init = {0};
	    init.Speed = GPIO_SPEED_FREQ_HIGH;
	    init.Pin = BME_POWER_Pin;
	    init.Mode = GPIO_MODE_OUTPUT_PP;
	    init.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(BME_POWER_GPIO_Port, &init);
	    HAL_GPIO_WritePin(BME_POWER_GPIO_Port, BME_POWER_Pin, GPIO_PIN_SET);
	    MX_I2C2_Init();
	    bool success = 1;
	    MX_I2C2_DeInit();
	    HAL_GPIO_WritePin(BME_POWER_GPIO_Port, BME_POWER_Pin, GPIO_PIN_RESET);
	    if (success) {
	        appSetSKU(SKU_REFERENCE);
	    } else {
	        return false;
	    }

	    // Register the app
	    schedAppConfig config = {
	        .name = "bme",
	        .activationPeriodSecs = ACTIVATIONPERIOD,
	        .pollPeriodSecs = POLLINGPERIOD,
	        .activateFn = NULL,
	        .interruptFn = NULL,
	        .pollFn = bmePoll,
	        .responseFn = bmeResponse,
	    };
	    appID = schedRegisterApp(&config);
	    if (appID < 0) {
	        return false;
	    }

	    // Done

	    return true;


}

// Poller
void bmePoll(int appID, int state, void *appContext)
{

    // Disable if this isn't a reference sensor
    if (appSKU() != SKU_REFERENCE) {
        schedDisable(appID);
        return;
    }

    // Switch based upon state
    switch (state) {

    case STATE_ACTIVATED:
        if (!templateRegistered) {
            registerNotefileTemplate();
            schedSetCompletionState(appID, STATE_ACTIVATED, STATE_DEACTIVATED);
            APP_PRINTF("RIKA500: template registration request\r\n");
            break;
        }
        if (!addNote()) {
            schedSetState(appID, STATE_DEACTIVATED, "RIKA500: update failure");
        } else {
            schedSetCompletionState(appID, STATE_DEACTIVATED, STATE_DEACTIVATED);//deactivate the application after a successful queue
            APP_PRINTF("RIKA500: note queued\r\n");
        }
        break;

    }

}

// Register the notefile template for our data
static bool registerNotefileTemplate()
{

    // Create the request
    J *req = NoteNewRequest("note.template");
    if (req == NULL) {
        return false;
    }

    // Create the body
    J *body = JCreateObject();
    if (body == NULL) {
        JDelete(req);
        return false;
    }

    // Add an ID to the request, which will be echo'ed
    // back in the response by the notecard itself.  This
    // helps us to identify the asynchronous response
    // without needing to have an additional state.
    JAddNumberToObject(req, "id", REQUESTID_TEMPLATE);

    // Fill-in request parameters.  Note that in order to minimize
    // the size of the over-the-air JSON we're using a special format
    // for the "file" parameter implemented by the gateway, in which
    // a "file" parameter beginning with * will have that character
    // substituted with the textified sensor address.
    JAddStringToObject(req, "file", SENSORDATA_NOTEFILE);

    // Fill-in the body template
    JAddNumberToObject(body, "salinity", TFLOAT32);
    JAddNumberToObject(body, "voltage", TFLOAT32);
    JAddNumberToObject(body, "attempts", TINT16);

    // Attach the body to the request, and send it to the gateway
    JAddItemToObject(req, "body", body);
    noteSendToGatewayAsync(req, true);
    return true;

}

// Gateway Response handler
void bmeResponse(int appID, J *rsp, void *appContext)
{

    // See if there's an error
    char *err = JGetString(rsp, "err");
    if (err[0] != '\0') {
        APP_PRINTF("RIKA500: gateway returned error: %s\r\n", err);
        return;
    }

    // Flash the LED if this is a response to this specific ping request
    switch (JGetInt(rsp, "id")) {

    case REQUESTID_TEMPLATE:
        templateRegistered = true;
        APP_PRINTF("RIKA500: SUCCESSFUL template registration\r\n");
        break;
    }

}

// Send the sensor data
static bool addNote()
{

    // Measure the sensor values
    MX_USART1_UART_Init();
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 16);
	bool success = sendData(TxData);
	MX_USART1_UART_DeInit();
    if (!success) {
    	//if there wasn't a successful measurement taken in MAXNUMPOLLS attempts,
    	//create a note request saying the update failed. if it hasn't been MAXNUMPOLLS yet,
    	//then return false and take another measurement
        APP_PRINTF("RIKA500: update failed\r\n");
        if(numPolls >= MAXNUMPOLLS){
        	// Create the request
        	    J *req = NoteNewRequest("note.add");
        	    if (req == NULL) {
        	        return false;
        	    }

        	    // Create the body
        	    J *body = JCreateObject();
        	    if (body == NULL) {
        	        JDelete(req);
        	        return false;
        	    }

        	    // Set the target notefile
        	    JAddStringToObject(req, "file", SENSORDATA_NOTEFILE);

        	    // If immediate, sync now
        	    if (syncNow) {
        	        syncNow = false;
        	        JAddBoolToObject(req, "sync", true);
        	    }

        	    // Fill-in the body
        	    JAddNumberToObject(body, "salinity", -1);

        	    // Add the voltage, just for convenient reference
        	#ifdef USE_SPARROW
        	    JAddNumberToObject(body, "voltage", MX_ADC_A0_Voltage());
        	#endif
        	    JAddNumberToObject(body, "attempts", numPolls);
        	    // Attach the body to the request, and send it to the gateway
        	    JAddItemToObject(req, "body", body);
        	    noteSendToGatewayAsync(req, false);
        	    numPolls = 0;//reset numPolls
        	    return true;
        }
        return false;
    }

    // Create the request
    J *req = NoteNewRequest("note.add");
    if (req == NULL) {
        return false;
    }

    // Create the body
    J *body = JCreateObject();
    if (body == NULL) {
        JDelete(req);
        return false;
    }

    // Set the target notefile
    JAddStringToObject(req, "file", SENSORDATA_NOTEFILE);

    // If immediate, sync now
    if (syncNow) {
        syncNow = false;
        JAddBoolToObject(req, "sync", true);
    }

    // Fill-in the body
    JAddNumberToObject(body, "salinity", salinity);

    // Add the voltage, just for convenient reference
#ifdef USE_SPARROW
    JAddNumberToObject(body, "voltage", MX_ADC_A0_Voltage());
#endif
    JAddNumberToObject(body, "attempts", numPolls);
    // Attach the body to the request, and send it to the gateway
    JAddItemToObject(req, "body", body);
    noteSendToGatewayAsync(req, false);
    numPolls = 0;//reset numPolls
    return true;


}


//-----------------------------------------------------------------------------------------
//NOT USING ANY OF THE FUNCTIONS BELOW FOR RIKA500
//-----------------------------------------------------------------------------------------


// Update the static temp/humidity/pressure values with the most accurate
// values that we can by averaging several samples.
bool bmeUpdate()
{
    bool success = false;

    // Determine whether it's on primary or secondary address
    struct bme280_dev dev;
    dev.intf = BME280_I2C_INTF;
    dev.read = bme280_i2c_read;
    dev.write = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;
    dev.intf_ptr = &bme_dev_addr;
    bme_dev_addr = BME280_I2C_ADDR_PRIM;
    if (bme280_init(&dev) != BME280_INTF_RET_SUCCESS) {
        return success;
    }

    // Allocate a sample buffer
    lastBME.temperature = lastBME.humidity = lastBME.pressure = 0.0;

    // Ignore the first two readings for settling purposes
    struct bme280_data comp_data;
    bme280_read(&dev, &comp_data);
    bme280_read(&dev, &comp_data);

    // Take a set of measurements, discarding data that is very high or very low.  We
    // do this to get the most accurate sample possible.
    int validSamples = 0;
    int totalRetries = 0;
    int maxRetries = (sizeof(envSamples) / sizeof(envSamples[0]))*2;
    struct bme280_data prev_data = {0};
    int samples = sizeof(envSamples) / sizeof(envSamples[0]);
    for (int i=0; i<samples; i++) {

        // Read the sample, and retry if I2C read failure
        if (!bme280_read(&dev, &comp_data)) {
            if (totalRetries++ < maxRetries) {
                --i;
            }
            continue;
        }

        // If we haven't yet converged, retry
        if (totalRetries < maxRetries && i > 0) {
            double tempPct = fabs((comp_data.temperature-prev_data.temperature)/prev_data.temperature);
            double pressPct = fabs((comp_data.pressure-prev_data.pressure)/prev_data.pressure);
            double humidPct = fabs((comp_data.humidity-prev_data.humidity)/prev_data.humidity);
            bool retry = false;
            if (tempPct >= 0.001) {
                retry = true;
            }
            if (pressPct >= 0.0001) {
                retry = true;
            }
            if (humidPct >= 0.005) {
                retry = true;
            }
            if (retry) {
                totalRetries++;
                i = -1;
                validSamples = 0;
                continue;
            }
        }
        memcpy(&prev_data, &comp_data, sizeof(struct bme280_data));

        // Store the sample
        envSamples[validSamples].temperature = comp_data.temperature;
        envSamples[validSamples].pressure = comp_data.pressure;
        envSamples[validSamples].humidity = comp_data.humidity;
        validSamples++;

    }

    // Average the samples (assuming lastBME is already zero'ed)
    if (validSamples) {
        for (int i=0; i<validSamples; i++) {
            lastBME.temperature += envSamples[i].temperature;
            lastBME.pressure += envSamples[i].pressure;
            lastBME.humidity += envSamples[i].humidity;
        }
        lastBME.temperature /= validSamples;
        lastBME.pressure /= validSamples;
        lastBME.humidity /= validSamples;
        success = true;
    }

    // Put the sensor to sleep, to save power if we're leaving it on
    bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);

    // Done
    return success;

}

// BME280 sensor read
bool bme280_read(struct bme280_dev *dev, struct bme280_data *comp_data)
{
    int8_t rslt;
    uint8_t settings_sel;

    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_INTF_RET_SUCCESS) {
        return false;
    }
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
    if (rslt != BME280_INTF_RET_SUCCESS) {
        return false;
    }

    // Delay while the sensor completes a measurement
    dev->delay_us(70000, dev->intf_ptr);
    memset(comp_data, 0, sizeof(struct bme280_data));
    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, dev);
    if (rslt != BME280_INTF_RET_SUCCESS) {
        return false;
    }

    // If the data looks bad, don't accept it.  (Humidity does operate
    // at the extremes, but these do not and we've seen these failures
    // concurrently, where temp == -40 and press == 110000 && humid == 100%)
    if (comp_data->temperature == -40           // temperature_min
            || comp_data->pressure == 30000.0       // pressure_min
            || comp_data->pressure == 110000.0) {   // pressure_max
        return false;
    }

    return true;
}

// Delay
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    HAL_DelayUs(period);
}

// Read from sensor
int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bool success = MY_I2C2_ReadRegister(bme_dev_addr, reg_addr, reg_data, len, 5000);
    return (success ? BME280_INTF_RET_SUCCESS : !BME280_INTF_RET_SUCCESS);
}

// Write to sensor
int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bool success = MY_I2C2_WriteRegister(bme_dev_addr, reg_addr, (uint8_t *) reg_data, len, 5000);
    return (success ? BME280_INTF_RET_SUCCESS : !BME280_INTF_RET_SUCCESS);
}
