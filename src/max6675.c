#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include "max6675.h"

#define TAG "max6675"
#define MAX_DEVICES (3)

struct max6675_state {
	TimerHandle_t update_timer;
	struct max6675 max6675;
	unsigned instance;
	spi_device_handle_t spi;
};

static void max6675_init_spi(struct max6675_state *state);
static void update_timer_callback(TimerHandle_t timer);
static struct max6675_state max6675s[MAX_DEVICES] = { { 0 } };
static unsigned instances = 0;

int max6675_init(struct max6675 *max6675)
{
	char name[32];
	struct max6675_state *state;

	if (!max6675) {
		ESP_LOGE(TAG, "Init struct not set!");
		return -1;
	}
	if (instances >= MAX_DEVICES) {
		ESP_LOGE(TAG, "Too many devices!");
		return -1;
	}
	state = &max6675s[instances];
	state->instance = instances++;
	memcpy(&state->max6675, max6675, sizeof(*max6675));
	snprintf(name, sizeof(name), TAG "_%u", instances++);

	max6675_init_spi(state);

	state->update_timer =
		xTimerCreate(pcTaskGetName(NULL), pdMS_TO_TICKS(100), pdTRUE,
			     state, update_timer_callback);
	if (xTimerStart(state->update_timer, portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG, "Failed to create timer!");
		return -1;
	}

	return state->instance;
}

static void max6675_init_spi(struct max6675_state *state)
{
	int ret;
	spi_bus_config_t bus_cfg = {
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};
	spi_device_interface_config_t dev_cfg = {
		.queue_size = 7,
		.clock_speed_hz = 1 * 1000 * 1000,
		.mode = 0,
	};

	if (!state) {
		ESP_LOGE(TAG, "State struct not set!");
		return;
	}

	/* From https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/spi_master.html#overview */
	switch (state->max6675.spi) {
	case HSPI_HOST:
		bus_cfg.miso_io_num = 12;
		bus_cfg.mosi_io_num = 13;
		bus_cfg.sclk_io_num = 14;
		dev_cfg.spics_io_num = 15;
		break;
	case VSPI_HOST:
		bus_cfg.miso_io_num = 19;
		bus_cfg.mosi_io_num = 23;
		bus_cfg.sclk_io_num = 18;
		dev_cfg.spics_io_num = 5;
		break;
	default:
		ESP_LOGE(TAG, "Unsupported SPI interface!");
		return;
	}

	ret = spi_bus_initialize(state->max6675.spi, &bus_cfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	ret = spi_bus_add_device(state->max6675.spi, &dev_cfg, &state->spi);
	ESP_ERROR_CHECK(ret);
}

static void update_timer_callback(TimerHandle_t timer)
{
	struct max6675_state *state;
	int ret;
	spi_transaction_t trans = {
		.length = 2,
		.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
	};
	state = (struct max6675_state *)pvTimerGetTimerID(timer);
	ret = spi_device_polling_transmit(state->spi, &trans);
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "Received %u bytes: 1: 0x%02X 2: 0x%02X", trans.rxlength, trans.rx_data[0],
		 trans.rx_data[1]);
}
