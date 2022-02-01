#pragma once

#include <stdint.h>
#include <hal/spi_types.h>

struct max6675 {
	spi_host_device_t spi;
};

int max6675_init(struct max6675 *max6675);
