#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t omron_i2c_init(void);
void omron_i2c_scan(void);
esp_err_t omron_read(void);

#ifdef __cplusplus
}
#endif
