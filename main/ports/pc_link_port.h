#ifndef PC_LINK_PORT_H
#define PC_LINK_PORT_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t pc_link_port_init(void);
bool pc_link_port_ready(void);
esp_err_t pc_link_port_send(const uint8_t *bytes, size_t size);
int pc_link_port_read(uint8_t *bytes, size_t capacity);

#endif
