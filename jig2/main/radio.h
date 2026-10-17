// radio.h — shared Wi-Fi bring-up and RC vendor-IE beacon TX (see radio.c).

#ifndef RADIO_H
#define RADIO_H

#include <stddef.h>
#include <stdint.h>

void radio_init_once(void);
void radio_start_sta(uint8_t channel);
int  radio_send_beacon(const uint8_t *body, size_t len);

#endif  // RADIO_H
