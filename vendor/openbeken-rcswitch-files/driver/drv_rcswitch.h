#ifndef DRV_RCSWITCH_H
#define DRV_RCSWITCH_H

// RC switch driver: listens in 802.11 monitor mode for the RC broadcast
// protocol (0x06 RelaysUpdate / 0x07 RelaysLedUpdate) and drives the relay +
// status LED. See src/driver/protocol.* and controller.*.

void RCSwitch_Init(void);
void RCSwitch_OnEverySecond(void);
void RCSwitch_RunQuickTick(void);
void RCSwitch_Stop(void);
void RCSwitch_AppendInformationToHTTPIndexPage(void *request, int bPreState);

#endif
