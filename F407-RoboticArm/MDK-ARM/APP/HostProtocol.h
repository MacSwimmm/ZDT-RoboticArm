#ifndef __HOST_PROTOCOL_H
#define __HOST_PROTOCOL_H

#include "main.h"

/* PC <-> F407 protocol on USART2 (DAP-Link virtual COM port). */
void HostProtocol_Init(void);
void HostProtocol_Process(void);

#endif
