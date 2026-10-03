#pragma once
#include <nds.h>
#include <libtwl/rtos/rtosMutex.h>

// Protects all Slot-1 card-bus transactions (DLDI SD + DSPico USB).
extern rtos_mutex_t gCardMutex;
