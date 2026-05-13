#ifndef __OTA_UPDATE_H
#define __OTA_UPDATE_H

#include "Arduino.h"

bool performOTAIfAvailable(const char* firmwareDevice,
                           const char* firmwareVersion,
                           const char* githubRepo,
                           volatile bool* isUpToDate);

#endif
