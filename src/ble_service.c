// ble_service.h
#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include "rep_detection.h"

#ifdef __cplusplus
extern "C" {
#endif

int ble_service_init(void);

// Exemple de fonction pour envoyer les données "rep_stats" en BLE
int ble_service_send_rep_data(const struct rep_stats *stats);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVICE_H
