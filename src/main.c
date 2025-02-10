#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <stdio.h>
#include "sensor_lsm6dsox.h"
#include "rep_detection.h"
#include "ble_service.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Intervalle d'échantillonnage (en millisecondes) pour la lecture du capteur
#define SENSOR_READ_INTERVAL_MS  10  // ~100 Hz (à adapter selon vos besoins)

void main(void)
{
    int err;

    LOG_INF("FitBuddy (Priorité 1) - Démarrage du firmware");

    // 1. Initialisation du capteur IMU (LSM6DSOX)
    err = sensor_lsm6dsox_init();
    if (err) {
        LOG_ERR("Erreur init LSM6DSOX: %d", err);
        return;
    }

    // 2. Initialisation BLE (ou BLE Mesh)
    err = ble_service_init();
    if (err) {
        LOG_ERR("Erreur init BLE: %d", err);
        return;
    }

    // 3. Boucle principale
    while (1) {
        // Lecture IMU
        struct imu_data data;
        err = sensor_lsm6dsox_read(&data);
        if (err == 0) {
            // Détection et mise à jour des statistiques sur les répétitions
            update_repetition_stats(&data);

            // Récupération et affichage périodique des stats
            static uint32_t last_print_ms = 0;
            uint32_t now_ms = k_uptime_get_32();
            if ((now_ms - last_print_ms) >= 1000) {
                last_print_ms = now_ms;

                struct rep_stats stats = get_current_rep_stats();
                LOG_INF("Reps totales: %d - Reps serie: %d - Temps serie (ms): %u",
                        stats.total_reps, stats.current_reps, stats.current_series_time);
                LOG_INF("Vitesse moy. rep: %f - Amplitude moy.: %f",
                        stats.avg_speed, stats.avg_amplitude);

                // Transmission BLE (simple exemple, vous pouvez envoyer
                // la structure stats en notification)
                ble_service_send_rep_data(&stats);
            }
        }

        // Pause avant la prochaine lecture
        k_msleep(SENSOR_READ_INTERVAL_MS);
    }
}
