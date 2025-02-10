// rep_detection.h
#ifndef REP_DETECTION_H
#define REP_DETECTION_H

#include "sensor_lsm6dsox.h"

#ifdef __cplusplus
extern "C" {
#endif

struct rep_stats {
    int total_reps;            // Nombre total de reps toutes séries confondues
    int current_reps;          // Nombre de reps dans la série en cours
    uint32_t current_series_time; // Durée de la série en millisecondes
    float avg_speed;           // Vitesse moyenne (simplifiée)
    float avg_amplitude;       // Amplitude moyenne
    // ... Ajoutez ce dont vous avez besoin
};

// A appeler périodiquement après chaque lecture IMU
void update_repetition_stats(const struct imu_data *data);

// Récupère l’état courant des stats
struct rep_stats get_current_rep_stats(void);

#ifdef __cplusplus
}
#endif

#endif // REP_DETECTION_H

