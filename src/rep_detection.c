// rep_detection.c
#include "rep_detection.h"
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(main);

// Paramètres de détection
#define ACC_THRESHOLD  1.2f // 1.2 g par ex. (à adapter)
#define MIN_REP_INTERVAL_MS 300  // On évite de détecter de fausses reps trop rapprochées

static struct rep_stats g_stats = {0};
static bool in_rep = false;
static uint32_t last_rep_time = 0;
static float speed_sum = 0.0f;
static float amplitude_sum = 0.0f;
static int rep_count_in_series = 0;

// On mémorise le temps de début de la série pour calculer la durée
static uint32_t series_start_time = 0;

// Fonction appelée par main()
void update_repetition_stats(const struct imu_data *data)
{
    // Calcul du vecteur accélération total (approx.)
    float accel_mag = sqrtf(data->accel_x * data->accel_x +
                            data->accel_y * data->accel_y +
                            data->accel_z * data->accel_z);

    // Estimation rapide de vitesse = dérivée d’accélération
    // (Ici simplifié: on peut se baser sur l’accel. instantanée comme "pseudo-speed")
    float speed_estimate = accel_mag; 

    // Vérifie si on dépasse un certain seuil => mouvement significatif
    bool rep_detected = (accel_mag >= ACC_THRESHOLD);

    uint32_t now_ms = k_uptime_get_32();

    // Initialisation si nouvelle série ?
    if (g_stats.current_reps == 0 && !in_rep) {
        // On considère qu’on commence une série
        series_start_time = now_ms;
    }

    if (rep_detected && !in_rep) {
        // Début d’une répétition
        // Vérifie qu’on n’a pas déjà compté une rep récemment
        if ((now_ms - last_rep_time) > MIN_REP_INTERVAL_MS) {
            in_rep = true;
            last_rep_time = now_ms;
            rep_count_in_series++;
        }
    } else if (!rep_detected && in_rep) {
        // Fin de la répétition
        in_rep = false;
        // Fin de la rep => actualise statistiques
        g_stats.total_reps++;
        g_stats.current_reps = rep_count_in_series;

        // On accumule la "vitesse" moyenne
        speed_sum += speed_estimate;
        // On accumule l’amplitude
        amplitude_sum += (accel_mag);

        // Possibilité de calculer un "niveau de difficulté" = ex. pic d’accélération
        // Dans l’exemple simplifié, on pourrait le logguer directement
        LOG_INF("Niveau de difficulté approx.: %f", accel_mag);
    }

    // Mise à jour temps de série
    if (rep_count_in_series > 0) {
        g_stats.current_series_time = (now_ms - series_start_time);
    } else {
        g_stats.current_series_time = 0;
    }

    // Exemple d’une détection de fin de série (pause trop longue)
    // => On réinitialise si plus de 3 secondes sans rep.
    // C’est l’occasion de calculer le temps de pause, etc.
    if (!in_rep && rep_count_in_series > 0) {
        if ((now_ms - last_rep_time) > 3000) {
            // Fin de série
            rep_count_in_series = 0;
            speed_sum = 0.0f;
            amplitude_sum = 0.0f;
        }
    }
}

struct rep_stats get_current_rep_stats(void)
{
    struct rep_stats stats_copy = g_stats;

    // Calcule la vitesse/amplitude moyennes sur la série en cours
    if (rep_count_in_series > 0) {
        stats_copy.avg_speed = speed_sum / (float)rep_count_in_series;
        stats_copy.avg_amplitude = amplitude_sum / (float)rep_count_in_series;
    } else {
        stats_copy.avg_speed = 0.0f;
        stats_copy.avg_amplitude = 0.0f;
    }

    return stats_copy;
}
