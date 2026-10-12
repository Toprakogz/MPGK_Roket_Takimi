// Write your flight algorithm in this file.
// Keep every piece of algorithm state here and reset all of it in flight_algorithm_init().
#include "flight_algorithm.h"
#include "bench.h"  // nvm_write() / nvm_read(): simulated non-volatile memory
#include "cmath"
#include "cstring"
struct PersistentState {
    int current_state;
    bool drogue_deployed;
    bool main_deployed;
};
static FlightState current_state = STATE_PAD;
static bool drogue_commanded = false;
static bool main_commanded = false;

static uint32_t launch_counter = 0;
static uint32_t apogee_counter = 0;
static uint32_t main_chute_counter = 0;
static uint32_t landed_counter = 0;

static uint32_t drogue_fire_start_ms = 0;
static uint32_t main_fire_start_ms = 0;
void flight_algorithm_init(void) {
    // TODO: reset all algorithm state (and restore what you need with nvm_read())
    PersistentState p_state;
    if (nvm_read(&p_state, sizeof(PersistentState))) {
        current_state = (FlightState)p_state.current_state;
        drogue_commanded = p_state.drogue_deployed;
        main_commanded = p_state.main_deployed;
    } else {
        current_state = STATE_PAD;
        drogue_commanded = false;
        main_commanded = false;
        
        PersistentState initial_p = { (int)current_state, drogue_commanded, main_commanded };
        nvm_write(&initial_p, sizeof(PersistentState));
    }

    launch_counter = 0;
    apogee_counter = 0;
    main_chute_counter = 0;
    landed_counter = 0;
    drogue_fire_start_ms = 0;
    main_fire_start_ms = 0;
}

void flight_algorithm_step(const SensorSample *sample, const FilterOutput *filtered, AlgorithmOutput *out) {
    (void)sample;
    (void)filtered;
    // TODO: decide the flight state and the pyro outputs
    out->state = current_state;
    out->drogue_primary = false;
    out->drogue_backup = false;
    out->main_primary = false;
    out->main_backup = false;

    switch (current_state) {
        case STATE_PAD:
            if (filtered->vertical_accel_mps2 > 15.0f && filtered->vertical_speed_mps > 10.0f) {
                launch_counter++;
                if (launch_counter >= 3) {
                    current_state = STATE_BOOST;
                    launch_counter = 0;
                }
            } else {
                launch_counter = 0;
            }
            break;

        case STATE_BOOST:
            if (sample->t_ms > 3500 && filtered->vertical_accel_mps2 < 2.0f) {
                current_state = STATE_COAST;
            }
            break;

        case STATE_COAST:
            if (filtered->vertical_speed_mps <= 0.5f && filtered->altitude_m > 500.0f) {
                apogee_counter++;
                if (apogee_counter >= 5) {
                    current_state = STATE_DROGUE_DESCENT;
                    drogue_commanded = true;
                    drogue_fire_start_ms = sample->t_ms;
                    apogee_counter = 0;
                }
            } else {
                apogee_counter = 0;
            }
            break;

        case STATE_DROGUE_DESCENT:
            if (filtered->altitude_m <= 600.0f) {
                main_chute_counter++;
                if (main_chute_counter >= 5) {
                    current_state = STATE_MAIN_DESCENT;
                    main_commanded = true;
                    main_fire_start_ms = sample->t_ms;
                    main_chute_counter = 0;
                }
            } else {
                main_chute_counter = 0;
            }
            break;

        case STATE_MAIN_DESCENT:
            if (filtered->altitude_m <= 15.0f) {
                landed_counter++;
                if (landed_counter >= 30) {
                    current_state = STATE_LANDED;
                }
            } else {
                landed_counter = 0;
            }
            break;

        case STATE_LANDED:
            break;
    }
    if (drogue_commanded) {
        if (sample->t_ms - drogue_fire_start_ms < 500) {
            out->drogue_primary = true;
            out->drogue_backup = true;
        }
    }

    if (main_commanded) {
        if (sample->t_ms - main_fire_start_ms < 500) {
            out->main_primary = true;
            out->main_backup = true;
        }
    }

    out->state = current_state;
    PersistentState current_p = { (int)current_state, drogue_commanded, main_commanded };
    nvm_write(&current_p, sizeof(PersistentState));
}
const char *flight_state_name(int state) {
    switch (state) {
        case STATE_PAD: return "RAMPA";
        case STATE_BOOST: return "YUKSELIS";
        case STATE_COAST: return "SUZULME";
        case STATE_DROGUE_DESCENT: return "SURUKLENME_PARASUTU";
        case STATE_MAIN_DESCENT: return "ANA_PARASUT";
        case STATE_LANDED: return "YERDE";
        default: return "ARA_DURUM";  // add names for your own states here
    }
}
