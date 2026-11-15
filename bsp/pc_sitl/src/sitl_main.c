#include "main_brain.h"
#include "sitl_physics.h"

#include <stdio.h>

extern hal_sys_t sitl_sys;
extern hal_gpio_t sitl_gpio;
extern hal_uart_t sitl_uart_sim;
extern hal_pwm_t sitl_pwm_esc;
extern hal_adc_t sitl_adc;
extern hal_storage_t sitl_storage;
extern hal_wireless_t sitl_wireless;
extern const imu_vtable_t sitl_imu_vtable;
sitl_physics_t *sitl_get_physics(void);
void sitl_sync_brain_sensors(pan_brain_system_t *brain);

static pan_brain_system_t g_sitl_brain;

int main(void)
{
    sitl_physics_t *phys = sitl_get_physics();
    sitl_physics_init(phys, 15.364700, 75.124000);

    imu_universal_register_virtual(&g_sitl_brain.imu, &sitl_imu_vtable, phys);

    pan_brain_bsp_t bsp = {
        .sys = &sitl_sys,
        .gpio = &sitl_gpio,
        .uart_sim = &sitl_uart_sim,
        .uart_ext = NULL,
        .i2c = NULL,
        .spi_imu = NULL,
        .spi_optflow = NULL,
        .pwm_esc = &sitl_pwm_esc,
        .pwm_gimbal = NULL,
        .adc = &sitl_adc,
        .storage = &sitl_storage,
        .flash = NULL,
        .wireless = &sitl_wireless,
        .pin_sim_pwrkey = 4,
        .pin_sim_dtr = 25,
        .pin_imu_int = 34,
        .pin_led = 12,
        .led_active_low = true
    };

    pan_brain_init(&g_sitl_brain, &bsp);
    g_sitl_brain.rc.sim_override = true;
    g_sitl_brain.rc.state.signal_valid = true;

    printf("=== PAN-BRAIN PC SITL 6-DOF MISSION VERIFICATION ===\n");

    /* Run 40 seconds of simulated flight at 500Hz (20,000 steps) */
    for (int step = 0; step < 20000; step++) {
        float t_s = (float)step * 0.002f;

        if (t_s >= 5.0f && t_s < 15.0f) {
            /* Phase A: Arm + Takeoff in Loiter to 15m */
            g_sitl_brain.rc.state.raw_ch[4] = 1900; /* ARM */
            g_sitl_brain.rc.state.raw_ch[5] = 1500; /* LOITER */
            g_sitl_brain.rc.state.raw_ch[2] = 1750; /* Climb stick */
            g_sitl_brain.rc.state.signal_valid = true;
        } else if (t_s >= 15.0f && t_s < 20.0f) {
            /* Phase B: Fly Waypoint 80m North-East */
            if (step == 7500) {
                mission_add_waypoint(&g_sitl_brain.mission, 15.365300, 75.124600, 20.0f, true);
                g_sitl_brain.flight_mode = PAN_MODE_MISSION;
                printf("[t=%.1fs] Waypoint Mission Dispatched\n", t_s);
            }
            g_sitl_brain.rc.state.raw_ch[2] = 1500;
        } else if (t_s >= 20.0f) {
            /* Phase C: Simulate Dual Loss (RC dropped + MQTT dropped) -> Smart RTH */
            if (step == 10000) {
                g_sitl_brain.rc.state.signal_valid = false;
                g_sitl_brain.modem.is_mqtt_connected = false;
                printf("[t=%.1fs] Triggered RC + LTE Dual-Loss Failsafe!\n", t_s);
            }
        }

        sitl_physics_step(phys, 0.002f);
        sitl_sync_brain_sensors(&g_sitl_brain);

        pan_brain_step_500hz(&g_sitl_brain);
        if ((step % 5) == 0) {
            pan_brain_step_100hz_nav(&g_sitl_brain);
            pan_brain_step_100hz_comms(&g_sitl_brain);
        }

        if ((step % 1000) == 0) {
            printf("[t=%4.1fs] Mode=%2u Armed=%d Alt=%5.2fm Pos=(%6.1f,%6.1f)m VBat=%.2fV LED_Pat=%u\n",
                   t_s,
                   (unsigned)g_sitl_brain.flight_mode,
                   g_sitl_brain.esc.armed ? 1 : 0,
                   g_sitl_brain.ekf.est_alt_agl_m,
                   phys->pn_m, phys->pe_m,
                   phys->vbat_mv * 0.001f,
                   (unsigned)g_sitl_brain.led.pattern);
        }
    }

    printf("=== SITL RUN COMPLETE ===\n");
    return 0;
}

