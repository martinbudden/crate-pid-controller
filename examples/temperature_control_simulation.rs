// This example simulates a temperature control system using a PID controller.
// It is based on the Pidgeon](https://crates.io/crates/pidgeon) example at <https://github.com/security-union/pidgeon/blob/main/crates/pidgeon/examples/debug_temperature_control.rs>

use pidsk_controller::{PidControllerf32, PidGainsf32};
use std::{thread::sleep, time::Duration};

fn main() {
    println!();
    println!("Temperature Control Simulation");
    println!("==============================");
    println!("Target temperature:   {:.1}°C", TARGET_TEMPERATURE);
    println!("Starting temperature: {:.1}°C", STARTING_TEMPERATURE);
    println!("Ambient temperature:  {:.1}°C", AMBIENT_TEMPERATURE);
    println!();

    let mut pid_controller = PidControllerf32::new()
        .with_gains(PidGainsf32::new().with_kp(2.0).with_ki(0.1).with_kd(0.05))
        .with_integral_limits(50.0, -50.0);

    pid_controller.set_setpoint(TARGET_TEMPERATURE);
    pid_controller.set_initial_measurement(STARTING_TEMPERATURE);

    let dt = DT;
    let mut temperature = STARTING_TEMPERATURE;
    let thermal_mass = 5.0; // simulated thermal mass (higher = slower changes)

    println!("Time(s) | Temperature(°C) | Control Signal(%) | HVAC Mode");
    println!("--------|-----------------|-------------------|----------");

    let mut t = 0.0;
    let mut loop_counter = 0;
    loop {
        // Calculate control signal using the current temperature
        let control_signal = pid_controller.update(temperature, dt);

        let hvac_mode = if control_signal > 1.0 {
            "Heating"
        } else if control_signal < -1.0 {
            "Cooling"
        } else {
            "Idle"
        };

        // Natural heat loss/gain
        let ambient_effect = (AMBIENT_TEMPERATURE - temperature) * 0.01;
        temperature += ambient_effect;
        temperature += control_signal * HVAC_POWER / thermal_mass;
        println!(
            "{:6.1} | {:15.2} | {:17.1} | {}",
            t, temperature, control_signal, hvac_mode
        );

        // Simulate a disturbance (window opens)
        if loop_counter == DISTURBANCE_ITERATION {
            println!(">>> Window opened! Temperature dropped 2°C");
            temperature -= 2.0;
        }

        t += dt;
        if t > SIMULATION_DURATION_SECONDS {
            break;
        }
        loop_counter += 1;
        sleep(Duration::from_secs_f32(0.02));
    }
}

const TARGET_TEMPERATURE: f32 = 22.0;
const AMBIENT_TEMPERATURE: f32 = 15.0;
const STARTING_TEMPERATURE: f32 = 10.0;
const HVAC_POWER: f32 = 0.5;
const DT: f32 = 0.1;
const SIMULATION_DURATION_SECONDS: f32 = 20.0;
const DISTURBANCE_ITERATION: u32 = 100;
