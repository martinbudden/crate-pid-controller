# `pidsk-controller`

[![Crates.io](https://img.shields.io/crates/v/pidsk-controller.svg)](https://crates.io/crates/pidsk-controller)
[![Documentation](https://docs.rs/pidsk-controller/badge.svg)](https://docs.rs/pidsk-controller)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/martinbudden/pidsk-controller)

`pidsk-controller` provides a PID controller together with features useful for real-world control applications,
including variable loop timing, integral anti-windup, user-controlled derivative filtering, runtime gain changes, and access to individual PID terms.

The controller is available for both `f32` and `f64`.

## Features

- **P, I, and D control** using independent `kp`, `ki`, and `kd` gains.
- **Setpoint feed-forward** using the `ks` gain.
- **Setpoint derivative kick** using the `kk` gain. Allows intentional derivative kick.
- **Derivative on measurement**, avoids unintentional derivative kick when the setpoint changes.
- **Variable loop timing** by supplying `delta_t` to `update()`.
- **Integral anti-windup** using integral limits, output saturation, or both.
- **User-controlled D-term filtering** through `update_delta()`.
- **Dynamic PID control**, including runtime gain changes.
- **Runtime integration control**, allowing the I-term to be switched on or off.
- **Custom I-term error** through `update_delta_iterm()`, allowing I-term relaxation.
- **Access to individual PID terms** for tuning, telemetry, and testing.
- **Optimized update functions** for common **P**, **PI**, **PD**, and other configurations.
- **`f32` and `f64` variants**.

## Controller formulation

The controller calculates its output as:

```text
output =
    kp * error
  + ki * error_integral
  + kd * error_derivative
  + ks * setpoint
  + kk * setpoint_derivative
```

The first three terms are the conventional PID terms:

- `P` — proportional error
- `I` — accumulated integral error
- `D` — derivative of the measured value

The additional terms are:

- `S` — setpoint (aka open-loop or feed-forward)
- `K` — setpoint derivative kick

Setting `ks` and `kk` to zero gives a conventional PID controller.

Setting `kp`, `ki`, `kd`, and `kk` to zero gives pure open-loop control.

The controller uses the **independent PID** notation, where the gains are `kp`, `ki`, and `kd`.

(This differs from the dependent/ISA notation, where the terms `kc`, `tau_i`, and `tau_d` are used).

## Quick start

A basic PID controller can be created by specifying its gains and then calling `update()` from the control loop:

```rust
use pidsk_controller::PidControllerf32;

let mut pid_controller = PidControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05);

let delta_t = 0.01; // seconds

// Set the desired value.
pid_controller.set_setpoint(10.0);

// Simulated measurement.
let measurement = 8.5;

// Call update() from the control loop.
let control_signal = pid_controller.update(measurement, delta_t);

// Apply control_signal to the system being controlled.
// ...
```

`delta_t` is supplied to each call, so the controller does not require a perfectly periodic control loop.
Small variations in loop timing (jitter) are handled correctly.

## `f32` and `f64`

The crate provides `f32` and `f64` variants of the main controller types:

1. `PidControllerf32`, `PidControllerf64`
2. `PidGainsf32`, `PidGainsf64`
3. `PidLimitsf32`, `PidLimitsf64`
4. `PidErrorsf32`, `PidErrorsf64`

These are type aliases for the corresponding generic types.
So `PidControllerf32` is an alias for `PidController<f32>`, but that is transparent to the user.

## Setpoint control

The S-term provides a component of the output based directly on the setpoint rather than on the control error.

This is known as open loop control, or feed-forward control.

It is useful when part of the required control output can be predicted from the desired operating point,
for example setting the control surfaces on a fixed-wing aircraft.

## Setpoint derivative and derivative kick

The D-term is calculated from the **measurement**, rather than from the error.
This avoids *unintentional* derivative kick when the setpoint changes.

If setpoint derivative kick is desired, it can be added explicitly using the K-term.

(Note that Betaflight uses the term "feed-forward" for setpoint derivative kick.)

## Integral anti-windup

Integral windup can occur when the controller output is unable to reach the value requested by the PID calculation,
for example when an actuator has reached its limit.

`pidsk-controller` provides two ways of limiting the integral term:

1. **Integral limits** — constrain the accumulated integral value.
2. **Output saturation** — prevent integration when the controller output has saturated.

For example:

```rust
use pidsk_controller::PidControllerf32;

let mut pid_controller = PidControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05)
   .with_integral_limits(50.0, -50.0);

// or

let mut pid_controller = PidControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05)
   .with_output_saturation(0.8);
```

The Integral Limits and Output Saturation can be used together, if desired.

## Filtering the D-term

`pidsk-controller` deliberately does not impose a particular D-term filter.

Instead, `update_delta()` allows the application to provide its own filter:

```rust
use pidsk_controller::{PidControllerf32};
use signal_filters::{Pt1Filterf32, UpdateFilter};

let mut pid_controller = PidControllerf32::new();

// Create a filter for the D-term
let mut dterm_filter = Pt1Filterf32::with_k(0.9);

// 1000 Hz update rate
let delta_t = 0.001;

// Simulated measurement
let measurement = 1.2;

let measurement_delta = measurement - pid_controller.previous_measurement();

// Filter the D-term before updating the PID controller.
let filtered_measurement_delta = measurement_delta.filter_using(&mut dterm_filter);

let command = pid_controller.update_delta(
    measurement,
    filtered_measurement_delta,
    delta_t,
);
```

```text

                     ┌──────────────────────┐
                     │   pidsk-controller   │
                     │                      │  command  ┌────────────┐
Setpoint ───────────►│  P + I + D + S + K   │──────────►│ Controlled │
(desired value)      │                      │           │   System   │
                     └──┬─────────┬─────────┘           └─────┬──────┘
                        ▲         ▲ filtered                  ▼
                        │         │ measurement delta         │ measurement
                        │    ┌────┴───┐                       │
                        │    │ D-term │                       │
                        │    │ filter │                       │
                        │    └────▲───┘                       │
                        │         │ measurement delta         │
                        │    ┌────┴──────┐                    │
                        └─── │measurement│────────────────────┘
                             └───────────┘
```

## Customizing the I-term

`update_delta_iterm()` extends `update_delta()` with an `iterm_error` parameter:

```text
let output = pid_controller.update_delta_iterm(
    measurement,
    measurement_delta,
    iterm_error,
    delta_t,
);
```

This allows  **I-term relaxation**, where integration is reduced or disabled under particular operating conditions,
for example during aggressive manuevers by an aircraft.

## Switching off integration

Integration can also be switched off and on at runtime:

```rust
use pidsk_controller::{PidControllerf32};
let mut pid_controller = PidControllerf32::new();

pid_controller.switch_integration_off();

// ...

pid_controller.switch_integration_on();
```

This is useful to avoid integral wind-up when the system is temporarily constrained,
for example when a quadcopter is on the ground before take-off.

## Dynamic PID control

PID gains can be changed while the controller is running.

For example:

```text
pid_controller.update_gains(new_gains);
```

`update_gains()` is intended for changing gains without introducing an unwanted output bump.

This is useful for applications where PID gains are changed during operation.
For example a Flight Controller for a quadcopter may reduce the PID gains at high throttle values.

## Choosing an update function

For most applications, the standard `update()` function is suitable.

```text
let output = pid_controller.update(measurement, delta_t);
```

Use the other update functions when your application needs a specific calculation or when avoiding unnecessary work is important.

| Function               | Purpose                                                               |
| ---------------------- | --------------------------------------------------------------------- |
| `update()`             | General PID update                                                    |
| `update_delta()`       | Supply a measurement delta, allowing application-controlled filtering |
| `update_delta_iterm()` | Supply both a measurement delta and custom I-term error               |
| `update_p()`           | Optimized P-only controller                                           |
| `update_sp()`          | Optimized P + setpoint feed-forward controller                        |
| `update_spd()`         | Optimized P + S + D controller                                        |
| `update_skpd()`        | Optimized form with integration disabled                              |

The optimized functions are particularly useful in applications with high control-loop frequency
(ie in the kilohertz range) where avoiding unnecessary calculations is important.

## Inspecting PID terms

The current PID terms can be obtained using:

```rust
use pidsk_controller::{PidControllerf32};
let mut pid_controller = PidControllerf32::new();

let errors = pid_controller.error();

// or

let raw_errors = pid_controller.error_raw();
```

`error()` returns the P, I, D, S, and K terms after their respective gains have been applied.

`error_raw()` returns the unscaled errors.

These values can be useful for:

- PID tuning
- telemetry
- diagnostics
- control-loop testing
- observing which terms are contributing to the output

## Resetting the controller

The controller provides several reset operations.

`reset_integral()` clears only the accumulated integral:

`reset()` clears the controller's historical state so that it can be used for bumpless transfer:

`reset_all()` resets all state and is primarily intended for testing.

## Examples

See the [`examples`](examples/).

## API documentation

For the complete API, see the documentation on [docs.rs](https://docs.rs/pidsk-controller).

The main types are:

- [`PidController`](https://docs.rs/pidsk-controller/latest/pidsk_controller/struct.PidController.html)
- [`PidGains`](https://docs.rs/pidsk-controller/latest/pidsk_controller/struct.PidGains.html)
- [`PidLimits`](https://docs.rs/pidsk-controller/latest/pidsk_controller/struct.PidLimits.html)
- [`PidErrors`](https://docs.rs/pidsk-controller/latest/pidsk_controller/struct.PidErrors.html)

## Original implementation

`pidsk-controller` was originally implemented as a C++ library:

[Library-PID](https://github.com/martinbudden/Library-PID).

## License

Licensed under either of:

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
