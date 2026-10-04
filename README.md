# `pidsk-controller` Rust Crate<br>[![Crates.io](https://img.shields.io/crates/v/pidsk-controller.svg)](https://crates.io/crates/pidsk-controller) [![Documentation](https://docs.rs/pidsk-controller/badge.svg)](https://docs.rs/pidsk-controller) [![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

`pidsk-controller` is a PID controller with additional features useful for real-world control applications,
including: variable loop timing, integral anti-windup, user-controlled derivative filtering, and runtime gain changes.

The controller is available for both `f32` and `f64`.

This crate is `no_std`, `no alloc`, and the Minimum Supported Rust Version (MSRV) is `Rust 1.89`.

## Features

- **P, I, and D control** using independent `kp`, `ki`, and `kd` gains.
- **Setpoint feed-forward** (openloop control) using the `ks` gain.
- **Setpoint derivative kick** using the `kk` gain. Allows *intentional* derivative kick.
- **Calculates derivative on measurement**, avoids *unintentional* derivative kick when the setpoint changes.
- **Variable loop timing** by supplying `dt` to `update()`.
- **Integral anti-windup** using integral limits, output saturation, or both.
- **User-controlled D-term filtering** through `update_delta()`.
- **Dynamic PID control**, including runtime gain changes with no output jump.
- **Runtime integration control**, allowing the I-term to be switched on or off.
- **Custom I-term error** through `update_delta_iterm()`, allowing I-term relaxation.
- **Access to individual PID terms** for logging, tuning, telemetry, and testing.
- **Optimized update functions** for common **P**, **PI**, **PD**, and other configurations.
- **`f32` and `f64` variants**.
- **Optional `serde` support**: enable the `serde` feature to add `Serialize` and `Deserialize`.

For cases where the full functionality of a `PidskController` is required, partial forms are provided:

- `PController` - a pure P-controller.
- `PdController` - a PD-controller.
- `PidController` - a traditional PID controller, ie a `PidskController` without the S-term and K-term.

These forms are particularly useful for implementing Dual-Ring Cascaded PID Loops, where, because of the cascade,
some of the gains are redundant.

## Controller formulation

The controller calculates its output as:

```text
output =
    kp * error
  + ki * error_integral
  - kd * measurement_derivative
  + ks * setpoint
  + kk * setpoint_derivative
```

Or, expressed mathematically,:

![PID equation](https://raw.githubusercontent.com/martinbudden/crate-pid-controller/main/docs/equations/pidsk.svg)

Where:

- C(t) = control output, the output to the actuator.
- P(t) = process variable, the measured value.
- e(t) = error = S(t) - P(t)
- S(t) = set point, the desired target for the process variable.

`kp`/`ki`/`kd`/`ks`/`kk` can be changed during operation and can therefore be a function
of time.

The first three terms are the conventional PID terms:

- `P` - proportional error
- `I` - accumulated integral error
- `D` - derivative of the measured value

The additional terms are:

- `S` - setpoint (aka open-loop or feed-forward)
- `K` - setpoint derivative (kick)

Setting `ks` and `kk` to zero gives a conventional PID controller.

Setting `kp`, `ki`, `kd`, and `kk` to zero gives pure open-loop control.

The controller uses the **independent PID** notation, where the gains are `kp`, `ki`, and `kd`.

(In the dependent/ISA notation, the terms `kc`, `tau_i`, and `tau_d` are used,
where `kc = kp`, `tau_i = kp / ki` and `tau_d = kd / kp`).

## Quick start

A basic PID controller can be created by specifying its gains and then calling `update()` from the control loop:

```rust
use pidsk_controller::PidskControllerf32;

let mut pid_controller = PidskControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05);

let dt = 0.01; // seconds

// Set the desired value.
pid_controller.set_setpoint(10.0);

// Simulated measurement.
let measurement = 8.5;

// Call update() from the control loop.
let command = pid_controller.update(measurement, dt);

// Apply command to the system being controlled.
// ...
```

`dt` is supplied to each call, so the controller does not require a perfectly periodic control loop.
Variations in loop timing (jitter) are handled automatically.

## `f32` and `f64`

The crate provides `f32` and `f64` variants of the main controller types:

1. `PidskControllerf32`, `PidskControllerf64`
2. `PidskGainsf32`, `PidskGainsf64`
3. `PidskErrorsf32`, `PidskErrorsf64`
4. `PidLimitsf32`, `PidLimitsf64`

These are type aliases for the corresponding generic types.
So `PidskControllerf32` is an alias for `PidskController<f32>`, but that is transparent to the user.

## Setpoint control

The S-term provides a component of the output based directly on the setpoint rather than on the control error.

This is known as open loop control, or feed-forward control.

It is useful when part of the required control output can be predicted from the desired operating point,
for example setting the control surfaces on a fixed-wing aircraft, or setting the power of a heater based on the current temperature.

## Setpoint derivative and derivative kick

The D-term is calculated from the **measurement**, rather than from the error.
This avoids *unintentional* derivative kick when the setpoint changes.

If setpoint derivative kick is desired, it can be added explicitly using the K-term.

Flight controllers (such as Betaflight) use derivative kick to make the aircraft more responsive to control-stick inputs.
(Note that Betaflight uses the term "feed-forward" for setpoint derivative kick.)

## Integral anti-windup

Integral windup can occur when the controller output is unable to reach the value requested by the PID calculation,
for example when an actuator has reached its limit, or a heater has reached its maximum power.

`pidsk-controller` provides two ways of limiting the integral term:

1. **Integral limits** - constrains the accumulated integral value.
2. **Output saturation** - prevents integration when the controller output has saturated.

For example:

```rust
use pidsk_controller::PidskControllerf32;

let mut pid_controller = PidskControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05)
   .with_integral_limits(50.0, -50.0);

// or with asymmetric limits:

let mut pid_controller = PidskControllerf32::new()
   .with_kp(2.0)
   .with_ki(0.1)
   .with_kd(0.05)
   .with_integral_limits(50.0, 0.0);

// or with output saturation

let mut pid_controller = PidskControllerf32::new()
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
use pidsk_controller::PidskControllerf32;
use signal_filters::{Pt1Filterf32, UpdateFilter};

let mut pid_controller = PidskControllerf32::new();

// Create a filter for the D-term
let mut dterm_filter = Pt1Filterf32::new().with_k(0.9);

// 1000 Hz update rate
let dt = 0.001;

// Simulated measurement
let measurement = 1.2;

let measurement_delta = measurement - pid_controller.previous_measurement();

// Filter the D-term before updating the PID controller.
let filtered_measurement_delta = measurement_delta.filter_using(&mut dterm_filter);

let command = pid_controller.update_delta(
    measurement,
    filtered_measurement_delta,
    dt,
);
```

The above code shows the intermediate steps for clarity. It can be written more compactly:

```rust
use pidsk_controller::PidskControllerf32;
use signal_filters::{Pt1Filterf32, UpdateFilter};

let mut pid_controller = PidskControllerf32::new();
let mut dterm_filter = Pt1Filterf32::new().with_k(0.9);
let dt = 0.001;

// Simulated measurement
let measurement = 1.2;

let command = pid_controller.update_delta(
    measurement,
    (measurement - pid_controller.previous_measurement()).filter_using(&mut dterm_filter),
    dt,
);
```

```text

               ┌────────────────────┐
               │  pidsk-controller  │
               │                    │  command  ┌────────────┐
Setpoint ─────►│ P + I + D + S + K  │──────────►│ Controlled │
(desired       │                    │           │   System   │
 value)        └──┬─────────┬───────┘           └─────┬──────┘
                  ▲         ▲ filtered                ▼
                  │         │ measurement delta       │ measurement
                  │    ┌────┴───┐                     │
                  │    │ D-term │                     │
                  │    │ filter │                     │
                  │    └────▲───┘                     │
                  │         │ measurement delta       │
                  │    ┌────┴──────┐                  │
                  └─── │measurement│──────────────────┘
                       └───────────┘
```

## Customizing the I-term

`update_delta_iterm()` extends `update_delta()` with an `iterm_error` parameter:

```text
let output = pid_controller.update_delta_iterm(
    measurement,
    measurement_delta,
    iterm_error,
    dt,
);
```

This allows  **I-term relaxation**, where integration is reduced or disabled under particular operating conditions,
for example during aggressive manuevers by an aircraft.

## Switching off integration

Integration can also be switched off and on at runtime:

```rust
use pidsk_controller::PidskControllerf32;
let mut pid_controller = PidskControllerf32::new();

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
Or the gains may be changed during in-flight PID tuning.

## Choosing an update function

For most applications, use either the standard `update()`, or `update_delta()` if you want to filter the D-term.

```text
let output = pid_controller.update(measurement, dt);
```

Use the other update functions when your application needs a specific calculation or when avoiding unnecessary work is important.

| Function               | Purpose                                                                      |
| ---------------------- | ---------------------------------------------------------------------------- |
| `update()`             | General PID update                                                           |
| `update_delta()`       | Supply a measurement delta, allowing application-controlled D-term filtering |
| `update_delta_iterm()` | Supply both a measurement delta and custom I-term error                      |
| `update_p()`           | Optimized P-only controller                                                  |
| `update_sp()`          | Optimized P + setpoint feed-forward controller                               |
| `update_spi()`         | Optimized P + S + I controller                                               |
| `update_spd()`         | Optimized P + S + D controller                                               |
| `update_skpd()`        | Optimized form with integration disabled                                     |

The optimized functions are particularly useful in applications with high control-loop frequency
(ie in the kilohertz range) where avoiding unnecessary calculations is important.

### Examples

**Quadcopter with 8kHz gyro/PID loop**: use `update_delta_iterm` for full control on the Roll and Pitch axes,
use `update_sp` for optimized calculation on the Yaw axis.

**Self-balancing robot**: use `update_delta` on the Pitch axis and `update_sp` on the Yaw axis.

**Thermostat**: use `update` for simplicity. Use `ks`, `kp` and `ki` for some openloop control. `kd` and `kk` are set to zero.

**Aircraft altitude hold**: use a Dual-Ring cascaded PID Loop. The outer (altitude) loop is a pure P-controller, so used `update_p`.
The inner (vertical speed) loop is a PID-controller, so use `update`.

## Inspecting PID terms

The current PID terms can be obtained using:

```rust
use pidsk_controller::PidskControllerf32;
let mut pid_controller = PidskControllerf32::new();

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

See:

- [examples](examples/).
- [motor-mixers](https://crates.io/crates/motor-mixers): uses `PidControllerf32` to implement a Dynamic Idle Controller
  to ensure the motors do not go below their minimum allowed RPM.
- [Protoflight](https://crates.io/crates/protoflight): uses `PidskControllerf32` to stabilize the aircraft along its
  roll, pitch, and yaw axes. It uses `PControllerf32` and `PidControllerf32` as part of its altitude control
  Dual-Ring Cascaded PID Loop.

## API documentation

For the complete API, see the documentation on [docs.rs](https://docs.rs/pidsk-controller).

## Original implementation

`pidsk-controller` was originally implemented as a C++ library:

[Library-PID](https://github.com/martinbudden/Library-PID).

## License

Licensed under either of:

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
