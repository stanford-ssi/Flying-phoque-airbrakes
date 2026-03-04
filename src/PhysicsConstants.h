#pragma once

// Dry air specific gas constant
const float Rd = 287.05f;  // J/(kg·K)

// Standard gravity
const float g0 = 9.80665f;  // m/s^2

// Celsius to Kelvin offset
const float CELSIUS_TO_KELVIN = 273.15f;

// Simulation reference conditions (ISA sea level)
const float SIM_PRESSURE_REF = 1013.25f;      // hPa
const float SIM_TEMPERATURE_REF = 293.15f;     // K (~20°C)
