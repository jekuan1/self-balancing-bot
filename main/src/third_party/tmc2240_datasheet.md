[cite_start]The TMC2240 is a high-performance stepper driver supporting **SPI** and **Single-Wire UART** for configuration[cite: 247, 467]. Implementation requires managing precise timing and specific register bits for advanced features like **StealthChop2**, **SpreadCycle**, and **StallGuard**.

---

## 1. Interface Configuration & Control
[cite_start]The hardware interface is determined by the **UART_EN** pin[cite: 468].

### SPI Interface (UART_EN = 0)
* [cite_start]**Datagram:** Always 40 bits (8-bit address/command + 32-bit data)[cite: 471].
* [cite_start]**Speed:** Maximum SCK frequency is **10MHz**[cite: 394].
* [cite_start]**Timing Requirements:** * **tCC (CSN to SCK):** Must be valid before/after SCK change[cite: 394].
    * [cite_start]**tCSH (CSN High Time):** Minimum $4 \times T_{CLK}$[cite: 394].
    * [cite_start]**Data Setup/Hold:** 10ns minimum[cite: 394].

### UART Single-Wire Interface (UART_EN = 1)
* [cite_start]**Baud Rate:** Adaptive; no manual baud rate configuration required[cite: 472].
* [cite_start]**Addressing:** Supports up to 255 nodes using **AD0, AD1, and AD2** pins as address inputs[cite: 420, 421, 422].
* [cite_start]**Integrity:** Uses **CRC checking** for all datagrams[cite: 467].

---

## 2. Core Motion & Current Control
### Step/Direction (S/D) Interface
* [cite_start]**Microstepping:** Configurable up to **256 microsteps** per fullstep[cite: 477].
* [cite_start]**`dedge` bit:** If set to 1, both rising and falling edges of the STEP signal trigger a step, halving the required toggle rate[cite: 474].
* [cite_start]**Timing:** Minimum STEP high/low times are $t_{CLK} + 20ns$[cite: 395].

### Current Scaling
* [cite_start]**`IFS` (Full-Scale Current):** Set via an external resistor on the **IREF** pin (range 12kΩ to 60kΩ)[cite: 255, 389].
* **Global Current:** Regulated via the internal DAC; [cite_start]$I_{RMS}$ max is 2.1A[cite: 253, 301].
* [cite_start]**Standstill:** Configure automatic current reduction to minimize heat (e.g., 50% current reduces power to ~25%)[cite: 480, 481].

---

## 3. Implementing Advanced Features
[cite_start]The TMC2240 uses proprietary algorithms to optimize motor performance[cite: 463].


### StealthChop2 & SpreadCycle
* **StealthChop2:** Noiseless operation. [cite_start]Requires **Automatic Tuning (AT#1 and AT#2)** to calibrate `PWM_GRAD` and `PWM_OFS`[cite: 278, 351].
* **SpreadCycle:** High-dynamic current control. [cite_start]Use the `TPWMTHRS` register to set the velocity threshold for switching between StealthChop2 and SpreadCycle[cite: 351].

### StallGuard & CoolStep
* [cite_start]**StallGuard2/4:** Sensorless load detection[cite: 281]. [cite_start]Monitor the load value to detect stalls or perform sensorless homing[cite: 333].
* [cite_start]**CoolStep:** Uses StallGuard feedback to adaptively scale motor current[cite: 282].
    * [cite_start]**Implementation:** Requires tuning `SEM_MIN`, `SEM_MAX`, and `SEUP/SEDN` bits for current increment/decrement response times[cite: 333].

---

## 4. Diagnostics and Protection
[cite_start]Code should actively monitor the **DIAG0** and **DIAG1** pins/registers for[cite: 258, 432, 434]:
* [cite_start]**Overcurrent (OCP):** Thresholds depend on full-scale current settings[cite: 357].
* [cite_start]**Thermal Shutdown:** Occurs at **165°C** (20°C hysteresis)[cite: 388].
* [cite_start]**UVLO:** Driver disables if $V_S$ falls below ~3.9V[cite: 388].

### Register Management during Sleep
* [cite_start]**SLEEPN Pin:** Pull low to enter sleep mode (<18µA consumption)[cite: 377, 436].
* [cite_start]**CRITICAL:** **Register content is lost** during sleep[cite: 439]. [cite_start]You must reconfigure the IC after wake-up (wait ~2.5ms for `TWAKE`)[cite: 390, 439].

To implement TMC2240 features into code, you must manage its registers via SPI or UART and handle specific timing sequences for its advanced "Trinamic" features.

### 1. Register Access & Interface Timing
The TMC2240 uses a **40-bit datagram** for SPI and a **variable-length datagram with CRC** for UART.

* **SPI Timing:**
    * **Clock Frequency ($f_{CLK}$):** Supports up to 10MHz. 
    * **CSN to SCK ($t_{CC}$):** Minimum 20ns. Ensure Chip Select is low before the first clock edge.
    * **Clock High/Low ($t_{CH}/t_{CL}$):** Minimum 10ns.
* **UART Configuration:**
    * **Single-Wire:** Data is sent and received on the same wire. Implement a "read-back" ignore in your code during transmission.
    * **Baud Rate:** Automatic detection based on the sync byte `0x05`.
* **Addressing:**
    * Registers are accessed via an **8-bit address** (MSB is R/W bit: $1$ for Write, $0$ for Read).
    * Example: To write to `CHOPCONF`, use address `0xEC` (0x6C | 0x80).

---

### 2. Implementing StealthChop2 (Silent Mode)
StealthChop2 requires an initialization and tuning sequence to function correctly. 


* **Key Registers:** `PWMCONF`, `PWM_SCALE`, `PWM_AUTO`.
* **Implementation Steps:**
    1.  **Enable:** Set `en_pwm_mode = 1` in `GCONF`.
    2.  **Initial Tuning (AT#1):** Power on the motor and let it stand still for ~130ms. The driver measures the coil resistance.
    3.  **Dynamic Tuning (AT#2):** Move the motor at a medium velocity (avoid resonance frequencies). The driver adjusts `PWM_GRAD_AUTO` and `PWM_OFS_AUTO`.
    4.  **Persistent Storage:** For faster startups, read the tuned `PWM_GRAD_AUTO` and `PWM_OFS_AUTO` values and write them to `PWM_GRAD` and `PWM_OFS` on the next boot.

---

### 3. Implementing SpreadCycle (High Dynamic Mode)
SpreadCycle is a cycle-by-cycle current control that is better for high-speed or high-torque requirements.

* **Key Register:** `CHOPCONF`.
* **Relevant Bits to Access:**
    * `TOFF` (Bits 0-3): Sets the off-time (the "chopper frequency"). A value of `0` disables the driver. Recommended: `3` to `5`.
    * `HSTRT` / `HEND` (Hysteresis): Controls the current ripple. Use `HSTRT=4` and `HEND=1` as a starting point.
    * `TBL` (Blanking Time): Prevents false triggering from switching spikes. Set to `1` or `2` (24 or 36 clock cycles).

---

### 4. Switching Modes (Hybrid Mode)
You can automatically switch between StealthChop2 (quiet at low speeds) and SpreadCycle (powerful at high speeds) using the **Velocity Threshold**.

* **Logic:**
    * `V < TPWMTHRS`: Driver uses StealthChop2.
    * `V > TPWMTHRS`: Driver uses SpreadCycle.
* **Code Implementation:** Calculate the `TSTEP` value (time between microsteps) corresponding to your desired switchover speed and write it to the `TPWMTHRS` register.

---

### 5. StallGuard4 & CoolStep (Sensorless Load)
StallGuard4 provides a "load value" (`SG_RESULT`) that indicates motor stress.

* **Sensorless Homing:**
    1.  Monitor `SG_RESULT`. As the motor hits a physical stop, `SG_RESULT` drops toward zero.
    2.  Set the `SGT` (StallGuard Threshold) in `SG4_THRS`. 
    3.  Configure `DIAG0` to trigger an interrupt when a stall is detected (`diag0_stall = 1` in `GCONF`).
* **CoolStep Implementation:**
    * Set `SEMIN` and `SEMAX` in the `COOLCONF` register.
    * The driver will automatically decrease motor current when `SG_RESULT > SEMAX` and increase it when `SG_RESULT < SEMIN`.

### 6. Critical Timings for Code Loops
* **Watchdog:** If using the internal watchdog, ensure your SPI/UART "heartbeat" writes to the `WSEC` register every <1 second.
* **Standstill:** Use `IHOLD` and `IRUN` registers to automatically reduce current when the `STEP` pin is inactive. The delay before reduction is set by `IHOLDDELAY`.

To implement the advanced features from pages 41-60 of the TMC2240 datasheet, your code must handle the fine-tuning of PWM regulators, manage high-velocity transitions, and potentially customize the microstep look-up table (LUT).

### 1. Advanced StealthChop2 Tuning (`PWMCONF`)
While StealthChop2 is "plug-and-play," precision applications require accessing the internal regulation loop coefficients to ensure stability during rapid acceleration.

* **`PWM_REG` (Bits 17-20):** This is the P-coefficient of the PWM amplitude regulator. 
    * **Implementation:** If the motor "hunts" or vibrates during velocity changes, increase this value (range 1–15). High values (e.g., >8) allow the driver to reach the target current faster but may cause overshoot.
* **`PWM_LIM` (Bits 24-27):** Controls the "current jerk" when switching from SpreadCycle back to StealthChop2. 
    * **Implementation:** If you hear a "clunk" when the motor slows down past the `TPWMTHRS` threshold, reduce this value to smooth the transition.
* **`FREEWHEEL` (Bits 20-21):** Defines standstill behavior when `IHOLD=0`.
    * **0:** Normal operation (current reduced to 0).
    * **1:** Freewheeling (coils disconnected; motor can be moved easily).
    * **2:** Passive Braking (LS FETs on; motor is hard to move).

### 2. Velocity-Based Switching (`TPWMTHRS`)
To implement "Hybrid Mode" (StealthChop for silence, SpreadCycle for torque), you must calculate the `TSTEP` value.

* **Formula:** $TSTEP = f_{CLK} / f_{STEP}$
* **Logic:**
    ```c
    // Example: Switch to SpreadCycle above 500 RPM
    // 1. Calculate TSTEP at the limit velocity
    // 2. Write to TPWMTHRS register
    writeRegister(TMC2240_TPWMTHRS, calculated_tstep_value);
    ```
* **Note:** If `TSTEP` < `TPWMTHRS`, StealthChop2 is disabled and SpreadCycle takes over.

### 3. Microstep Table (LUT) Customization
The TMC2240 uses a programmable Look-Up Table (LUT) to translate the 10-bit microstep counter (`MSCNT`) into coil currents.



* **`MSLUT[0...7]`:** These registers store the actual wave shape. By default, they are a sine wave.
* **Non-Standard Motors:** If using a motor with non-sinusoidal torque characteristics (e.g., some thin disk-magnet motors), you can write custom values to these registers to improve smoothness.
* **`OFFSET_SIN90`:** Used to compensate for motor manufacturing tolerances.
    * **Implementation:** Use `SG4_IND` (individual StallGuard values for Coil A and B) to detect phase asymmetry. Adjust `OFFSET_SIN90` until the load values for both coils are equal.

### 4. Direct Current Access for Diagnostics
For real-time monitoring or custom control loops, you can read the instantaneous coil currents.
* **`CUR_A` and `CUR_B` (Registers 0x66, 0x67):**
    * These provide the 9-bit signed current values currently being applied to the coils.
    * **Timing:** These registers update every microstep. Do not poll these faster than your microstep rate to avoid redundant SPI/UART traffic.

### 5. Implementation Summary Table

| Feature | Key Register | Bit/Field | Action |
| :--- | :--- | :--- | :--- |
| **Regulation Speed** | `PWMCONF` | `PWM_REG` | Increase for faster current adaptation. |
| **Switch Smoothness**| `PWMCONF` | `PWM_LIM` | Decrease to stop "clicking" during mode swaps. |
| **Phase Tuning** | `MSLUT0` | `OFFSET_SIN90` | Tune using `SG4_IND` to reduce vibration. |
| **Safety** | `GCONF` | `drv_enn` | Soft-disable the power stage via code without cutting $V_S$. |

**Critical Code Sequence for Init:**
1. Set `GCONF` (interface and global settings).
2. Set `CHOPCONF` (basic motor parameters/SpreadCycle).
3. Set `PWMCONF` (StealthChop2 settings).
4. Set `IHOLD_IRUN` (current levels).
5. Set `TPWMTHRS` (transition velocity).
6. **Last Step:** Enable the driver stage by pulling the `ENN` pin low or clearing the `drv_enn` bit.

To implement the features detailed in the final segment of the datasheet (pages 61–80), your code must handle global configuration flags, emergency stop functions, and "Direct Mode" for specialized control.

### 1. Global Configuration (`GCONF` Register)
The `GCONF` register (Address `0x00`) is the most important register for defining the driver's behavior.

* **`direct_mode` (Bit 16):** * **Implementation:** Set this bit to `1` to bypass the internal microstep sequencer. You then write signed 9-bit current values directly to the `DIRECT_MODE` register (`0x2D`).
    * **Timing:** Use this for high-speed custom control loops or when your MCU is handling the motion profile entirely. Note that StealthChop2 auto-regulation is limited in this mode.
* **`stop_enable` (Bit 15):** * **Logic:** When enabled, a high signal on the `ENCA` pin acts as a "Hard Stop." The sequencer stops executing steps immediately, and the motor enters a standstill state. This is critical for safety interrupts.
* **`diag0_error` & `diag0_otpw` (Bits 8–9):**
    * **Implementation:** Route these to an MCU interrupt pin. `diag0_error` triggers on any critical driver fault (overcurrent, thermal shutdown), while `diag0_otpw` provides an early warning when the chip reaches its pre-warning temperature.

### 2. Output Driver Configuration (`DRV_CONF`)
This register controls the physical characteristics of the H-Bridge MOSFETs.

* **`slope_control` (Bits 8–11):** * **Usage:** Adjusts the switching speed (Slew Rate) of the MOSFETs. 
    * **Code Implementation:** If you experience high Electromagnetic Interference (EMI), reduce the slope. If the driver is running too hot at high PWM frequencies, increase the slope to reduce switching losses.
* **`bbm_time` (Break-Before-Make):**
    * **Safety:** This adds a "dead time" (in clock cycles) to prevent "shoot-through" (where high-side and low-side FETs are on simultaneously). The default is usually safe, but must be checked if using non-standard supply voltages.

### 3. Integrated Current Sensing (ICS)
Unlike older TMC drivers, the TMC2240 features integrated sensing, meaning you don't need external sense resistors.

* **`GLOBAL_SCALER` (Register `0x71`):**
    * **Implementation:** This provides a 0–255 scaling factor for the motor current. 
    * **Math:** $I_{RMS} = (GLOBAL\_SCALER / 256) \times I_{FS}$ (where $I_{FS}$ is the full-scale current set by the `IREF` resistor). 
    * **Use Case:** Use this for global torque adjustments without changing the `IRUN`/`IHOLD` settings.

### 4. Advanced Diagnostics & Status Polling
In your main execution loop, you should poll the `DRV_STATUS` register (`0x6F`) to monitor the health of the system.

* **Bits to Monitor:**
    * `s2vsa` / `s2vsb`: Short to supply (indicates a wiring fault).
    * `s2ga` / `s2gb`: Short to ground.
    * `ot`: Overtemperature shutdown (165°C).
    * `otpw`: Overtemperature pre-warning (143°C).
    * `stst`: Standstill indicator. Use this to verify the motor has actually stopped before changing parameters like `CHOPCONF`.

### 5. SPI/UART Communication "Heartbeat"
If you enable the **Watchdog (`WSEC`)**, you must implement a periodic write in your code.

* **Requirement:** If `WSEC` is non-zero, the driver will disable the power stage if it doesn't receive a valid SPI/UART datagram within the specified time.
* **Code Logic:**
    ```c
    void loop() {
        // ... motion logic ...
        if (millis() - last_heartbeat > 500) {
            readRegister(TMC2240_GSTAT); // Reading a register resets the watchdog
            last_heartbeat = millis();
        }
    }
    ```

### Summary of Register Bit-Access for Implementation

| Bitfield | Register | Bit(s) | Implementation Action |
| :--- | :--- | :--- | :--- |
| `en_pwm_mode` | `GCONF` | 0 | Set to 1 to enable StealthChop2. |
| `diag1_steps_skipped`| `GCONF` | 12 | Route to DIAG1 to detect lost steps via StallGuard. |
| `multistep_filt` | `CHOPCONF`| 28 | Enable to filter STEP pulses and reduce jitter. |
| `reset` | `GSTAT` | 0 | Read to check if a Power-On Reset occurred (re-init required). |

By managing these registers, you can transition the TMC2240 from a basic "Step/Dir" driver into a fully monitored, intelligent motion controller.

To implement the features from the final technical section (pages 81–100), your code needs to focus on **interrupt-driven diagnostics**, **SpreadCycle fine-tuning**, and the **microstep look-up table (LUT)** structure.

### 1. Interrupt & Diagnostic Configuration (`GCONF` & `IOIN`)
The registers in this section define how the driver communicates errors and status via the physical `DIAG0` and `DIAG1` pins.

* **Configuring `DIAG1` for Feedback:**
    * **`diag1_index` (Bit 9):** Set to `1` to output a pulse when the microstep counter is at position 0. This is useful for synchronizing multiple drivers or high-precision homing.
    * **`diag1_stall` (Bit 8):** Set to `1` to trigger the pin when a stall is detected. 
    * **Logic:** Your MCU should attach an ISR (Interrupt Service Routine) to the `DIAG1` pin to stop motion instantly when the stall flag hits, preventing mechanical damage.
* **Reading `IOIN` (Address 0x06):**
    * This register allows your code to "see" the state of the physical pins (`DIAG0`, `DIAG1`, `STEP`, `DIR`, `ENN`). Use this for sanity checks during initialization.

### 2. SpreadCycle Fine-Tuning (`CHOPCONF`)
The `CHOPCONF` register (0x6C) is the heart of the constant-off-time chopper. Implementing this requires setting specific bitfields to match your motor's inductance:

* **`TOFF` (Bits 0–3):** This is the main chopper switch. 
    * **Implementation:** Set this to a non-zero value to enable the driver. A value of `3` to `5` is typically ideal for 24V/36V systems.
* **`HSTRT` (Bits 4–6) & `HEND` (Bits 7–10):**
    * These control the hysteresis of the current ripple. 
    * **Code Strategy:** For high-speed torque, increase `HSTRT`. For lower noise in SpreadCycle, keep the sum of `HSTRT + HEND` small.
* **`TBL` (Blanking Time, Bits 15–16):**
    * Crucial for code stability: Set this to `1` (24 clocks) or `2` (36 clocks) to ignore the "ringing" that occurs immediately after a MOSFET switches. Without this, the driver may experience false overcurrent trips.

### 3. Customizing the Microstep Table (`MSLUT`)
The TMC2240 allows you to redefine the sine wave shape to compensate for motor "cogging" or non-linear torque.

* **`MSLUT[0...7]` (Addresses 0x70–0x77):** * These 32-bit registers store the "slopes" between microstep positions.
    * **Bit Encoding:** Each bit defines whether the next microstep is a +0, +1, +2, or -1 change in current relative to the previous one.
* **`MSLUTSEL` (Address 0x78):**
    * This register divides the 256-microstep wave into four segments. You can assign different "lookup widths" to each segment to give higher resolution at the peak of the sine wave and lower resolution at the zero-crossing.

### 4. Global Current Scaling (`GLOBAL_SCALER`)
Instead of recalculating all `IRUN` and `IHOLD` values when you want to change power, use the `GLOBAL_SCALER` (Address 0x71).

* **Implementation:** * Write a value from `0` to `255`. 
    * Value `256` (coded as 0) represents 100% of the current set by the `IREF` resistor.
    * **Utility:** Use this for a "battery saver" mode in code or to temporarily boost torque during a difficult move.

### 5. Summary of Bitfields for Firmware Constants
When writing your header file or driver class, ensure these addresses and bitmasks are mapped:

| Function | Register | Bitmask | Recommended Start Value |
| :--- | :--- | :--- | :--- |
| **Enable Chopper** | `CHOPCONF` | `0xF` (TOFF) | `0x3` |
| **Blanking Time** | `CHOPCONF` | `0x18000` (TBL) | `0x2` (36 clocks) |
| **Stall to DIAG1** | `GCONF` | `0x100` | `1` |
| **Index to DIAG1** | `GCONF` | `0x200` | `1` (for homing sync) |
| **Software Reset** | `GSTAT` | `0x1` | Read only (1 = reset occurred) |

**Final Implementation Tip:** Always read the `GSTAT` (Global Status) register at the start of your main loop. If the `reset` bit is `1`, it means the driver experienced a power brown-out and has lost its register settings; your code must trigger a full re-initialization sequence.

The final sections of the datasheet (pages 101–127) provide the granular register map and a "Quick Configuration Guide." To implement these into high-density, production-ready code, you should focus on the bit-level formatting of the microstep LUT, the CoolStep adaptive scaling logic, and the essential power-up sequence.

### 1. The Microstep LUT Structure (`MSLUT`)
Implementing custom wave shapes requires understanding the differential bit-coding used in registers `0x60` to `0x67`.

* **Differential Encoding:** Each bit in the `MSLUT` registers represents a change in the current value for the next microstep.
    * The meaning of a `0` or `1` bit depends on the `W` bits in `MSLUTSEL` (0x68). 
    * **Example Code Logic:** For a standard sine wave, you define four segments in `MSLUTSEL`. In segment 0, a bit value of `1` typically represents a step of `+1` in current, while `0` represents `+0`.
* **Initialization:** You must set `START_SIN` and `START_SIN90` (Register 0x69). These provide the absolute starting current values because the LUT only stores the *differences* between steps.

### 2. CoolStep Adaptive Current Scaling (`COOLCONF`)
CoolStep allows the driver to automatically reduce current during light loads to save power and heat.

* **`SEMIN` (Bits 0–3):** If the `SG_RESULT` (StallGuard value) falls below this threshold, the driver increases current.
* **`SEMAX` (Bits 8–11):** If `SG_RESULT` stays above (`SEMIN + SEMAX + 1`) * 32, the driver decreases current.
* **Implementation Note:** Set `SEMIN` to roughly 1/4 to 1/2 of your typical StallGuard reading during a "normal" move. Setting `SEMIN = 0` disables CoolStep entirely.

### 3. Quick Configuration "Boilerplate"
Based on the datasheet's guide, your initialization function should follow this exact register-write order to ensure the analog-to-digital converters and FET drivers stabilize correctly:

1.  **Global Reset Check:** Read `GSTAT` (0x01). If the `reset` bit is set, clear it by writing 1 and proceed.
2.  **Current Scaling:** * Set `GLOBAL_SCALER` (0x71) to calibrate for your specific motor RMS current.
    * Set `IHOLD_IRUN` (0x10). Recommendation: Set `IHOLD` to ~50% of `IRUN` with a `IHOLDDELAY` of 1-5 to prevent motor drop when stopping.
3.  **Chopper Configuration:** * Write to `CHOPCONF` (0x6C). Set `TOFF` to a value between 2 and 5 to enable the H-Bridges.
    * Set `TBL` (Blanking time) to 2 (36 clock cycles) for standard 24V setups.
4.  **Mode Thresholds:** * Set `TPWMTHRS` (0x13) to the velocity where you want to transition from StealthChop2 to SpreadCycle.
5.  **Enable Driver:** Pull the physical `ENN` pin low or clear the `drv_enn` bit in `GCONF`.

### 4. Advanced "Direct Mode" (Register 0x2D)
If your application requires non-standard motion (like a haptic feedback device or a linear actuator with variable force), you can use `direct_mode`.

* **Access:** Set `direct_mode = 1` in `GCONF`.
* **Control:** Write to `DIRECT_MODE` (0x2D).
    * **Bits 8:0:** Signed current for Coil A.
    * **Bits 24:16:** Signed current for Coil B.
* **Timing:** The driver updates the current at the moment the SPI/UART write completes. Ensure your MCU loop has a stable frequency (e.g., 1kHz to 10kHz) for smooth movement.

### 5. Critical Troubleshooting Bits
| Register | Bitfield | Purpose in Code |
| :--- | :--- | :--- |
| `DRV_STATUS` (0x6F) | `otpw` | Interrupt trigger: Slow down motor to prevent total thermal shutdown. |
| `DRV_STATUS` (0x6F) | `s2vsa` / `s2ga` | Safety check: Shutdown system if short to supply or ground is detected. |
| `IOIN` (0x06) | `VERSION` | Firmware check: Verify the IC is a TMC2240 (Value should be `0x40`). |



### Implementation Timing Requirements
* **Wait after Power-up:** Wait at least **2.5ms** (`tWAKE`) after $V_{CC}$ is stable before sending the first SPI/UART command.
* **Register Persistence:** All registers are volatile. If the `SLEEPN` pin is toggled or power is lost, your software must re-run the entire initialization sequence.