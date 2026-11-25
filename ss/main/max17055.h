/**
 * @file    max17055.h
 * @brief   MAX17055 fuel gauge driver — public API
 *
 * Developer      : Eric Mulwa
 * Date           : 18 May 2026
 * Last Modified  : 18 May 2026
 *
 * Platform-agnostic driver for the Maxim MAX17055 1-Cell Fuel Gauge
 * using the ModelGauge m5 EZ algorithm.
 *
 * Design goals
 * ------------
 *  • Zero platform dependencies — all I/O goes through user-supplied
 *    read/write callbacks stored in @ref max17055_ctx_t.
 *  • All raw register functions return `int32_t` (0 = OK, <0 = error)
 *    so callers can chain error checks easily.
 *  • High-level helpers return physical values (mAh, %, mV, mA, °C, s)
 *    directly, accepting NaN / negative sentinel on error.
 *  • Initialization follows the Maxim-recommended EZ-Config sequence
 *    from the Software Implementation Guide (UG6365; Rev 2; 4/19).
 *
 * Usage (minimal example)
 * -----------------------
 * @code
 *   max17055_ctx_t ctx = {
 *       .write_reg  = my_i2c_write,
 *       .read_reg   = my_i2c_read,
 *       .handle     = NULL,
 *       .rsense_mohm = 10,           // 10 mΩ sense resistor
 *       .design_cap_mah = 3000,      // 3 000 mAh battery
 *       .charge_voltage_mv = 4200,   // 4.2 V charger
 *       .empty_voltage_mv = 3300,    // VEmpty = 3.3 V
 *       .ichg_term_ma = 150,         // charge termination at 150 mA
 *   };
 *   max17055_init(&ctx);
 *
 *   float soc = max17055_get_soc(&ctx);           // e.g. 83.5 %
 *   float cap = max17055_get_rep_cap_mah(&ctx);   // e.g. 2505.0 mAh
 *   float tte = max17055_get_tte_s(&ctx) / 3600;  // hours
 * @endcode
 */

#ifndef MAX17055_H
#define MAX17055_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "max17055_reg.h"

/* =========================================================================
 * PLATFORM I/O CALLBACKS
 * ========================================================================= */

/**
 * @brief  Write callback type.
 * @param  handle  Opaque pointer supplied by the user (e.g. I2C bus handle).
 * @param  reg     Register address (1 byte).
 * @param  data    Data bytes to write (little-endian, LSB first).
 * @param  len     Number of bytes (always 2 for 16-bit registers).
 * @return 0 on success, negative on error.
 */
typedef int32_t (*max17055_write_fn)(void *handle, uint8_t reg,
                                     uint8_t *data, uint16_t len);

/**
 * @brief  Read callback type.
 * @param  handle  Opaque pointer supplied by the user.
 * @param  reg     Register address (1 byte).
 * @param  data    Buffer to receive data bytes (little-endian, LSB first).
 * @param  len     Number of bytes to read (always 2 for 16-bit registers).
 * @return 0 on success, negative on error.
 */
typedef int32_t (*max17055_read_fn)(void *handle, uint8_t reg,
                                    uint8_t *data, uint16_t len);

/* =========================================================================
 * CONFIGURATION & CONTEXT
 * ========================================================================= */

/**
 * @brief  Driver context — one instance per physical device.
 *
 * Fill every field before calling @ref max17055_init().
 * The context may be stack-allocated or static; the driver never
 * modifies it after initialisation (except for @c hibcfg_saved which is
 * captured during init).
 */
typedef struct {
    /* --- Platform callbacks (REQUIRED) --- */
    max17055_write_fn  write_reg;     /**< Platform I2C write function */
    max17055_read_fn   read_reg;      /**< Platform I2C read function */
    void              *handle;        /**< Opaque handle passed to callbacks */

    /* --- Application parameters (REQUIRED) --- */
    uint16_t  rsense_mohm;            /**< Sense resistor value in mΩ (e.g. 10) */
    uint16_t  design_cap_mah;         /**< Battery label/design capacity in mAh */
    uint16_t  charge_voltage_mv;      /**< Charger target voltage in mV
                                           (e.g. 4200 or 4350/4400) */
    uint16_t  empty_voltage_mv;       /**< Empty voltage threshold in mV
                                           (e.g. 3300); recovery = empty + 580 mV */
    uint16_t  ichg_term_ma;           /**< Charge termination current in mA */

    /* --- Optional parameters --- */
    max17055_model_id_t model_id;     /**< Cell chemistry (default: LITHIUM_COBALT) */

    /* --- Internal state (do not set, managed by driver) --- */
    uint16_t  _hibcfg_saved;          /**< HibCfg saved before EZ-init sequence */
    bool      _initialised;           /**< True after successful max17055_init() */
} max17055_ctx_t;

/**
 * @brief  Learned parameters to save to NVM and restore after power loss.
 *
 * Save these values whenever bit 6 of the Cycles register toggles
 * (every 64% of a cycle).  Restore them at startup when the same
 * battery pack is recognised.
 */
typedef struct {
    uint16_t rcomp0;       /**< REG 0x38 */
    uint16_t tempco;       /**< REG 0x39 */
    uint16_t full_cap_rep; /**< REG 0x10 */
    uint16_t cycles;       /**< REG 0x17 */
    uint16_t full_cap_nom; /**< REG 0x23 */
} max17055_learned_params_t;

/* =========================================================================
 * ERROR / SENTINEL VALUES
 * ========================================================================= */

/** Returned by float helpers when a register read fails. */
#define MAX17055_INVALID_FLOAT  (-999.0f)

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief  Initialise the MAX17055 using the EZ-Config sequence.
 *
 * Follows the Maxim Software Implementation Guide (UG6365, Rev 2):
 *  1. Check for POR bit; skip if device was not reset.
 *  2. Wait for FSTAT.DNR == 0 (data not ready).
 *  3. Exit hibernate, write EZ-Config registers, poll ModelCfg.Refresh.
 *  4. Clear POR bit.
 *
 * This function blocks until the IC completes model loading
 * (typically < 500 ms).
 *
 * @param  ctx  Pointer to an initialised context struct.
 * @return 0 on success, <0 if communication failed or timed out.
 */
int32_t max17055_init(max17055_ctx_t *ctx);

/**
 * @brief  Verify the device is reachable and return the DevName register.
 *
 * Can be called before @ref max17055_init() to confirm I2C connectivity.
 * The initial silicon has DevName = 0x4010.
 *
 * @param  ctx      Pointer to context.
 * @param  devname  Output: raw DevName register value.
 * @return 0 on success, <0 on error.
 */
int32_t max17055_check_device(max17055_ctx_t *ctx, uint16_t *devname);

/* =========================================================================
 * HIGH-LEVEL MEASUREMENT ACCESSORS  (return physical values)
 * ========================================================================= */

/**
 * @defgroup measurements High-level measurement functions
 *
 * All functions convert raw register values to physical units using
 * the sense resistor value stored in the context.
 * On I2C error they return @ref MAX17055_INVALID_FLOAT.
 * @{
 */

/**
 * @brief  Reported State of Charge (RepSOC) in percent.
 * @return 0.0–100.0 %, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_soc(max17055_ctx_t *ctx);

/**
 * @brief  Reported remaining capacity (RepCap) in mAh.
 * @return Capacity in mAh, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_rep_cap_mah(max17055_ctx_t *ctx);

/**
 * @brief  Full charge capacity (FullCapRep) in mAh.
 *
 * Updated at the end of each charge cycle.
 * @return Full capacity in mAh, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_full_cap_mah(max17055_ctx_t *ctx);

/**
 * @brief  Time-to-empty (TTE) in seconds.
 * @return Seconds, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_tte_s(max17055_ctx_t *ctx);

/**
 * @brief  Time-to-full (TTF) in seconds.
 * @return Seconds, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_ttf_s(max17055_ctx_t *ctx);

/**
 * @brief  Cell voltage (VCell) in millivolts.
 * @return Voltage in mV, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_voltage_mv(max17055_ctx_t *ctx);

/**
 * @brief  Average cell voltage (AvgVCell) in millivolts.
 * @return Voltage in mV, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_avg_voltage_mv(max17055_ctx_t *ctx);

/**
 * @brief  Instantaneous current in milliamperes (signed).
 *
 * Positive = charging, negative = discharging.
 * @return Current in mA, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_current_ma(max17055_ctx_t *ctx);

/**
 * @brief  Average current in milliamperes (signed).
 * @return Current in mA, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_avg_current_ma(max17055_ctx_t *ctx);

/**
 * @brief  Temperature in degrees Celsius (signed).
 * @return Temperature in °C, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_temperature_c(max17055_ctx_t *ctx);

/**
 * @brief  Battery age as a percentage of design capacity.
 *
 *   Age(%) = 100 × (FullCapRep / DesignCap)
 *
 * @return 0–100+ %, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_age_pct(max17055_ctx_t *ctx);

/**
 * @brief  Cycle count (charge/discharge cycles).
 *
 * 100 in the raw register = 1 complete cycle (1% LSB).
 * @return Cycles as a float (e.g. 1.5 = one and a half cycles),
 *         or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_cycles(max17055_ctx_t *ctx);

/**
 * @brief  Cell internal resistance (RCell) in milliohms.
 * @return Resistance in mΩ, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_internal_resistance_mohm(max17055_ctx_t *ctx);

/**
 * @brief  Available SOC (AvSOC) — unfiltered, may jump on load changes.
 * @return 0.0–100.0 %, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_av_soc(max17055_ctx_t *ctx);

/**
 * @brief  Available capacity (AvCap) — unfiltered, in mAh.
 * @return Capacity in mAh, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_av_cap_mah(max17055_ctx_t *ctx);

/**
 * @brief  Instantaneous power in milliwatts.
 *
 * LSB = 0.8 mW with a 10 mΩ sense resistor.
 * @return Power in mW, or MAX17055_INVALID_FLOAT on error.
 */
float max17055_get_power_mw(max17055_ctx_t *ctx);

/** @} */ /* end defgroup measurements */

/* =========================================================================
 * STATUS / FLAG HELPERS
 * ========================================================================= */

/**
 * @brief  Read and parse the Status register.
 * @param  ctx  Pointer to context.
 * @param  st   Output status struct.
 * @return 0 on success, <0 on error.
 */
int32_t max17055_get_status(max17055_ctx_t *ctx, max17055_status_t *st);

/**
 * @brief  Clear specific bits in the Status register.
 *
 * Typically used to clear POR, dSOCi, Bi, Br after handling them.
 *
 * @param  ctx   Pointer to context.
 * @param  mask  Bitmask of bits to clear (0 = clear, 1 = leave unchanged
 *               — the function performs a read-AND-NAND-write).
 * @return 0 on success, <0 on error.
 */
int32_t max17055_clear_status_bits(max17055_ctx_t *ctx, uint16_t mask);

/**
 * @brief  Read the FStat register (algorithm / data-ready flags).
 * @param  ctx  Pointer to context.
 * @param  fs   Output FStat struct.
 * @return 0 on success, <0 on error.
 */
int32_t max17055_get_fstat(max17055_ctx_t *ctx, max17055_fstat_t *fs);

/**
 * @brief  Check whether the fuel gauge data is ready after a reset.
 *
 * The DNR bit is 1 for ~710 ms after power-up / cell insertion.
 *
 * @param  ctx    Pointer to context.
 * @param  ready  Output: true when data-not-ready bit is cleared.
 * @return 0 on success, <0 on error.
 */
int32_t max17055_is_data_ready(max17055_ctx_t *ctx, bool *ready);

/* =========================================================================
 * LEARNED PARAMETERS — SAVE & RESTORE
 * ========================================================================= */

/**
 * @brief  Read the five learned parameters from the IC.
 *
 * Save these to NVM whenever bit 6 of the Cycles register toggles
 * (i.e. every 64% change).  Restore them on power-up if the same
 * battery pack is detected.
 *
 * @param  ctx     Pointer to context.
 * @param  params  Output: learned parameter snapshot.
 * @return 0 on success, <0 on error.
 */
int32_t max17055_save_learned_params(max17055_ctx_t *ctx,
                                     max17055_learned_params_t *params);

/**
 * @brief  Write previously saved learned parameters back to the IC.
 *
 * Should be called during initialisation when the same battery pack
 * is recognised.  Follow with a 350 ms delay and then call
 * @ref max17055_restore_capacity().
 *
 * @param  ctx     Pointer to context.
 * @param  params  Snapshot previously obtained from max17055_save_learned_params().
 * @return 0 on success, <0 on error.
 */
int32_t max17055_restore_learned_params(max17055_ctx_t *ctx,
                                        const max17055_learned_params_t *params);

/**
 * @brief  Recalculate and restore MixCap and FullCapRep after learned-param restore.
 *
 * Must be called at least 350 ms after @ref max17055_restore_learned_params().
 * Also restores dQAcc and dPAcc as recommended by the implementation guide.
 *
 * @param  ctx     Pointer to context.
 * @param  params  The same snapshot used in max17055_restore_learned_params().
 * @return 0 on success, <0 on error.
 */
int32_t max17055_restore_capacity(max17055_ctx_t *ctx,
                                  const max17055_learned_params_t *params);

/* =========================================================================
 * LOW-LEVEL RAW REGISTER ACCESS
 *
 * Use these when you need direct register control.  All values are in
 * raw register units — refer to the datasheet for scaling.
 * ========================================================================= */

/** @defgroup raw_regs Low-level register read/write functions @{ */

int32_t max17055_reg_read(max17055_ctx_t *ctx, uint8_t reg, uint16_t *val);
int32_t max17055_reg_write(max17055_ctx_t *ctx, uint8_t reg, uint16_t val);

/**
 * @brief  Write a register and verify the write succeeded (up to 3 retries).
 *
 * Recommended by the implementation guide for safety-critical writes
 * such as FullCapRep, Cycles, etc.
 *
 * @param  ctx  Pointer to context.
 * @param  reg  Register address.
 * @param  val  Value to write.
 * @return 0 on success, <0 if write could not be verified.
 */
int32_t max17055_reg_write_verify(max17055_ctx_t *ctx, uint8_t reg, uint16_t val);

/* Specific raw getters/setters used internally and exposed for advanced use */
int32_t max17055_get_rep_soc_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_rep_cap_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_full_cap_rep_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_vcell_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_avg_vcell_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_current_raw(max17055_ctx_t *ctx, int16_t *val);
int32_t max17055_get_avg_current_raw(max17055_ctx_t *ctx, int16_t *val);
int32_t max17055_get_temp_raw(max17055_ctx_t *ctx, int16_t *val);
int32_t max17055_get_tte_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_ttf_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_age_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_cycles_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_rcell_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_mix_soc_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_av_soc_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_mix_cap_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_av_cap_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_full_cap_nom_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_vfocv_raw(max17055_ctx_t *ctx, uint16_t *val);
int32_t max17055_get_vfsoc_raw(max17055_ctx_t *ctx, uint16_t *val);

/** @} */ /* end defgroup raw_regs */

/* =========================================================================
 * UTILITY CONVERSION HELPERS
 * ========================================================================= */

/**
 * @brief  Convert a raw capacity register value to mAh.
 * @param  raw          Raw 16-bit register value.
 * @param  rsense_mohm  Sense resistor in mΩ.
 * @return Capacity in mAh.
 */
float max17055_raw_to_mah(uint16_t raw, uint16_t rsense_mohm);

/**
 * @brief  Convert mAh to raw capacity register value.
 * @param  mah          Capacity in mAh.
 * @param  rsense_mohm  Sense resistor in mΩ.
 * @return Raw register value.
 */
uint16_t max17055_mah_to_raw(float mah, uint16_t rsense_mohm);

/**
 * @brief  Convert a raw current register value (signed) to mA.
 * @param  raw          Signed raw register value.
 * @param  rsense_mohm  Sense resistor in mΩ.
 * @return Current in mA (positive = charge, negative = discharge).
 */
float max17055_raw_to_ma(int16_t raw, uint16_t rsense_mohm);

/**
 * @brief  Convert a raw voltage register value to mV.
 * @param  raw  Raw 16-bit register value.
 * @return Voltage in mV.
 */
float max17055_raw_to_mv(uint16_t raw);

/**
 * @brief  Convert a raw temperature register value (signed) to °C.
 * @param  raw  Signed raw register value.
 * @return Temperature in °C.
 */
float max17055_raw_to_degc(int16_t raw);

/**
 * @brief  Convert a raw time register value to seconds.
 * @param  raw  Raw 16-bit register value.
 * @return Time in seconds.
 */
float max17055_raw_to_seconds(uint16_t raw);

/**
 * @brief  Convert a raw percentage register value to percent.
 * @param  raw  Raw 16-bit register value.
 * @return Percentage (0.0–255.99 %).
 */
float max17055_raw_to_pct(uint16_t raw);

/**
 * @brief  Convert a raw resistance register value to mΩ.
 * @param  raw  Raw 16-bit register value.
 * @return Resistance in mΩ.
 */
float max17055_raw_to_mohm(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* MAX17055_H */
