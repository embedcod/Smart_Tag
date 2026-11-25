/**
 * @file    max17055.c
 * @brief   MAX17055 fuel gauge driver — implementation
 *
 * Developer      : Eric Mulwa
 * Date           : 18 May 2026
 * Last Modified  : 18 May 2026
 *
 * EZ-Config initialization sequence follows:
 *   MAX17055 Software Implementation Guide, UG6365, Rev 2, 4/19
 *
 * Scaling constants follow:
 *   MAX17055 Datasheet, Table 6 (Standard Register Formats)
 *   MAX17055 User Guide, Table 3
 */

#include "max17055.h"

#include <string.h>   /* memcpy */
#include <stddef.h>

/* =========================================================================
 * PRIVATE HELPERS
 * ========================================================================= */

/** Millisecond delay — provided by the user via a weak symbol or macro.
 *  Override MAX17055_DELAY_MS in your build system if needed. */
#ifndef MAX17055_DELAY_MS
  #ifdef __ZEPHYR__
    #include <zephyr/kernel.h>
    #define MAX17055_DELAY_MS(ms)  k_msleep(ms)
  #elif defined(ESP_PLATFORM)
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #define MAX17055_DELAY_MS(ms)  vTaskDelay(pdMS_TO_TICKS(ms))
  #elif defined(USE_HAL_DRIVER)      /* STM32 HAL */
    #include "stm32_hal.h"
    #define MAX17055_DELAY_MS(ms)  HAL_Delay(ms)
  #else
    /* Fallback: busy-wait — replace with your RTOS delay */
    #include <stdint.h>
    static void _max17055_busy_wait(uint32_t ms)
    {
        volatile uint32_t i;
        for (i = 0; i < ms * 8000UL; i++) { (void)i; }
    }
    #define MAX17055_DELAY_MS(ms)  _max17055_busy_wait(ms)
  #endif
#endif

/* Timeout for polling loops (ms) */
#define POLL_TIMEOUT_MS   1000u
#define POLL_STEP_MS      10u

/* Write-verify retry count (Maxim recommendation: up to 3 attempts) */
#define WRITE_VERIFY_RETRIES  3u

/* -------------------------------------------------------------------------
 * Internal I/O shims
 * ------------------------------------------------------------------------- */

static int32_t _read16(max17055_ctx_t *ctx, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0};
    int32_t ret = ctx->read_reg(ctx->handle, reg, buf, 2);
    if (ret == 0) {
        *out = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    }
    return ret;
}

static int32_t _write16(max17055_ctx_t *ctx, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val & 0xFFu);
    buf[1] = (uint8_t)(val >> 8);
    return ctx->write_reg(ctx->handle, reg, buf, 2);
}

/* =========================================================================
 * PUBLIC — LOW-LEVEL RAW REGISTER ACCESS
 * ========================================================================= */

int32_t max17055_reg_read(max17055_ctx_t *ctx, uint8_t reg, uint16_t *val)
{
    return _read16(ctx, reg, val);
}

int32_t max17055_reg_write(max17055_ctx_t *ctx, uint8_t reg, uint16_t val)
{
    return _write16(ctx, reg, val);
}

int32_t max17055_reg_write_verify(max17055_ctx_t *ctx, uint8_t reg, uint16_t val)
{
    for (uint8_t attempt = 0; attempt < WRITE_VERIFY_RETRIES; attempt++) {
        int32_t ret = _write16(ctx, reg, val);
        if (ret != 0) {
            return ret;
        }
        MAX17055_DELAY_MS(1);
        uint16_t readback = 0;
        ret = _read16(ctx, reg, &readback);
        if (ret != 0) {
            return ret;
        }
        if (readback == val) {
            return 0;  /* verified OK */
        }
    }
    return -1;  /* write could not be verified */
}

/* =========================================================================
 * PUBLIC — UTILITY CONVERSION HELPERS
 * ========================================================================= */

/*
 * Capacity: LSB = 5.0 µVh / Rsense_Ω
 *         = 5.0 µVh / (rsense_mohm × 10^-3 Ω)
 *         = 5.0e-6 Vh / (rsense_mohm × 10^-3)
 *         = 5.0e-3 mVh / (rsense_mohm × 10^-3)
 *         = 5.0 mVh / rsense_mohm
 *  mAh = raw × 5.0 / rsense_mohm
 */
float max17055_raw_to_mah(uint16_t raw, uint16_t rsense_mohm)
{
    if (rsense_mohm == 0) { return 0.0f; }
    return (float)raw * 5.0f / (float)rsense_mohm;
}

uint16_t max17055_mah_to_raw(float mah, uint16_t rsense_mohm)
{
    if (rsense_mohm == 0) { return 0; }
    float raw = mah * (float)rsense_mohm / 5.0f;
    return (uint16_t)(raw + 0.5f);
}

/*
 * Current: LSB = 1.5625 µV / Rsense_Ω  (signed two's-complement)
 *        = 1.5625e-6 V / (rsense_mohm × 10^-3)
 *        = 1.5625e-3 mV / (rsense_mohm × 10^-3)
 *        = 1.5625 mV / rsense_mohm
 *  mA = raw_signed × 1.5625 / rsense_mohm
 */
float max17055_raw_to_ma(int16_t raw, uint16_t rsense_mohm)
{
    if (rsense_mohm == 0) { return 0.0f; }
    return (float)raw * 1.5625f / (float)rsense_mohm;
}

/*
 * Voltage: LSB = 78.125 µV  →  mV = raw × 0.078125
 */
float max17055_raw_to_mv(uint16_t raw)
{
    return (float)raw * 0.078125f;
}

/*
 * Temperature: LSB = 1/256 °C  (signed two's-complement)
 */
float max17055_raw_to_degc(int16_t raw)
{
    return (float)raw / 256.0f;
}

/*
 * Time: LSB = 5.625 s
 */
float max17055_raw_to_seconds(uint16_t raw)
{
    return (float)raw * 5.625f;
}

/*
 * Percentage: LSB = 1/256 %
 */
float max17055_raw_to_pct(uint16_t raw)
{
    return (float)raw / 256.0f;
}

/*
 * Resistance: LSB = 1/4096 Ω  →  mΩ = raw × 1000/4096
 */
float max17055_raw_to_mohm(uint16_t raw)
{
    return (float)raw * (1000.0f / 4096.0f);
}

/* =========================================================================
 * PUBLIC — RAW REGISTER GETTERS
 * ========================================================================= */

int32_t max17055_get_rep_soc_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_REP_SOC, val); }

int32_t max17055_get_rep_cap_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_REP_CAP, val); }

int32_t max17055_get_full_cap_rep_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_FULL_CAP_REP, val); }

int32_t max17055_get_vcell_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_VCELL, val); }

int32_t max17055_get_avg_vcell_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_AVG_VCELL, val); }

int32_t max17055_get_current_raw(max17055_ctx_t *ctx, int16_t *val)
{
    uint16_t raw = 0;
    int32_t ret = _read16(ctx, MAX17055_REG_CURRENT, &raw);
    *val = (int16_t)raw;
    return ret;
}

int32_t max17055_get_avg_current_raw(max17055_ctx_t *ctx, int16_t *val)
{
    uint16_t raw = 0;
    int32_t ret = _read16(ctx, MAX17055_REG_AVG_CURRENT, &raw);
    *val = (int16_t)raw;
    return ret;
}

int32_t max17055_get_temp_raw(max17055_ctx_t *ctx, int16_t *val)
{
    uint16_t raw = 0;
    int32_t ret = _read16(ctx, MAX17055_REG_TEMP, &raw);
    *val = (int16_t)raw;
    return ret;
}

int32_t max17055_get_tte_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_TTE, val); }

int32_t max17055_get_ttf_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_TTF, val); }

int32_t max17055_get_age_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_AGE, val); }

int32_t max17055_get_cycles_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_CYCLES, val); }

int32_t max17055_get_rcell_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_RCELL, val); }

int32_t max17055_get_mix_soc_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_MIX_SOC, val); }

int32_t max17055_get_av_soc_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_AV_SOC, val); }

int32_t max17055_get_mix_cap_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_MIX_CAP, val); }

int32_t max17055_get_av_cap_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_AV_CAP, val); }

int32_t max17055_get_full_cap_nom_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_FULL_CAP_NOM, val); }

int32_t max17055_get_vfocv_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_VFOCV, val); }

int32_t max17055_get_vfsoc_raw(max17055_ctx_t *ctx, uint16_t *val)
    { return _read16(ctx, MAX17055_REG_VFSOC, val); }

/* =========================================================================
 * PUBLIC — HIGH-LEVEL MEASUREMENT ACCESSORS
 * ========================================================================= */

float max17055_get_soc(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_REP_SOC, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_pct(raw);
}

float max17055_get_rep_cap_mah(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_REP_CAP, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mah(raw, ctx->rsense_mohm);
}

float max17055_get_full_cap_mah(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_FULL_CAP_REP, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mah(raw, ctx->rsense_mohm);
}

float max17055_get_tte_s(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_TTE, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_seconds(raw);
}

float max17055_get_ttf_s(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_TTF, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_seconds(raw);
}

float max17055_get_voltage_mv(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_VCELL, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mv(raw);
}

float max17055_get_avg_voltage_mv(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_AVG_VCELL, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mv(raw);
}

float max17055_get_current_ma(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_CURRENT, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_ma((int16_t)raw, ctx->rsense_mohm);
}

float max17055_get_avg_current_ma(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_AVG_CURRENT, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_ma((int16_t)raw, ctx->rsense_mohm);
}

float max17055_get_temperature_c(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_TEMP, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_degc((int16_t)raw);
}

float max17055_get_age_pct(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_AGE, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_pct(raw);
}

float max17055_get_cycles(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_CYCLES, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    /* Cycles register: 1% LSB, so 100 raw = 1 full cycle */
    return (float)raw / 100.0f;
}

float max17055_get_internal_resistance_mohm(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_RCELL, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mohm(raw);
}

float max17055_get_av_soc(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_AV_SOC, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_pct(raw);
}

float max17055_get_av_cap_mah(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_AV_CAP, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    return max17055_raw_to_mah(raw, ctx->rsense_mohm);
}

float max17055_get_power_mw(max17055_ctx_t *ctx)
{
    uint16_t raw = 0;
    if (_read16(ctx, MAX17055_REG_POWER, &raw) != 0) {
        return MAX17055_INVALID_FLOAT;
    }
    /* Power LSB = 0.8 mW with 10 mΩ sense resistor.
     * General: LSB = 8.0 mW·mΩ / rsense_mohm  */
    return (float)(int16_t)raw * 8.0f / (float)ctx->rsense_mohm;
}

/* =========================================================================
 * PUBLIC — STATUS & FLAG HELPERS
 * ========================================================================= */

int32_t max17055_get_status(max17055_ctx_t *ctx, max17055_status_t *st)
{
    return _read16(ctx, MAX17055_REG_STATUS, &st->val);
}

int32_t max17055_clear_status_bits(max17055_ctx_t *ctx, uint16_t mask)
{
    uint16_t current = 0;
    int32_t ret = _read16(ctx, MAX17055_REG_STATUS, &current);
    if (ret != 0) { return ret; }
    /* Write back with requested bits cleared */
    return max17055_reg_write_verify(ctx, MAX17055_REG_STATUS,
                                     current & ~mask);
}

int32_t max17055_get_fstat(max17055_ctx_t *ctx, max17055_fstat_t *fs)
{
    return _read16(ctx, MAX17055_REG_FSTAT, &fs->val);
}

int32_t max17055_is_data_ready(max17055_ctx_t *ctx, bool *ready)
{
    uint16_t fstat = 0;
    int32_t ret = _read16(ctx, MAX17055_REG_FSTAT, &fstat);
    if (ret != 0) { return ret; }
    *ready = ((fstat & 0x0001u) == 0u);   /* DNR bit = 0 means ready */
    return 0;
}

/* =========================================================================
 * PUBLIC — INITIALISATION
 * ========================================================================= */

int32_t max17055_check_device(max17055_ctx_t *ctx, uint16_t *devname)
{
    return _read16(ctx, MAX17055_REG_DEV_NAME, devname);
}

/**
 * @brief  Encode VEmpty register value from mV targets.
 *
 * VEmpty register format (Table 2, datasheet):
 *   D15–D7: VE  = empty voltage,    10 mV/LSB  (9 bits)
 *   D6 –D0: VR  = recovery voltage, 40 mV/LSB  (7 bits)
 *
 * Recovery voltage default = 3880 mV.  We set it to empty + 580 mV,
 * which matches the datasheet default of 3.3 V empty / 3.88 V recovery.
 */
static uint16_t _encode_vempty(uint16_t empty_mv)
{
    uint16_t ve = (empty_mv / 10u) & 0x1FFu;           /* 9 bits, 10 mV/LSB */
    uint16_t recovery_mv = empty_mv + 580u;
    /* Clamp recovery to 5080 mV (max for 7-bit 40 mV/LSB) */
    if (recovery_mv > 5080u) { recovery_mv = 5080u; }
    uint16_t vr = (recovery_mv / 40u) & 0x7Fu;         /* 7 bits, 40 mV/LSB */
    return (ve << 7) | vr;
}

/**
 * @brief  Encode IChgTerm register from mA, using rsense.
 *
 * Current LSB = 1.5625 µV / Rsense_Ω
 * raw = ma × rsense_mohm / 1.5625
 */
static uint16_t _encode_ichgterm(uint16_t ichg_ma, uint16_t rsense_mohm)
{
    float raw = (float)ichg_ma * (float)rsense_mohm / 1.5625f;
    return (uint16_t)(raw + 0.5f);
}

int32_t max17055_init(max17055_ctx_t *ctx)
{
    int32_t ret = 0;

    /* ---------------------------------------------------------------
     * Step 1 — Check for Power-On Reset
     * --------------------------------------------------------------- */
    uint16_t status_raw = 0;
    ret = _read16(ctx, MAX17055_REG_STATUS, &status_raw);
    if (ret != 0) { return ret; }

    /*if ((status_raw & 0x0002u) == 0u) { // TODO
        // POR bit is 0: device has already been configured — skip init
        ctx->_initialised = true;
        return 0;
    }*/

	bool force_init = true;
	if (!force_init && ((status_raw & 0x0002u) == 0u)) {
	    ctx->_initialised = true;
	    return 0;
	}

    /* ---------------------------------------------------------------
     * Step 2 — Wait for FSTAT.DNR == 0 (data not ready after POR)
     * --------------------------------------------------------------- */
    uint32_t elapsed = 0;
    uint16_t fstat = 0;
    do {
        MAX17055_DELAY_MS(POLL_STEP_MS);
        elapsed += POLL_STEP_MS;
        ret = _read16(ctx, MAX17055_REG_FSTAT, &fstat);
        if (ret != 0) { return ret; }
    } while ((fstat & 0x0001u) && (elapsed < POLL_TIMEOUT_MS));

    if (fstat & 0x0001u) {
        return -2;   /* timeout waiting for DNR */
    }

    /* ---------------------------------------------------------------
     * Step 3 — Exit Hibernate, write EZ-Config registers
     * --------------------------------------------------------------- */

    /* 3a — save and disable hibernate */
    ret = _read16(ctx, MAX17055_REG_HIB_CFG, &ctx->_hibcfg_saved);
    if (ret != 0) { return ret; }

    /* Soft-wakeup sequence (Section 1.10, User Guide) */
    ret = _write16(ctx, MAX17055_REG_SOFT_WAKEUP, MAX17055_CMD_SOFT_WAKEUP);
    if (ret != 0) { return ret; }
    ret = _write16(ctx, MAX17055_REG_HIB_CFG, 0x0000u);
    if (ret != 0) { return ret; }
    ret = _write16(ctx, MAX17055_REG_SOFT_WAKEUP, MAX17055_CMD_CLEAR);
    if (ret != 0) { return ret; }

    /* 3b — EZ-Config Option 1 (no INI file required)
     *
     *  Registers to write:
     *    DesignCap (0x18) — label capacity in raw units
     *    dQAcc     (0x45) — DesignCap / 32
     *    IChgTerm  (0x1E) — charge termination current
     *    VEmpty    (0x3A) — empty / recovery voltage
     *    dPAcc     (0x46) — 44138/DesignCap × dQAcc   (4.2 V)
     *                    or 51200/DesignCap × dQAcc   (>4.275 V)
     *    ModelCfg  (0xDB) — 0x8000 (4.2 V) or 0x8400 (4.3–4.4 V)
     */
    uint16_t design_cap_raw = max17055_mah_to_raw(
                                (float)ctx->design_cap_mah, ctx->rsense_mohm);
    uint16_t dqacc          = design_cap_raw / 16u;
    uint16_t ichgterm_raw   = _encode_ichgterm(ctx->ichg_term_ma,
                                               ctx->rsense_mohm);
    uint16_t vempty_raw     = _encode_vempty(ctx->empty_voltage_mv);

    ret = _write16(ctx, MAX17055_REG_DESIGN_CAP, design_cap_raw);
    if (ret != 0) { return ret; }

    ret = _write16(ctx, MAX17055_REG_DQACC, dqacc);
    if (ret != 0) { return ret; }

    ret = _write16(ctx, MAX17055_REG_ICHG_TERM, ichgterm_raw);
    if (ret != 0) { return ret; }

    ret = _write16(ctx, MAX17055_REG_VEMPTY, vempty_raw);
    if (ret != 0) { return ret; }

    /* ModelCfg and dPAcc depend on charge voltage */
    max17055_modelcfg_t modelcfg = { .val = 0 };
    modelcfg.bits.Refresh  = 1;
    modelcfg.bits.ModelID  = (uint16_t)ctx->model_id;

    uint16_t dpacc;
    if (ctx->charge_voltage_mv > 4275u) {
        /* High-voltage charge (4.3–4.4 V) */
        modelcfg.bits.VChg = 1;
        /* dPAcc = dQAcc × 51200 / DesignCap  (keep only integer part) */
        dpacc = (uint16_t)(((uint32_t)dqacc * 51200u) / design_cap_raw);
    } else {
        /* Standard charge (≤4.2 V) */
        modelcfg.bits.VChg = 0;
        /* dPAcc = dQAcc × 44138 / DesignCap */
        dpacc = (uint16_t)(((uint32_t)dqacc * 44138u) / design_cap_raw);
    }

    ret = _write16(ctx, MAX17055_REG_DPACC, dpacc);
    if (ret != 0) { return ret; }

    ret = _write16(ctx, MAX17055_REG_MODEL_CFG, modelcfg.val);
    if (ret != 0) { return ret; }

    /* 3c — Poll ModelCfg.Refresh until it clears (model loading complete) */
    elapsed = 0;
    uint16_t mcfg = 0;
    do {
        MAX17055_DELAY_MS(POLL_STEP_MS);
        elapsed += POLL_STEP_MS;
        ret = _read16(ctx, MAX17055_REG_MODEL_CFG, &mcfg);
        if (ret != 0) { return ret; }
    } while ((mcfg & 0x8000u) && (elapsed < POLL_TIMEOUT_MS));

    if (mcfg & 0x8000u) {
        return -3;   /* timeout: model never finished loading */
    }

    /* 3d — Restore original HibCfg */
    ret = _write16(ctx, MAX17055_REG_HIB_CFG, ctx->_hibcfg_saved);
    if (ret != 0) { return ret; }

    /* ---------------------------------------------------------------
     * Step 4 — Clear POR bit to mark configuration as complete
     * --------------------------------------------------------------- */
    ret = _read16(ctx, MAX17055_REG_STATUS, &status_raw);
    if (ret != 0) { return ret; }

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_STATUS,
                                    status_raw & ~0x0002u);
    if (ret != 0) { return ret; }

    ctx->_initialised = true;
    return 0;
}

/* =========================================================================
 * PUBLIC — LEARNED PARAMETERS SAVE / RESTORE
 * ========================================================================= */

int32_t max17055_save_learned_params(max17055_ctx_t *ctx,
                                     max17055_learned_params_t *params)
{
    int32_t ret;

    ret = _read16(ctx, MAX17055_REG_RCOMP0,       &params->rcomp0);
    if (ret != 0) { return ret; }
    ret = _read16(ctx, MAX17055_REG_TEMP_CO,      &params->tempco);
    if (ret != 0) { return ret; }
    ret = _read16(ctx, MAX17055_REG_FULL_CAP_REP, &params->full_cap_rep);
    if (ret != 0) { return ret; }
    ret = _read16(ctx, MAX17055_REG_CYCLES,       &params->cycles);
    if (ret != 0) { return ret; }
    ret = _read16(ctx, MAX17055_REG_FULL_CAP_NOM, &params->full_cap_nom);
    if (ret != 0) { return ret; }

    return 0;
}

int32_t max17055_restore_learned_params(max17055_ctx_t *ctx,
                                        const max17055_learned_params_t *params)
{
    int32_t ret;

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_RCOMP0,
                                    params->rcomp0);
    if (ret != 0) { return ret; }

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_TEMP_CO,
                                    params->tempco);
    if (ret != 0) { return ret; }

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_FULL_CAP_NOM,
                                    params->full_cap_nom);
    if (ret != 0) { return ret; }

    return 0;
    /* NOTE: caller must wait ≥ 350 ms then call max17055_restore_capacity() */
}

int32_t max17055_restore_capacity(max17055_ctx_t *ctx,
                                  const max17055_learned_params_t *params)
{
    int32_t ret;

    /* Recalculate MixCap:  MixCap = (MixSOC_raw × FullCapNom) / 25600
     * 25600 = 100% × 256 (RepSOC LSB denominator)                        */
    uint16_t mix_soc_raw = 0;
    ret = _read16(ctx, MAX17055_REG_MIX_SOC, &mix_soc_raw);
    if (ret != 0) { return ret; }

    uint16_t mix_cap = (uint16_t)(((uint32_t)mix_soc_raw
                                  * params->full_cap_nom) / 25600u);

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_MIX_CAP, mix_cap);
    if (ret != 0) { return ret; }

    ret = max17055_reg_write_verify(ctx, MAX17055_REG_FULL_CAP_REP,
                                    params->full_cap_rep);
    if (ret != 0) { return ret; }

    /* Write dQAcc and dPAcc to 200% of capacity (per implementation guide) */
    uint16_t dqacc = params->full_cap_nom / 16u;
    ret = max17055_reg_write_verify(ctx, MAX17055_REG_DPACC, 0x0C80u);
    if (ret != 0) { return ret; }
    ret = max17055_reg_write_verify(ctx, MAX17055_REG_DQACC, dqacc);
    if (ret != 0) { return ret; }

    /* Caller should wait another 350 ms, then restore Cycles */
    ret = max17055_reg_write_verify(ctx, MAX17055_REG_CYCLES,
                                    params->cycles);
    return ret;
}
