/**
 * @file    max17055_reg.h
 * @brief   MAX17055 register map and bitfield definitions
 *
 *
 * Developer      : Eric Mulwa
 * Date           : 18 May 2026
 * Last Modified  : 18 May 2026
 *
 * All register addresses and bitfield unions for the MAX17055 1-Cell
 * Fuel Gauge with ModelGauge m5 EZ algorithm.
 *
 * Reference:
 *   - MAX17055 Datasheet (19-8726; Rev 0; 1/17)
 *   - MAX17055 User Guide (Rev.0, December 2016)
 *   - MAX17055 Software Implementation Guide (UG6365; Rev 2; 4/19)
 */

#ifndef MAX17055_REG_H
#define MAX17055_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * REGISTER ADDRESSES
 * ========================================================================= */

/* --- Fuel Gauge Output Registers --- */
#define MAX17055_REG_STATUS          0x00  /**< Status flags (alerts, POR, battery) */
#define MAX17055_REG_VALRT_TH        0x01  /**< Voltage alert thresholds */
#define MAX17055_REG_TALRT_TH        0x02  /**< Temperature alert thresholds */
#define MAX17055_REG_SALRT_TH        0x03  /**< SOC alert thresholds */
#define MAX17055_REG_AT_RATE         0x04  /**< Hypothetical load current for AtRate calcs */
#define MAX17055_REG_REP_CAP         0x05  /**< Reported remaining capacity (mAh) */
#define MAX17055_REG_REP_SOC         0x06  /**< Reported state-of-charge (%) */
#define MAX17055_REG_AGE             0x07  /**< Battery age (% of design capacity) */
#define MAX17055_REG_TEMP            0x08  /**< Temperature (°C) */
#define MAX17055_REG_VCELL           0x09  /**< Cell voltage (V) */
#define MAX17055_REG_CURRENT         0x0A  /**< Instantaneous current (A) */
#define MAX17055_REG_AVG_CURRENT     0x0B  /**< Average current (A) */
#define MAX17055_REG_QRESIDUAL       0x0C  /**< Charge that cannot be extracted */
#define MAX17055_REG_MIX_SOC         0x0D  /**< Mixed SOC before empty compensation */
#define MAX17055_REG_AV_SOC          0x0E  /**< Available SOC (unfiltered) */
#define MAX17055_REG_MIX_CAP         0x0F  /**< Mixed capacity before empty compensation */

/* --- Capacity / Time Registers --- */
#define MAX17055_REG_FULL_CAP_REP    0x10  /**< Full capacity (for reporting) */
#define MAX17055_REG_TTE             0x11  /**< Time-to-empty (s) */
#define MAX17055_REG_QR_TABLE00      0x12  /**< QR characterisation table entry 0 */
#define MAX17055_REG_FULL_SOC_THR    0x13  /**< Full SOC threshold for EOC detection */
#define MAX17055_REG_RCELL           0x14  /**< Cell internal resistance (Ω) */
#define MAX17055_REG_AVG_TA          0x16  /**< Average temperature (°C) */
#define MAX17055_REG_CYCLES          0x17  /**< Cycle counter */
#define MAX17055_REG_DESIGN_CAP      0x18  /**< Design / label capacity */
#define MAX17055_REG_AVG_VCELL       0x19  /**< Average cell voltage (V) */
#define MAX17055_REG_MAX_MIN_TEMP    0x1A  /**< Max/min temperature record */
#define MAX17055_REG_MAX_MIN_VOLT    0x1B  /**< Max/min voltage record */
#define MAX17055_REG_MAX_MIN_CURR    0x1C  /**< Max/min current record */
#define MAX17055_REG_CONFIG          0x1D  /**< Configuration register */
#define MAX17055_REG_ICHG_TERM       0x1E  /**< Charge termination current */
#define MAX17055_REG_AV_CAP          0x1F  /**< Available capacity (unfiltered, mAh) */
#define MAX17055_REG_TTF             0x20  /**< Time-to-full (s) */
#define MAX17055_REG_DEV_NAME        0x21  /**< Device name / revision */
#define MAX17055_REG_QR_TABLE10      0x22  /**< QR characterisation table entry 1 */
#define MAX17055_REG_FULL_CAP_NOM    0x23  /**< Full capacity nominal */
#define MAX17055_REG_AIN             0x27  /**< AIN thermistor ratio */
#define MAX17055_REG_LEARN_CFG       0x28  /**< Learning algorithm config */
#define MAX17055_REG_FILTER_CFG      0x29  /**< ADC / mixing filter config */
#define MAX17055_REG_RELAX_CFG       0x2A  /**< Relaxation detection config */
#define MAX17055_REG_MISC_CFG        0x2B  /**< Miscellaneous config */
#define MAX17055_REG_TGAIN           0x2C  /**< Temperature gain (thermistor) */
#define MAX17055_REG_TOFF            0x2D  /**< Temperature offset (thermistor) */
#define MAX17055_REG_CGAIN           0x2E  /**< Current gain */
#define MAX17055_REG_COFF            0x2F  /**< Current offset */
#define MAX17055_REG_QR_TABLE20      0x32  /**< QR characterisation table entry 2 */
#define MAX17055_REG_DIE_TEMP        0x34  /**< Die temperature */
#define MAX17055_REG_FULL_CAP        0x35  /**< Full capacity (condition-compensated) */
#define MAX17055_REG_RCOMP0          0x38  /**< OCV model compensation */
#define MAX17055_REG_TEMP_CO         0x39  /**< Temperature compensation for RComp0 */
#define MAX17055_REG_VEMPTY          0x3A  /**< Empty voltage / recovery voltage */
#define MAX17055_REG_FSTAT           0x3D  /**< Fuel-gauge status flags */
#define MAX17055_REG_TIMER           0x3E  /**< Timer (0–3.2 h) */
#define MAX17055_REG_SHDN_TIMER      0x3F  /**< Shutdown timer config */
#define MAX17055_REG_USER_MEM1       0x40  /**< User memory 1 */
#define MAX17055_REG_QR_TABLE30      0x42  /**< QR characterisation table entry 3 */
#define MAX17055_REG_RGAIN           0x43  /**< Dynamic power resistance gain */
#define MAX17055_REG_DQACC           0x45  /**< Charge accumulator */
#define MAX17055_REG_DPACC           0x46  /**< Percentage accumulator */
#define MAX17055_REG_CONVG_CFG       0x49  /**< Converge-to-empty config */
#define MAX17055_REG_VF_REM_CAP      0x4A  /**< Voltage-FG remaining capacity */
#define MAX17055_REG_QH              0x4D  /**< Raw coulomb count */

/* --- Extended Registers (0xB0 page) --- */
#define MAX17055_REG_STATUS2         0xB0  /**< Firmware status flags */
#define MAX17055_REG_POWER           0xB1  /**< Instantaneous power (mW) */
#define MAX17055_REG_ID_USER_MEM2    0xB2  /**< User memory 2 / serial ID */
#define MAX17055_REG_AVG_POWER       0xB3  /**< Average power (mW) */
#define MAX17055_REG_IALRT_TH        0xB4  /**< Current alert thresholds */
#define MAX17055_REG_TTF_CFG         0xB5  /**< TTF config */
#define MAX17055_REG_CV_MIX_CAP      0xB6  /**< CV mix capacity */
#define MAX17055_REG_CV_HALF_TIME    0xB7  /**< CV half-time */
#define MAX17055_REG_CG_TEMP_CO      0xB8  /**< Copper-trace temperature coeff */
#define MAX17055_REG_CURVE           0xB9  /**< Thermistor / metal curve */
#define MAX17055_REG_HIB_CFG         0xBA  /**< Hibernate mode config */
#define MAX17055_REG_CONFIG2         0xBB  /**< Configuration 2 */
#define MAX17055_REG_VRIPPLE         0xBC  /**< Voltage ripple (RMS) */
#define MAX17055_REG_RIPPLE_CFG      0xBD  /**< Ripple measurement config */
#define MAX17055_REG_TIMER_H         0xBE  /**< Long-duration timer (LSb = 3.2 h) */

/* --- Extended Registers (0xD0 page) --- */
#define MAX17055_REG_RSENSE_MEM3     0xD0  /**< Sense resistor / user memory 3 */
#define MAX17055_REG_SC_OCV_LIM      0xD1  /**< LiFePO4 OCV keep-out window */
#define MAX17055_REG_SOC_HOLD        0xD3  /**< SOC hold (empty / 99%) config */
#define MAX17055_REG_MAX_PEAK_PWR    0xD4  /**< Max peak power (mW) */
#define MAX17055_REG_SUS_PEAK_PWR    0xD5  /**< Sustained peak power (mW) */
#define MAX17055_REG_PACK_RESISTANCE 0xD6  /**< Pack resistance (mΩ) */
#define MAX17055_REG_SYS_RESISTANCE  0xD7  /**< System resistance (mΩ) */
#define MAX17055_REG_MIN_SYS_VOLTAGE 0xD8  /**< Minimum system voltage */
#define MAX17055_REG_MPP_CURRENT     0xD9  /**< Max peak current (mA) */
#define MAX17055_REG_SPP_CURRENT     0xDA  /**< Sustained peak current (mA) */
#define MAX17055_REG_MODEL_CFG       0xDB  /**< Model / algorithm config */
#define MAX17055_REG_AT_QRESIDUAL    0xDC  /**< AtRate residual charge */
#define MAX17055_REG_AT_TTE          0xDD  /**< AtRate time-to-empty */
#define MAX17055_REG_AT_AV_SOC       0xDE  /**< AtRate available SOC */
#define MAX17055_REG_AT_AV_CAP       0xDF  /**< AtRate available capacity */

/* --- Special Registers --- */
#define MAX17055_REG_VFSOC           0xFF  /**< Voltage fuel-gauge SOC */
#define MAX17055_REG_VFOCV           0xFB  /**< Voltage fuel-gauge OCV */

/* Command register (not a standard data register) */
#define MAX17055_REG_SOFT_WAKEUP     0x60  /**< Command register for soft-wakeup */
#define MAX17055_CMD_SOFT_WAKEUP     0x0090u
#define MAX17055_CMD_CLEAR           0x0000u

/* I2C slave address (7-bit) */
#define MAX17055_I2C_ADDR            0x36u

/* =========================================================================
 * STANDARD REGISTER LSB SCALING CONSTANTS
 *
 * Per Table 6 (datasheet) / Table 3 (user guide):
 *   Capacity  : 5.0 µVh / Rsense   →  with 10 mΩ  → 0.5 mAh / LSB
 *   Percentage: 1/256 %             →  1% at bit D8
 *   Voltage   : 78.125 µV           →  1.25 mV at bit D4
 *   Current   : 1.5625 µV / Rsense  →  156.25 µA with 10 mΩ (signed)
 *   Temperature:1/256 °C            →  1°C at bit D8 (signed)
 *   Resistance: 1/4096 Ω
 *   Time      : 5.625 s
 * ========================================================================= */
#define MAX17055_LSB_CAPACITY_UVH    5.0f       /**< µVh per LSB (divide by Rsense_Ω for mAh) */
#define MAX17055_LSB_PERCENTAGE      (1.0f/256.0f) /**< % per LSB */
#define MAX17055_LSB_VOLTAGE_UV      78.125f    /**< µV per LSB */
#define MAX17055_LSB_CURRENT_UV      1.5625f    /**< µV per LSB (divide by Rsense_Ω for mA) */
#define MAX17055_LSB_TEMP_DEG        (1.0f/256.0f) /**< °C per LSB */
#define MAX17055_LSB_RESISTANCE_OHM  (1.0f/4096.0f) /**< Ω per LSB */
#define MAX17055_LSB_TIME_S          5.625f     /**< seconds per LSB */

/* =========================================================================
 * BITFIELD UNIONS
 * ========================================================================= */

/** Helper union: raw 16-bit word with byte access */
typedef union {
    uint16_t u16;
    uint8_t  u8[2];
} max17055_word_t;

/** Helper union: signed 16-bit word with byte access */
typedef union {
    int16_t  i16;
    uint8_t  u8[2];
} max17055_sword_t;

/* --- Status Register (0x00) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t x0        : 1;  /**< bit 0: don't care */
        uint16_t POR       : 1;  /**< bit 1: power-on reset occurred */
        uint16_t Imn       : 1;  /**< bit 2: min current alert */
        uint16_t Bst       : 1;  /**< bit 3: battery absent (host-side) */
        uint16_t x4        : 1;  /**< bit 4: don't care */
        uint16_t x5        : 1;  /**< bit 5: don't care */
        uint16_t Imx       : 1;  /**< bit 6: max current alert */
        uint16_t dSOCi     : 1;  /**< bit 7: 1% SOC change alert */
        uint16_t Vmn       : 1;  /**< bit 8: min voltage alert */
        uint16_t Tmn       : 1;  /**< bit 9: min temp alert */
        uint16_t Smn       : 1;  /**< bit 10: min SOC alert */
        uint16_t Bi        : 1;  /**< bit 11: battery insertion */
        uint16_t Vmx       : 1;  /**< bit 12: max voltage alert */
        uint16_t Tmx       : 1;  /**< bit 13: max temp alert */
        uint16_t Smx       : 1;  /**< bit 14: max SOC alert */
        uint16_t Br        : 1;  /**< bit 15: battery removal */
    } bits;
} max17055_status_t;

/* --- FStat Register (0x3D) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t DNR       : 1;  /**< bit 0: data-not-ready after reset */
        uint16_t x1        : 4;
        uint16_t RelDt2    : 1;  /**< bit 5: long relaxation detected */
        uint16_t EDet      : 1;  /**< bit 6: empty detected */
        uint16_t FQ        : 1;  /**< bit 7: full qualified (EOC) */
        uint16_t RelDt     : 1;  /**< bit 8: relaxed cell detected */
        uint16_t x9        : 7;
    } bits;
} max17055_fstat_t;

/* --- ModelCfg Register (0xDB) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t x0        : 4;
        uint16_t ModelID   : 4;  /**< bits 4-7: cell chemistry model */
        uint16_t x8        : 2;
        uint16_t VChg      : 1;  /**< bit 10: 1 = 4.3–4.4 V charge, 0 = 4.2 V */
        uint16_t x11       : 4;
        uint16_t Refresh   : 1;  /**< bit 15: set to reload model */
    } bits;
} max17055_modelcfg_t;

/* --- Config Register (0x1D) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t Ber       : 1;  /**< bit 0: alert on battery removal */
        uint16_t Bei       : 1;  /**< bit 1: alert on battery insertion */
        uint16_t Aen       : 1;  /**< bit 2: alert pin enable */
        uint16_t FTHRM     : 1;  /**< bit 3: force thermistor bias */
        uint16_t ETHRM     : 1;  /**< bit 4: enable thermistor */
        uint16_t x5        : 1;
        uint16_t COMMSH    : 1;  /**< bit 6: comm-shutdown enable */
        uint16_t SHDN      : 1;  /**< bit 7: force shutdown */
        uint16_t Tex       : 1;  /**< bit 8: temperature external */
        uint16_t Ten       : 1;  /**< bit 9: enable temperature channel */
        uint16_t AINSH     : 1;  /**< bit 10: AIN pin shutdown */
        uint16_t IS        : 1;  /**< bit 11: current alert sticky */
        uint16_t VS        : 1;  /**< bit 12: voltage alert sticky */
        uint16_t TS        : 1;  /**< bit 13: temperature alert sticky */
        uint16_t SS        : 1;  /**< bit 14: SOC alert sticky */
        uint16_t TSel      : 1;  /**< bit 15: 0 = die temp, 1 = thermistor */
    } bits;
} max17055_config_t;

/* --- Config2 Register (0xBB) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t x0        : 1;
        uint16_t CPMode    : 1;  /**< bit 1: constant-power mode */
        uint16_t x2        : 2;  /* reserved: must write 10b */
        uint16_t x4        : 1;  /* reserved: must write 1 */
        uint16_t LDMDL     : 1;  /**< bit 5: load model initiate */
        uint16_t TAlrtEn   : 1;  /**< bit 6: temperature alert enable */
        uint16_t dSOCen    : 1;  /**< bit 7: SOC-change alert enable */
        uint16_t POWR      : 4;  /**< bits 8-11: AvgPower time constant */
        uint16_t DPEn      : 1;  /**< bit 12: dynamic power enable */
        uint16_t AtRateEn  : 1;  /**< bit 13: AtRate calc enable */
        uint16_t x14       : 2;
    } bits;
} max17055_config2_t;

/* --- LearnCfg Register (0x28) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t x0        : 4;
        uint16_t LS        : 3;  /**< bits 4-6: learn stage (0–7) */
        uint16_t x7        : 9;
    } bits;
} max17055_learncfg_t;

/* --- VEmpty Register (0x3A) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t VR        : 7;  /**< bits 0-6: recovery voltage (40 mV/LSB) */
        uint16_t VE        : 9;  /**< bits 7-15: empty voltage target (10 mV/LSB) */
    } bits;
} max17055_vempty_t;

/* --- HibCfg Register (0xBA) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t HibConfig : 15;
        uint16_t EnHib     : 1;  /**< bit 15: enable hibernate mode */
    } bits;
} max17055_hibcfg_t;

/* --- Status2 Register (0xB0) --- */
typedef union {
    uint16_t val;
    struct {
        uint16_t x0        : 1;
        uint16_t Hib       : 1;  /**< bit 1: hibernate status */
        uint16_t x2        : 3;
        uint16_t FullDet   : 1;  /**< bit 5: full detected */
        uint16_t x6        : 2;
        uint16_t SNReady   : 1;  /**< bit 8: serial number ready */
        uint16_t x9        : 3;
        uint16_t DPReady   : 1;  /**< bit 12: dynamic power ready */
        uint16_t AtRateReady:1;  /**< bit 13: AtRate output ready */
        uint16_t x14       : 2;
    } bits;
} max17055_status2_t;

/* --- ModelID values for ModelCfg --- */
typedef enum {
    MAX17055_MODEL_LITHIUM_COBALT = 0,  /**< Li-cobalt-oxide (most Li cells) */
    MAX17055_MODEL_LITHIUM_NCR    = 2,  /**< Panasonic NCR/NCA */
    MAX17055_MODEL_LIFEPO4        = 6,  /**< LiFePO4 (custom char. recommended) */
} max17055_model_id_t;

#ifdef __cplusplus
}
#endif

#endif /* MAX17055_REG_H */
