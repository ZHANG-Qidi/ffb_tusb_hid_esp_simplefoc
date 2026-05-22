#pragma once
#include <stdint.h>

typedef struct {
    uint16_t vid;
    uint16_t pid;
} wheel_id_t;

typedef enum {
    // Fanatec / FTEC
    WHEEL_FTEC_CLUBSPORT_V1,
    WHEEL_FTEC_CLUBSPORT_V2,
    WHEEL_FTEC_CLUBSPORT_V3,
    WHEEL_FTEC_CSL,
    WHEEL_FTEC_CSL_PS4,
    WHEEL_FTEC_CSR_ELITE,
    WHEEL_FTEC_PORSCHE,
    WHEEL_FTEC_DD_F1_PC,
    WHEEL_FTEC_DD_2_PC,

    // Thrustmaster
    WHEEL_TM_F430,
    WHEEL_TM_T150,
    WHEEL_TM_T300_RS,
    WHEEL_TM_TS_PC,
    WHEEL_TM_T80,
    WHEEL_TM_TMX,
    WHEEL_TM_F458,
    WHEEL_TM_T500_RS,
    WHEEL_TM_T500_RS_F1,
    WHEEL_TM_TX,
    WHEEL_TM_T_GT,

    // Logitech
    WHEEL_LG_G920,
    WHEEL_LG_G29,
    WHEEL_LG_G25,
    WHEEL_LG_G27,
    WHEEL_LG_DF_EX,
    WHEEL_LG_DF_GT,
    WHEEL_LG_DF_PRO,
    WHEEL_LG_MOMO,
    WHEEL_LG_G923_XONE,
    WHEEL_LG_G923_PS4,

    // Microsoft
    WHEEL_MS_SIDEWINDER_FFB,

    // SimXperience / Simucube
    WHEEL_SIMX_ACCUPRO,
    WHEEL_SIMUCUBE_OSW_MMOS,
    WHEEL_SIMUCUBE_OSW,
    WHEEL_SIMUCUBE2_SPORT,
    WHEEL_SIMUCUBE2_PRO,
    WHEEL_SIMUCUBE2_ULTIMATE,

    // Sim Steering
    WHEEL_SIM_STEERING_V1,
    WHEEL_SIM_STEERING_V2,
    WHEEL_SIM_STEERING_V3,

    WHEEL_MAX
} wheel_type_t;

extern const wheel_id_t wheel_table[WHEEL_MAX];
