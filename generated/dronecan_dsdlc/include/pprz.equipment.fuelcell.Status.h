#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define PPRZ_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE 11
#define PPRZ_EQUIPMENT_FUELCELL_STATUS_SIGNATURE (0x448664047118AB12ULL)
#define PPRZ_EQUIPMENT_FUELCELL_STATUS_ID 20141

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class pprz_equipment_fuelcell_Status_cxx_iface;
#endif

struct pprz_equipment_fuelcell_Status {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = pprz_equipment_fuelcell_Status_cxx_iface;
#endif
    uint8_t tank_pressure;
    uint8_t regulated_pressure;
    uint16_t battery_voltage;
    uint16_t output_power;
    uint16_t spm_power;
    int16_t battery_power;
    uint8_t psu_state;
    uint8_t error_code;
    uint8_t sub_code;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t pprz_equipment_fuelcell_Status_encode(struct pprz_equipment_fuelcell_Status* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool pprz_equipment_fuelcell_Status_decode(const CanardRxTransfer* transfer, struct pprz_equipment_fuelcell_Status* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _pprz_equipment_fuelcell_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_equipment_fuelcell_Status* msg, bool tao);
static inline bool _pprz_equipment_fuelcell_Status_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_equipment_fuelcell_Status* msg, bool tao);
void _pprz_equipment_fuelcell_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct pprz_equipment_fuelcell_Status* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->tank_pressure);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->regulated_pressure);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 10, &msg->battery_voltage);
    *bit_ofs += 10;
    canardEncodeScalar(buffer, *bit_ofs, 14, &msg->output_power);
    *bit_ofs += 14;
    canardEncodeScalar(buffer, *bit_ofs, 13, &msg->spm_power);
    *bit_ofs += 13;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->battery_power);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->psu_state);
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->error_code);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sub_code);
    *bit_ofs += 8;
}

/*
 decode pprz_equipment_fuelcell_Status, return true on failure, false on success
*/
bool _pprz_equipment_fuelcell_Status_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct pprz_equipment_fuelcell_Status* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->tank_pressure);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->regulated_pressure);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 10, false, &msg->battery_voltage);
    *bit_ofs += 10;

    canardDecodeScalar(transfer, *bit_ofs, 14, false, &msg->output_power);
    *bit_ofs += 14;

    canardDecodeScalar(transfer, *bit_ofs, 13, false, &msg->spm_power);
    *bit_ofs += 13;

    canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->battery_power);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->psu_state);
    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->error_code);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sub_code);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_equipment_fuelcell_Status sample_pprz_equipment_fuelcell_Status_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(pprz_equipment_fuelcell_Status, PPRZ_EQUIPMENT_FUELCELL_STATUS_ID, PPRZ_EQUIPMENT_FUELCELL_STATUS_SIGNATURE, PPRZ_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE);
#endif
#endif
