#define CANARD_DSDLC_INTERNAL
#include <pprz.equipment.fuelcell.Status.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t pprz_equipment_fuelcell_Status_encode(struct pprz_equipment_fuelcell_Status* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, PPRZ_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE);
    _pprz_equipment_fuelcell_Status_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool pprz_equipment_fuelcell_Status_decode(const CanardRxTransfer* transfer, struct pprz_equipment_fuelcell_Status* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > PPRZ_EQUIPMENT_FUELCELL_STATUS_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_pprz_equipment_fuelcell_Status_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct pprz_equipment_fuelcell_Status sample_pprz_equipment_fuelcell_Status_msg(void) {
    struct pprz_equipment_fuelcell_Status msg;

    msg.tank_pressure = (uint8_t)random_bitlen_unsigned_val(7);
    msg.regulated_pressure = (uint8_t)random_bitlen_unsigned_val(8);
    msg.battery_voltage = (uint16_t)random_bitlen_unsigned_val(10);
    msg.output_power = (uint16_t)random_bitlen_unsigned_val(14);
    msg.spm_power = (uint16_t)random_bitlen_unsigned_val(13);
    msg.battery_power = (int16_t)random_bitlen_signed_val(16);
    msg.psu_state = (uint8_t)random_bitlen_unsigned_val(4);
    msg.error_code = (uint8_t)random_bitlen_unsigned_val(8);
    msg.sub_code = (uint8_t)random_bitlen_unsigned_val(8);
    return msg;
}
#endif
