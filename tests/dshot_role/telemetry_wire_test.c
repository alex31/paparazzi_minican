// Verify NaN survives the shared float16 codec, and the documented FlexDebug wire size.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <dronecan.protocol.FlexDebug.h>
#include <uavcan.equipment.esc.Status.h>

int main(void)
{
    struct uavcan_equipment_esc_Status status = {
        .esc_index = 4, .rpm = 857, .voltage = NAN, .current = NAN, .temperature = NAN
    }, decoded_status = {0};
    uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];
    CanardRxTransfer transfer = {.payload_head = buffer, .tao = true};
    transfer.payload_len = uavcan_equipment_esc_Status_encode(&status, buffer, true);
    assert(!uavcan_equipment_esc_Status_decode(&transfer, &decoded_status));
    assert(decoded_status.esc_index == 4 && decoded_status.rpm == 857);
    assert(isnan(decoded_status.voltage) && isnan(decoded_status.current) && isnan(decoded_status.temperature));

    struct dronecan_protocol_FlexDebug debug = {
        .id = 2004, .u8 = {.len = 5, .data = {1, 4, 3, 100, 0xEB}}
    }, decoded_debug = {0};
    for (unsigned tao = 0; tao < 2; ++tao) {
        transfer.tao = tao;
        transfer.payload_len = dronecan_protocol_FlexDebug_encode(&debug, buffer, tao);
        assert(transfer.payload_len == (tao ? 7 : 8));
        assert(buffer[0] == 0xD4 && buffer[1] == 0x07); // ID 2004, little endian.
        assert(!dronecan_protocol_FlexDebug_decode(&transfer, &decoded_debug));
        assert(decoded_debug.id == debug.id && decoded_debug.u8.len == 5);
        assert(memcmp(decoded_debug.u8.data, debug.u8.data, 5) == 0);
    }
    puts("DShot wire: shared Status float16 NaN and FlexDebug codecs (with/without TAO) OK");
}
