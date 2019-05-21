/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h"
#include "telemetry.h"

#ifdef PROTO_HAS_NRF24L01

#ifdef EMULATOR
#define USE_FIXED_MFGID
#define Q90C_BIND_COUNT      20
#define Q90C_PACKET_PERIOD   100
#define Q90C_BIND_PACKET_PERIOD 100
#define dbgprintf printf
#else
#define Q90C_BIND_COUNT      250
#define Q90C_PACKET_PERIOD   7336  // Timeout for callback in uSec
#define Q90C_BIND_PACKET_PERIOD    3300
// printf inside an interrupt handler is really dangerous
// this shouldn't be enabled even in debug builds without explicitly
// turning it on
#define dbgprintf if (0) printf
#endif

#define Q90C_INITIAL_WAIT    500
#define Q90C_PACKET_SIZE     12
#define Q90C_RF_BIND_CHANNEL 0x33
#define Q90C_NUM_CHANNELS    3
#define Q90C_ADDRESS_LENGTH  5

static u8 tx_power;
static u8 packet[Q90C_PACKET_SIZE];
static u8 hopping_frequency_no;
static u8 rx_tx_addr[Q90C_ADDRESS_LENGTH];
static u8 hopping_frequency[Q90C_NUM_CHANNELS];
static u16 bind_counter;
static u8 phase;

enum {
    Q90C_BIND,
    Q90C_DATA
};

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
};

#define CHANNEL_FMODE   CHANNEL5  // 3 position flight mode (Angle - Horizon - Acro)
#define CHANNEL_VTX     CHANNEL6  // Toggle HIGH = next vTX frequency

// Bit vector from bit position
#define BV(bit) (1 << bit)

#define CHAN_RANGE (CHAN_MAX_VALUE - CHAN_MIN_VALUE)
static u16 scale_channel(u8 ch, u16 destMin, u16 destMax)
{
    s32 chanval = Channels[ch];
    s32 range = (s32)destMax - (s32)destMin;

    if (chanval < CHAN_MIN_VALUE)
        chanval = CHAN_MIN_VALUE;
    else if (chanval > CHAN_MAX_VALUE)
        chanval = CHAN_MAX_VALUE;
    return (range * (chanval - CHAN_MIN_VALUE)) / CHAN_RANGE + destMin;
}

#define GET_FLAG(ch, mask) (Channels[ch] > 0 ? mask : 0)
static void Q90C_send_packet(u8 bind)
{
    static u8 packet_counter;
    if (bind) {
        memcpy(packet, rx_tx_addr, 4);
        memcpy(&packet[4], hopping_frequency, 3);
        packet[7] = 0x1e;
        packet[8] = 0;
        packet[9] = 0;
        packet[10] = rx_tx_addr[4];
        packet[11] = 0x3a;  // initial checksum value ?
    }
    else
    {
        packet[0] = scale_channel(CHANNEL3, 0, 0xff);
        packet[1] = 0x7a;
        packet[2] = 0x88;
        packet[3] = 0x88;
        packet[4] = 0x1e;
        packet[5] = 0x1e;
        packet[6] = 0x1e;
        packet[7] = 0x1e;
        packet[8] = 0x00; // flags
        packet[9] = 0x00;
        packet[10] = packet_counter++;
        packet[11] = 0x9c;  // initial checksum value ?
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++]);
        if(hopping_frequency_no >= Q90C_NUM_CHANNELS)
            hopping_frequency_no = 0;
    }
    // checksum
    for(u8 i=0; i<Q90C_PACKET_SIZE-1; i++)
        packet[11] += packet[i];
    
    XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WriteEnhancedPayload(packet, Q90C_PACKET_SIZE, 0);

    if (tx_power != Model.tx_power) {
        //Keep transmit power updated
        tx_power = Model.tx_power;
        NRF24L01_SetPower(tx_power);
    }
}

static u16 Q90C_callback()
{
    switch (phase) {
        case Q90C_BIND:
            Q90C_send_packet(1);
            bind_counter--;
            if (bind_counter == 0) {
                PROTOCOL_SetBindState(0);
                XN297_SetTXAddr(rx_tx_addr, Q90C_ADDRESS_LENGTH);
                phase = Q90C_DATA;
            }
            break;
        case Q90C_DATA:
            Q90C_send_packet(0);
            break;
    }
    return Q90C_BIND_PACKET_PERIOD;
}

static void Q90C_init()
{
    NRF24L01_Initialize();
    XN297_SetTXAddr((u8*)"\x4F\x43\x54\x81\x81", Q90C_ADDRESS_LENGTH);  // bind address
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, Q90C_RF_BIND_CHANNEL);
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_250K);           // 250Kbps
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_SetPower(tx_power);
}

static void Q90C_initialize_txid()
{
    memcpy(rx_tx_addr, (u8*)"\x24\x03\x01\x82\x4B", Q90C_ADDRESS_LENGTH);  // stock Tx
    memcpy(hopping_frequency, (u8*)"\x18\x26\x37", Q90C_NUM_CHANNELS);  // stock Tx
}

static void initialize(u8 bind) {
    CLOCK_StopTimer();
    Q90C_initialize_txid();
    tx_power = Model.tx_power;
    hopping_frequency_no = 0;
    Q90C_init();
    if (bind) {
        bind_counter = Q90C_BIND_COUNT;
        phase = Q90C_BIND;
        PROTOCOL_SetBindState(Q90C_BIND_COUNT * Q90C_PACKET_PERIOD / 1000);
    }
    else
    {
        phase = Q90C_DATA;
        XN297_SetTXAddr(rx_tx_addr, Q90C_ADDRESS_LENGTH);
    }
    CLOCK_StartTimer(Q90C_INITIAL_WAIT, Q90C_callback);
}

uintptr_t Q90C_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
        case PROTOCMD_INIT:  initialize(0); return 0;
        case PROTOCMD_DEINIT:
        case PROTOCMD_RESET:
            CLOCK_StopTimer();
            return (NRF24L01_Reset() ? 1 : -1);
        case PROTOCMD_CHECK_AUTOBIND: return 0;
        case PROTOCMD_BIND:  initialize(1); return 0;
        case PROTOCMD_NUMCHAN: return 6;  // A,E,T,R,flight mode,vTX Ch++
        case PROTOCMD_DEFAULT_NUMCHAN: return 6;
        case PROTOCMD_CURRENT_ID: return Model.fixed_id;
        case PROTOCMD_TELEMETRYSTATE: return PROTO_TELEM_UNSUPPORTED;
        case PROTOCMD_CHANNELMAP: return AETRG;
        default: break;
    }
    return 0;
}

#endif
