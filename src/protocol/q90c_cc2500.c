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

#ifdef PROTO_HAS_CC2500

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
//  printf inside an interrupt handler is really dangerous
//  this shouldn't be enabled even in debug builds without explicitly
//  turning it on
#define dbgprintf if (0) printf
#endif

#define Q90C_INITIAL_WAIT    500
#define Q90C_PACKET_SIZE     12
#define Q90C_RF_BIND_CHANNEL 0x33
#define Q90C_NUM_CHANNELS    3
#define Q90C_ADDRESS_LENGTH  5

const char * const q90c_opts[] = {
    _tr_noop("Freq-Fine"),  "-127", "127", NULL,
    NULL
};

enum {
    PROTOOPTS_FREQFINE,
    LAST_PROTO_OPT
};
ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

static u8 tx_power;
static u8 packet[Q90C_PACKET_SIZE];
static u8 hopping_frequency_no;
static u8 rx_tx_addr[Q90C_ADDRESS_LENGTH];
static u8 hopping_frequency[Q90C_NUM_CHANNELS];
static u16 bind_counter;
static u8 phase;
static u8 fine;

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
    CHANNEL6,
};

#define CHANNEL_FMODE   CHANNEL5  // 3 position flight mode (Angle - Horizon - Acro)
#define CHANNEL_VTX     CHANNEL6  // Toggle HIGH = next vTX frequency

// Bit vector from bit position
#define BV(bit) (1 << bit)

// xn297 emulation
////////////////////
static u8 xn297_addr_len;
static u8 xn297_tx_addr[5];

static void XN297L_init()
{
    CC2500_Reset();
    CC2500_Strobe(CC2500_SIDLE);

    // Address Config = No address check
    // Base Frequency = 2400
    // CRC Autoflush = false
    // CRC Enable = false
    // Channel Spacing = 333.251953
    // Data Format = Normal mode
    // Data Rate = 249.939
    // Deviation = 126.953125
    // Device Address = 0
    // Manchester Enable = false
    // Modulated = true
    // Modulation Format = GFSK
    // Packet Length Mode = Variable packet length mode. Packet length configured by the first byte after sync word
    // RX Filter BW = 203.125000
    // Sync Word Qualifier Mode = No preamble/sync
    // TX Power = 0
    // Whitening = false

    CC2500_WriteReg(CC2500_08_PKTCTRL0, 0x01);   // Packet Automation Control
    CC2500_WriteReg(CC2500_0B_FSCTRL1, 0x0A);   // Frequency Synthesizer Control
    CC2500_WriteReg(CC2500_0C_FSCTRL0, 0x00);   // Frequency Synthesizer Control
    CC2500_WriteReg(CC2500_0D_FREQ2, 0x5C);   // Frequency Control Word, High Byte
    CC2500_WriteReg(CC2500_0E_FREQ1, 0x4E);   // Frequency Control Word, Middle Byte
    CC2500_WriteReg(CC2500_0F_FREQ0, 0xC3);   // Frequency Control Word, Low Byte
    CC2500_WriteReg(CC2500_10_MDMCFG4, 0x8D);   // Modem Configuration
    CC2500_WriteReg(CC2500_11_MDMCFG3, 0x3B);   // Modem Configuration
    CC2500_WriteReg(CC2500_12_MDMCFG2, 0x10);   // Modem Configuration
    CC2500_WriteReg(CC2500_13_MDMCFG1, 0x23);   // Modem Configuration
    CC2500_WriteReg(CC2500_14_MDMCFG0, 0xA4);   // Modem Configuration
    CC2500_WriteReg(CC2500_15_DEVIATN, 0x62);   // Modem Deviation Setting
    CC2500_WriteReg(CC2500_18_MCSM0, 0x18);   // Main Radio Control State Machine Configuration
    CC2500_WriteReg(CC2500_19_FOCCFG, 0x1D);   // Frequency Offset Compensation Configuration
    CC2500_WriteReg(CC2500_1A_BSCFG, 0x1C);   // Bit Synchronization Configuration
    CC2500_WriteReg(CC2500_1B_AGCCTRL2, 0xC7);   // AGC Control
    CC2500_WriteReg(CC2500_1C_AGCCTRL1, 0x00);   // AGC Control
    CC2500_WriteReg(CC2500_1D_AGCCTRL0, 0xB0);   // AGC Control
    CC2500_WriteReg(CC2500_21_FREND1, 0xB6);   // Front End RX Configuration
    CC2500_WriteReg(CC2500_23_FSCAL3, 0xEA);   // Frequency Synthesizer Calibration
    CC2500_WriteReg(CC2500_25_FSCAL1, 0x00);   // Frequency Synthesizer Calibration
    CC2500_WriteReg(CC2500_26_FSCAL0, 0x11);   // Frequency Synthesizer Calibration

    CC2500_SetTxRxMode(TX_EN);
}

static void XN297L_SetTXAddr(const u8* addr, u8 len)
{
    if (len > 5) len = 5;
    if (len < 3) len = 3;
    xn297_addr_len = len;
    memcpy(xn297_tx_addr, addr, len);
}

static void XN297L_WriteEnhancedPayload(const u8* msg, u8 len, u8 noack)
{
    u8 buf[32];
    u8 last = 0;
    u8 i;
    u8 scramble_index = 0;
    static u8 pid = 0;
    static const u16 initial = 0xb5d2;

    // address
    for (i = 0; i < xn297_addr_len; ++i) {
        buf[last++] = xn297_tx_addr[xn297_addr_len - i - 1] ^ xn297_scramble[scramble_index++];
    }

    // pcf
    buf[last] = (len << 1) | (pid >> 1); // pcf msb
    buf[last] ^= xn297_scramble[scramble_index++];
    last++;
    buf[last] = (pid << 7) | (noack << 6); // pcf lsb (2bit)

    // payload
    buf[last] |= bit_reverse(msg[0]) >> 2; // first 6 bit of payload
    buf[last] ^= xn297_scramble[scramble_index++];

    for (i = 0; i < len - 1; ++i) {
        last++;
        buf[last] = (bit_reverse(msg[i]) << 6) | (bit_reverse(msg[i + 1]) >> 2);
        
        buf[last] ^= xn297_scramble[scramble_index++];
    }

    last++;
    buf[last] = bit_reverse(msg[len - 1]) << 6; // last 2 bit of payload
    
    buf[last] ^= xn297_scramble[scramble_index++] & 0xc0;
        
    u16 crc = initial;
    for (i = 0; i < last; ++i) {
        crc = crc16_update(crc, buf[i], 8);
    }
    crc = crc16_update(crc, buf[last] & 0xc0, 2);
        
    crc ^= xn297_crc_xorout_scrambled_enhanced[xn297_addr_len - 3 + len];
       
    buf[last++] |= (crc >> 8) >> 2;
    buf[last++] = ((crc >> 8) << 6) | ((crc & 0xff) >> 2);
    buf[last++] = (crc & 0xff) << 6;

    pid++;
    if (pid > 3)
        pid = 0;

    // stop TX/RX
    CC2500_Strobe(CC2500_SIDLE);
    // flush tx FIFO
    CC2500_Strobe(CC2500_SFTX);
    // packet length
    CC2500_WriteReg(CC2500_3F_TXFIFO, last + 3);
    // xn297L preamble
    CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, (u8*)"\x71\x0f\x55", 3);
    // xn297 packet
    CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, buf, last);
    // transmit
    CC2500_Strobe(CC2500_STX);
}

// end of xn297 emulation
///////////////////////////

static u16 scale_channel(s32 chanval, s32 inMin, s32 inMax, u16 destMin, u16 destMax)
{
    s32 range = (s32)destMax - (s32)destMin;
    s32 chanrange = inMax - inMin;

    if (chanval < inMin)
        chanval = inMin;
    else if (chanval > inMax)
        chanval = inMax;
    return (range * (chanval - inMin)) / chanrange + destMin;
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
        packet[0] = scale_channel(Channels[CHANNEL3], CHAN_MIN_VALUE, CHAN_MAX_VALUE, 0, 0xff);  // throttle
        // A,E,R have weird scaling
        if(Channels[CHANNEL4] <= 0)
            packet[1] = scale_channel(Channels[CHANNEL4], CHAN_MIN_VALUE, 0, 0, 0x7a);  // rudder neutral = 0x7a
        else
            packet[1] = scale_channel(Channels[CHANNEL4], 0, CHAN_MAX_VALUE, 0x7a, 0xff);
        if (Channels[CHANNEL2] <= 0)
            packet[2] = scale_channel(Channels[CHANNEL2], CHAN_MIN_VALUE, 0, 0, 0x88);  // elevator neutral = 0x88
        else
            packet[2] = scale_channel(Channels[CHANNEL2], 0, CHAN_MAX_VALUE, 0x88, 0xff);
        if (Channels[CHANNEL1] <= 0)
            packet[3] = scale_channel(Channels[CHANNEL1], CHAN_MIN_VALUE, 0, 0, 0x88);  // aileron neutral = 0x88
        else
            packet[3] = scale_channel(Channels[CHANNEL1], 0, CHAN_MAX_VALUE, 0x88, 0xff);
        
        //packet[2] = 0x88; // scale_channel(CHANNEL2, 0, 0xff);  // elevator neutral = 0x88
        //packet[3] = 0x88; // scale_channel(CHANNEL1, 0, 0xff);  // aileron neutral = 0x88;
        packet[4] = 0x1e;
        packet[5] = 0x1e;
        packet[6] = 0x1e;
        packet[7] = 0x1e;
        packet[8] = 0x00; // flags
        packet[9] = 0x00;
        packet[10] = packet_counter++;
        packet[11] = 0x9c;  // initial checksum value ?
        CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[hopping_frequency_no++] * 3);
        if (hopping_frequency_no >= Q90C_NUM_CHANNELS)
            hopping_frequency_no = 0;
    }
    // checksum
    for (u8 i = 0; i < Q90C_PACKET_SIZE - 1; i++)
        packet[11] += packet[i];

    XN297L_WriteEnhancedPayload(packet, Q90C_PACKET_SIZE, 0);

    if (tx_power != Model.tx_power) {
        //Keep transmit power updated
        tx_power = Model.tx_power;
        CC2500_SetPower(tx_power);
    }
}

static u16 Q90C_callback()
{
    if (fine != (s8)Model.proto_opts[PROTOOPTS_FREQFINE]) {
        fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
        CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    }
    switch (phase) {
    case Q90C_BIND:
        Q90C_send_packet(1);
        bind_counter--;
        if (bind_counter == 0) {
            PROTOCOL_SetBindState(0);
            XN297L_SetTXAddr(rx_tx_addr, Q90C_ADDRESS_LENGTH);
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
    XN297L_init();
    XN297L_SetTXAddr((u8*)"\x4F\x43\x54\x81\x81", Q90C_ADDRESS_LENGTH);  // bind address
    CC2500_WriteReg(CC2500_0A_CHANNR, Q90C_RF_BIND_CHANNEL * 3);
    CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    CC2500_SetPower(tx_power);
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
    fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
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
        XN297L_SetTXAddr(rx_tx_addr, Q90C_ADDRESS_LENGTH);
    }
    CLOCK_StartTimer(Q90C_INITIAL_WAIT, Q90C_callback);
}

uintptr_t Q90CC_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
    case PROTOCMD_INIT:  initialize(0); return 0;
    case PROTOCMD_DEINIT:
    case PROTOCMD_RESET:
        CLOCK_StopTimer();
        return (CC2500_Reset() ? 1 : -1);
    case PROTOCMD_CHECK_AUTOBIND: return 0;
    case PROTOCMD_BIND:  initialize(1); return 0;
    case PROTOCMD_NUMCHAN: return 6;  // A,E,T,R,flight mode,vTX Ch++
    case PROTOCMD_DEFAULT_NUMCHAN: return 6;
    case PROTOCMD_CURRENT_ID: return Model.fixed_id;
    case PROTOCMD_GETOPTIONS: return (uintptr_t)q90c_opts;
    case PROTOCMD_TELEMETRYSTATE: return PROTO_TELEM_UNSUPPORTED;
    case PROTOCMD_CHANNELMAP: return AETRG;
    default: break;
    }
    return 0;
}

#endif
