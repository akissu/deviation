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

#ifdef PROTO_HAS_CC2500

#ifdef EMULATOR
#define USE_FIXED_MFGID
#define BIND_COUNT 5
#define PACKET_PERIOD    150
#define dbgprintf printf
#else
#define BIND_COUNT 300
#define PACKET_PERIOD    6465  // Timeout for callback in uSec
 // printf inside an interrupt handler is really dangerous
 // this shouldn't be enabled even in debug builds without explicitly
 // turning it on
#define dbgprintf if (0) printf
#endif

#define INITIAL_WAIT        500
#define PACKET_SIZE         15
#define RF_NUM_CHANNELS     4
#define ADDRESS_LENGTH      4
#define RF_BIND_CHANNEL     0x32

const char * const jjrch31_opts[] = {
    _tr_noop("Freq-Fine"),  "-127", "127", NULL,
    NULL
};

enum {
    PROTOOPTS_FREQFINE = 0,
    LAST_PROTO_OPT
};
ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

// For code readability
enum {
    CHANNEL1 = 0,  // Aileron
    CHANNEL2,      // Elevator
    CHANNEL3,      // Throttle
    CHANNEL4,      // Rudder
    CHANNEL5,      // 
    CHANNEL6,      // Flip
    CHANNEL7,      // Still camera
    CHANNEL8,      // Video camera
    CHANNEL9,      // Headless
    CHANNEL10,     // Return To Home
};

#define CHANNEL_FLIP        CHANNEL6
#define CHANNEL_PICTURE     CHANNEL7
#define CHANNEL_VIDEO       CHANNEL8
#define CHANNEL_HEADLESS    CHANNEL9
#define CHANNEL_RTH         CHANNEL10

// Bit vector from bit position
#define BV(bit) (1 << bit)

static u8 packet[PACKET_SIZE];
static u8 tx_power;
static u8 fine;
//static u8 sync[2];
//static u8 txid[4];
static u16 counter;
static u8 rx_tx_addr[ADDRESS_LENGTH];
u8 hopping_frequency_no;
u8 rf_chan[RF_NUM_CHANNELS];

// LT8910 emulation functions taken from the MULTI project
// tweaked for CC2500
static u8 LT8900_buffer[64];
static u8 LT8900_buffer_start;
static u16 LT8900_buffer_overhead_bits;
static u8 LT8900_addr[8];
static u8 LT8900_addr_size;
static u8 LT8900_Preamble_Len;
static u8 LT8900_Tailer_Len;
static u8 LT8900_CRC_Initial_Data;
static u8 LT8900_Flags;
#define LT8900_CRC_ON 6
#define LT8900_SCRAMBLE_ON 5
#define LT8900_PACKET_LENGTH_EN 4
#define LT8900_DATA_PACKET_TYPE_1 3
#define LT8900_DATA_PACKET_TYPE_0 2
#define LT8900_FEC_TYPE_1 1
#define LT8900_FEC_TYPE_0 0

static void LT8900_Config(u8 preamble_len, u8 trailer_len, u8 flags, u8 crc_init)
{
    //Preamble 1 to 8 bytes
    LT8900_Preamble_Len = preamble_len;
    //Trailer 4 to 18 bits
    LT8900_Tailer_Len = trailer_len;
    //Flags
    // CRC_ON: 1 on, 0 off
    // SCRAMBLE_ON: 1 on, 0 off
    // PACKET_LENGTH_EN: 1 1st byte of payload is payload size
    // DATA_PACKET_TYPE: 00 NRZ, 01 Manchester, 10 8bit/10bit line code, 11 interleave data type
    // FEC_TYPE: 00 No FEC, 01 FEC13, 10 FEC23, 11 reserved
    LT8900_Flags = flags;
    //CRC init constant
    LT8900_CRC_Initial_Data = crc_init;
}

static void LT8900_SetChannel(u8 channel)
{
    if (channel > 85)
        channel = 85;
    // channel spacing is 333.25 MHz
    CC2500_WriteReg(CC2500_0A_CHANNR, channel * 3);
}

static void LT8900_BuildOverhead()
{
    uint8_t pos, i;

    // Build overhead
    // preamble
    memset(LT8900_buffer, LT8900_addr[0] & 0x01 ? 0xAA : 0x55, LT8900_Preamble_Len /* -1 */);
    pos = LT8900_Preamble_Len /* -1 */;
    // address
    for (i = 0; i < LT8900_addr_size; i++)
    {
        LT8900_buffer[pos] = bit_reverse(LT8900_addr[i]);
        pos++;
    }
    // trailer
    memset(LT8900_buffer + pos, (LT8900_buffer[pos - 1] & 0x01) == 0 ? 0xAA : 0x55, 3);
    LT8900_buffer_overhead_bits = pos * 8 + LT8900_Tailer_Len;
    // nrf address length max is 5
    //pos += LT8900_Tailer_Len / 8;
    
    //LT8900_buffer_start = pos > 5 ? 5 : pos;
    LT8900_buffer_start = 0;
}

static void LT8900_SetAddress(u8 *address, u8 addr_size)
{
    //u8 addr[5];
    u8 i;

    // Address size (SyncWord) 2 to 8 bytes, 16/32/48/64 bits
    LT8900_addr_size = addr_size;
    for (i = 0; i < addr_size; i++)
        LT8900_addr[i] = address[addr_size - 1 - i];

    // Build overhead
    LT8900_BuildOverhead();

    // Set NRF RX&TX address based on overhead content
    //NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, LT8900_buffer_start - 2);
    //for (i = 0; i < LT8900_buffer_start; i++)	// reverse bytes order
    //    addr[i] = LT8900_buffer[LT8900_buffer_start - i - 1];
    //NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, addr, LT8900_buffer_start);
    //NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, addr, LT8900_buffer_start);
}

static void LT8900_WritePayload(uint8_t* msg, uint8_t len)
{
    u16 crc = LT8900_CRC_Initial_Data, a, mask;
    u8 i, pos = 0, tmp, buffer[64], pos_final, shift;
    // Add packet len
    if (LT8900_Flags&BV(LT8900_PACKET_LENGTH_EN))
    {
        tmp = bit_reverse(len);
        buffer[pos++] = tmp;
        crc = crc16_update(crc, tmp, 8);
    }
    // Add payload
    for (i = 0; i < len; i++)
    {
        tmp = bit_reverse(msg[i]);
        buffer[pos++] = tmp;
        crc = crc16_update(crc, tmp, 8);
    }
    // Add CRC
    if (LT8900_Flags&BV(LT8900_CRC_ON))
    {
        buffer[pos++] = crc >> 8;
        buffer[pos++] = crc;
    }
    // Shift everything to fit behind the trailer (4 to 18 bits)
    shift = LT8900_buffer_overhead_bits & 0x7;
    pos_final = LT8900_buffer_overhead_bits / 8;
    mask = ~(0xFF << (8 - shift));
    LT8900_buffer[pos_final + pos] = 0xFF;
    for (i = pos - 1; i != 0xFF; i--)
    {
        a = buffer[i] << (8 - shift);
        LT8900_buffer[pos_final + i] = (LT8900_buffer[pos_final + i] & mask >> 8) | a >> 8;
        LT8900_buffer[pos_final + i + 1] = (LT8900_buffer[pos_final + i + 1] & mask) | a;
    }
    if (shift)
        pos++;
    // Send everything
    CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, LT8900_buffer + LT8900_buffer_start, pos_final + pos - LT8900_buffer_start);
    CC2500_Strobe(CC2500_STX);
}

// end of LT8910 emulation

static void send_packet(u8 bind)
{
    if (bind) {
        memcpy(packet, (u8*)"\x1d\x55\x3f\x45\x49\x50\x11\x20\x00\x00\x57\x00\x00\xaa\xe4", 15);
    }
    else {
        memcpy(packet, (u8*)"\x1d\x01\x64\x64\x64\x00\xaa\x00\x00\x00\x00\x00\x00\x85\xe4", 15);
    }
    // frequency hopping
    LT8900_SetChannel(bind ? RF_BIND_CHANNEL : rf_chan[hopping_frequency_no++ % RF_NUM_CHANNELS]);
    // halt Tx/Rx
    CC2500_Strobe(CC2500_SIDLE);
    // flush tx FIFO
    CC2500_Strobe(CC2500_SFTX);
    // send packet
    LT8900_WritePayload(packet, PACKET_SIZE);

    // Keep transmit power updated
    if (tx_power != Model.tx_power) {
        tx_power = Model.tx_power;
        CC2500_SetPower(tx_power);
    }
}

static u16 jjrch31_callback()
{
    if (fine != (s8)Model.proto_opts[PROTOOPTS_FREQFINE]) {
        fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
        CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    }

    if (counter > 0) {
        counter--;
        send_packet(1);
        if (counter == 1)
        {
            rx_tx_addr[0] = 0x1d;
            rx_tx_addr[1] = 0xe4;
            LT8900_SetAddress(rx_tx_addr, ADDRESS_LENGTH);
            PROTOCOL_SetBindState(0);
        }
    }
    else {
        send_packet(0);
    }
    return PACKET_PERIOD;
}

static void jjrch31_init()
{
    //
    // Rf settings for CC2500
    //
    // Base Frequency = 2402.000000 (same as LT8910)
    // CRC Enable = false
    // Channel Spacing = 333.251953
    // Data Format = Normal mode (FIFO)
    // Data Rate = 62.4847
    // Deviation = 228.515625 (same as LT8910)
    // Manchester Enable = false
    // Modulated = true
    // Modulation Format = GFSK
    // Packet Length Mode = Variable packet length mode.Packet length configured by the first byte after sync word
    // Sync Word Qualifier Mode = No preamble / sync
    // Whitening = false
    
    CC2500_Reset();
    CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x00);  // Packet Automation Control
    CC2500_WriteReg(CC2500_08_PKTCTRL0, 0x01);  // Packet Automation Control
    CC2500_WriteReg(CC2500_0B_FSCTRL1,  0x0B);  // Frequency Synthesizer Control 
    CC2500_WriteReg(CC2500_0D_FREQ2,    0x5C);  // Frequency Control Word, High Byte 
    CC2500_WriteReg(CC2500_0E_FREQ1,    0x62);  // Frequency Control Word, Middle Byte 
    CC2500_WriteReg(CC2500_0F_FREQ0,    0x76);  // Frequency Control Word, Low Byte 
    CC2500_WriteReg(CC2500_10_MDMCFG4,  0x7B);  // Modem Configuration 
    CC2500_WriteReg(CC2500_11_MDMCFG3,  0x3B);  // Modem Configuration 
    CC2500_WriteReg(CC2500_12_MDMCFG2,  0x90);  // Modem Configuration
    CC2500_WriteReg(CC2500_13_MDMCFG1,  0x03);  // Modem Configuration
    CC2500_WriteReg(CC2500_14_MDMCFG0,  0xA4);  // Modem Configuration 
    CC2500_WriteReg(CC2500_15_DEVIATN,  0x71);  // Modem Deviation Setting 
    CC2500_WriteReg(CC2500_18_MCSM0,    0x18);  // Main Radio Control State Machine Configuration 
    CC2500_WriteReg(CC2500_19_FOCCFG,   0x16);  // Frequency Offset Compensation Configuration
    CC2500_WriteReg(CC2500_1B_AGCCTRL2, 0x43);  // AGC Control
    CC2500_WriteReg(CC2500_25_FSCAL1,   0x00);  // Frequency Synthesizer Calibration 
    CC2500_WriteReg(CC2500_26_FSCAL0,   0x11);  // Frequency Synthesizer Calibration 

    CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    CC2500_SetPower(tx_power);
    CC2500_SetTxRxMode(TX_EN);
    
    // LT8910 settings
    // 3 byte preamble, 4 bit trailer, CRC init = 0, size of packet = 1st byte of payload
    LT8900_Config(3, 4, BV(LT8900_CRC_ON) | BV(LT8900_PACKET_LENGTH_EN), 0x00);
    // bind syncword
    memcpy(rx_tx_addr, (u8*)"\x03\x80\x47\x43", ADDRESS_LENGTH);
    LT8900_SetAddress(rx_tx_addr, ADDRESS_LENGTH);
}

static void initialize_txid()
{
    rf_chan[0] = 0x2d;
    rf_chan[1] = 0x35;
    rf_chan[2] = 0x3c;
    rf_chan[3] = 0x46;
}

static void initialize()
{
    CLOCK_StopTimer();
    tx_power = Model.tx_power;
    fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
    counter = BIND_COUNT;
    initialize_txid();
    jjrch31_init();
    PROTOCOL_SetBindState(BIND_COUNT * PACKET_PERIOD / 1000);
    CLOCK_StartTimer(INITIAL_WAIT, jjrch31_callback);
}

uintptr_t JJRCH31_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
    case PROTOCMD_INIT:  initialize(); return 0;
    case PROTOCMD_DEINIT:
    case PROTOCMD_RESET:
        CLOCK_StopTimer();
        return (CC2500_Reset() ? 1 : -1);
    case PROTOCMD_CHECK_AUTOBIND: return 1;
    case PROTOCMD_BIND:  initialize(); return 0;
    case PROTOCMD_NUMCHAN: return 10;
    case PROTOCMD_DEFAULT_NUMCHAN: return 10;
    case PROTOCMD_CURRENT_ID: return Model.fixed_id;
    case PROTOCMD_GETOPTIONS: return (uintptr_t)jjrch31_opts;
    case PROTOCMD_TELEMETRYSTATE: return PROTO_TELEM_UNSUPPORTED;
    case PROTOCMD_CHANNELMAP: return AETRG;
    default: break;
    }
    return 0;
}

#endif
