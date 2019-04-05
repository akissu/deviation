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
#include "pages.h"
#include "config/model.h"

static struct xn297dump_obj * const gui = &gui_objs.u.xn297dump;

#include "../common/_xn297dump_page.c"

static const char *mode_cb(guiObject_t *obj, int dir, void *data)
{
    (void)obj;
    (void)data;
    xn297dump.mode = GUI_TextSelectHelper(xn297dump.mode, 0, 3, dir, 1, 1, NULL);
    if (xn297dump.mode > XN297DUMP_2MBPS)
        xn297dump.mode = XN297DUMP_OFF;
    _dump_enable(xn297dump.mode);
    
    const void * modelbl[4] = { "Off", "1M", "250K", "2M" };
    return modelbl[xn297dump.mode];
}

static const char *channel_cb(guiObject_t *obj, int dir, void *data)
{
    (void)obj;
    (void)data;
    xn297dump.channel = GUI_TextSelectHelper(xn297dump.channel, 0, 125, dir, 1, 10, NULL);
    snprintf(tempstring, 7, "Ch %d", xn297dump.channel);
    memset(xn297dump.packet, 0, sizeof(xn297dump.packet));  // clear old packet data
    return tempstring;
}

static const char *pktlen_cb(guiObject_t *obj, int dir, void *data)
{
    (void)obj;
    (void)data;
    xn297dump.pkt_len = GUI_TextSelectHelper(xn297dump.pkt_len, 1, 32, dir, 1, 5, NULL);
    snprintf(tempstring, 7, "Len %d", xn297dump.pkt_len);
    memset(xn297dump.packet, 0, sizeof(xn297dump.packet));  // clear old packet data
    return tempstring;
}

static void _draw_page()
{
    PAGE_ShowHeader(PAGE_GetName(PAGEID_XN297DUMP));
    GUI_CreateTextSelectPlate(&gui->mode, 0, HEADER_HEIGHT, 40, LINE_HEIGHT, &TEXTSEL_FONT, NULL, mode_cb, NULL);
    GUI_CreateTextSelectPlate(&gui->channel, LCD_WIDTH/2 - 23, HEADER_HEIGHT, 42, LINE_HEIGHT, &TEXTSEL_FONT, scan_cb, channel_cb, NULL);
    GUI_CreateTextSelectPlate(&gui->pkt_len, LCD_WIDTH - 44, HEADER_HEIGHT, 44, LINE_HEIGHT, &TEXTSEL_FONT, NULL, pktlen_cb, NULL);
    for (int i = 0; i < 4; i++) {
        GUI_CreateLabelBox(&gui->packetdata[i], 0, HEADER_HEIGHT + 14 + 7 * i, LCD_WIDTH, 7, &TINY_FONT, packetdata_cb, NULL, (void *)(uintptr_t)i);
    }
    GUI_CreateLabelBox(&gui->status, 0, HEADER_HEIGHT + 43, LCD_WIDTH, 7, &TINY_FONT, status_cb, NULL, NULL);
}
