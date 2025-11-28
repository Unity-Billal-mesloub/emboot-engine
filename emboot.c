/**
 * Copyright (c) 2024, liujitong, <sulfurandcu@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// rt-baremetal
// fal
// ymodem
// hpatchlite
// qled -> ikLed
// nr_micro_shell -> embush

#include <rtconfig.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <emboot.h>
#include <fal.h>
#include <ymodem.h>

#include <hpatch_impl.h>
#include <qled.h>
#include <nr_micro_shell.h>

#ifndef EMBOOT_HPATCH_CATCH_SIZE
#define EMBOOT_HPATCH_CATCH_SIZE        1024
#endif

#ifndef EMBOOT_DECOMPRESS_CACHE_SIZE
#define EMBOOT_DECOMPRESS_CACHE_SIZE    1024
#endif

#ifndef EMBOOT_CRC_POLY
#define EMBOOT_CRC_POLY                 0x04C11DB7          // CRC-32/MPEG-2
#endif
#ifndef EMBOOT_CRC_INIT
#define EMBOOT_CRC_INIT                 0xFFFFFFFF          // CRC-32/MPEG-2
#endif
#ifndef EMBOOT_MOV_ADDR
#define EMBOOT_MOV_ADDR                 1024                // copy the emboot header to the upctrl partition, offset xxx bytes.
#endif
#ifndef EMBOOT_MAX_TRYS
#define EMBOOT_MAX_TRYS                 2
#endif

#ifndef EMBOOT_MSP_MASK
#define EMBOOT_MSP_MASK                 0x00000000
#endif

#ifndef EMBOOT_MSP_DATA
#define EMBOOT_MSP_DATA                 0x20000000
#endif

#ifndef EMBOOT_APP_MASK
#define EMBOOT_APP_MASK                 0x00000000
#endif

#ifndef EMBOOT_APP_DATA
#define EMBOOT_APP_DATA                 0x00000000
#endif

#ifndef EMBOOT_UPCTRL_PART
#define EMBOOT_UPCTRL_PART              "update"
#endif

#ifndef EMBOOT_RUNAPP_PART
#define EMBOOT_RUNAPP_PART              "runapp"
#endif

#ifndef EMBOOT_BACKUP_PART
#define EMBOOT_BACKUP_PART              "backup"
#endif

#ifndef EMBOOT_DECODE_PART
#define EMBOOT_DECODE_PART              "decode"
#endif

#ifndef EMBOOT_EXPORT
#define EMBOOT_EXPORT(cmd, func)        NR_SHELL_CMD_EXPORT(cmd, func)
#endif

#ifndef emboot_env_init
#define emboot_env_init()               do {\
                                            shell_init();\
                                            fal_init();\
                                        } while(0)
#endif

#ifndef emboot_env_fini
#define emboot_env_fini()               do {\
                                            rt_device_control(rt_console_get_device(), RT_DEVICE_CTRL_CLOSE, RT_NULL); \
                                        } while(0)
#endif

#ifndef emboot_bash
#define emboot_bash(c)                  shell(c)
#endif

#ifndef emboot_printf_i
#define emboot_printf_i(fmt, args...)   rt_kprintf(fmt, ##args);
#endif
#ifndef emboot_printf_e
#define emboot_printf_e(fmt, args...)   rt_kprintf(fmt, ##args);
#endif
#ifndef emboot_printf_d
#define emboot_printf_d(fmt, args...)   rt_kprintf(fmt, ##args);
#endif

void emboot_key_init(void)
{

}

#ifdef EMBOOT_LED_PIN
static rt_tick_t emboot_led_tick = 0;
static int emboot_led_mark = 0;
#endif

void emboot_led_init(void)
{
#ifdef EMBOOT_LED_PIN
    qled_init();

    qled_add(EMBOOT_LED_PIN, 0);
    qled_set_blink(EMBOOT_LED_PIN, 100, 100);
    emboot_led_mark = 1;
#endif
}

void emboot_led_task(void)
{
#ifdef EMBOOT_LED_PIN
    if (!emboot_led_mark)
    {
        return;
    }

    if (rt_tick_get() - emboot_led_tick >= QLED_TIME_UNIT_MS)
    {
        qled_run();
        emboot_led_tick = rt_tick_get();
    }
#endif
}

void emboot_led_fast(void)
{
#ifdef EMBOOT_LED_PIN
    qled_set_blink(EMBOOT_LED_PIN, 50, 50);
#endif
}

#ifdef EMBOOT_WDT_PIN
static int emboot_wdt_mark = 0;
static int emboot_wdt_data = 0;
#endif

static rt_device_t emboot_wdt = RT_NULL;

void emboot_wdt_init(void)
{
#ifdef EMBOOT_WDT_PIN
    rt_pin_mode(EMBOOT_WDT_PIN, PIN_MODE_OUTPUT);
    emboot_wdt_mark = 1;
#endif

#ifdef EMBOOT_WDT_DEV
    emboot_wdt = rt_device_find(EMBOOT_WDT_DEV);
    if (!emboot_wdt) return;
    rt_device_init(emboot_wdt);
#endif
}

void emboot_wdt_feed(void)
{
#ifdef EMBOOT_WDT_PIN
    if (!emboot_wdt_mark) return;
    rt_pin_write(EMBOOT_WDT_PIN, emboot_wdt_data = !emboot_wdt_data);
#endif

#ifdef EMBOOT_WDT_DEV
    if (!emboot_wdt) return;
    rt_device_control(emboot_wdt, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
#endif
}

typedef struct hpatch_handle_t
{
    hpatchi_listener_t parent;

    int patch_file_offset;
    int patch_file_length;
    int newer_file_length;

    int patch_file_rd_pos;
    int newer_file_wr_pos;

} hpatch_handle_t;

typedef int (*emboot_get_t)(unsigned int addr, unsigned char *data, unsigned int size);
typedef int (*emboot_set_t)(unsigned int addr, unsigned char *data, unsigned int size);

static unsigned char emboot_copy_buffer[1024];
static unsigned char emboot_head_buffer[1024];
static unsigned char emboot_ctrl_buffer[__update_zone_size];

static uint32_t embcrc_table[256];
static uint32_t embcrc_mark;

static void embcrc_init(void)
{
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t crc = i << 24;
        for (int k = 0; k < 8; k++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ EMBOOT_CRC_POLY;
            }
            else
            {
                crc <<= 1;
            }
        }
        embcrc_table[i] = crc;
    }
}

static uint32_t embcrc(const uint8_t *data, size_t len, uint32_t crc)
{
    if (!embcrc_mark)
    {
        embcrc_mark = 1;
        embcrc_init();
    }

    while (len--)
    {
        crc = (crc << 8) ^ embcrc_table[((crc >> 24) ^ *data) & 0xFF];
        data++;
    }

    return crc;
}

static int emboot_calc_hash(int remain, int offset, emboot_get_t embget)
{
    int blkmax = sizeof(emboot_copy_buffer);
    int blklen;
    int pkglen = remain;
    int crcval = EMBOOT_CRC_INIT;

    emboot_printf_i("00%%");
    while (remain > 0)
    {
        int percent = offset * 100 / pkglen;
        if (percent % 5 == 0 && percent < 100)
        {
            emboot_printf_i("\b\b\b%02d%%", percent);
        }

        blklen = remain > blkmax ? blkmax : remain;
        embget(offset, emboot_copy_buffer, blklen);
        crcval = embcrc(emboot_copy_buffer, blklen, crcval);
        offset += blklen;
        remain -= blklen;
    }
    emboot_printf_i("\b\b\b100%% ");

    return crcval;
}

static int emboot_copy_data(int remain, int offset, emboot_get_t embget, emboot_set_t embset)
{
    int blkmax = sizeof(emboot_copy_buffer);
    int blklen;
    int pkglen = remain;

    emboot_printf_i("00%%");
    while (remain > 0)
    {
        int percent = offset * 100 / pkglen;
        if (percent % 5 == 0 && percent < 100)
        {
            emboot_printf_i("\b\b\b%02d%%", percent);
        }

        blklen = remain > blkmax ? blkmax : remain;
        embget(offset, emboot_copy_buffer, blklen);
        embset(offset, emboot_copy_buffer, blklen);
        offset += blklen;
        remain -= blklen;
    }
    emboot_printf_i("\b\b\b100%% ");
    emboot_printf_d("(copied size = 0x%08X) ", pkglen);

    return 0;
}

int emboot_upctrl_erase(void) { return fal_partition_erase_all(fal_partition_find(EMBOOT_UPCTRL_PART)); }
int emboot_runapp_erase(void) { return fal_partition_erase_all(fal_partition_find(EMBOOT_RUNAPP_PART)); }
int emboot_backup_erase(void) { return fal_partition_erase_all(fal_partition_find(EMBOOT_BACKUP_PART)); }
int emboot_decode_erase(void) { return fal_partition_erase_all(fal_partition_find(EMBOOT_DECODE_PART)); }

int emboot_upctrl_read (unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_read (fal_partition_find(EMBOOT_UPCTRL_PART), addr, data, size); }
int emboot_runapp_read (unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_read (fal_partition_find(EMBOOT_RUNAPP_PART), addr, data, size); }
int emboot_backup_read (unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_read (fal_partition_find(EMBOOT_BACKUP_PART), addr, data, size); }
int emboot_decode_read (unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_read (fal_partition_find(EMBOOT_DECODE_PART), addr, data, size); }

int emboot_upctrl_write(unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_write(fal_partition_find(EMBOOT_UPCTRL_PART), addr, data, size); }
int emboot_runapp_write(unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_write(fal_partition_find(EMBOOT_RUNAPP_PART), addr, data, size); }
int emboot_backup_write(unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_write(fal_partition_find(EMBOOT_BACKUP_PART), addr, data, size); }
int emboot_decode_write(unsigned int addr, unsigned char *data, unsigned int size) { return fal_partition_write(fal_partition_find(EMBOOT_DECODE_PART), addr, data, size); }

int embget_runapp_size(void)
{
    const struct fal_partition *part = fal_partition_find(EMBOOT_RUNAPP_PART);
    if (part == RT_NULL)
    {
        return 0;
    }
    return part->len;
}

int embget_runapp_data(void)
{
    int data;
    emboot_runapp_read(0, (uint8_t *)&data, sizeof(data));
    return data;
}

int embset_update_step(emboot_step_t step, int erase)
{
    if (erase)
    {
        emboot_ctrl_t *emboot_ctrl = (emboot_ctrl_t *)emboot_ctrl_buffer;
        emboot_upctrl_read(0, emboot_ctrl_buffer, sizeof(emboot_ctrl_buffer));
        emboot_ctrl->update_step = step;
        emboot_upctrl_erase();
        emboot_upctrl_write(0, emboot_ctrl_buffer, sizeof(emboot_ctrl_buffer));
    }
    else
    {
        emboot_ctrl_t emboot_ctrl = {0};
        emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
        emboot_ctrl.update_step = step;
        emboot_upctrl_write(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    }
    return 0;
}

int embget_update_step(void)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return emboot_ctrl.update_step;
}

int embset_update_stay(int stay)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    emboot_ctrl.update_stay = stay;
    emboot_upctrl_write(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return 0;
}

int embget_update_stay(void)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    int stay = (int)emboot_ctrl.update_stay != -1 && (int)emboot_ctrl.update_stay != 0;
    if (stay)
    {
        embset_update_stay(0);
    }
    return stay;
}

int embset_patchi_indx(int index)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    emboot_ctrl.patchi_indx = index;
    emboot_upctrl_write(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return 0;
}

int embget_patchi_indx(void)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return emboot_ctrl.patchi_indx;
}

int embset_backup_info(uint32_t size, uint32_t hash)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    emboot_ctrl.backup_size = size;
    emboot_ctrl.backup_hash = hash;
    emboot_upctrl_write(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return 0;
}

int embset_decode_info(uint32_t size, uint32_t hash)
{
    emboot_ctrl_t emboot_ctrl = {0};
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    emboot_ctrl.decode_size = size;
    emboot_ctrl.decode_hash = hash;
    emboot_upctrl_write(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));
    return 0;
}

hpi_BOOL hpatch_stream_read_empty(struct hpatchi_listener_t *listener, hpi_pos_t addr, hpi_byte *data, hpi_size_t size)
{
    memset(data, 0, size);
    return hpi_TRUE;
}

hpi_BOOL hpatch_stream_read_old(struct hpatchi_listener_t *listener, hpi_pos_t addr, hpi_byte *data, hpi_size_t size)
{
    int result = emboot_runapp_read(addr, data, size);
    if (result < 0) { return hpi_FALSE; }
    return hpi_TRUE;
}

hpi_BOOL hpatch_stream_read_patch(hpi_TInputStreamHandle input_stream, hpi_byte *data, hpi_size_t *size)
{
    hpatch_handle_t *hpatch = (hpatch_handle_t *)input_stream;

    if ((hpatch->patch_file_rd_pos + *size) > hpatch->patch_file_length)
    {
        *size = hpatch->patch_file_length - hpatch->patch_file_rd_pos;
    }

    int result = emboot_backup_read(hpatch->patch_file_offset + hpatch->patch_file_rd_pos, data, *size);
    if (result < 0) { return hpi_FALSE; }
    hpatch->patch_file_rd_pos += *size;
    return hpi_TRUE;
}

hpi_BOOL hpatch_stream_write_new(struct hpatchi_listener_t *listener, const hpi_byte *data, hpi_size_t size)
{
    hpatch_handle_t *hpatch = (hpatch_handle_t *)listener;

    int percent = hpatch->newer_file_wr_pos * 100 / hpatch->newer_file_length;
    if (percent % 5 == 0 && percent < 100)
    {
        emboot_printf_i("\b\b\b%02d%%", percent);
    }

    int result = emboot_decode_write(hpatch->newer_file_wr_pos, (unsigned char *)data, size);
    if (result < 0) { return hpi_FALSE; }
    hpatch->newer_file_wr_pos += size;
    return hpi_TRUE;
}

int emboot_verify_precheck(void)
{
    emboot_head_t *emboot_head = (emboot_head_t *)emboot_head_buffer;
    int first8B = sizeof(emboot_head->header_size) + sizeof(emboot_head->header_hash);

    emboot_backup_read(0, (uint8_t *)emboot_head, first8B);
    emboot_printf_i("precheck package head: ");
    if (emboot_head->header_size > sizeof(emboot_head_buffer))
    {
        emboot_printf_i("error size!\n");
        return -1;
    }
    emboot_backup_read(0, (uint8_t *)emboot_head, emboot_head->header_size);
    if (emboot_head->header_hash != embcrc((uint8_t *)emboot_head + first8B, emboot_head->header_size - first8B, EMBOOT_CRC_INIT))
    {
        emboot_printf_i("error hash!\n");
        return -1;
    }
    emboot_printf_i("ok!\n");

    emboot_printf_i("precheck package body: ");
    if (emboot_head->remain_hash != emboot_calc_hash(emboot_head->remain_size, emboot_head->header_size, emboot_backup_read))
    {
        emboot_printf_i("error hash!\n");
        return -1;
    }
    emboot_printf_i("ok!\n");

    int err = 0;
    int crc = 0;

    emboot_printf_i("\n");
    emboot_printf_i("verify\n");
    emboot_printf_i("######\n");

retry_precheck_dnload:
    emboot_printf_i("verify [dnload/backup] ");
    if (emboot_head->remain_hash != (crc = emboot_calc_hash(emboot_head->remain_size, emboot_head->header_size, emboot_backup_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect remain size = 0x%08X]\n", emboot_head->remain_size);
        emboot_printf_d("@DEBUG [expect remain hash = 0x%08X]\n", emboot_head->remain_hash);
        emboot_printf_d("@DEBUG [actual remain hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i("\n");
            return -1;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_precheck_dnload;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
    }

    err = 0;

retry_precheck_oldapp:
    for (int i = 0; i < emboot_head->patchx_nums; ++i)
    {
        emboot_printf_i("verify [curent/runapp] ");
        emboot_printf_i("%d/%d ", i+1, emboot_head->patchx_nums);

        if (emboot_head->patchx_data[i].oldapp_size == 0x00000000 ||
            emboot_head->patchx_data[i].oldapp_size == 0xFFFFFFFF ||
            emboot_head->patchx_data[i].oldapp_hash == emboot_calc_hash(emboot_head->patchx_data[i].oldapp_size, 0, emboot_runapp_read))
        {
            emboot_printf_i("ok!\n");
            emboot_printf_i("######\n");
            emboot_printf_i("verify done!\n");
            return 0;
        }
        else
        {
            emboot_printf_i("incorrect!\n");
        }
    }

    err++;
    if (err >= EMBOOT_MAX_TRYS)
    {
        emboot_printf_i("error!\n");
        emboot_printf_i("\n");
        return -1;
    }
    else
    {
        emboot_printf_i("retry: %d\n", err);
        goto retry_precheck_oldapp;
    }

    return -1;
}

static int emboot_verify(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;

    emboot_printf_i("\n");
    emboot_printf_i("update start:\n");

    emboot_printf_i("\n");
    emboot_printf_i("verify\n");
    emboot_printf_i("######\n");

retry_verify_dnload:
    emboot_printf_i("verify [dnload/backup] ");
    if (emboot_head->remain_hash != (crc = emboot_calc_hash(emboot_head->remain_size, emboot_head->header_size, emboot_backup_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect remain size = 0x%08X]\n", emboot_head->remain_size);
        emboot_printf_d("@DEBUG [expect remain hash = 0x%08X]\n", emboot_head->remain_hash);
        emboot_printf_d("@DEBUG [actual remain hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_verify_dnload;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
    }

    err = 0;

retry_verify_oldapp:
    for (int i = 0; i < emboot_head->patchx_nums; ++i)
    {
        emboot_printf_i("verify [curent/runapp] ");
        emboot_printf_i("%d/%d ", i+1, emboot_head->patchx_nums);

        if (emboot_head->patchx_data[i].oldapp_size == 0x00000000 ||
            emboot_head->patchx_data[i].oldapp_size == 0xFFFFFFFF ||
            emboot_head->patchx_data[i].oldapp_hash == emboot_calc_hash(emboot_head->patchx_data[i].oldapp_size, 0, emboot_runapp_read))
        {
            emboot_printf_i("ok!\n");
            embset_patchi_indx(i);
            embset_update_step(emboot_step_decode, 0);
            emboot_printf_i("######\n");
            emboot_printf_i("verify done! ");

            if (emboot_head->patchx_data[i].patchi_type == patchi_type_full_image)
            {
                emboot_printf_i("(this is a full update image)\n"); // package = emboot_header + main.bin
            }
            if (emboot_head->patchx_data[i].patchi_type == patchi_type_full_patch)
            {
                emboot_printf_i("(this is a full update patch)\n"); // package = emboot_header + diff_with_empty.patch
            }
            if (emboot_head->patchx_data[i].patchi_type > 0)
            {
                emboot_printf_i("(this is a diff update patch)\n"); // package = emboot_header + diff_with_older.patch
            }

            // copy the emboot header to [upctrl], as the [dnload/backup] will be erased when backing up the old firmware.
            emboot_upctrl_write(EMBOOT_MOV_ADDR, (uint8_t *)emboot_head, emboot_head->header_size);
            return emboot_stat_busy;
        }
        else
        {
            emboot_printf_i("incorrect!\n");
        }
    }

    err++;
    if (err >= EMBOOT_MAX_TRYS)
    {
        emboot_printf_i("error!\n");
        emboot_printf_i(NR_SHELL_USER_NAME);
        embset_update_step(emboot_step_finish, 0);
        return emboot_stat_idle;
    }
    else
    {
        emboot_printf_i("retry: %d\n", err);
        goto retry_verify_oldapp;
    }
}

static int emboot_decode(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;
    int idx = embget_patchi_indx();

    emboot_printf_i("\n");
    emboot_printf_i("decode\n");
    emboot_printf_i("######\n");

retry_decode:
    emboot_printf_i("erases [decode/newapp]\n");
    emboot_decode_erase();

    hpatch_handle_t hpatch = {0};
    hpatch.patch_file_offset  = emboot_head->header_size + emboot_head->patchx_data[idx].patchi_addr;
    hpatch.patch_file_length     = emboot_head->patchx_data[idx].patchi_size;
    hpatch.newer_file_length     = emboot_head->patchx_data[idx].newapp_size;

    int type = emboot_head->patchx_data[idx].patchi_type;

    if (type <  0)  // full update with image file
    {
        emboot_printf_i("unpack [decode/newapp] <- [dnload/FullUpdateIMAGE] [copying...] ");
        emboot_copy_data(emboot_head->remain_size, emboot_head->header_size, emboot_backup_read, emboot_decode_write);
    }
    if (type == 0)  // full update with patch file
    {
        emboot_printf_i("hpatch [decode/newapp] <- [dnload/FullUpdatePATCH] ");
        emboot_printf_i("00%%");
        hpi_patch(&hpatch.parent, EMBOOT_HPATCH_CATCH_SIZE, EMBOOT_DECOMPRESS_CACHE_SIZE, hpatch_stream_read_patch, hpatch_stream_read_empty, hpatch_stream_write_new);
        emboot_printf_i("\b\b\b100%%\n");
    }
    if (type >  0)  // diff update with patch file
    {
        emboot_printf_i("hpatch [decode/newapp] <- [dnload/DiffUpdatePATCH] ");
        emboot_printf_i("00%%");
        hpi_patch(&hpatch.parent, EMBOOT_HPATCH_CATCH_SIZE, EMBOOT_DECOMPRESS_CACHE_SIZE, hpatch_stream_read_patch, hpatch_stream_read_old, hpatch_stream_write_new);
        emboot_printf_i("\b\b\b100%%\n");
    }

    emboot_printf_i("verify [decode/newapp] ");
    if (emboot_head->patchx_data[idx].newapp_hash != (crc = emboot_calc_hash(emboot_head->patchx_data[idx].newapp_size, 0, emboot_decode_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect newapp size = 0x%08X]\n", emboot_head->patchx_data[idx].newapp_size);
        emboot_printf_d("@DEBUG [expect newapp hash = 0x%08X]\n", emboot_head->patchx_data[idx].newapp_hash);
        emboot_printf_d("@DEBUG [actual newapp hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_decode;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
        embset_update_step(emboot_step_backup, 0);
    }

    emboot_printf_i("######\n");
    emboot_printf_i("decode done!\n");

    embset_decode_info(emboot_head->patchx_data[idx].newapp_size, emboot_head->patchx_data[idx].newapp_hash);

    return emboot_stat_busy;
}

static int emboot_backup(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    emboot_printf_i("\n");
    emboot_printf_i("backup\n");
    emboot_printf_i("######\n");

    emboot_printf_i("erases [dnload/backup]\n");
    emboot_backup_erase();

    emboot_printf_i("backup [dnload/backup] <- [curent/runapp] ");
    emboot_copy_data(embget_runapp_size(), 0, emboot_runapp_read, emboot_backup_write);
    emboot_printf_i("\n");

    emboot_printf_i("hasher [curent/runapp] ");
    int crc = emboot_calc_hash(embget_runapp_size(), 0, emboot_runapp_read);
    emboot_printf_i("\n");
    embset_backup_info(embget_runapp_size(), crc);

    embset_update_step(emboot_step_docopy, 0);
    emboot_printf_i("######\n");
    emboot_printf_i("backup done!\n");

    return emboot_stat_busy;
}

static int emboot_docopy(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;
    int idx = embget_patchi_indx();

    emboot_printf_i("\n");
    emboot_printf_i("docopy\n");
    emboot_printf_i("######\n");

retry_docopy:
    emboot_printf_i("erases [curent/runapp]\n");
    emboot_runapp_erase();

    emboot_printf_i("docopy [curent/runapp] <- [decode/newapp] ");
    emboot_copy_data(emboot_head->patchx_data[idx].newapp_size, 0, emboot_decode_read, emboot_runapp_write);
    emboot_printf_i("\n");

    emboot_printf_i("verify [curent/runapp] ");
    if (emboot_head->patchx_data[idx].newapp_hash != (crc = emboot_calc_hash(emboot_head->patchx_data[idx].newapp_size, 0, emboot_runapp_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect runapp size = 0x%08X]\n", emboot_head->patchx_data[idx].newapp_size);
        emboot_printf_d("@DEBUG [expect runapp hash = 0x%08X]\n", emboot_head->patchx_data[idx].newapp_hash);
        emboot_printf_d("@DEBUG [actual runapp hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_revert, 0);
            return emboot_stat_busy;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_docopy;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
        embset_update_step(emboot_step_finish, 0);
    }

    emboot_printf_i("######\n");
    emboot_printf_i("docopy done!\n");

    emboot_printf_i("\n");
    emboot_printf_i("update success!\n");

    return emboot_stat_done;
}

static int emboot_revert(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;

    emboot_printf_i("\n");
    emboot_printf_i("revert (undo/rollback)\n");
    emboot_printf_i("######\n");

retry_verify_backup:
    emboot_printf_i("verify [backup/oldapp] ");
    if (emboot_ctrl->backup_size == 0x00000000 ||
        emboot_ctrl->backup_size == 0xFFFFFFFF ||
        emboot_ctrl->backup_hash != (crc = emboot_calc_hash(emboot_ctrl->backup_size, 0, emboot_backup_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect backup size = 0x%08X]\n", emboot_ctrl->backup_size);
        emboot_printf_d("@DEBUG [expect backup hash = 0x%08X]\n", emboot_ctrl->backup_hash);
        emboot_printf_d("@DEBUG [actual backup hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_verify_backup;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
    }

retry_revert:
    emboot_printf_i("erases [curent/runapp]\n");
    emboot_runapp_erase();

    emboot_printf_i("revert [curent/runapp] <- [backup/oldapp] ");
    emboot_copy_data(emboot_ctrl->backup_size, 0, emboot_backup_read, emboot_runapp_write);
    emboot_printf_i("\n");

    emboot_printf_i("verify [curent/runapp] ");
    if (emboot_ctrl->backup_hash != (crc = emboot_calc_hash(emboot_ctrl->backup_size, 0, emboot_runapp_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect runapp size = 0x%08X]\n", emboot_ctrl->backup_size);
        emboot_printf_d("@DEBUG [expect runapp hash = 0x%08X]\n", emboot_ctrl->backup_hash);
        emboot_printf_d("@DEBUG [actual runapp hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_revert;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
        embset_update_step(emboot_step_finish, 0);
    }

    emboot_printf_i("######\n");
    emboot_printf_i("revert done!\n");

    return emboot_stat_done;
}

static int emboot_recopy(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;

    emboot_printf_i("\n");
    emboot_printf_i("recopy (redo/rollforward)\n");
    emboot_printf_i("######\n");

retry_verify_decode:
    emboot_printf_i("verify [decode/newapp] ");
    if (emboot_ctrl->decode_size == 0x00000000 ||
        emboot_ctrl->decode_size == 0xFFFFFFFF ||
        emboot_ctrl->decode_hash != (crc = emboot_calc_hash(emboot_ctrl->decode_size, 0, emboot_decode_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect decode size = 0x%08X]\n", emboot_ctrl->decode_size);
        emboot_printf_d("@DEBUG [expect decode hash = 0x%08X]\n", emboot_ctrl->decode_hash);
        emboot_printf_d("@DEBUG [actual decode hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_verify_decode;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
    }

retry_recopy:
    emboot_printf_i("erases [curent/runapp]\n");
    emboot_runapp_erase();

    emboot_printf_i("recopy [curent/runapp] <- [decode/newapp] ");
    emboot_copy_data(emboot_ctrl->decode_size, 0, emboot_decode_read, emboot_runapp_write);
    emboot_printf_i("\n");

    emboot_printf_i("verify [curent/runapp] ");
    if (emboot_ctrl->decode_hash != (crc = emboot_calc_hash(emboot_ctrl->decode_size, 0, emboot_runapp_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect runapp size = 0x%08X]\n", emboot_ctrl->decode_size);
        emboot_printf_d("@DEBUG [expect runapp hash = 0x%08X]\n", emboot_ctrl->decode_hash);
        emboot_printf_d("@DEBUG [actual runapp hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_recopy;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
        embset_update_step(emboot_step_finish, 0);
    }

    emboot_printf_i("######\n");
    emboot_printf_i("recopy done!\n");

    return emboot_stat_done;
}

static int emboot_rocopy(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int err = 0;
    int crc = 0;

    emboot_printf_i("\n");
    emboot_printf_i("recopy (redo/rollforward -f)\n");
    emboot_printf_i("######\n");

    emboot_printf_i("hasher [decode/newapp] ");
    int decode_hash = emboot_calc_hash(embget_runapp_size(), 0, emboot_decode_read);
    emboot_printf_i("\n");

retry_recopy:
    emboot_printf_i("erases [curent/runapp]\n");
    emboot_runapp_erase();

    emboot_printf_i("recopy [curent/runapp] <- [decode/newapp] ");
    emboot_copy_data(embget_runapp_size(), 0, emboot_decode_read, emboot_runapp_write);
    emboot_printf_i("\n");

    emboot_printf_i("verify [curent/runapp] ");
    if (decode_hash != (crc = emboot_calc_hash(embget_runapp_size(), 0, emboot_runapp_read)))
    {
        emboot_printf_i("error!\n");
        emboot_printf_d("@DEBUG [expect runapp size = 0x%08X]\n", embget_runapp_size());
        emboot_printf_d("@DEBUG [expect runapp hash = 0x%08X]\n", decode_hash);
        emboot_printf_d("@DEBUG [actual runapp hash = 0x%08X]\n", crc);
        err++;
        if (err >= EMBOOT_MAX_TRYS)
        {
            emboot_printf_i(NR_SHELL_USER_NAME);
            embset_update_step(emboot_step_finish, 0);
            return emboot_stat_idle;
        }
        else
        {
            emboot_printf_i("retry: %d\n", err);
            goto retry_recopy;
        }
    }
    else
    {
        emboot_printf_i("ok!\n");
        embset_update_step(emboot_step_finish, 0);
    }

    emboot_printf_i("######\n");
    emboot_printf_i("recopy done!\n");

    return emboot_stat_done;
}

typedef int (*method_t)(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head);

typedef struct update_t
{
    uint32_t                            step;
    method_t                            method;
} update_t;

static const update_t update[] =
{
    {emboot_step_verify, emboot_verify,},
    {emboot_step_decode, emboot_decode,}, // dnload -> decode
    {emboot_step_backup, emboot_backup,}, // runapp -> backup
    {emboot_step_docopy, emboot_docopy,}, // decode -> runapp
    {emboot_step_revert, emboot_revert,}, // backup -> runapp
    {emboot_step_recopy, emboot_recopy,}, // decode -> runapp (copy decode_size bytes)
    {emboot_step_rocopy, emboot_rocopy,}, // decode -> runapp (copy runapp_size bytes)
};

int emboot_header(emboot_ctrl_t *emboot_ctrl, emboot_head_t *emboot_head)
{
    int first8B = sizeof(emboot_head->header_size) + sizeof(emboot_head->header_hash);
    if (emboot_ctrl->update_step == emboot_step_revert ||
        emboot_ctrl->update_step == emboot_step_recopy)
    {
        // no need update header.
    }
    else
    if (emboot_ctrl->update_step == emboot_step_verify)
    {
        // get update header from [dnload/backup].
        emboot_backup_read(0, (uint8_t *)emboot_head, first8B);
        if (emboot_head->header_size > sizeof(emboot_head_buffer))
        {
            emboot_printf_e("dnload [packet:header] error size!\n");
            embset_update_step(emboot_step_finish, 0);
            return -1;
        }
        emboot_backup_read(0, (uint8_t *)emboot_head, emboot_head->header_size);
        if (emboot_head->header_hash != embcrc((uint8_t *)emboot_head + first8B, emboot_head->header_size - first8B, EMBOOT_CRC_INIT))
        {
            emboot_printf_e("dnload [packet:header] error hash!\n");
            embset_update_step(emboot_step_finish, 0);
            return -1;
        }
    }
    else
    {
        // get update header from [upctrl].
        emboot_upctrl_read(EMBOOT_MOV_ADDR, (uint8_t *)emboot_head, first8B);
        if (emboot_head->header_size > sizeof(emboot_head_buffer))
        {
            emboot_printf_e("upctrl [packet:header] error size!\n");
            embset_update_step(emboot_step_finish, 0);
            return -1;
        }
        emboot_upctrl_read(EMBOOT_MOV_ADDR, (uint8_t *)emboot_head, emboot_head->header_size);
        if (emboot_head->header_hash != embcrc((uint8_t *)emboot_head + first8B, emboot_head->header_size - first8B, EMBOOT_CRC_INIT))
        {
            emboot_printf_e("upctrl [packet:header] error hash!\n");
            embset_update_step(emboot_step_finish, 0);
            return -1;
        }
    }
    return 0;
}

int emboot_update(void)
{
    emboot_ctrl_t emboot_ctrl = {0};
    memset(&emboot_ctrl, 0xff, sizeof(emboot_ctrl_t));
    emboot_upctrl_read(0, (uint8_t *)&emboot_ctrl, sizeof(emboot_ctrl_t));

    for (int i = 0; i < sizeof(update) / sizeof(update[0]); ++i)
    {
        if (emboot_ctrl.update_step == update[i].step && update[i].method)
        {
            emboot_head_t *emboot_head = (emboot_head_t *)emboot_head_buffer;

            int result = emboot_header(&emboot_ctrl, emboot_head);
            if (result < 0)
            {
                return emboot_stat_idle;
            }

            emboot_led_fast();
            return update[i].method(&emboot_ctrl, emboot_head);
        }
    }

    return emboot_stat_idle;
}

int emboot_mark;
int emboot_over;
rt_tick_t emboot_time;

/**
 * Before jumping to the application, all multiplexed pins must be de-initialized.
 * otherwise, the pins may still be controlled by their previously assigned peripheral functions.
 */
void emboot_fini(void)
{
    emboot_env_fini();
}

rt_ssize_t emboot_getc(rt_uint8_t *data, rt_size_t size)
{
    return rt_device_read(rt_console_get_device(), 0, data, size);
}

bool emboot_wait_keyboard_input(int timeout)
{
    rt_tick_t tick = rt_tick_get();
    int idx = 0;
    uint8_t input[8] = {0};
    while (rt_tick_get() - tick <= timeout)
    {
        if (emboot_getc(&input[idx], 1) == 1)
        {
            idx++;
        }
        if (idx > 2)
        {
            break;
        }
    }

    if (((input[0] == 0x0d) && (input[1] == 0x0d)) || ((input[0] == 0x0d) && (input[1] == 0x0a)))
    {
        return true;
    }

    return false;
}

void emboot_jump_call(void)
{
    emboot_over = 1;
}

void emboot_jump(void)
{
    typedef void (*emboot_jump_t)(void);
    emboot_jump_t JumpToApplication = (emboot_jump_t)(*((__IO uint32_t *)(__runapp_zone_addr + 4)));
    uint32_t Msp4B = *((__IO uint32_t *)(__runapp_zone_addr + 0));
    uint32_t App4B = *((__IO uint32_t *)(__runapp_zone_addr + 4));

#if EMBOOT_MSP_MASK != 0
    if ((Msp4B & EMBOOT_MSP_MASK) != EMBOOT_MSP_DATA)
    {
        rt_kprintf("\nBOOTLOADER JumpTo App Failed: MSP Invalid!\n");
        return;
    }
#endif

#if EMBOOT_APP_MASK != 0
    if ((App4B & EMBOOT_APP_MASK) != EMBOOT_APP_DATA)
    {
        rt_kprintf("\nBOOTLOADER JumpTo App Failed: APP Invalid!\n");
        return;
    }
#endif

    __disable_irq();
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    for (int i = 0; i < sizeof(NVIC->ICER)/sizeof(NVIC->ICER[0]); ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    __set_CONTROL(0);
    __set_MSP(Msp4B);

    JumpToApplication();
}

void emboot_fast_boot(void)
{
    emboot_ctrl_t *ctrl = (emboot_ctrl_t *)__update_zone_addr;
    int data = *(uint32_t *)__runapp_zone_addr;
    int step = ctrl->update_step;
    int stay = ctrl->update_stay;
    int jump = (data != -1) && (step == -1 || step == 0) && (stay == -1 || stay == 0);
    if (jump)
    {
        emboot_jump();
    }
}

void emboot_full_boot(void)
{
    int data = embget_runapp_data();
    int step = embget_update_step();
    int stay = embget_update_stay();
    int jump = (data != -1) && (step == -1 || step == 0) && (!stay);
    if (jump)
    {
        rt_kprintf("\nBOOTLOADER JumpTo App!\n\n");

        emboot_fini();

        emboot_jump();
    }
}

void emboot_init(void)
{
    emboot_env_init();

    emboot_key_init();
    emboot_led_init();
    emboot_wdt_init();

    emboot_time = rt_tick_get();
    emboot_over = !(embget_update_stay() | emboot_wait_keyboard_input(1000));
}

#ifdef EMBOOT_DTM_SECTION
char const emboot_meta[][16] __attribute__((section(".emboot"))) =
{
    {
        __DATE_yyyy_mm_dd__,            // compile date (str): 2024-10-24
        0,0,
        __DATE_0xYYYYMMDD__,            // compile date (hex): 0x20241024
    },
    {
        __TIME_t_hh_mm_ss__,            // compile time (str): T.10:24:05
        0,0,
        __TIME_0x00HHMMSS__,            // compile time (hex): 0x00102405
    },
    #ifdef EMBOOT_DTM_VERSION
    {
        EMBOOT_DTM_VERSION
    },
    #endif
    #ifdef EMBOOT_DTM_RLSDATE
    {
        EMBOOT_DTM_RLSDATE
    },
    #endif
};
#endif

void emboot_info(void)
{
    rt_kprintf("\n");
#ifdef EMBOOT_DTM_VERSION
    rt_kprintf("BOOTLOADER %s (BUILD %s %s)\n", EMBOOT_DTM_VERSION, __DATE_yyyy_mm_dd__, __TIME__);
#else
    rt_kprintf("BOOTLOADER %s (BUILD %s %s)\n",                     __DATE_yyyy_mm_dd__, __TIME__);
#endif
}

void emboot_core(void)
{
    if (!emboot_mark)
    {
        emboot_init();
        emboot_info();
        emboot_mark = 1;
    }

    unsigned char c;
    if (emboot_getc(&c, 1) == 1)
    {
        emboot_bash(c);
        emboot_time = rt_tick_get();
    }

    int emboot_stat = emboot_update();
    if (emboot_stat == emboot_stat_busy)
    {
        emboot_over = 0;
        emboot_time = rt_tick_get();
    }
    if (emboot_stat == emboot_stat_done)
    {
        emboot_over = 1;
    }

    if (emboot_over || (rt_tick_get() - emboot_time > EMBOOT_RUN_TIMEOUT))
    {
        emboot_full_boot();
    }
}

void emboot_loop(void)
{
    while (1)
    {
        emboot_core();
    }
}

void emboot_tick(void)
{
    emboot_led_task();
    emboot_wdt_feed();
}

int embrym_recv_idx;

static enum rym_code embrym_recv_bgn(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    fal_partition_t part = (fal_partition_t)fal_partition_find(EMBOOT_BACKUP_PART);
    if (part == RT_NULL) return RYM_ERR_CAN;

    fal_partition_erase_all(part);

    embrym_recv_idx = 0;
    return RYM_CODE_ACK;
}

static enum rym_code embrym_recv_txt(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    fal_partition_t part = (fal_partition_t)fal_partition_find(EMBOOT_BACKUP_PART);
    if (part == RT_NULL) return RYM_ERR_CAN;

    int writeLen = fal_partition_write(part, embrym_recv_idx, buf, len);
    if (writeLen != len) return RYM_ERR_CAN;

    embrym_recv_idx += len;
    return RYM_CODE_ACK;
}

static enum rym_code embrym_recv_end(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    return RYM_CODE_ACK;
}

void embrym_recv(void)
{
    rt_device_t dev = rt_console_get_device();
    if (!dev) return;

    shell_printf("press 'u' to abort\n");

    struct rym_ctx ctx;
    rt_err_t result = rym_recv_on_device(&ctx, dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_RX_NON_BLOCKING,
                                         embrym_recv_bgn,
                                         embrym_recv_txt,
                                         embrym_recv_end, 1000);

    shell_printf("\ndownload ");
    if (result != RT_EOK)
    {
        shell_printf("fail!\n");
    }
    else
    {
        shell_printf("success!\n");
        if (emboot_verify_precheck() == 0)
        {
            emboot_upctrl_erase();
            embset_update_step(emboot_step_verify, 0);
        }
    }
}

void embcmd_reboot(char argc, char *argv)
{
    if (argc == 1)
    {
        rt_hw_cpu_reset();
    }
}

void embcmd_jump(char argc, char *argv)
{
    if (argc == 1)
    {
        emboot_jump_call();
    }
}

void embcmd_redo(char argc, char *argv)
{
    if (argc == 1)
    {
        embset_update_step(emboot_step_recopy, 1);
    }
    else
    if (argc == 2 && !strcmp("-f", &argv[(int)argv[1]]))
    {
        embset_update_step(emboot_step_rocopy, 1);
    }
}

void embcmd_undo(char argc, char *argv)
{
    if (argc == 1)
    {
        embset_update_step(emboot_step_revert, 1);
    }
}

void embcmd_download(char argc, char *argv)
{
    if (argc == 1)
    {
        embrym_recv();
    }
}

EMBOOT_EXPORT(reboot, embcmd_reboot);
EMBOOT_EXPORT(jump, embcmd_jump);
EMBOOT_EXPORT(redo, embcmd_redo);
EMBOOT_EXPORT(undo, embcmd_undo);
EMBOOT_EXPORT(download, embcmd_download);
