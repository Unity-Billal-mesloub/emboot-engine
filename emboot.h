/**
 * Copyright (c) 2024, liujitong, <sulfurandcu@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __emboot_h__
#define __emboot_h__

#include <stdbool.h>
#include <stdint.h>

typedef enum emboot_stat_t
{
    emboot_stat_idle,
    emboot_stat_done,
    emboot_stat_busy,

} emboot_stat_t;

typedef enum emboot_step_t
{
    emboot_step_verify = 0x7FFFFFFF,
    emboot_step_decode = 0x0000FFFF,
    emboot_step_backup = 0x00000FFF,
    emboot_step_docopy = 0x000000FF,
    emboot_step_revert = 0x0000000F,
    emboot_step_recopy = 0x00000007,
    emboot_step_rocopy = 0x00000003,
    emboot_step_finish = 0x00000000,
} emboot_step_t;

typedef struct emboot_ctrl_t
{
    uint32_t update_step;
    uint32_t update_stay;
    uint32_t patchi_indx;
    uint32_t backup_size;
    uint32_t backup_hash;
    uint32_t decode_size;
    uint32_t decode_hash;
} emboot_ctrl_t;

typedef enum patchi_type_t
{
    patchi_type_full_image = 0xFFFFFFFF,
    patchi_type_full_patch = 0x00000000,
    patchi_type_diff_patch = 0x00000001,
} patchi_type_t;

typedef struct patchi_data_t
{
    uint32_t                            patchi_type;        // 0xFFFFFFFF:FULL_IMAGE, 0x00000000:FULL_PATCH, 0xXXXXXXXX:DIFF_PATCH
    uint32_t                            patchi_addr;

    uint32_t                            patchi_size;
    uint32_t                            patchi_hash;

    uint32_t                            oldapp_size;        // 0xFFFFFFFF:FULL_IMAGE, 0x00000000:FULL_PATCH, 0xXXXXXXXX:DIFF_PATCH
    uint32_t                            oldapp_hash;

    uint32_t                            newapp_size;
    uint32_t                            newapp_hash;

} patchi_data_t;

typedef struct emboot_head_t
{
    uint32_t                            header_size;
    uint32_t                            header_hash;        // skip first 8 byte
    uint32_t                            remain_size;
    uint32_t                            remain_hash;

    uint32_t                            header_code;
    uint32_t                            device_code;
    uint32_t                            patchx_size;
    uint32_t                            patchx_nums;

    uint32_t                            Reserved_C1;
    uint32_t                            Reserved_C2;
    uint32_t                            Reserved_C3;
    uint32_t                            Reserved_C4;

    uint32_t                            Reserved_D1;
    uint32_t                            Reserved_D2;
    uint32_t                            Reserved_D3;
    uint32_t                            Reserved_D4;

    patchi_data_t                       patchx_data[];

} emboot_head_t;

void emboot_core(void);
void emboot_loop(void);
void emboot_tick(void);
void emboot_fast_boot(void);


#define __MONH__    ((__DATE__[0]+__DATE__[1]+__DATE__[2]) == 281 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 269 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 288 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 291 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 295 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 301 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 299 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 285 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 296 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 294 ? '1' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 307 ? '1' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 268 ? '1' \
                    : '0')
#define __MONL__    ((__DATE__[0]+__DATE__[1]+__DATE__[2]) == 281 ? '1' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 269 ? '2' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 288 ? '3' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 291 ? '4' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 295 ? '5' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 301 ? '6' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 299 ? '7' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 285 ? '8' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 296 ? '9' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 294 ? '0' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 307 ? '1' \
                    :(__DATE__[0]+__DATE__[1]+__DATE__[2]) == 268 ? '2' \
                    : '0')
#define __DAYH__    (__DATE__[0x04] == ' ' ? '0' : __DATE__[0x04])
#define __DAYL__    (__DATE__[0x05])

#define __DATE_yh__ __DATE__[0x07], __DATE__[0x08]
#define __DATE_yl__ __DATE__[0x09], __DATE__[0x0A]
#define __DATE_mm__ __MONH__,       __MONL__
#define __DATE_dd__ __DAYH__,       __DAYL__
#define __TIME_hh__ __TIME__[0x00], __TIME__[0x01]
#define __TIME_mm__ __TIME__[0x03], __TIME__[0x04]
#define __TIME_ss__ __TIME__[0x06], __TIME__[0x07]

#define __DATE_YH__ (((__DATE__[0x07] - '0') << 4) | (__DATE__[0x08] - '0'))
#define __DATE_YL__ (((__DATE__[0x09] - '0') << 4) | (__DATE__[0x0A] - '0'))
#define __DATE_MM__ (((__MONH__       - '0') << 4) | (__MONL__       - '0'))
#define __DATE_DD__ (((__DAYH__       - '0') << 4) | (__DAYL__       - '0'))
#define __TIME_HH__ (((__TIME__[0x00] - '0') << 4) | (__TIME__[0x01] - '0'))
#define __TIME_MM__ (((__TIME__[0x03] - '0') << 4) | (__TIME__[0x04] - '0'))
#define __TIME_SS__ (((__TIME__[0x06] - '0') << 4) | (__TIME__[0x07] - '0'))

#define __DATE_yyyy_mm_dd__ __DATE_yh__, __DATE_yl__, '-', __DATE_mm__, '-', __DATE_dd__
#define __TIME_t_hh_mm_ss__ 'T', '.',    __TIME_hh__, ':', __TIME_mm__, ':', __TIME_ss__

#define __DATE_0xYYYYMMDD__ __DATE_YH__, __DATE_YL__, __DATE_MM__, __DATE_DD__
#define __TIME_0x00HHMMSS__         'T', __TIME_HH__, __TIME_MM__, __TIME_SS__

#endif /* __emboot_h__ */
