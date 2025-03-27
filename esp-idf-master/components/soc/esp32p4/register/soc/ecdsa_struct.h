/**
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/** Group: Data Memory */

/** Group: Configuration registers */
/** Type of conf register
 *  ECDSA configure register
 */
typedef union {
    struct {
        /** work_mode : R/W; bitpos: [1:0]; default: 0;
         *  The work mode bits of ECDSA Accelerator. 0: Signature Verify Mode. 1: Signature
         *  Generate Mode. 2: Export Public Key Mode. 3: invalid.
         */
        uint32_t work_mode:2;
        /** ecc_curve : R/W; bitpos: [2]; default: 0;
         *  The ecc curve select bit of ECDSA Accelerator.  0: P-192.  1: P-256.
         */
        uint32_t ecc_curve:1;
        /** software_set_k : R/W; bitpos: [3]; default: 0;
         *  The source of k select bit. 0: k is automatically generated by hardware. 1: k is
         *  written by software.
         */
        uint32_t software_set_k:1;
        /** software_set_z : R/W; bitpos: [4]; default: 0;
         *  The source of z select bit. 0: z is generated from SHA result. 1: z is written by
         *  software.
         */
        uint32_t software_set_z:1;
        /** deterministic_k : R/W; bitpos: [5]; default: 0;
         *  The source of hardware generated k. 0: k is generated by TRNG. 1: k is generated by
         *  deterministic derivation algorithm.
         */
        uint32_t deterministic_k:1;
        /** deterministic_loop : R/W; bitpos: [21:6]; default: 0;
         *  The (loop number - 1) value in the deterministic derivation algorithm to derive k.
         */
        uint32_t deterministic_loop:16;
        uint32_t reserved_22:10;
    };
    uint32_t val;
} ecdsa_conf_reg_t;

/** Type of start register
 *  ECDSA start register
 */
typedef union {
    struct {
        /** start : WT; bitpos: [0]; default: 0;
         *  Write 1 to start calculation of ECDSA Accelerator. This bit will be self-cleared
         *  after configuration.
         */
        uint32_t start:1;
        /** load_done : WT; bitpos: [1]; default: 0;
         *  Write 1 to input load done signal of ECDSA Accelerator. This bit will be
         *  self-cleared after configuration.
         */
        uint32_t load_done:1;
        /** get_done : WT; bitpos: [2]; default: 0;
         *  Write 1 to input get done signal of ECDSA Accelerator. This bit will be
         *  self-cleared after configuration.
         */
        uint32_t get_done:1;
        uint32_t reserved_3:29;
    };
    uint32_t val;
} ecdsa_start_reg_t;


/** Group: Clock and reset registers */
/** Type of clk register
 *  ECDSA clock gate register
 */
typedef union {
    struct {
        /** clk_gate_force_on : R/W; bitpos: [0]; default: 0;
         *  Write 1 to force on register clock gate.
         */
        uint32_t clk_gate_force_on:1;
        uint32_t reserved_1:31;
    };
    uint32_t val;
} ecdsa_clk_reg_t;


/** Group: Interrupt registers */
/** Type of int_raw register
 *  ECDSA interrupt raw register, valid in level.
 */
typedef union {
    struct {
        /** calc_done_int_raw : RO/WTC/SS; bitpos: [0]; default: 0;
         *  The raw interrupt status bit  for the ecdsa_calc_done_int interrupt
         */
        uint32_t calc_done_int_raw:1;
        /** sha_release_int_raw : RO/WTC/SS; bitpos: [1]; default: 0;
         *  The raw interrupt status bit  for the ecdsa_sha_release_int interrupt
         */
        uint32_t sha_release_int_raw:1;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_int_raw_reg_t;

/** Type of int_st register
 *  ECDSA interrupt status register.
 */
typedef union {
    struct {
        /** calc_done_int_st : RO; bitpos: [0]; default: 0;
         *  The masked interrupt status bit  for the ecdsa_calc_done_int interrupt
         */
        uint32_t calc_done_int_st:1;
        /** sha_release_int_st : RO; bitpos: [1]; default: 0;
         *  The masked interrupt status bit  for the ecdsa_sha_release_int interrupt
         */
        uint32_t sha_release_int_st:1;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_int_st_reg_t;

/** Type of int_ena register
 *  ECDSA interrupt enable register.
 */
typedef union {
    struct {
        /** calc_done_int_ena : R/W; bitpos: [0]; default: 0;
         *  The interrupt enable bit  for the ecdsa_calc_done_int interrupt
         */
        uint32_t calc_done_int_ena:1;
        /** sha_release_int_ena : R/W; bitpos: [1]; default: 0;
         *  The interrupt enable bit  for the ecdsa_sha_release_int interrupt
         */
        uint32_t sha_release_int_ena:1;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_int_ena_reg_t;

/** Type of int_clr register
 *  ECDSA interrupt clear register.
 */
typedef union {
    struct {
        /** calc_done_int_clr : WT; bitpos: [0]; default: 0;
         *  Set this bit to clear the ecdsa_calc_done_int interrupt
         */
        uint32_t calc_done_int_clr:1;
        /** sha_release_int_clr : WT; bitpos: [1]; default: 0;
         *  Set this bit to clear the ecdsa_sha_release_int interrupt
         */
        uint32_t sha_release_int_clr:1;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_int_clr_reg_t;


/** Group: Status registers */
/** Type of state register
 *  ECDSA status register
 */
typedef union {
    struct {
        /** busy : RO; bitpos: [1:0]; default: 0;
         *  The status bits of ECDSA Accelerator. ECDSA is at 0: IDLE, 1: LOAD, 2: GET, 3: BUSY
         *  state.
         */
        uint32_t busy:2;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_state_reg_t;


/** Group: Result registers */
/** Type of result register
 *  ECDSA result register
 */
typedef union {
    struct {
        /** operation_result : RO/SS; bitpos: [0]; default: 0;
         *  The operation result bit of ECDSA Accelerator, only valid when ECDSA calculation is
         *  done.
         */
        uint32_t operation_result:1;
        /** k_value_warning : RO/SS; bitpos: [1]; default: 0;
         *  The k value warning bit of ECDSA Accelerator, valid when k value is bigger than the
         *  curve order, then actually taken k = k mod n.
         */
        uint32_t k_value_warning:1;
        uint32_t reserved_2:30;
    };
    uint32_t val;
} ecdsa_result_reg_t;


/** Group: SHA register */
/** Type of sha_mode register
 *  ECDSA control SHA register
 */
typedef union {
    struct {
        /** sha_mode : R/W; bitpos: [2:0]; default: 0;
         *  The work mode bits of SHA Calculator in ECDSA Accelerator. 1: SHA-224. 2: SHA-256.
         *  Others: invalid.
         */
        uint32_t sha_mode:3;
        uint32_t reserved_3:29;
    };
    uint32_t val;
} ecdsa_sha_mode_reg_t;

/** Type of sha_start register
 *  ECDSA control SHA register
 */
typedef union {
    struct {
        /** sha_start : WT; bitpos: [0]; default: 0;
         *  Write 1 to start the first calculation of SHA Calculator in ECDSA Accelerator. This
         *  bit will be self-cleared after configuration.
         */
        uint32_t sha_start:1;
        uint32_t reserved_1:31;
    };
    uint32_t val;
} ecdsa_sha_start_reg_t;

/** Type of sha_continue register
 *  ECDSA control SHA register
 */
typedef union {
    struct {
        /** sha_continue : WT; bitpos: [0]; default: 0;
         *  Write 1 to start the latter calculation of SHA Calculator in ECDSA Accelerator. This
         *  bit will be self-cleared after configuration.
         */
        uint32_t sha_continue:1;
        uint32_t reserved_1:31;
    };
    uint32_t val;
} ecdsa_sha_continue_reg_t;

/** Type of sha_busy register
 *  ECDSA status register
 */
typedef union {
    struct {
        /** sha_busy : RO; bitpos: [0]; default: 0;
         *  The busy status bit of SHA Calculator in ECDSA Accelerator. 1:SHA is in
         *  calculation. 0: SHA is idle.
         */
        uint32_t sha_busy:1;
        uint32_t reserved_1:31;
    };
    uint32_t val;
} ecdsa_sha_busy_reg_t;


/** Group: Version register */
/** Type of date register
 *  Version control register
 */
typedef union {
    struct {
        /** date : R/W; bitpos: [27:0]; default: 36716656;
         *  ECDSA version control register
         */
        uint32_t date:28;
        uint32_t reserved_28:4;
    };
    uint32_t val;
} ecdsa_date_reg_t;


typedef struct {
    uint32_t reserved_000;
    volatile ecdsa_conf_reg_t conf;
    volatile ecdsa_clk_reg_t clk;
    volatile ecdsa_int_raw_reg_t int_raw;
    volatile ecdsa_int_st_reg_t int_st;
    volatile ecdsa_int_ena_reg_t int_ena;
    volatile ecdsa_int_clr_reg_t int_clr;
    volatile ecdsa_start_reg_t start;
    volatile ecdsa_state_reg_t state;
    volatile ecdsa_result_reg_t result;
    uint32_t reserved_028[53];
    volatile ecdsa_date_reg_t date;
    uint32_t reserved_100[64];
    volatile ecdsa_sha_mode_reg_t sha_mode;
    uint32_t reserved_204[3];
    volatile ecdsa_sha_start_reg_t sha_start;
    volatile ecdsa_sha_continue_reg_t sha_continue;
    volatile ecdsa_sha_busy_reg_t sha_busy;
    uint32_t reserved_21c[25];
    volatile uint32_t message[8];
    uint32_t reserved_2a0[472];
    volatile uint32_t r[8];
    volatile uint32_t s[8];
    volatile uint32_t z[8];
    volatile uint32_t qax[8];
    volatile uint32_t qay[8];
} ecdsa_dev_t;


#ifndef __cplusplus
_Static_assert(sizeof(ecdsa_dev_t) == 0xaa0, "Invalid size of ecdsa_dev_t structure");
#endif

#ifdef __cplusplus
}
#endif
