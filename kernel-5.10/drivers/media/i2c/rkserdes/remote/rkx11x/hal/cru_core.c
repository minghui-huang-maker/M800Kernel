// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Rockchip Electronics Co. Ltd.
 *
 * Author: Joseph Chen <chenjh@rock-chips.com>
 */

#include "cru_core.h"

/********************* Private MACRO Definition ******************************/
#define REG_X4(i) ((i) * 4)

#define PWRDOWN_SHIFT      13
#define PWRDOWN_MASK       (1 << PWRDOWN_SHIFT)
#define PLL_POSTDIV1_SHIFT 12
#define PLL_POSTDIV1_MASK  (0x7 << PLL_POSTDIV1_SHIFT)
#define PLL_FBDIV_SHIFT    0
#define PLL_FBDIV_MASK     (0xfff << PLL_FBDIV_SHIFT)
#define PLL_POSTDIV2_SHIFT 6
#define PLL_POSTDIV2_MASK  (0x7 << PLL_POSTDIV2_SHIFT)
#define PLL_REFDIV_SHIFT   0
#define PLL_REFDIV_MASK    (0x3f << PLL_REFDIV_SHIFT)
#define PLL_DSMPD_SHIFT    12
#define PLL_DSMPD_MASK     (1 << PLL_DSMPD_SHIFT)
#define PLL_FRAC_SHIFT     0
#define PLL_FRAC_MASK      (0xffffff << PLL_FRAC_SHIFT)

#define EXPONENT_OF_FRAC_PLL 24
#define RK_PLL_MODE_SLOW     0
#define RK_PLL_MODE_NORMAL   1
#define RK_PLL_MODE_DEEP     2
#define MHZ                  1000000

#define PLL_GET_PLLMODE(val, shift, mask) \
        (((uint32_t)(val) & mask) >> shift)
#define PLL_GET_FBDIV(x) \
        (((uint32_t)(x) & (PLL_FBDIV_MASK)) >> PLL_FBDIV_SHIFT)
#define PLL_GET_REFDIV(x) \
        (((uint32_t)(x) & (PLL_REFDIV_MASK)) >> PLL_REFDIV_SHIFT)
#define PLL_GET_POSTDIV1(x) \
        (((uint32_t)(x) & (PLL_POSTDIV1_MASK)) >> PLL_POSTDIV1_SHIFT)
#define PLL_GET_POSTDIV2(x) \
        (((uint32_t)(x) & (PLL_POSTDIV2_MASK)) >> PLL_POSTDIV2_SHIFT)
#define PLL_GET_DSMPD(x) \
        (((uint32_t)(x) & (PLL_DSMPD_MASK)) >> PLL_DSMPD_SHIFT)
#define PLL_GET_FRAC(x) \
        (((uint32_t)(x) & (PLL_FRAC_MASK)) >> PLL_FRAC_SHIFT)
#define CRU_PLL_ROUND_UP_TO_KHZ(x) \
        (HAL_DIV_ROUND_UP((x), 1000) * 1000)

static struct PLL_CONFIG g_PllCfg_AutoTable;

/********************* Private Function Definition ***************************/
uint32_t RKX11x_HAL_CRU_CORE_Read(struct hwclk *hw, uint32_t reg)
{
    uint32_t val;
    int ret;

    ret = hw->xfer.read(hw->xfer.client, reg, &val);
    if (ret) {
        CRU_ERR("%s: read reg=0x%08x failed, ret=%d\n", hw->name, reg, ret);
    }

    CRU_DBGR("%s: %s: reg=0x%08x, val=0x%08x\n", __func__, hw->name, reg, val);

    return val;
}

uint32_t RKX11x_HAL_CRU_CORE_Write(struct hwclk *hw, uint32_t reg, uint32_t val)
{
    int ret;

    CRU_DBGR("%s: %s: reg=0x%08x, val=0x%08x\n", __func__, hw->name, reg, val);

    ret = hw->xfer.write(hw->xfer.client, reg, val);
    if (ret) {
        CRU_ERR("%s: write reg=0x%08x, val=0x%08x failed, ret=%d\n",
                hw->name, reg, val, ret);
    }

    return ret;
}

uint32_t RKX11x_HAL_CRU_CORE_WriteMask(struct hwclk *hw, uint32_t reg,
                                       uint32_t msk, uint32_t val)
{
    return RKX11x_HAL_CRU_CORE_Write(hw, reg, VAL_MASK_WE(msk, val));
}

static int isBetterFreq(uint32_t now, uint32_t new, uint32_t best)
{
    return (new <= now && new > best);
}

int RKX11x_HAL_CRU_CORE_FreqGetMux4(struct hwclk *hw,
                                    uint32_t freq, uint32_t freq0,
                                    uint32_t freq1, uint32_t freq2, uint32_t freq3)
{
    uint32_t best = 0;

    if (isBetterFreq(freq, freq0, best)) {
        best = freq0;
    }

    if (isBetterFreq(freq, freq1, best)) {
        best = freq1;
    }

    if (isBetterFreq(freq, freq2, best)) {
        best = freq2;
    }

    if (isBetterFreq(freq, freq3, best)) {
        best = freq3;
    }

    if (best == freq0) {
        return 0;
    } else if (best == freq1) {
        return 1;
    } else if (best == freq2) {
        return 2;
    } else if (best == freq3) {
        return 3;
    }

    return HAL_INVAL;
}

int RKX11x_HAL_CRU_CORE_FreqGetMux3(struct hwclk *hw,
                                    uint32_t freq, uint32_t freq0,
                                    uint32_t freq1, uint32_t freq2)
{
    return RKX11x_HAL_CRU_CORE_FreqGetMux4(hw, freq, freq0, freq1, freq2, freq2);
}

int RKX11x_HAL_CRU_CORE_FreqGetMux2(struct hwclk *hw,
                                    uint32_t freq, uint32_t freq0, uint32_t freq1)
{
    return RKX11x_HAL_CRU_CORE_FreqGetMux4(hw, freq, freq0, freq1, freq1, freq1);
}

uint32_t RKX11x_HAL_CRU_CORE_MuxGetFreq4(struct hwclk *hw, uint32_t muxName,
                                         uint32_t freq0, uint32_t freq1,
                                         uint32_t freq2, uint32_t freq3)
{
    switch (RKX11x_HAL_CRU_CORE_ClkGetMux(hw, muxName)) {
    case 0:
        return freq0;

    case 1:
        return freq1;

    case 2:
        return freq2;

    case 3:
        return freq3;
    }

    return HAL_INVAL;
}

uint32_t RKX11x_HAL_CRU_CORE_MuxGetFreq3(struct hwclk *hw, uint32_t muxName,
                                         uint32_t freq0, uint32_t freq1, uint32_t freq2)
{
    return RKX11x_HAL_CRU_CORE_MuxGetFreq4(hw, muxName, freq0, freq1, freq2, freq2);
}

uint32_t RKX11x_HAL_CRU_CORE_MuxGetFreq2(struct hwclk *hw, uint32_t muxName,
                                         uint32_t freq0, uint32_t freq1)
{
    return RKX11x_HAL_CRU_CORE_MuxGetFreq4(hw, muxName, freq0, freq1, freq1, freq1);
}

int RKX11x_HAL_CRU_CORE_RoundFreqGetMux4(struct hwclk *hw, uint32_t freq,
                                         uint32_t pFreq0, uint32_t pFreq1,
                                         uint32_t pFreq2, uint32_t pFreq3, uint32_t *pFreqOut)
{
    uint32_t mux;

    if (pFreq3 && (pFreq3 % freq == 0)) {
        *pFreqOut = pFreq3;
        mux = 3;
    } else if (pFreq2 && (pFreq2 % freq == 0)) {
        *pFreqOut = pFreq2;
        mux = 2;
    } else if (pFreq1 % freq == 0) {
        *pFreqOut = pFreq1;
        mux = 1;
    } else {
        *pFreqOut = pFreq0;
        mux = 0;
    }

    return mux;
}

int RKX11x_HAL_CRU_CORE_RoundFreqGetMux3(struct hwclk *hw, uint32_t freq,
                                         uint32_t pFreq0, uint32_t pFreq1,
                                         uint32_t pFreq2, uint32_t *pFreqOut)
{
    return RKX11x_HAL_CRU_CORE_RoundFreqGetMux4(hw, freq, pFreq0, pFreq1, pFreq2, 0, pFreqOut);
}

int RKX11x_HAL_CRU_CORE_RoundFreqGetMux2(struct hwclk *hw, uint32_t freq,
                                         uint32_t pFreq0, uint32_t pFreq1, uint32_t *pFreqOut)
{
    return RKX11x_HAL_CRU_CORE_RoundFreqGetMux4(hw, freq, pFreq0, pFreq1, 0, 0, pFreqOut);
}

/******************************************************************************/
uint32_t RKX11x_HAL_CRU_CORE_GetPllFreq(struct hwclk *hw, struct PLL_SETUP *pSetup)
{
    uint32_t refDiv, fbDiv, postdDv1, postDiv2, frac, dsmpd;
    uint32_t mode = 0, rate = OSC_24M;

    mode = PLL_GET_PLLMODE(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->modeOffset),
                           pSetup->modeShift,
                           pSetup->modeMask);

    switch (mode) {
    case RK_PLL_MODE_SLOW:
        rate = OSC_24M;
        break;
    case RK_PLL_MODE_NORMAL:
        postdDv1 = PLL_GET_POSTDIV1(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset0));
        fbDiv = PLL_GET_FBDIV(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset0));
        postDiv2 = PLL_GET_POSTDIV2(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset1));
        refDiv = PLL_GET_REFDIV(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset1));
        dsmpd = PLL_GET_DSMPD(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset1));
        frac = PLL_GET_FRAC(RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset2));
        rate = (rate / refDiv) * fbDiv;
        if (dsmpd == 0) {
            uint64_t fracRate = OSC_24M;

            fracRate *= frac;
            fracRate = fracRate >> EXPONENT_OF_FRAC_PLL;
            fracRate = fracRate / refDiv;
            rate += fracRate;
        }
        rate = rate / (postdDv1 * postDiv2);
        rate = CRU_PLL_ROUND_UP_TO_KHZ(rate);
        break;
    case RK_PLL_MODE_DEEP:
    default:
        rate = 32768;
        break;
    }

    return rate;
}

static uint32_t CRU_Gcd(uint32_t m, uint32_t n)
{
    int t;

    while (m > 0) {
        if (n > m) {
            t = m;
            m = n;
            n = t;
        }
        m -= n;
    }

    return n;
}

static uint64_t HAL_DivU64Rem(uint64_t numerator, uint32_t denominator, uint32_t *pRemainder)
{
    uint64_t remainder = numerator;
    uint64_t b = denominator;
    uint64_t result;
    uint64_t d = 1;
    uint32_t high = numerator >> 32;

    result = 0;
    if (high >= denominator) {
        high /= denominator;
        result = (uint64_t)high << 32;
        remainder -= (uint64_t)(high * denominator) << 32;
    }

    while ((int64_t)b > 0 && b < remainder) {
        b = b + b;
        d = d + d;
    }

    do {
        if (remainder >= b) {
            remainder -= b;
            result += d;
        }
        b >>= 1;
        d >>= 1;
    } while (d);

    if (pRemainder) {
        *pRemainder = remainder;
    }

    return result;
}

static inline uint64_t HAL_DivU64(uint64_t numerator, uint32_t denominator)
{
    return HAL_DivU64Rem(numerator, denominator, HAL_NULL);
}

static const struct PLL_CONFIG *CRU_CORE_PllSetByAuto(struct hwclk *hw,
                                                      struct PLL_SETUP *pSetup,
                                                      uint32_t finHz,
                                                      uint32_t foutHz)
{
    struct PLL_CONFIG *rateTable = &g_PllCfg_AutoTable;
    uint32_t refDiv, fbDiv, dsmpd;
    uint32_t postDiv1, postDiv2;
    uint32_t clkGcd = 0;
    uint64_t foutVco;
    uint64_t intVco;
    uint64_t frac64;

    CRU_DBGF("%s: %s: pll=%d, refdiv: [%u, %u], foutVco: [%u, %u], fout: [%u, %u]\n",
             __func__, hw->name, pSetup->id, pSetup->minRefdiv, pSetup->maxRefdiv,
             pSetup->minVco, pSetup->maxVco, pSetup->minFout, pSetup->maxFout);

    if (finHz == 0 || foutHz == 0 || foutHz == finHz) {
        return HAL_NULL;
    }

    if ((foutHz < pSetup->minFout) || (foutHz > pSetup->maxFout)) {
        return HAL_NULL;
    }

    for (postDiv1 = 1; postDiv1 <= 7; postDiv1++) {
        for (postDiv2 = 1; postDiv2 <= 7; postDiv2++) {
            if (postDiv1 < postDiv2) {
                CRU_DBGF("%s: %s: pll=%d, Invalid postDiv1(%u) < postDiv2(%u)\n",
                         __func__, hw->name, pSetup->id, postDiv1, postDiv2);
                continue;
            }

            foutVco = (uint64_t)foutHz * postDiv1 * postDiv2;
            if ((foutVco < pSetup->minVco) || (foutVco > pSetup->maxVco)) {
                CRU_DBGF("%s: %s: pll=%d, Invalid foutVco(%llu), min_max[%u, %u]\n",
                         __func__, hw->name, pSetup->id, foutVco, pSetup->minVco, pSetup->maxVco);
                continue;
            }

            intVco = foutVco / MHZ * MHZ;
            clkGcd = CRU_Gcd(finHz, intVco);
            refDiv = finHz / clkGcd;
            fbDiv = intVco / clkGcd;

            if ((refDiv < pSetup->minRefdiv) || (refDiv > pSetup->maxRefdiv)) {
                CRU_DBGF("%s: %s: pll=%d, Invalid refDiv(%u), min_max[%u, %u]\n",
                         __func__, hw->name, pSetup->id, refDiv, pSetup->minRefdiv, pSetup->maxRefdiv);
                continue;
            }

            if (foutHz / MHZ * MHZ == foutHz) {
                dsmpd = 1;
                frac64 = 0;
            } else {
                frac64 = (foutVco % MHZ) * refDiv;
                frac64 = HAL_DivU64(frac64 << EXPONENT_OF_FRAC_PLL, OSC_24M);
                if (frac64 > 0) {
                    dsmpd = 0;
                } else {
                    dsmpd = 1;
                }
            }

            if (dsmpd && !(fbDiv >= 16 && fbDiv <= 2500)) {
                CRU_DBGF("%s: %s: pll=%d, Invalid fbDiv(%u) on int mode, min_max[16, 2500]\n",
                         __func__, hw->name, pSetup->id, fbDiv);
                continue;
            }
            if (!dsmpd && !(fbDiv >= 20 && fbDiv <= 500)) {
                CRU_DBGF("%s: %s: pll=%d, Invalid fbDiv(%u) on frac mode, min_max[20, 500]\n",
                         __func__, hw->name, pSetup->id, fbDiv);
                continue;
            }
            if (frac64 > 0x00ffffff) {
                CRU_DBGF("%s: %s: pll=%d, Invalid frac64(0x%llx) over max 0x00ffffff\n",
                         __func__, hw->name, pSetup->id, frac64);
                continue;
            }

            rateTable->refDiv = refDiv;
            rateTable->fbDiv = fbDiv;
            rateTable->postDiv1 = postDiv1;
            rateTable->postDiv2 = postDiv2;
            rateTable->dsmpd = dsmpd;
            rateTable->frac = frac64;

            return rateTable;
        }
    }

    return HAL_NULL;
}

static const struct PLL_CONFIG *CRU_CORE_PllGetSettings(struct hwclk *hw,
                                                        struct PLL_SETUP *pSetup,
                                                        uint32_t rate)
{
    const struct PLL_CONFIG *rateTable = pSetup->rateTable;

    if (!rateTable) {
        CRU_ERR("%s: %s: Unavailable pll=%d rate table\n", __func__, hw->name, pSetup->id);

        return HAL_NULL;
    }

    while (rateTable->rate) {
        if (rateTable->rate == rate) {
            return rateTable;
        }
        rateTable++;
    }

    rateTable = CRU_CORE_PllSetByAuto(hw, pSetup, OSC_24M, rate);
    if (!rateTable) {
        CRU_ERR("%s: %s: Unsupported pll=%d rate %d\n", __func__, hw->name, pSetup->id, rate);

        return HAL_NULL;
    } else {
        CRU_MSG("%s: Auto PLL=%d: (%d, %d, %d, %d, %d, %d, %d)\n",
                hw->name, pSetup->id, rate, rateTable->refDiv, rateTable->fbDiv,
                rateTable->postDiv1, rateTable->postDiv2,
                rateTable->dsmpd, rateTable->frac);
    }

    return rateTable;
}

/*
 * Force PLL into slow mode
 * Pll Power down
 * Pll Config fbDiv, refDiv, postdDv1, postDiv2, dsmpd, frac
 * Pll Power up
 * Waiting for pll lock
 * Force PLL into normal mode
 */
HAL_Status RKX11x_HAL_CRU_CORE_SetPllFreq(struct hwclk *hw, struct PLL_SETUP *pSetup, uint32_t rate)
{
    const struct PLL_CONFIG *pConfig;
    int delay = 2400;

    if (rate == RKX11x_HAL_CRU_CORE_GetPllFreq(hw, pSetup)) {
        return HAL_OK;
    }

    pConfig = CRU_CORE_PllGetSettings(hw, pSetup, rate);
    if (!pConfig) {
        return HAL_ERROR;
    }

    /* Force PLL into slow mode to ensure output stable clock */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->modeOffset, pSetup->modeMask, RK_PLL_MODE_SLOW << pSetup->modeShift);

    /* Pll Power down */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PWRDOWN_MASK, 1 << PWRDOWN_SHIFT);

    /* Pll Config */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PLL_POSTDIV2_MASK, pConfig->postDiv2 << PLL_POSTDIV2_SHIFT);
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PLL_REFDIV_MASK, pConfig->refDiv << PLL_REFDIV_SHIFT);
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset0, PLL_POSTDIV1_MASK, pConfig->postDiv1 << PLL_POSTDIV1_SHIFT);
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset0, PLL_FBDIV_MASK, pConfig->fbDiv << PLL_FBDIV_SHIFT);
    if (pSetup->sscgEn) {
        RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PLL_DSMPD_MASK, 0 << PLL_DSMPD_SHIFT);
    } else {
        RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PLL_DSMPD_MASK, pConfig->dsmpd << PLL_DSMPD_SHIFT);
    }

    if (pConfig->frac) {
        RKX11x_HAL_CRU_CORE_Write(hw, pSetup->conOffset2, (RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset2) & 0xff000000) | pConfig->frac);
    }

    /* Pll Power up */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PWRDOWN_MASK, 0 << PWRDOWN_SHIFT);

    /* Waiting for pll lock */
    while (delay > 0) {
        if (pSetup->stat0) {
            if (RKX11x_HAL_CRU_CORE_Read(hw, pSetup->stat0) & (1 << pSetup->lockShift)) {
                break;
            }
        } else {
            if (RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset1) & (1 << pSetup->lockShift)) {
                break;
            }
        }
        HAL_DelayUs(2);
        delay--;
    }
    if (delay == 0) {
        return HAL_TIMEOUT;
    }

    /* Force PLL into normal mode */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->modeOffset, pSetup->modeMask, RK_PLL_MODE_NORMAL << pSetup->modeShift);

    return HAL_OK;
}

HAL_Status RKX11x_HAL_CRU_CORE_SetPllPowerUp(struct hwclk *hw, struct PLL_SETUP *pSetup)
{
    int delay = 2400;

    /* Pll Power up */
    RKX11x_HAL_CRU_CORE_WriteMask(hw, pSetup->conOffset1, PWRDOWN_MASK, 0 << PWRDOWN_SHIFT);

    /* Waiting for pll lock */
    while (delay > 0) {
        if (pSetup->stat0) {
            if (RKX11x_HAL_CRU_CORE_Read(hw, pSetup->stat0) & (1 << pSetup->lockShift)) {
                break;
            }
        } else {
            if (RKX11x_HAL_CRU_CORE_Read(hw, pSetup->conOffset1) & (1 << pSetup->lockShift)) {
                break;
            }
        }
        HAL_DelayUs(1000);
        delay--;
    }
    if (delay == 0) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

HAL_Status RKX11x_HAL_CRU_CORE_SetPllPowerDown(struct hwclk *hw, struct PLL_SETUP *pSetup)
{
    RKX11x_HAL_CRU_CORE_Write(hw, pSetup->conOffset1, VAL_MASK_WE(PWRDOWN_MASK, 1 << PWRDOWN_SHIFT));

    return HAL_OK;
}

HAL_Check RKX11x_HAL_CRU_CORE_ClkIsEnabled(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_GATE_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_GATE_GET_BITS_SHIFT(clk);

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_FALSE;

    return (HAL_Check)((RKX11x_HAL_CRU_CORE_Read(hw, hw->gate_con0 + REG_X4(index)) & (1 << shift)) >> shift);
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkEnable(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_GATE_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_GATE_GET_BITS_SHIFT(clk);
    uint32_t gid = 16 * index + shift;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    if (gid >= hw->num_gate) {
        CRU_ERR("%s: %s: clock(#%08x) is unsupported, gid=%d\n", __func__, hw->name, clk, gid);

        return HAL_INVAL;
    }

    if (hw->gate[gid].enable_count == 0) {
        CRU_DBGF("%s: %s: clock(#%08x), gid=%d\n", __func__, hw->name, clk, gid);
        RKX11x_HAL_CRU_CORE_Write(hw, hw->gate_con0 + REG_X4(index), VAL_MASK_WE(1U << shift, 0U << shift));
    }

    hw->gate[gid].enable_count++;

    return HAL_OK;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkDisable(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_GATE_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_GATE_GET_BITS_SHIFT(clk);
    uint32_t gid = 16 * index + shift;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    if (gid >= hw->num_gate) {
        CRU_ERR("%s: %s: clock(#%08x) is unsupported\n", __func__, hw->name, clk);

        return HAL_INVAL;
    }

    if (hw->gate[gid].enable_count == 0) {
        CRU_ERR("%s: %s: clock(#%08x) is already disabled\n", __func__, hw->name, clk);

        return HAL_OK;
    }

    hw->gate[gid].enable_count--;
    if (hw->gate[gid].enable_count == 0) {
        CRU_DBGF("%s: %s: clock(#%08x), gid=%d\n", __func__, hw->name, clk, gid);
        RKX11x_HAL_CRU_CORE_Write(hw, hw->gate_con0 + REG_X4(index), VAL_MASK_WE(1U << shift, 1U << shift));
    }

    return HAL_OK;
}

HAL_Check RKX11x_HAL_CRU_CORE_ClkIsReset(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_GATE_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_GATE_GET_BITS_SHIFT(clk);

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_FALSE;

    return (HAL_Check)((RKX11x_HAL_CRU_CORE_Read(hw, hw->softrst_con0 + REG_X4(index)) & (1 << shift)) >> shift);
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkResetAssert(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_RESET_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_RESET_GET_BITS_SHIFT(clk);

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    RKX11x_HAL_CRU_CORE_Write(hw, hw->softrst_con0 + REG_X4(index), VAL_MASK_WE(1U << shift, 1U << shift));

    return HAL_OK;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkResetDeassert(struct hwclk *hw, uint32_t clk)
{
    uint32_t index = CLK_RESET_GET_REG_OFFSET(clk);
    uint32_t shift = CLK_RESET_GET_BITS_SHIFT(clk);

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    RKX11x_HAL_CRU_CORE_Write(hw, hw->softrst_con0 + REG_X4(index), VAL_MASK_WE(1U << shift, 0U << shift));

    return HAL_OK;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkSetDiv(struct hwclk *hw, uint32_t divName, uint32_t divValue)
{
    uint32_t shift, mask, index;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    index = CLK_DIV_GET_REG_OFFSET(divName);
    shift = CLK_DIV_GET_BITS_SHIFT(divName);
    mask = CLK_DIV_GET_MASK(divName);
    if (divValue > mask) {
        divValue = mask;
    }

    CRU_DBGF("%s: %s: clock(#%08x), selcon%d=0x%08x, shift=%d, mask=0x%x, div=%d\n",
             __func__, hw->name, divName, index, hw->sel_con0 + REG_X4(index), shift, mask, divValue);

    RKX11x_HAL_CRU_CORE_Write(hw, hw->sel_con0 + REG_X4(index), VAL_MASK_WE(mask, (divValue - 1U) << shift));

    return HAL_OK;
}

uint32_t RKX11x_HAL_CRU_CORE_ClkGetDiv(struct hwclk *hw, uint32_t divName)
{
    uint32_t shift, mask, index, divValue;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    index = CLK_DIV_GET_REG_OFFSET(divName);
    shift = CLK_DIV_GET_BITS_SHIFT(divName);
    mask = CLK_DIV_GET_MASK(divName);

    divValue = ((RKX11x_HAL_CRU_CORE_Read(hw, hw->sel_con0 + REG_X4(index)) & mask) >> shift) + 1;

    return divValue;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkSetMux(struct hwclk *hw, uint32_t muxName, uint32_t muxValue)
{
    uint32_t shift, mask, index;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    index = CLK_MUX_GET_REG_OFFSET(muxName);
    shift = CLK_MUX_GET_BITS_SHIFT(muxName);
    mask = CLK_MUX_GET_MASK(muxName);

    CRU_DBGF("%s: %s: clock(#%08x), selcon%d=0x%08x, shift=%d, mask=0x%x, mux=%d\n",
             __func__, hw->name, muxName, index, hw->sel_con0 + REG_X4(index), shift, mask, muxValue);

    RKX11x_HAL_CRU_CORE_Write(hw, hw->sel_con0 + REG_X4(index), VAL_MASK_WE(mask, muxValue << shift));

    return HAL_OK;
}

uint32_t RKX11x_HAL_CRU_CORE_ClkGetMux(struct hwclk *hw, uint32_t muxName)
{
    uint32_t shift, mask, index, muxValue;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    index = CLK_MUX_GET_REG_OFFSET(muxName);
    shift = CLK_MUX_GET_BITS_SHIFT(muxName);
    mask = CLK_MUX_GET_MASK(muxName);

    muxValue = (RKX11x_HAL_CRU_CORE_Read(hw, hw->sel_con0 + REG_X4(index)) & mask) >> shift;

    return muxValue;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkSetTestout(struct hwclk *hw, uint32_t clockName,
                                             uint32_t muxValue, uint32_t divValue)
{
    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    if (hw->ops->clkInitTestout) {
        hw->ops->clkInitTestout(hw, clockName, muxValue, divValue);
    }

    return HAL_OK;
}

void RKX11x_HAL_CRU_CORE_ClkDumpTree(struct hwclk *hw)
{
    struct clkTable *c;
    const char *parent;
    uint32_t clkMux, mux;
    uint32_t freq;

    if (!hw || (hw->type == CLK_UNDEF)) {
        return;
    }

    if (!hw->ops || !hw->ops->clkTable) {
        return;
    }

    if (!(hw->flags & CLK_FLG_CRU_INITED)) {
        CRU_ERR("%s: CRU is Uninitialized\n", __func__);
        return;
    }

    CRU_MSG("  ================== %s ==================\n", hw->name);
    CRU_MSG("  Name                           Rate:hz      Parent\n");
    CRU_MSG("  ====================================================\n");

    for (c = hw->ops->clkTable; c->name; c++) {
        if (c->type == DUMP_INT) {
            if (c->numParents == 1) {
                mux = 0;
            } else {
                clkMux = CLK_GET_MUX(c->clk);
                mux = RKX11x_HAL_CRU_CORE_ClkGetMux(hw, clkMux);
            }
            freq = RKX11x_HAL_CRU_CORE_ClkGetFreq(hw, c->clk);
            parent = c->parents[mux];
        } else if (c->type == DUMP_EXT) {
            freq = c->getFreq(hw, c->clk);
            parent = c->extParent ? c->extParent : "ext-in";
        } else {
            continue;
        }

        CRU_MSG("  %-30s %10d   %s\n", c->name, freq, parent);
    }
    CRU_MSG("\n");
}

HAL_Status RKX11x_HAL_CRU_CORE_SetGlbSrst(struct hwclk *hw, eCRU_GlbSrstType type)
{
    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    if (type == GLB_SRST_FST) {
        RKX11x_HAL_CRU_CORE_Write(hw, hw->gbl_srst_fst, GLB_SRST_FST);
    }

    return HAL_INVAL;
}

uint32_t RKX11x_HAL_CRU_CORE_ClkGetFreq(struct hwclk *hw, uint32_t clockName)
{
    uint32_t rate;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    rate = hw->ops->clkGetFreq(hw, clockName);

    CRU_DBGF("%s: %s: clock(#%08x) get rate: %d\n", __func__, hw->name, clockName, rate);

    return rate;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkSetFreq(struct hwclk *hw, uint32_t clockName, uint32_t rate)
{
    HAL_Status ret;
    uint32_t now;

    if (!(hw->flags & CLK_FLG_CRU_INITED))
        return HAL_ERROR;

    CRU_DBGF("%s: %s: clock(#%08x) set rate: %d\n", __func__, hw->name, clockName, rate);

    ret = hw->ops->clkSetFreq(hw, clockName, rate);
    if (hw->flags & CLK_FLG_SET_RATE_VERIFY) {
        now = hw->ops->clkGetFreq(hw, clockName);
        if (rate != now) {
            CRU_MSG("%s: Set rate %d, but %d returned\n",
                    hw->name, rate, now);
            ret = HAL_ERROR;
        }
    }

    return ret;
}

HAL_Status RKX11x_HAL_CRU_CORE_Register(struct hwclk *hw, struct xferclk *xfer)
{
    if (xfer == NULL || hw == NULL) {
        return HAL_INVAL;
    }

    if (!xfer->read || !xfer->write || !xfer->client || !xfer->name[0]) {
        return HAL_INVAL;
    }

    if (xfer->type == CLK_UNDEF || xfer->type >= CLK_MAX) {
        return HAL_INVAL;
    }

    memset(hw, 0, sizeof(struct hwclk));
    hw->type = xfer->type;
    memcpy(&hw->xfer, xfer, sizeof(struct xferclk));

    switch (xfer->type) {
    case CLK_RKX110:
        if (xfer->version == RKX110_SER_CLK) {
            hw->ops = &rkx110_ser_clk_ops;
        } else if (xfer->version == RKX111_SER_CLK) {
            hw->ops = &rkx111_ser_clk_ops;
        } else {
            CRU_ERR("%s: Invalid version : %d\n", __func__, xfer->version);
            return HAL_INVAL;
	}
        break;

    default:
        CRU_ERR("%s: Invalid type: %d\n", __func__, xfer->type);

        return HAL_INVAL;
    }

    if (!hw->ops || !hw->ops->clkInit || !hw->ops->clkGetFreq || !hw->ops->clkSetFreq) {
        CRU_ERR("No available clkOps or .clkInit() or .clkGetFreq() or .clkSetFreq()\n");

        return HAL_INVAL;
    }

    HAL_MutexInit(&hw->lock);

    CRU_MSG("CLK: register xfer '%s' (chip v%d) with client 0x%08lx successfully.\n",
            hw->xfer.name, hw->xfer.version, (unsigned long)hw->xfer.client);

    return 0;
}

HAL_Status RKX11x_HAL_CRU_CORE_ClkInit(struct hwclk *hw)
{
    int ret = 0;

    if (!hw || (hw->type == CLK_UNDEF)) {
        return HAL_INVAL;
    }

    if (!hw->ops || !hw->ops->clkInit) {
        return HAL_INVAL;
    }

    ret = hw->ops->clkInit(hw, &hw->xfer);
    if (ret) {
        CRU_ERR("%s: Init clock failed, ret=%d\n", hw->name, ret);

        return HAL_INVAL;
    }

    if (!hw->num_gate) {
        CRU_ERR("No available .gate\n");

        return HAL_INVAL;
    }

#if HAL_ENABLE_SET_RATE_VERIFY
    hw->flags |= CLK_FLG_SET_RATE_VERIFY;
#endif

    hw->flags |= CLK_FLG_CRU_INITED;

    CRU_MSG("CLK: hwclk '%s' init successfully.\n", hw->name);

#if RKX11X_DEBUG_CRU_INIT
    RKX11x_HAL_CRU_CORE_ClkDumpTree(hw);
#endif

    return 0;
}
