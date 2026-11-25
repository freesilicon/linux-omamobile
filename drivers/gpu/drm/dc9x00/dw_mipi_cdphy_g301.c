// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room Synopsys DesignWare MIPI CDPHY G301 driver for Tensor G5 Laguna.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "dc9x00_internal.h"
#include "dc9x00_regs.h"

#define G301_MIN_RATE_MBPS		80
#define G301_MAX_RATE_MBPS		4500
#define G301_LOOP_COMP_MIN_KHZ		19200
#define G301_LOOP_COMP_MAX_KHZ		64000
#define G301_MAX_N			16
#define G301_VCO_OSC_MIN_KHZ		2000000
#define G301_VCO_OSC_MAX_KHZ		4500000
#define G301_PHY_STATUS_TIMEOUT_US	50000

#define G301_LP_DCO_PERIOD_PS		4770
#define G301_LPX_IO_SR0_DELAY_PS	12500
#define G301_D2A_HS_TX_DELAY		3
#define G301_T_LPX_PS			55000
#define G301_T_CLK_PREPARE_PS		73150
#define G301_T_CLK_TRAIL_PS		60000
#define G301_T_CLK_PREPARE_ZERO_PS	300000

struct g301_cp_entry {
	u32 vco_min_khz;
	u32 vco_max_khz;
	u32 vco_5_3;
	u32 vco_0_2;
	u32 cpbias;
	u32 gmp;
	u32 integ;
	u32 prop;
};

static const struct g301_cp_entry g301_cp_table[] = {
	{ 1706250, 2250000, 0, 0,  0, 1, 8, 16 },
	{ 1413750, 1793750, 0, 0,  0, 1, 8, 16 },
	{ 1170000, 1486260, 0, 3,  0, 1, 8, 16 },
	{  975000, 1230000, 0, 7, 16, 1, 8, 16 },
	{  853125, 1025000, 1, 0,  0, 1, 8, 16 },
	{  706875,  896875, 1, 0,  0, 1, 8, 16 },
	{  585000,  743125, 1, 3,  0, 1, 8, 16 },
	{  487500,  615000, 1, 7, 16, 1, 8, 16 },
	{  426560,  512500, 2, 0,  0, 1, 8, 16 },
	{  353400,  484400, 2, 0,  0, 1, 8, 16 },
	{  292500,  371500, 2, 3,  0, 1, 8, 16 },
	{  243750,  307500, 2, 7, 16, 1, 8, 16 },
	{  213300,  256250, 3, 0,  0, 1, 8, 16 },
	{  176720,  224200, 3, 0,  0, 1, 8, 16 },
	{  146250,  185780, 3, 3,  0, 1, 8, 16 },
	{  121880,  153750, 3, 7, 16, 1, 8, 16 },
	{  106640,  125120, 4, 0,  0, 1, 8, 16 },
	{   88360,  112100, 4, 0,  0, 1, 8, 16 },
	{   73130,   92900, 4, 3,  0, 1, 8, 16 },
	{   60930,   76870, 4, 7, 16, 1, 8, 16 },
	{   53320,   64000, 5, 0,  0, 1, 8, 16 },
	{   44180,   56000, 5, 0,  0, 1, 8, 16 },
	{   40000,   46440, 5, 3,  0, 1, 8, 16 },
};

struct g301_pll_cfg {
	u32 n;
	u32 m;
	u32 vco_5_3;
	u32 fracn_en;
	u32 mint;
	u32 frac_quot;
	const struct g301_cp_entry *cp;
};

struct g301_dphy_timings {
	u32 clk_trail;
	u32 clk_zero;
	u32 clk_pre;
	u32 clk_overlap;
	u32 clk_lpx;
	u32 clk_trail_dco;
	u32 clk_lp11_end;
	u32 clk_post;
	u32 clk_prpr;
	u32 clk_lp11_init;
	u32 clk_hsexit;
	u32 data_trail;
	u32 data_zero;
	u32 data_overlap;
	u32 data_lpx;
	u32 data_trail_dco;
	u32 data_lp11_end;
	u32 data_prpr;
	u32 data_lp11_init;
	u32 data_hsexit;
};

struct dw_cdphy_g301 {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
	void __iomem *apb;
	struct clk *pllref;
	u32 datarate_mbps;
	u32 lanes;
	bool configured;
	bool clk_enabled;
	bool pll_boot_on;
	struct g301_pll_cfg pll;
	struct g301_dphy_timings timings;
};

static inline void cdphy_rmw(void __iomem *reg, u32 mask, u32 val)
{
	u32 cur = readl(reg);

	cur = (cur & ~mask) | (val & mask);
	writel(cur, reg);
}

static void cdphy_rmw_lanes(void __iomem *base, u32 reg0, unsigned int count,
			    u32 mask, u32 val)
{
	unsigned int n;

	for (n = 0; n < count; n++)
		cdphy_rmw(base + reg0 + n * 0x800, mask, val);
}

static void cdphy_set_lane_din(struct dw_cdphy_g301 *ctx, bool dp, bool dn)
{
	u32 val = (dp ? CDPHY_LANE_2_3_LPTX_DIN_DP_VAL : 0) |
		  (dn ? CDPHY_LANE_2_3_LPTX_DIN_DN_VAL : 0);

	cdphy_rmw_lanes(ctx->base, CDPHY_AFE_LANE_CTRL_2_3(0), 5,
			CDPHY_LANE_2_3_LPTX_DIN_MASK, val);
}

static void cdphy_prepare_lp11_ovr(struct dw_cdphy_g301 *ctx)
{
	void __iomem *b = ctx->base;

	/* Drive LP00 -> LP10 -> LP11 on the AFE before releasing reset. */
	cdphy_set_lane_din(ctx, false, false);
	cdphy_rmw_lanes(b, CDPHY_AFE_LANE_CTRL_2_4(0), 5,
			CDPHY_LANE_2_4_LPTX_DIN_OVR_EN_MASK,
			CDPHY_LANE_2_4_LPTX_DIN_OVR_EN_MASK);
	cdphy_rmw_lanes(b, CDPHY_AFE_LANE_CTRL_2_7(0), 5,
			CDPHY_LANE_2_7_LPTX_EN_PON_VAL_MASK,
			CDPHY_LANE_2_7_LPTX_EN_PON_VAL_MASK);
	cdphy_rmw_lanes(b, CDPHY_AFE_LANE_CTRL_2_8(0), 5,
			CDPHY_LANE_2_8_LPTX_EN_PON_OVR_MASK,
			CDPHY_LANE_2_8_LPTX_EN_PON_OVR_MASK);
	udelay(1);
	cdphy_set_lane_din(ctx, true, false);
	udelay(1);
	cdphy_set_lane_din(ctx, true, true);
	udelay(1);
	cdphy_rmw_lanes(b, CDPHY_DPHY_PPI_LANE_OVR_0_8(0), 4,
			CDPHY_PPI_LANE_TURN_OVR_MASK,
			CDPHY_PPI_LANE_TURN_OVR_MASK);
}

static void cdphy_clear_lane_ovr(struct dw_cdphy_g301 *ctx)
{
	void __iomem *b = ctx->base;

	cdphy_rmw_lanes(b, CDPHY_AFE_LANE_CTRL_2_4(0), 5,
			CDPHY_LANE_2_4_LPTX_DIN_OVR_EN_MASK, 0);
	cdphy_rmw_lanes(b, CDPHY_AFE_LANE_CTRL_2_8(0), 5,
			CDPHY_LANE_2_8_LPTX_EN_PON_OVR_MASK, 0);
	cdphy_rmw_lanes(b, CDPHY_DPHY_PPI_LANE_OVR_0_8(0), 4,
			CDPHY_PPI_LANE_TURN_OVR_MASK, 0);
}

static void cdphy_dump_diag(struct dw_cdphy_g301 *ctx, const char *ctx_str)
{
	dev_err(ctx->dev,
		"CDPHY %s: sts=%#x c0=%#x e0=%#x e1=%#x ppi=%#x c9=%#x a2=%#x\n",
		ctx_str,
		readl(ctx->apb + CDPHY_PHY_STS),
		readl(ctx->apb + CDPHY_PHY_CTRL0),
		readl(ctx->apb + CDPHY_PHY_EXT_CTRL0),
		readl(ctx->apb + CDPHY_PHY_EXT_CTRL1),
		readl(ctx->base + CDPHY_COMMON_PPI_OVR_0_12),
		readl(ctx->base + CDPHY_COMMON_9),
		readl(ctx->base + CDPHY_ANACTRL_2));
}

static int cdphy_calc_pll(u32 datarate_mbps, u32 clkin_khz, struct g301_pll_cfg *out)
{
	u32 n = 1, log2_p, p, f_vco_in, f_vco_out, f_vco_osc = 0;
	u32 m_int, m_frac;
	const struct g301_cp_entry *cp = NULL;
	unsigned int i;

	if (datarate_mbps < G301_MIN_RATE_MBPS || datarate_mbps > G301_MAX_RATE_MBPS)
		return -EINVAL;
	if (clkin_khz < G301_LOOP_COMP_MIN_KHZ)
		return -EINVAL;

	while (clkin_khz % n == 0 && clkin_khz / n > G301_LOOP_COMP_MAX_KHZ && n <= G301_MAX_N)
		n++;

	if (clkin_khz % n != 0 || clkin_khz / n < G301_LOOP_COMP_MIN_KHZ || n > G301_MAX_N)
		return -EINVAL;

	f_vco_in = clkin_khz / n;
	f_vco_out = datarate_mbps * 500;

	for (log2_p = 1; log2_p <= 6; log2_p++) {
		p = BIT(log2_p);
		f_vco_osc = f_vco_out * p;
		if (f_vco_osc < G301_VCO_OSC_MIN_KHZ || f_vco_osc > G301_VCO_OSC_MAX_KHZ)
			continue;
		for (i = 0; i < ARRAY_SIZE(g301_cp_table); i++) {
			const struct g301_cp_entry *e = &g301_cp_table[i];

			if (e->vco_min_khz <= f_vco_out && f_vco_out <= e->vco_max_khz &&
			    e->vco_5_3 == log2_p - 1) {
				cp = e;
				break;
			}
		}
		if (cp)
			break;
	}
	if (!cp)
		return -EINVAL;

	memset(out, 0, sizeof(*out));
	out->n = n - 1;
	out->vco_5_3 = log2_p - 1;
	out->cp = cp;

	if (f_vco_osc % f_vco_in == 0) {
		m_int = f_vco_osc / f_vco_in;
		out->fracn_en = 0;
		out->m = m_int * 2 - 32;
	} else {
		m_int = (f_vco_osc * 2) / f_vco_in;
		m_frac = ((f_vco_osc * 2 - f_vco_in * m_int) * BIT(16)) / f_vco_in;
		out->fracn_en = 1;
		out->mint = m_int - 32;
		out->frac_quot = m_frac;
	}

	return 0;
}

static void cdphy_calc_dphy_timings(u32 datarate_mbps, struct g301_dphy_timings *t)
{
	u32 ui_ps = DIV_ROUND_CLOSEST(1000000, datarate_mbps);
	u32 t_word = 8 * ui_ps;
	u32 t_hs_prpr = DIV_ROUND_CLOSEST((40000 + 4 * ui_ps + 85000 + 6 * ui_ps) * 11, 20);
	u32 t_clk_post = DIV_ROUND_CLOSEST((60000 + 52 * ui_ps) * 11, 10);
	u32 t_hs_zero = (145000 + 10 * ui_ps) - t_hs_prpr;
	u32 t_hs_trail = max_t(u32, 8 * ui_ps, 60000 + 4 * ui_ps);
	u32 t_clk_zero = DIV_ROUND_CLOSEST((G301_T_CLK_PREPARE_ZERO_PS - G301_T_CLK_PREPARE_PS) * 11, 10);
	u32 t_eot = 105000 + 12 * ui_ps;
	u32 t_clk_trail_adj = DIV_ROUND_CLOSEST(G301_T_CLK_TRAIL_PS + t_eot, 2);
	u32 t_hs_trail_adj = DIV_ROUND_CLOSEST(t_hs_trail + t_eot, 2);

	t->clk_overlap = 2;
	t->clk_lp11_init = 52;
	t->clk_lpx = 11;
	t->clk_prpr = 17;
	t->clk_pre = G301_D2A_HS_TX_DELAY;
	t->clk_zero = DIV_ROUND_UP(G301_T_LPX_PS + G301_T_CLK_PREPARE_PS + t_clk_zero +
				   5 * G301_LP_DCO_PERIOD_PS - 3 * t_word, t_word) - 1;
	t->clk_trail = DIV_ROUND_UP(t_clk_trail_adj, t_word) - 1 + G301_D2A_HS_TX_DELAY;
	t->clk_trail_dco = DIV_ROUND_CLOSEST(G301_D2A_HS_TX_DELAY * t_word + t_clk_trail_adj -
					     t_word - 4 * G301_LP_DCO_PERIOD_PS,
					     G301_LP_DCO_PERIOD_PS) - 1;
	t->clk_post = DIV_ROUND_UP(t_clk_post, t_word) - 3;
	t->clk_lp11_end = 52;
	t->clk_hsexit = 23;

	t->data_overlap = 2;
	t->data_lp11_init = 52;
	t->data_lpx = 11;
	t->data_prpr = DIV_ROUND_UP(G301_LPX_IO_SR0_DELAY_PS + t_hs_prpr, G301_LP_DCO_PERIOD_PS) - 1;
	t->data_zero = max_t(int, 0,
			     (int)DIV_ROUND_UP(G301_T_LPX_PS + t_hs_prpr + t_hs_zero +
					       5 * G301_LP_DCO_PERIOD_PS - 3 * t_word, t_word) - 1);
	t->data_trail = DIV_ROUND_UP(t_hs_trail_adj, t_word) - 1 + G301_D2A_HS_TX_DELAY;
	t->data_trail_dco = DIV_ROUND_CLOSEST(G301_D2A_HS_TX_DELAY * t_word + t_hs_trail_adj -
					      t_word - 4 * G301_LP_DCO_PERIOD_PS,
					      G301_LP_DCO_PERIOD_PS) - 1;
	t->data_lp11_end = 52;
	t->data_hsexit = 23;
}

static void cdphy_program_common(struct dw_cdphy_g301 *ctx)
{
	void __iomem *b = ctx->base;

	cdphy_rmw(b + CDPHY_PPI_STARTUP_DPHY_2, GENMASK(7, 0), 3);
	cdphy_rmw(b + CDPHY_PPI_STARTUP_DPHY_3, GENMASK(7, 0), 38);
	cdphy_rmw(b + CDPHY_PPI_STARTUP_DPHY_6, GENMASK(7, 0), 16);
	cdphy_rmw(b + CDPHY_PPI_STARTUP_DPHY_A, GENMASK(7, 0), 33);
	cdphy_rmw(b + CDPHY_PPI_STARTUP_DPHY_10, GENMASK(7, 0), 47);
	cdphy_rmw(b + CDPHY_PPI_STARTUP_1_1, GENMASK(11, 0), 563);
	cdphy_rmw(b + CDPHY_PPI_CALIB_BG_0, GENMASK(8, 0), 500);
	cdphy_rmw(b + CDPHY_LPDCOCAL_TIMEBASE, GENMASK(9, 0), 153);
	cdphy_rmw(b + CDPHY_LPDCOCAL_NREF, GENMASK(10, 0), 800);
	cdphy_rmw(b + CDPHY_LPDCOCAL_NREF_RANGE, GENMASK(4, 0), 27);
	cdphy_rmw(b + CDPHY_LPDCOCAL_TWAIT_CFG, GENMASK(15, 0), 47 | (127 << 9));
	cdphy_rmw(b + CDPHY_LPDCOCAL_VT_CFG, GENMASK(15, 0),
		  BIT(1) | (27 << 2) | (47 << 7));
	cdphy_rmw(b + CDPHY_LPDCOCAL_COARSE_CFG, GENMASK(1, 0), 1);
	cdphy_rmw(b + CDPHY_COMMON_CFG, GENMASK(1, 0), 3);
	cdphy_rmw(b + CDPHY_TERMCAL_CFG_0, GENMASK(6, 0), 38);
	cdphy_rmw(b + CDPHY_PLL_STARTUP_CFG_0, GENMASK(9, 0), 383);
	cdphy_rmw(b + CDPHY_PLL_STARTUP_CFG_1, GENMASK(9, 0), 191);
	cdphy_rmw(b + CDPHY_PLL_STARTUP_CFG_2, GENMASK(9, 0), 0);
	cdphy_rmw(b + CDPHY_ANACTRL_0, GENMASK(7, 2), FIELD_PREP(GENMASK(7, 2), 63));
}

static void cdphy_program_dphy(struct dw_cdphy_g301 *ctx)
{
	const struct g301_dphy_timings *t = &ctx->timings;
	void __iomem *b = ctx->base;
	unsigned int n;

	for (n = 0; n < 4; n++) {
		cdphy_rmw(b + CDPHY_DLANE_LP_0(n), GENMASK(15, 8),
			  (6U << 8) | (1U << 12));
		cdphy_rmw(b + CDPHY_DLANE_LP_2(n), BIT(0), 0);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_0(n), GENMASK(15, 0), t->data_trail);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_1(n), GENMASK(15, 0), t->data_zero);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_3(n), GENMASK(7, 0), t->data_overlap);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_4(n), GENMASK(15, 0), t->data_lpx);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_5(n), GENMASK(15, 0), t->data_trail_dco);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_6(n), GENMASK(15, 0), t->data_lp11_end);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_9(n), GENMASK(15, 0), t->data_prpr);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_10(n), GENMASK(15, 0), t->data_lp11_init);
		cdphy_rmw(b + CDPHY_DLANE_HS_TX_12(n), GENMASK(15, 0), t->data_hsexit);
	}

	cdphy_rmw(b + CDPHY_CLK_LP_0, GENMASK(15, 12), 1U << 12);
	cdphy_rmw(b + CDPHY_CLK_LP_2, BIT(0), 0);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_0, GENMASK(15, 0), t->clk_trail);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_1, GENMASK(15, 0), t->clk_zero);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_2, GENMASK(15, 0), t->clk_pre);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_3, GENMASK(7, 0), t->clk_overlap);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_4, GENMASK(15, 0), t->clk_lpx);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_5, GENMASK(15, 0), t->clk_trail_dco);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_6, GENMASK(15, 0), t->clk_lp11_end);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_8, GENMASK(15, 0), t->clk_post);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_9, GENMASK(15, 0), t->clk_prpr);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_10, GENMASK(15, 0), t->clk_lp11_init);
	cdphy_rmw(b + CDPHY_CLK_HS_TX_12, GENMASK(15, 0), t->clk_hsexit);

	for (n = 0; n < 5; n++) {
		cdphy_rmw(b + CDPHY_AFE_LANE_CTRL_2_2(n), BIT(0), (n == 2) ? BIT(0) : 0);
		cdphy_rmw(b + CDPHY_AFE_LANE_CTRL_2_3(n),
			  CDPHY_LANE_2_3_HSTX_SEL_CLKLB | BIT(4),
			  (n == 2) ? 0 : BIT(4));
	}
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_4, CDPHY_CB_2_4_VCOMM_PROG_MASK,
		  FIELD_PREP(CDPHY_CB_2_4_VCOMM_PROG_MASK, 3));
}

static void cdphy_program_extra(struct dw_cdphy_g301 *ctx)
{
	void __iomem *b = ctx->base;

	cdphy_rmw(b + CDPHY_AFE_LANE2_CTRL_2_7, BIT(6) | BIT(7), BIT(6) | BIT(7));
	cdphy_rmw(b + CDPHY_AFE_LANE2_CTRL_2_8, GENMASK(3, 0), 0);

	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_5, BIT(8), 0);
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_3, BIT(8) | BIT(9), BIT(8) | BIT(9));
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_1, BIT(15), 0);
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_0, BIT(15), 0);
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_6, BIT(13) | BIT(14), BIT(13) | BIT(14));
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_7, BIT(9) | BIT(10), 0);
	cdphy_rmw(b + CDPHY_AFE_CB_CTRL_2_4, BIT(15), 0);
	cdphy_rmw_lanes(b, CDPHY_CLANE_LP_0(0), 3, GENMASK(15, 8), (6U << 8) | (1U << 12));

	cdphy_rmw(b + CDPHY_ANACTRL_2, GENMASK(13, 12), 0);
}

static void cdphy_program_pll(struct dw_cdphy_g301 *ctx)
{
	const struct g301_pll_cfg *p = &ctx->pll;
	void __iomem *a = ctx->apb;

	cdphy_rmw(a + CDPHY_PLL_CFG0, BIT(0) | GENMASK(2, 1) | GENMASK(8, 4) | BIT(27),
		  FIELD_PREP(GENMASK(8, 4), 16) | BIT(27));
	writel(3, a + CDPHY_PLL_CFG1);
	cdphy_rmw(a + CDPHY_PLL_CFG2, GENMASK(29, 0),
		  FIELD_PREP(GENMASK(13, 4), 1) |
		  FIELD_PREP(GENMASK(21, 14), 255) |
		  FIELD_PREP(GENMASK(29, 22), 3));

	cdphy_rmw(a + CDPHY_PLL_CFG3, GENMASK(31, 0),
		  FIELD_PREP(GENMASK(11, 0), p->mint) |
		  FIELD_PREP(GENMASK(23, 12), p->m) |
		  FIELD_PREP(GENMASK(29, 24), p->n) |
		  (p->fracn_en ? BIT(31) : 0));

	cdphy_rmw(a + CDPHY_PLL_CFG4, GENMASK(15, 0), 1);
	cdphy_rmw(a + CDPHY_PLL_CFG5, GENMASK(20, 0), 0);
	cdphy_rmw(a + CDPHY_PLL_CFG6, GENMASK(22, 0), 0);
	cdphy_rmw(a + CDPHY_PLL_CFG7, GENMASK(31, 0), p->frac_quot & 0xffff);
	cdphy_rmw(a + CDPHY_PLL_CFG3, BIT(30), BIT(30));

	cdphy_rmw(a + CDPHY_PLL_CP, GENMASK(26, 0),
		  FIELD_PREP(GENMASK(2, 0), p->cp->vco_0_2) |
		  FIELD_PREP(GENMASK(5, 3), p->vco_5_3) |
		  FIELD_PREP(GENMASK(11, 6), p->cp->prop) |
		  FIELD_PREP(GENMASK(17, 12), p->cp->integ) |
		  FIELD_PREP(GENMASK(19, 18), p->cp->gmp) |
		  FIELD_PREP(GENMASK(26, 20), p->cp->cpbias));
}

static int dw_cdphy_g301_init(struct phy *phy)
{
	struct dw_cdphy_g301 *ctx = phy_get_drvdata(phy);
	int ret;

	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret < 0)
		return ret;

	if (ctx->pllref && !ctx->clk_enabled) {
		ret = clk_prepare_enable(ctx->pllref);
		if (ret) {
			pm_runtime_put_sync(ctx->dev);
			return ret;
		}
		ctx->clk_enabled = true;
	}

	ctx->pll_boot_on = !!(readl(ctx->base + CDPHY_COMMON_9) & CDPHY_COMMON9_PLL_ENABLED);

	return 0;
}

static int dw_cdphy_g301_exit(struct phy *phy)
{
	struct dw_cdphy_g301 *ctx = phy_get_drvdata(phy);

	if (ctx->clk_enabled && ctx->pllref) {
		clk_disable_unprepare(ctx->pllref);
		ctx->clk_enabled = false;
	}
	pm_runtime_put_sync(ctx->dev);
	return 0;
}

static int dw_cdphy_g301_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct dw_cdphy_g301 *ctx = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *dphy = &opts->mipi_dphy;
	unsigned long rate_hz = 0;
	u32 clkin_khz, sts, stop_mask;
	int ret;

	if (dphy->hs_clk_rate)
		ctx->datarate_mbps = DIV_ROUND_CLOSEST(dphy->hs_clk_rate, 1000000);
	ctx->lanes = dphy->lanes ? clamp_t(u32, dphy->lanes, 1, 4) : 4;

	if (ctx->pllref)
		rate_hz = clk_get_rate(ctx->pllref);
	if (!rate_hz) {
		rate_hz = 38400000;
		dev_warn(ctx->dev, "pllref reported 0 Hz, assuming 38.4 MHz\n");
	}
	clkin_khz = rate_hz / 1000;

	ret = cdphy_calc_pll(ctx->datarate_mbps, clkin_khz, &ctx->pll);
	if (ret) {
		dev_err(ctx->dev, "PLL calc failed (rate=%u Mbps, clkin=%u kHz)\n",
			ctx->datarate_mbps, clkin_khz);
		return ret;
	}
	cdphy_calc_dphy_timings(ctx->datarate_mbps, &ctx->timings);

	cdphy_rmw(ctx->apb + CDPHY_PHY_CTRL0,
		  CDPHY_CTRL0_PHY_RST_N | CDPHY_CTRL0_SHUTDOWN_N |
		  CDPHY_CTRL0_PHY_MODE_CPHY | CDPHY_CTRL0_TEST_STOP_CLK_EN |
		  CDPHY_CTRL0_BIST_EN, 0);
	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL0, CDPHY_EXT0_CLEAR_MASK, 0);
	cdphy_rmw(ctx->base + CDPHY_HSTX_FIFO_CFG, BIT(0), 0);

	stop_mask = CDPHY_EXT1_FORCETXSTOP_DCK | CDPHY_EXT1_FORCETXSTOP_0 |
		    CDPHY_EXT1_FORCETXSTOP_1 | CDPHY_EXT1_FORCETXSTOP_2 |
		    CDPHY_EXT1_FORCETXSTOP_3;
	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL1,
		  CDPHY_EXT1_ENABLE_MASK | CDPHY_EXT1_CONT_EN_MASK | stop_mask,
		  stop_mask);
	cdphy_rmw(ctx->base + CDPHY_ANACTRL_2, GENMASK(13, 12), 0);

	cdphy_program_common(ctx);
	cdphy_program_dphy(ctx);
	cdphy_program_extra(ctx);
	cdphy_program_pll(ctx);

	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL1, GENMASK(23, 16),
		  CDPHY_EXT1_TXDATAWIDTHHS_ALL_16B);

	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL1, stop_mask, 0);
	cdphy_prepare_lp11_ovr(ctx);

	cdphy_rmw(ctx->apb + CDPHY_PHY_CTRL0,
		  CDPHY_CTRL0_PHY_MODE_CPHY | CDPHY_CTRL0_SHUTDOWN_N | CDPHY_CTRL0_PHY_RST_N,
		  CDPHY_CTRL0_SHUTDOWN_N | CDPHY_CTRL0_PHY_RST_N);
	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL1, stop_mask, stop_mask);

	ret = readl_poll_timeout(ctx->apb + CDPHY_PHY_STS, sts,
				 sts & CDPHY_STS_PHY_READY, 50, G301_PHY_STATUS_TIMEOUT_US);
	if (ret) {
		cdphy_dump_diag(ctx, "ready_timeout");
		return ret;
	}

	ret = readl_poll_timeout(ctx->apb + CDPHY_PHY_STS, sts,
				 sts & CDPHY_STS_PLL_LOCK, 50, G301_PHY_STATUS_TIMEOUT_US);
	if (ret) {
		cdphy_dump_diag(ctx, "pll_lock_timeout");
		return ret;
	}

	cdphy_rmw(ctx->apb + CDPHY_PLL_CFG0, GENMASK(2, 1), FIELD_PREP(GENMASK(2, 1), 1));
	ctx->configured = true;

	dev_info(ctx->dev, "CDPHY G301 locked at %u Mbps (ref=%u kHz, ppi=%#x, boot=%u)\n",
		 ctx->datarate_mbps, clkin_khz,
		 readl(ctx->base + CDPHY_COMMON_PPI_OVR_0_12) & 0x1f,
		 ctx->pll_boot_on);

	return 0;
}

static int dw_cdphy_g301_power_on(struct phy *phy)
{
	struct dw_cdphy_g301 *ctx = phy_get_drvdata(phy);
	u32 stop_mask = CDPHY_EXT1_FORCETXSTOP_DCK | CDPHY_EXT1_FORCETXSTOP_0 |
			CDPHY_EXT1_FORCETXSTOP_1 | CDPHY_EXT1_FORCETXSTOP_2 |
			CDPHY_EXT1_FORCETXSTOP_3;

	cdphy_clear_lane_ovr(ctx);
	cdphy_rmw(ctx->apb + CDPHY_PHY_EXT_CTRL1, stop_mask, 0);
	return 0;
}

static int dw_cdphy_g301_power_off(struct phy *phy)
{
	struct dw_cdphy_g301 *ctx = phy_get_drvdata(phy);

	cdphy_clear_lane_ovr(ctx);
	cdphy_rmw(ctx->apb + CDPHY_PHY_CTRL0,
		  CDPHY_CTRL0_SHUTDOWN_N | CDPHY_CTRL0_PHY_RST_N, 0);
	return 0;
}

static const struct phy_ops dw_cdphy_g301_ops = {
	.init = dw_cdphy_g301_init,
	.exit = dw_cdphy_g301_exit,
	.configure = dw_cdphy_g301_configure,
	.power_on = dw_cdphy_g301_power_on,
	.power_off = dw_cdphy_g301_power_off,
	.owner = THIS_MODULE,
};

static int dw_cdphy_g301_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct dw_cdphy_g301 *ctx;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->datarate_mbps = 1368;
	ctx->lanes = 4;

	ctx->base = devm_platform_ioremap_resource_byname(pdev, "cdphy-tx");
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	ctx->apb = devm_platform_ioremap_resource_byname(pdev, "u0-apb-regbank");
	if (IS_ERR(ctx->apb))
		return PTR_ERR(ctx->apb);

	ctx->pllref = devm_clk_get_optional(dev, "pllref");
	if (IS_ERR(ctx->pllref))
		return dev_err_probe(dev, PTR_ERR(ctx->pllref),
				     "failed to acquire pllref clock\n");

	pm_runtime_enable(dev);

	ctx->phy = devm_phy_create(dev, NULL, &dw_cdphy_g301_ops);
	if (IS_ERR(ctx->phy)) {
		pm_runtime_disable(dev);
		return PTR_ERR(ctx->phy);
	}

	phy_set_drvdata(ctx->phy, ctx);
	platform_set_drvdata(pdev, ctx);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		pm_runtime_disable(dev);
		return PTR_ERR(provider);
	}

	return 0;
}

static void dw_cdphy_g301_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id dw_cdphy_g301_of_match[] = {
	{ .compatible = "synopsys,dw-mipi-cdphy-v1" },
	{}
};
MODULE_DEVICE_TABLE(of, dw_cdphy_g301_of_match);

struct platform_driver dw_cdphy_g301_driver = {
	.probe = dw_cdphy_g301_probe,
	.remove = dw_cdphy_g301_remove,
	.driver = {
		.name = "dw-mipi-cdphy-g301",
		.of_match_table = dw_cdphy_g301_of_match,
	},
};
