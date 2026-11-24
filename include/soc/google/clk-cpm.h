/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SOC_GOOGLE_CLK_CPM_H_
#define _SOC_GOOGLE_CLK_CPM_H_

#include <linux/types.h>

struct clk;

int goog_cpm_clk_toggle_auto_gate(struct clk *clk, bool enable);
int goog_cpm_clk_toggle_qch_mode(struct clk *clk, bool enable);

#endif /* _SOC_GOOGLE_CLK_CPM_H_ */
