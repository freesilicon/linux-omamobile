// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pin controller definitions for Google Tensor G5 (LGA / Laguna).
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "pinctrl-google.h"

#define LGA_PIN_STRIDE		0x1000

GOOGLE_PINS(0);  GOOGLE_PINS(1);  GOOGLE_PINS(2);  GOOGLE_PINS(3);
GOOGLE_PINS(4);  GOOGLE_PINS(5);  GOOGLE_PINS(6);  GOOGLE_PINS(7);
GOOGLE_PINS(8);  GOOGLE_PINS(9);  GOOGLE_PINS(10); GOOGLE_PINS(11);
GOOGLE_PINS(12); GOOGLE_PINS(13); GOOGLE_PINS(14); GOOGLE_PINS(15);
GOOGLE_PINS(16); GOOGLE_PINS(17); GOOGLE_PINS(18); GOOGLE_PINS(19);
GOOGLE_PINS(20); GOOGLE_PINS(21); GOOGLE_PINS(22); GOOGLE_PINS(23);
GOOGLE_PINS(24); GOOGLE_PINS(25); GOOGLE_PINS(26); GOOGLE_PINS(27);
GOOGLE_PINS(28); GOOGLE_PINS(29); GOOGLE_PINS(30); GOOGLE_PINS(31);
GOOGLE_PINS(32); GOOGLE_PINS(33); GOOGLE_PINS(34); GOOGLE_PINS(35);
GOOGLE_PINS(36); GOOGLE_PINS(37); GOOGLE_PINS(38); GOOGLE_PINS(39);
GOOGLE_PINS(40); GOOGLE_PINS(41); GOOGLE_PINS(42); GOOGLE_PINS(43);
GOOGLE_PINS(44); GOOGLE_PINS(45); GOOGLE_PINS(46); GOOGLE_PINS(47);
GOOGLE_PINS(48); GOOGLE_PINS(49); GOOGLE_PINS(50); GOOGLE_PINS(51);
GOOGLE_PINS(52); GOOGLE_PINS(53); GOOGLE_PINS(54); GOOGLE_PINS(55);
GOOGLE_PINS(56); GOOGLE_PINS(57); GOOGLE_PINS(58); GOOGLE_PINS(59);
GOOGLE_PINS(60); GOOGLE_PINS(61); GOOGLE_PINS(62); GOOGLE_PINS(63);
GOOGLE_PINS(64); GOOGLE_PINS(65); GOOGLE_PINS(66); GOOGLE_PINS(67);
GOOGLE_PINS(68); GOOGLE_PINS(69); GOOGLE_PINS(70); GOOGLE_PINS(71);
GOOGLE_PINS(72); GOOGLE_PINS(73); GOOGLE_PINS(74); GOOGLE_PINS(75);
GOOGLE_PINS(76); GOOGLE_PINS(77); GOOGLE_PINS(78); GOOGLE_PINS(79);
GOOGLE_PINS(80); GOOGLE_PINS(81); GOOGLE_PINS(82); GOOGLE_PINS(83);
GOOGLE_PINS(84); GOOGLE_PINS(85); GOOGLE_PINS(86); GOOGLE_PINS(87);
GOOGLE_PINS(88); GOOGLE_PINS(89); GOOGLE_PINS(90);

/* HSION */
static const struct pinctrl_pin_desc google_lga_hsion[] = {
	PINCTRL_PIN(0, "XHSION_PCIE0_CLKREQN0"),
	PINCTRL_PIN(1, "XHSION_GPIO7"),
	PINCTRL_PIN(2, "XHSION_PCIE0_PERSTN0"),
	PINCTRL_PIN(3, "XHSION_GPIO8"),
	PINCTRL_PIN(4, "XHSION_GPIO0"),
	PINCTRL_PIN(5, "XHSION_GPIO1"),
	PINCTRL_PIN(6, "XHSION_GPIO2"),
	PINCTRL_PIN(7, "XHSION_GPIO3"),
	PINCTRL_PIN(8, "XHSION_GPIO4"),
	PINCTRL_PIN(9, "XHSION_GPIO5"),
	PINCTRL_PIN(10, "XHSION_GPIO6"),
	PINCTRL_PIN(11, "XHSION_ATB0"),
};

enum google_lga_hsion_funcs {
	google_pinmux_hsion_gpio,
	google_pinmux_hsion_pcie0,
	google_pinmux_hsion_atb0,
};

GOOGLE_FUNCTION_GROUPS(hsion_gpio, "XHSION_PCIE0_CLKREQN0", "XHSION_GPIO7",
		       "XHSION_PCIE0_PERSTN0", "XHSION_GPIO8", "XHSION_GPIO0",
		       "XHSION_GPIO1", "XHSION_GPIO2", "XHSION_GPIO3",
		       "XHSION_GPIO4", "XHSION_GPIO5", "XHSION_GPIO6");
GOOGLE_FUNCTION_GROUPS(hsion_pcie0, "XHSION_PCIE0_CLKREQN0",
		       "XHSION_PCIE0_PERSTN0");
GOOGLE_FUNCTION_GROUPS(hsion_atb0, "XHSION_ATB0");

static const struct google_pingroup google_lga_hsion_groups[] = {
	GOOGLE_PIN_GROUP(0, "XHSION_PCIE0_CLKREQN0", hsion_gpio, hsion_pcie0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XHSION_GPIO7", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XHSION_PCIE0_PERSTN0", hsion_gpio, hsion_pcie0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XHSION_GPIO8", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XHSION_GPIO0", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XHSION_GPIO1", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XHSION_GPIO2", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XHSION_GPIO3", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XHSION_GPIO4", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XHSION_GPIO5", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XHSION_GPIO6", hsion_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XHSION_ATB0", none, hsion_atb0, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_hsion_functions[] = {
	GOOGLE_FUNCTION(hsion_gpio),
	GOOGLE_FUNCTION(hsion_pcie0),
	GOOGLE_FUNCTION(hsion_atb0),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_hsion = {
	.pins = google_lga_hsion,
	.num_pins = ARRAY_SIZE(google_lga_hsion),
	.groups = google_lga_hsion_groups,
	.num_groups = ARRAY_SIZE(google_lga_hsion_groups),
	.funcs = google_lga_hsion_functions,
	.num_funcs = ARRAY_SIZE(google_lga_hsion_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "hsion",
};

/* HSIOS_STBY */
static const struct pinctrl_pin_desc google_lga_hsios_stby[] = {
	PINCTRL_PIN(0, "XHSIOS_CLKBUF_1"),
	PINCTRL_PIN(1, "XHSIOS_UFS_REFCLK"),
};

enum google_lga_hsios_stby_funcs {
	google_pinmux_hsios_stby_clkbuf,
	google_pinmux_hsios_stby_ufs,
};

GOOGLE_FUNCTION_GROUPS(hsios_stby_clkbuf, "XHSIOS_CLKBUF_1");
GOOGLE_FUNCTION_GROUPS(hsios_stby_ufs, "XHSIOS_UFS_REFCLK");

static const struct google_pingroup google_lga_hsios_stby_groups[] = {
	GOOGLE_PIN_GROUP(0, "XHSIOS_CLKBUF_1", none, hsios_stby_clkbuf, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XHSIOS_UFS_REFCLK", none, hsios_stby_ufs, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_hsios_stby_functions[] = {
	GOOGLE_FUNCTION(hsios_stby_clkbuf),
	GOOGLE_FUNCTION(hsios_stby_ufs),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_hsios_stby = {
	.pins = google_lga_hsios_stby,
	.num_pins = ARRAY_SIZE(google_lga_hsios_stby),
	.groups = google_lga_hsios_stby_groups,
	.num_groups = ARRAY_SIZE(google_lga_hsios_stby_groups),
	.funcs = google_lga_hsios_stby_functions,
	.num_funcs = ARRAY_SIZE(google_lga_hsios_stby_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "hsios_stby",
};

/* HSIOS */
static const struct pinctrl_pin_desc google_lga_hsios[] = {
	PINCTRL_PIN(0, "XHSIOS_UFS_RESETB"),
	PINCTRL_PIN(1, "XHSIOS_GPIO7"),
	PINCTRL_PIN(2, "XHSIOS_GPIO8"),
	PINCTRL_PIN(3, "XHSIOS_GPIO0"),
	PINCTRL_PIN(4, "XHSIOS_GPIO1"),
	PINCTRL_PIN(5, "XHSIOS_GPIO2"),
	PINCTRL_PIN(6, "XHSIOS_GPIO3"),
	PINCTRL_PIN(7, "XHSIOS_GPIO4"),
	PINCTRL_PIN(8, "XHSIOS_UFS_PWR_EN"),
	PINCTRL_PIN(9, "XHSIOS_ATB1"),
	PINCTRL_PIN(10, "XHSIOS_PCIE1_CLKREQN0"),
	PINCTRL_PIN(11, "XHSIOS_PCIE1_PERSTN0"),
	PINCTRL_PIN(12, "XHSIOS_GPIO5"),
	PINCTRL_PIN(13, "XHSIOS_GPIO6"),
	PINCTRL_PIN(14, "XHSIOS_VSYNC4"),
};

enum google_lga_hsios_funcs {
	google_pinmux_hsios_ufs,
	google_pinmux_hsios_gpio,
	google_pinmux_hsios_sd_data,
	google_pinmux_hsios_sd_cmd,
	google_pinmux_hsios_sd_fbclk,
	google_pinmux_hsios_sd_clk,
	google_pinmux_hsios_atb1,
	google_pinmux_hsios_pcie1,
	google_pinmux_hsios_vsync,
};

GOOGLE_FUNCTION_GROUPS(hsios_ufs, "XHSIOS_UFS_RESETB", "XHSIOS_UFS_PWR_EN");
GOOGLE_FUNCTION_GROUPS(hsios_gpio, "XHSIOS_GPIO7", "XHSIOS_GPIO8", "XHSIOS_GPIO0",
		       "XHSIOS_GPIO1", "XHSIOS_GPIO2", "XHSIOS_GPIO3",
		       "XHSIOS_GPIO4", "XHSIOS_PCIE1_CLKREQN0",
		       "XHSIOS_PCIE1_PERSTN0", "XHSIOS_GPIO5", "XHSIOS_GPIO6",
		       "XHSIOS_VSYNC4");
GOOGLE_FUNCTION_GROUPS(hsios_sd_data, "XHSIOS_GPIO7", "XHSIOS_GPIO8",
		       "XHSIOS_GPIO1", "XHSIOS_GPIO2");
GOOGLE_FUNCTION_GROUPS(hsios_sd_cmd, "XHSIOS_GPIO0");
GOOGLE_FUNCTION_GROUPS(hsios_sd_fbclk, "XHSIOS_GPIO3");
GOOGLE_FUNCTION_GROUPS(hsios_sd_clk, "XHSIOS_GPIO4");
GOOGLE_FUNCTION_GROUPS(hsios_atb1, "XHSIOS_ATB1");
GOOGLE_FUNCTION_GROUPS(hsios_pcie1, "XHSIOS_PCIE1_CLKREQN0",
		       "XHSIOS_PCIE1_PERSTN0");
GOOGLE_FUNCTION_GROUPS(hsios_vsync, "XHSIOS_VSYNC4");

static const struct google_pingroup google_lga_hsios_groups[] = {
	GOOGLE_PIN_GROUP(0, "XHSIOS_UFS_RESETB", hsios_ufs, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XHSIOS_GPIO7", hsios_gpio, none, hsios_sd_data, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XHSIOS_GPIO8", hsios_gpio, none, hsios_sd_data, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XHSIOS_GPIO0", hsios_gpio, none, hsios_sd_cmd, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XHSIOS_GPIO1", hsios_gpio, none, hsios_sd_data, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XHSIOS_GPIO2", hsios_gpio, none, hsios_sd_data, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XHSIOS_GPIO3", hsios_gpio, none, hsios_sd_fbclk, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XHSIOS_GPIO4", hsios_gpio, none, hsios_sd_clk, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XHSIOS_UFS_PWR_EN", hsios_ufs, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XHSIOS_ATB1", none, hsios_atb1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XHSIOS_PCIE1_CLKREQN0", hsios_gpio, hsios_pcie1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XHSIOS_PCIE1_PERSTN0", hsios_gpio, hsios_pcie1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XHSIOS_GPIO5", hsios_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(13, "XHSIOS_GPIO6", hsios_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(14, "XHSIOS_VSYNC4", hsios_gpio, hsios_vsync, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_hsios_functions[] = {
	GOOGLE_FUNCTION(hsios_ufs),
	GOOGLE_FUNCTION(hsios_gpio),
	GOOGLE_FUNCTION(hsios_sd_data),
	GOOGLE_FUNCTION(hsios_sd_cmd),
	GOOGLE_FUNCTION(hsios_sd_fbclk),
	GOOGLE_FUNCTION(hsios_sd_clk),
	GOOGLE_FUNCTION(hsios_atb1),
	GOOGLE_FUNCTION(hsios_pcie1),
	GOOGLE_FUNCTION(hsios_vsync),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_hsios = {
	.pins = google_lga_hsios,
	.num_pins = ARRAY_SIZE(google_lga_hsios),
	.groups = google_lga_hsios_groups,
	.num_groups = ARRAY_SIZE(google_lga_hsios_groups),
	.funcs = google_lga_hsios_functions,
	.num_funcs = ARRAY_SIZE(google_lga_hsios_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "hsios",
};

/* DPU */
static const struct pinctrl_pin_desc google_lga_dpu[] = {
	PINCTRL_PIN(0, "XDPU_TE0"),
	PINCTRL_PIN(1, "XDPU_TE1"),
};

enum google_lga_dpu_funcs {
	google_pinmux_dpu_gpio,
	google_pinmux_dpu_te,
};

GOOGLE_FUNCTION_GROUPS(dpu_gpio, "XDPU_TE0", "XDPU_TE1");
GOOGLE_FUNCTION_GROUPS(dpu_te, "XDPU_TE0", "XDPU_TE1");

static const struct google_pingroup google_lga_dpu_groups[] = {
	GOOGLE_PIN_GROUP(0, "XDPU_TE0", dpu_gpio, dpu_te, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XDPU_TE1", dpu_gpio, dpu_te, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_dpu_functions[] = {
	GOOGLE_FUNCTION(dpu_gpio),
	GOOGLE_FUNCTION(dpu_te),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_dpu = {
	.pins = google_lga_dpu,
	.num_pins = ARRAY_SIZE(google_lga_dpu),
	.groups = google_lga_dpu_groups,
	.num_groups = ARRAY_SIZE(google_lga_dpu_groups),
	.funcs = google_lga_dpu_functions,
	.num_funcs = ARRAY_SIZE(google_lga_dpu_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "dpu",
};

/* AOC */
static const struct pinctrl_pin_desc google_lga_aoc[] = {
	PINCTRL_PIN(0, "XAOC_PDM1_MIC_IN"),
	PINCTRL_PIN(1, "XAOC_PDM1_MIC_CLK"),
	PINCTRL_PIN(2, "XAOC_PDM2_MIC_IN"),
	PINCTRL_PIN(3, "XAOC_PDM2_MIC_CLK"),
	PINCTRL_PIN(4, "XAOC_PDM3_MIC_IN"),
	PINCTRL_PIN(5, "XAOC_PDM3_MIC_CLK"),
	PINCTRL_PIN(6, "XAOC_PDM0_MIC_IN"),
	PINCTRL_PIN(7, "XAOC_PDM0_MIC_CLK"),
	PINCTRL_PIN(8, "XAOC_I2S0_BCLK"),
	PINCTRL_PIN(9, "XAOC_I2S0_WS"),
	PINCTRL_PIN(10, "XAOC_I2S0_SDO"),
	PINCTRL_PIN(11, "XAOC_I2S0_SDI"),
	PINCTRL_PIN(12, "XAOC_GPIO0"),
	PINCTRL_PIN(13, "XAOC_GPIO1"),
	PINCTRL_PIN(14, "XAOC_UART0_RXD"),
	PINCTRL_PIN(15, "XAOC_UART0_TXD"),
	PINCTRL_PIN(16, "XAOC_UART0_RTSn"),
	PINCTRL_PIN(17, "XAOC_UART0_CTSn"),
	PINCTRL_PIN(18, "XAOC_UART3_RXD"),
	PINCTRL_PIN(19, "XAOC_UART3_TXD"),
	PINCTRL_PIN(20, "XAOC_UART3_RTSn"),
	PINCTRL_PIN(21, "XAOC_UART3_CTSn"),
	PINCTRL_PIN(22, "XAOC_GPIO2"),
	PINCTRL_PIN(23, "XAOC_GPIO3"),
	PINCTRL_PIN(24, "XAOC_GPIO4"),
	PINCTRL_PIN(25, "XAOC_GPIO5"),
	PINCTRL_PIN(26, "XAOC_SPI0_SCLK"),
	PINCTRL_PIN(27, "XAOC_SPI0_MOSI"),
	PINCTRL_PIN(28, "XAOC_SPI0_MISO"),
	PINCTRL_PIN(29, "XAOC_SPI0_SSB"),
	PINCTRL_PIN(30, "XAOC_OUT_MCLK"),
	PINCTRL_PIN(31, "XAOC_PDM0_FLCKR_IN"),
	PINCTRL_PIN(32, "XAOC_PDM0_FLCKR_CLK"),
	PINCTRL_PIN(33, "XAOC_I2C2_SCL"),
	PINCTRL_PIN(34, "XAOC_I2C2_SDA"),
	PINCTRL_PIN(35, "XAOC_I2C4_SCL"),
	PINCTRL_PIN(36, "XAOC_I2C4_SDA"),
	PINCTRL_PIN(37, "XAOC_TDM0_BCLK"),
	PINCTRL_PIN(38, "XAOC_TDM0_WS"),
	PINCTRL_PIN(39, "XAOC_TDM0_SDO"),
	PINCTRL_PIN(40, "XAOC_TDM0_SDI"),
	PINCTRL_PIN(41, "XAOC_GPIO6"),
	PINCTRL_PIN(42, "XAOC_GPIO7"),
	PINCTRL_PIN(43, "XAOC_SPI1_SCLK"),
	PINCTRL_PIN(44, "XAOC_SPI1_MOSI"),
	PINCTRL_PIN(45, "XAOC_SPI1_MISO"),
	PINCTRL_PIN(46, "XAOC_SPI1_SSB"),
	PINCTRL_PIN(47, "XAOC_TDM1_BCLK"),
	PINCTRL_PIN(48, "XAOC_TDM1_WS"),
	PINCTRL_PIN(49, "XAOC_TDM1_SDO"),
	PINCTRL_PIN(50, "XAOC_TDM1_SDI"),
	PINCTRL_PIN(51, "XAOC_SPI4_SCLK"),
	PINCTRL_PIN(52, "XAOC_SPI4_MOSI"),
	PINCTRL_PIN(53, "XAOC_SPI4_MISO"),
	PINCTRL_PIN(54, "XAOC_SPI4_SSB"),
	PINCTRL_PIN(55, "XAOC_GPIO8"),
	PINCTRL_PIN(56, "XAOC_GPIO9"),
	PINCTRL_PIN(57, "XAOC_GPIO10"),
	PINCTRL_PIN(58, "XAOC_GPIO11"),
	PINCTRL_PIN(59, "XAOC_I2S1_BCLK"),
	PINCTRL_PIN(60, "XAOC_I2S1_WS"),
	PINCTRL_PIN(61, "XAOC_I2S1_SDO"),
	PINCTRL_PIN(62, "XAOC_I2S1_SDI"),
	PINCTRL_PIN(63, "XAOC_I2C0_SCL"),
	PINCTRL_PIN(64, "XAOC_I2C0_SDA"),
	PINCTRL_PIN(65, "XAOC_GPIO12"),
	PINCTRL_PIN(66, "XAOC_GPIO13"),
	PINCTRL_PIN(67, "XAOC_I3C0_SCL"),
	PINCTRL_PIN(68, "XAOC_I3C0_SDA"),
	PINCTRL_PIN(69, "XAOC_I3C1_SCL"),
	PINCTRL_PIN(70, "XAOC_I3C1_SDA"),
	PINCTRL_PIN(71, "XAOC_SPI3_SCLK"),
	PINCTRL_PIN(72, "XAOC_SPI3_MOSI"),
	PINCTRL_PIN(73, "XAOC_SPI3_MISO"),
	PINCTRL_PIN(74, "XAOC_SPI3_SSB"),
	PINCTRL_PIN(75, "XAOC_SPI2_SCLK"),
	PINCTRL_PIN(76, "XAOC_SPI2_MOSI"),
	PINCTRL_PIN(77, "XAOC_SPI2_MISO"),
	PINCTRL_PIN(78, "XAOC_SPI2_SSB"),
	PINCTRL_PIN(79, "XAOC_UART1_RXD"),
	PINCTRL_PIN(80, "XAOC_UART1_TXD"),
	PINCTRL_PIN(81, "XAOC_UART1_RTSn"),
	PINCTRL_PIN(82, "XAOC_UART1_CTSn"),
	PINCTRL_PIN(83, "XAOC_UART4_RXD"),
	PINCTRL_PIN(84, "XAOC_UART4_TXD"),
	PINCTRL_PIN(85, "XAOC_UART4_RTSn"),
	PINCTRL_PIN(86, "XAOC_UART4_CTSn"),
	PINCTRL_PIN(87, "XAOC_SPI5_SCLK"),
	PINCTRL_PIN(88, "XAOC_SPI5_MOSI"),
	PINCTRL_PIN(89, "XAOC_SPI5_MISO"),
	PINCTRL_PIN(90, "XAOC_SPI5_SSB"),
};

enum google_lga_aoc_funcs {
	google_pinmux_aoc_int_gpio,
	google_pinmux_aoc_pdm1,
	google_pinmux_aoc_pdm2,
	google_pinmux_aoc_pdm3,
	google_pinmux_aoc_pdm,
	google_pinmux_aoc_i2s,
	google_pinmux_aoc_cpm_eint,
	google_pinmux_aoc_uart0,
	google_pinmux_aoc_cpm_uart,
	google_pinmux_aoc_uart3,
	google_pinmux_aoc_fiq,
	google_pinmux_aoc_spi0,
	google_pinmux_aoc_cli_int_0,
	google_pinmux_aoc_out_mclk,
	google_pinmux_aoc_pdm0_flckr,
	google_pinmux_aoc_i2c2,
	google_pinmux_aoc_i3c5,
	google_pinmux_aoc_elaoutput,
	google_pinmux_aoc_i2c4,
	google_pinmux_aoc_i3c4,
	google_pinmux_aoc_uart2,
	google_pinmux_aoc_tdm0,
	google_pinmux_aoc_spi1,
	google_pinmux_aoc_cli_int_1,
	google_pinmux_aoc_tdm1,
	google_pinmux_aoc_sdwire0,
	google_pinmux_aoc_sdwire1,
	google_pinmux_aoc_spi4,
	google_pinmux_aoc_i2c0,
	google_pinmux_aoc_i3c0,
	google_pinmux_aoc_i2c1,
	google_pinmux_aoc_i3c1,
	google_pinmux_aoc_i3c3,
	google_pinmux_aoc_spi3,
	google_pinmux_aoc_cli_int_3,
	google_pinmux_aoc_i3c2,
	google_pinmux_aoc_spi2,
	google_pinmux_aoc_cli_int_2,
	google_pinmux_aoc_uart1,
	google_pinmux_aoc_uart4,
	google_pinmux_aoc_uart5,
	google_pinmux_aoc_spi5,
	google_pinmux_aoc_i2c3,
	google_pinmux_aoc_cli_int_4,
};

GOOGLE_FUNCTION_GROUPS(aoc_int_gpio, "XAOC_PDM1_MIC_IN", "XAOC_PDM2_MIC_IN",
		       "XAOC_PDM3_MIC_IN", "XAOC_PDM0_MIC_IN", "XAOC_I2S0_WS",
		       "XAOC_I2S0_SDO", "XAOC_I2S0_SDI", "XAOC_GPIO0", "XAOC_GPIO1",
		       "XAOC_UART0_RXD", "XAOC_UART0_TXD", "XAOC_UART0_RTSn",
		       "XAOC_UART0_CTSn", "XAOC_UART3_RXD", "XAOC_UART3_TXD",
		       "XAOC_UART3_RTSn", "XAOC_UART3_CTSn", "XAOC_GPIO2",
		       "XAOC_GPIO3", "XAOC_GPIO4", "XAOC_GPIO5", "XAOC_SPI0_SCLK",
		       "XAOC_SPI0_MOSI", "XAOC_SPI0_MISO", "XAOC_SPI0_SSB",
		       "XAOC_PDM0_FLCKR_IN", "XAOC_PDM0_FLCKR_CLK", "XAOC_I2C2_SCL",
		       "XAOC_I2C2_SDA", "XAOC_I2C4_SCL", "XAOC_I2C4_SDA",
		       "XAOC_TDM0_WS", "XAOC_TDM0_SDO", "XAOC_TDM0_SDI",
		       "XAOC_GPIO6", "XAOC_GPIO7", "XAOC_SPI1_SCLK",
		       "XAOC_SPI1_MOSI", "XAOC_SPI1_MISO", "XAOC_SPI1_SSB",
		       "XAOC_TDM1_BCLK", "XAOC_TDM1_WS", "XAOC_TDM1_SDO",
		       "XAOC_TDM1_SDI", "XAOC_SPI4_SCLK", "XAOC_SPI4_MOSI",
		       "XAOC_SPI4_MISO", "XAOC_SPI4_SSB", "XAOC_GPIO8",
		       "XAOC_GPIO9", "XAOC_GPIO10", "XAOC_GPIO11", "XAOC_I2S1_WS",
		       "XAOC_I2S1_SDO", "XAOC_I2S1_SDI", "XAOC_I2C0_SCL",
		       "XAOC_I2C0_SDA", "XAOC_GPIO12", "XAOC_GPIO13",
		       "XAOC_I3C0_SCL", "XAOC_I3C0_SDA", "XAOC_I3C1_SCL",
		       "XAOC_I3C1_SDA", "XAOC_SPI3_SCLK", "XAOC_SPI3_MOSI",
		       "XAOC_SPI3_MISO", "XAOC_SPI3_SSB", "XAOC_SPI2_SCLK",
		       "XAOC_SPI2_MOSI", "XAOC_SPI2_MISO", "XAOC_SPI2_SSB",
		       "XAOC_UART1_RXD", "XAOC_UART1_TXD", "XAOC_UART1_RTSn",
		       "XAOC_UART1_CTSn", "XAOC_UART4_RXD", "XAOC_UART4_TXD",
		       "XAOC_UART4_RTSn", "XAOC_UART4_CTSn", "XAOC_SPI5_SCLK",
		       "XAOC_SPI5_MOSI", "XAOC_SPI5_MISO", "XAOC_SPI5_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_pdm1, "XAOC_PDM1_MIC_IN", "XAOC_PDM1_MIC_CLK");
GOOGLE_FUNCTION_GROUPS(aoc_pdm2, "XAOC_PDM2_MIC_IN", "XAOC_PDM2_MIC_CLK");
GOOGLE_FUNCTION_GROUPS(aoc_pdm3, "XAOC_PDM3_MIC_IN", "XAOC_PDM3_MIC_CLK");
GOOGLE_FUNCTION_GROUPS(aoc_pdm, "XAOC_PDM0_MIC_IN", "XAOC_PDM0_MIC_CLK");
GOOGLE_FUNCTION_GROUPS(aoc_i2s, "XAOC_I2S0_BCLK", "XAOC_I2S0_WS", "XAOC_I2S0_SDO",
		       "XAOC_I2S0_SDI", "XAOC_I2S1_BCLK", "XAOC_I2S1_WS",
		       "XAOC_I2S1_SDO", "XAOC_I2S1_SDI");
GOOGLE_FUNCTION_GROUPS(aoc_cpm_eint, "XAOC_GPIO0", "XAOC_GPIO1", "XAOC_GPIO2",
		       "XAOC_GPIO3", "XAOC_GPIO4", "XAOC_GPIO5", "XAOC_GPIO6",
		       "XAOC_GPIO7", "XAOC_GPIO8", "XAOC_GPIO9", "XAOC_GPIO10",
		       "XAOC_GPIO11", "XAOC_GPIO12", "XAOC_GPIO13");
GOOGLE_FUNCTION_GROUPS(aoc_uart0, "XAOC_UART0_RXD", "XAOC_UART0_TXD",
		       "XAOC_UART0_RTSn", "XAOC_UART0_CTSn");
GOOGLE_FUNCTION_GROUPS(aoc_cpm_uart, "XAOC_UART0_RXD", "XAOC_UART0_TXD",
		       "XAOC_UART0_RTSn", "XAOC_UART0_CTSn");
GOOGLE_FUNCTION_GROUPS(aoc_uart3, "XAOC_UART3_RXD", "XAOC_UART3_TXD",
		       "XAOC_UART3_RTSn", "XAOC_UART3_CTSn");
GOOGLE_FUNCTION_GROUPS(aoc_fiq, "XAOC_GPIO3", "XAOC_GPIO4", "XAOC_GPIO5",
		       "XAOC_GPIO6", "XAOC_GPIO7", "XAOC_TDM1_BCLK",
		       "XAOC_TDM1_WS", "XAOC_TDM1_SDO", "XAOC_SPI4_MISO",
		       "XAOC_GPIO8", "XAOC_GPIO9", "XAOC_GPIO10", "XAOC_GPIO12",
		       "XAOC_GPIO13", "XAOC_UART4_RXD", "XAOC_UART4_RTSn");
GOOGLE_FUNCTION_GROUPS(aoc_spi0, "XAOC_SPI0_SCLK", "XAOC_SPI0_MOSI",
		       "XAOC_SPI0_MISO", "XAOC_SPI0_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_cli_int_0, "XAOC_SPI0_SCLK", "XAOC_SPI0_MOSI",
		       "XAOC_SPI0_MISO", "XAOC_SPI0_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_out_mclk, "XAOC_OUT_MCLK");
GOOGLE_FUNCTION_GROUPS(aoc_pdm0_flckr, "XAOC_PDM0_FLCKR_IN",
		       "XAOC_PDM0_FLCKR_CLK");
GOOGLE_FUNCTION_GROUPS(aoc_i2c2, "XAOC_I2C2_SCL", "XAOC_I2C2_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i3c5, "XAOC_I2C2_SCL", "XAOC_I2C2_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_elaoutput, "XAOC_I2C2_SCL", "XAOC_I2C2_SDA",
		       "XAOC_I3C1_SCL", "XAOC_I3C1_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i2c4, "XAOC_I2C4_SCL", "XAOC_I2C4_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i3c4, "XAOC_I2C4_SCL", "XAOC_I2C4_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_uart2, "XAOC_I2C4_SCL", "XAOC_I2C4_SDA",
		       "XAOC_I2C0_SCL", "XAOC_I2C0_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_tdm0, "XAOC_TDM0_BCLK", "XAOC_TDM0_WS",
		       "XAOC_TDM0_SDO", "XAOC_TDM0_SDI");
GOOGLE_FUNCTION_GROUPS(aoc_spi1, "XAOC_SPI1_SCLK", "XAOC_SPI1_MOSI",
		       "XAOC_SPI1_MISO", "XAOC_SPI1_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_cli_int_1, "XAOC_SPI1_SCLK", "XAOC_SPI1_MOSI",
		       "XAOC_SPI1_MISO", "XAOC_SPI1_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_tdm1, "XAOC_TDM1_BCLK", "XAOC_TDM1_WS",
		       "XAOC_TDM1_SDO", "XAOC_TDM1_SDI", "XAOC_GPIO8",
		       "XAOC_GPIO9", "XAOC_GPIO10", "XAOC_GPIO11");
GOOGLE_FUNCTION_GROUPS(aoc_sdwire0, "XAOC_TDM1_BCLK", "XAOC_TDM1_WS",
		       "XAOC_GPIO8", "XAOC_GPIO9");
GOOGLE_FUNCTION_GROUPS(aoc_sdwire1, "XAOC_TDM1_SDO", "XAOC_TDM1_SDI",
		       "XAOC_GPIO10", "XAOC_GPIO11");
GOOGLE_FUNCTION_GROUPS(aoc_spi4, "XAOC_SPI4_SCLK", "XAOC_SPI4_MOSI",
		       "XAOC_SPI4_MISO", "XAOC_SPI4_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_i2c0, "XAOC_I2C0_SCL", "XAOC_I2C0_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i3c0, "XAOC_I3C0_SCL", "XAOC_I3C0_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i2c1, "XAOC_I3C1_SCL", "XAOC_I3C1_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i3c1, "XAOC_I3C1_SCL", "XAOC_I3C1_SDA");
GOOGLE_FUNCTION_GROUPS(aoc_i3c3, "XAOC_SPI3_SCLK", "XAOC_SPI3_MOSI");
GOOGLE_FUNCTION_GROUPS(aoc_spi3, "XAOC_SPI3_SCLK", "XAOC_SPI3_MOSI",
		       "XAOC_SPI3_MISO", "XAOC_SPI3_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_cli_int_3, "XAOC_SPI3_SCLK", "XAOC_SPI3_MOSI",
		       "XAOC_SPI3_MISO", "XAOC_SPI3_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_i3c2, "XAOC_SPI2_SCLK", "XAOC_SPI2_MOSI");
GOOGLE_FUNCTION_GROUPS(aoc_spi2, "XAOC_SPI2_SCLK", "XAOC_SPI2_MOSI",
		       "XAOC_SPI2_MISO", "XAOC_SPI2_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_cli_int_2, "XAOC_SPI2_SCLK", "XAOC_SPI2_MOSI",
		       "XAOC_SPI2_MISO", "XAOC_SPI2_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_uart1, "XAOC_UART1_RXD", "XAOC_UART1_TXD",
		       "XAOC_UART1_RTSn", "XAOC_UART1_CTSn");
GOOGLE_FUNCTION_GROUPS(aoc_uart4, "XAOC_UART4_RXD", "XAOC_UART4_TXD",
		       "XAOC_UART4_RTSn", "XAOC_UART4_CTSn");
GOOGLE_FUNCTION_GROUPS(aoc_uart5, "XAOC_SPI5_SCLK", "XAOC_SPI5_MOSI",
		       "XAOC_SPI5_MISO", "XAOC_SPI5_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_spi5, "XAOC_SPI5_SCLK", "XAOC_SPI5_MOSI",
		       "XAOC_SPI5_MISO", "XAOC_SPI5_SSB");
GOOGLE_FUNCTION_GROUPS(aoc_i2c3, "XAOC_SPI5_SCLK", "XAOC_SPI5_MOSI");
GOOGLE_FUNCTION_GROUPS(aoc_cli_int_4, "XAOC_SPI5_SCLK", "XAOC_SPI5_MOSI",
		       "XAOC_SPI5_MISO", "XAOC_SPI5_SSB");

static const struct google_pingroup google_lga_aoc_groups[] = {
	GOOGLE_PIN_GROUP(0, "XAOC_PDM1_MIC_IN", aoc_int_gpio, aoc_pdm1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XAOC_PDM1_MIC_CLK", none, aoc_pdm1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XAOC_PDM2_MIC_IN", aoc_int_gpio, aoc_pdm2, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XAOC_PDM2_MIC_CLK", none, aoc_pdm2, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XAOC_PDM3_MIC_IN", aoc_int_gpio, aoc_pdm3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XAOC_PDM3_MIC_CLK", none, aoc_pdm3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XAOC_PDM0_MIC_IN", aoc_int_gpio, aoc_pdm, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XAOC_PDM0_MIC_CLK", none, aoc_pdm, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XAOC_I2S0_BCLK", none, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XAOC_I2S0_WS", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XAOC_I2S0_SDO", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XAOC_I2S0_SDI", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XAOC_GPIO0", aoc_int_gpio, none, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(13, "XAOC_GPIO1", aoc_int_gpio, none, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(14, "XAOC_UART0_RXD", aoc_int_gpio, aoc_uart0, none, none, none, none, aoc_cpm_uart, none, none),
	GOOGLE_PIN_GROUP(15, "XAOC_UART0_TXD", aoc_int_gpio, aoc_uart0, none, none, none, none, aoc_cpm_uart, none, none),
	GOOGLE_PIN_GROUP(16, "XAOC_UART0_RTSn", aoc_int_gpio, aoc_uart0, none, none, none, none, aoc_cpm_uart, none, none),
	GOOGLE_PIN_GROUP(17, "XAOC_UART0_CTSn", aoc_int_gpio, aoc_uart0, none, none, none, none, aoc_cpm_uart, none, none),
	GOOGLE_PIN_GROUP(18, "XAOC_UART3_RXD", aoc_int_gpio, aoc_uart3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(19, "XAOC_UART3_TXD", aoc_int_gpio, aoc_uart3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(20, "XAOC_UART3_RTSn", aoc_int_gpio, aoc_uart3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(21, "XAOC_UART3_CTSn", aoc_int_gpio, aoc_uart3, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(22, "XAOC_GPIO2", aoc_int_gpio, none, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(23, "XAOC_GPIO3", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(24, "XAOC_GPIO4", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(25, "XAOC_GPIO5", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(26, "XAOC_SPI0_SCLK", aoc_int_gpio, none, aoc_spi0, none, none, aoc_cli_int_0, none, none, none),
	GOOGLE_PIN_GROUP(27, "XAOC_SPI0_MOSI", aoc_int_gpio, none, aoc_spi0, none, none, aoc_cli_int_0, none, none, none),
	GOOGLE_PIN_GROUP(28, "XAOC_SPI0_MISO", aoc_int_gpio, none, aoc_spi0, none, none, aoc_cli_int_0, none, none, none),
	GOOGLE_PIN_GROUP(29, "XAOC_SPI0_SSB", aoc_int_gpio, none, aoc_spi0, none, none, aoc_cli_int_0, none, none, none),
	GOOGLE_PIN_GROUP(30, "XAOC_OUT_MCLK", none, aoc_out_mclk, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(31, "XAOC_PDM0_FLCKR_IN", aoc_int_gpio, aoc_pdm0_flckr, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(32, "XAOC_PDM0_FLCKR_CLK", aoc_int_gpio, aoc_pdm0_flckr, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(33, "XAOC_I2C2_SCL", aoc_int_gpio, aoc_i2c2, aoc_i3c5, aoc_elaoutput, none, none, none, none, none),
	GOOGLE_PIN_GROUP(34, "XAOC_I2C2_SDA", aoc_int_gpio, aoc_i2c2, aoc_i3c5, aoc_elaoutput, none, none, none, none, none),
	GOOGLE_PIN_GROUP(35, "XAOC_I2C4_SCL", aoc_int_gpio, aoc_i2c4, aoc_i3c4, aoc_uart2, none, none, none, none, none),
	GOOGLE_PIN_GROUP(36, "XAOC_I2C4_SDA", aoc_int_gpio, aoc_i2c4, aoc_i3c4, aoc_uart2, none, none, none, none, none),
	GOOGLE_PIN_GROUP(37, "XAOC_TDM0_BCLK", none, aoc_tdm0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(38, "XAOC_TDM0_WS", aoc_int_gpio, aoc_tdm0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(39, "XAOC_TDM0_SDO", aoc_int_gpio, aoc_tdm0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(40, "XAOC_TDM0_SDI", aoc_int_gpio, aoc_tdm0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(41, "XAOC_GPIO6", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(42, "XAOC_GPIO7", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(43, "XAOC_SPI1_SCLK", aoc_int_gpio, none, aoc_spi1, none, none, aoc_cli_int_1, none, none, none),
	GOOGLE_PIN_GROUP(44, "XAOC_SPI1_MOSI", aoc_int_gpio, none, aoc_spi1, none, none, aoc_cli_int_1, none, none, none),
	GOOGLE_PIN_GROUP(45, "XAOC_SPI1_MISO", aoc_int_gpio, none, aoc_spi1, none, none, aoc_cli_int_1, none, none, none),
	GOOGLE_PIN_GROUP(46, "XAOC_SPI1_SSB", aoc_int_gpio, none, aoc_spi1, none, none, aoc_cli_int_1, none, none, none),
	GOOGLE_PIN_GROUP(47, "XAOC_TDM1_BCLK", aoc_int_gpio, aoc_tdm1, aoc_sdwire0, aoc_fiq, none, none, none, none, none),
	GOOGLE_PIN_GROUP(48, "XAOC_TDM1_WS", aoc_int_gpio, aoc_tdm1, aoc_sdwire0, aoc_fiq, none, none, none, none, none),
	GOOGLE_PIN_GROUP(49, "XAOC_TDM1_SDO", aoc_int_gpio, aoc_tdm1, aoc_sdwire1, aoc_fiq, none, none, none, none, none),
	GOOGLE_PIN_GROUP(50, "XAOC_TDM1_SDI", aoc_int_gpio, aoc_tdm1, aoc_sdwire1, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(51, "XAOC_SPI4_SCLK", aoc_int_gpio, none, aoc_spi4, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(52, "XAOC_SPI4_MOSI", aoc_int_gpio, none, aoc_spi4, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(53, "XAOC_SPI4_MISO", aoc_int_gpio, aoc_fiq, aoc_spi4, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(54, "XAOC_SPI4_SSB", aoc_int_gpio, none, aoc_spi4, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(55, "XAOC_GPIO8", aoc_int_gpio, none, aoc_tdm1, aoc_sdwire0, aoc_fiq, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(56, "XAOC_GPIO9", aoc_int_gpio, none, aoc_tdm1, aoc_sdwire0, aoc_fiq, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(57, "XAOC_GPIO10", aoc_int_gpio, none, aoc_tdm1, aoc_sdwire1, aoc_fiq, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(58, "XAOC_GPIO11", aoc_int_gpio, none, aoc_tdm1, aoc_sdwire1, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(59, "XAOC_I2S1_BCLK", none, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(60, "XAOC_I2S1_WS", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(61, "XAOC_I2S1_SDO", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(62, "XAOC_I2S1_SDI", aoc_int_gpio, aoc_i2s, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(63, "XAOC_I2C0_SCL", aoc_int_gpio, aoc_i2c0, aoc_uart2, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(64, "XAOC_I2C0_SDA", aoc_int_gpio, aoc_i2c0, aoc_uart2, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(65, "XAOC_GPIO12", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(66, "XAOC_GPIO13", aoc_int_gpio, aoc_fiq, none, none, none, none, aoc_cpm_eint, none, none),
	GOOGLE_PIN_GROUP(67, "XAOC_I3C0_SCL", aoc_int_gpio, aoc_i3c0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(68, "XAOC_I3C0_SDA", aoc_int_gpio, aoc_i3c0, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(69, "XAOC_I3C1_SCL", aoc_int_gpio, aoc_i2c1, aoc_i3c1, aoc_elaoutput, none, none, none, none, none),
	GOOGLE_PIN_GROUP(70, "XAOC_I3C1_SDA", aoc_int_gpio, aoc_i2c1, aoc_i3c1, aoc_elaoutput, none, none, none, none, none),
	GOOGLE_PIN_GROUP(71, "XAOC_SPI3_SCLK", aoc_int_gpio, aoc_i3c3, aoc_spi3, none, none, aoc_cli_int_3, none, none, none),
	GOOGLE_PIN_GROUP(72, "XAOC_SPI3_MOSI", aoc_int_gpio, aoc_i3c3, aoc_spi3, none, none, aoc_cli_int_3, none, none, none),
	GOOGLE_PIN_GROUP(73, "XAOC_SPI3_MISO", aoc_int_gpio, none, aoc_spi3, none, none, aoc_cli_int_3, none, none, none),
	GOOGLE_PIN_GROUP(74, "XAOC_SPI3_SSB", aoc_int_gpio, none, aoc_spi3, none, none, aoc_cli_int_3, none, none, none),
	GOOGLE_PIN_GROUP(75, "XAOC_SPI2_SCLK", aoc_int_gpio, aoc_i3c2, aoc_spi2, none, none, aoc_cli_int_2, none, none, none),
	GOOGLE_PIN_GROUP(76, "XAOC_SPI2_MOSI", aoc_int_gpio, aoc_i3c2, aoc_spi2, none, none, aoc_cli_int_2, none, none, none),
	GOOGLE_PIN_GROUP(77, "XAOC_SPI2_MISO", aoc_int_gpio, none, aoc_spi2, none, none, aoc_cli_int_2, none, none, none),
	GOOGLE_PIN_GROUP(78, "XAOC_SPI2_SSB", aoc_int_gpio, none, aoc_spi2, none, none, aoc_cli_int_2, none, none, none),
	GOOGLE_PIN_GROUP(79, "XAOC_UART1_RXD", aoc_int_gpio, aoc_uart1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(80, "XAOC_UART1_TXD", aoc_int_gpio, aoc_uart1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(81, "XAOC_UART1_RTSn", aoc_int_gpio, aoc_uart1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(82, "XAOC_UART1_CTSn", aoc_int_gpio, aoc_uart1, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(83, "XAOC_UART4_RXD", aoc_int_gpio, aoc_uart4, aoc_fiq, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(84, "XAOC_UART4_TXD", aoc_int_gpio, aoc_uart4, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(85, "XAOC_UART4_RTSn", aoc_int_gpio, aoc_uart4, aoc_fiq, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(86, "XAOC_UART4_CTSn", aoc_int_gpio, aoc_uart4, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(87, "XAOC_SPI5_SCLK", aoc_int_gpio, aoc_uart5, aoc_spi5, aoc_i2c3, none, aoc_cli_int_4, none, none, none),
	GOOGLE_PIN_GROUP(88, "XAOC_SPI5_MOSI", aoc_int_gpio, aoc_uart5, aoc_spi5, aoc_i2c3, none, aoc_cli_int_4, none, none, none),
	GOOGLE_PIN_GROUP(89, "XAOC_SPI5_MISO", aoc_int_gpio, aoc_uart5, aoc_spi5, none, none, aoc_cli_int_4, none, none, none),
	GOOGLE_PIN_GROUP(90, "XAOC_SPI5_SSB", aoc_int_gpio, aoc_uart5, aoc_spi5, none, none, aoc_cli_int_4, none, none, none),
};

static const struct google_pin_function google_lga_aoc_functions[] = {
	GOOGLE_FUNCTION(aoc_int_gpio),
	GOOGLE_FUNCTION(aoc_pdm1),
	GOOGLE_FUNCTION(aoc_pdm2),
	GOOGLE_FUNCTION(aoc_pdm3),
	GOOGLE_FUNCTION(aoc_pdm),
	GOOGLE_FUNCTION(aoc_i2s),
	GOOGLE_FUNCTION(aoc_cpm_eint),
	GOOGLE_FUNCTION(aoc_uart0),
	GOOGLE_FUNCTION(aoc_cpm_uart),
	GOOGLE_FUNCTION(aoc_uart3),
	GOOGLE_FUNCTION(aoc_fiq),
	GOOGLE_FUNCTION(aoc_spi0),
	GOOGLE_FUNCTION(aoc_cli_int_0),
	GOOGLE_FUNCTION(aoc_out_mclk),
	GOOGLE_FUNCTION(aoc_pdm0_flckr),
	GOOGLE_FUNCTION(aoc_i2c2),
	GOOGLE_FUNCTION(aoc_i3c5),
	GOOGLE_FUNCTION(aoc_elaoutput),
	GOOGLE_FUNCTION(aoc_i2c4),
	GOOGLE_FUNCTION(aoc_i3c4),
	GOOGLE_FUNCTION(aoc_uart2),
	GOOGLE_FUNCTION(aoc_tdm0),
	GOOGLE_FUNCTION(aoc_spi1),
	GOOGLE_FUNCTION(aoc_cli_int_1),
	GOOGLE_FUNCTION(aoc_tdm1),
	GOOGLE_FUNCTION(aoc_sdwire0),
	GOOGLE_FUNCTION(aoc_sdwire1),
	GOOGLE_FUNCTION(aoc_spi4),
	GOOGLE_FUNCTION(aoc_i2c0),
	GOOGLE_FUNCTION(aoc_i3c0),
	GOOGLE_FUNCTION(aoc_i2c1),
	GOOGLE_FUNCTION(aoc_i3c1),
	GOOGLE_FUNCTION(aoc_i3c3),
	GOOGLE_FUNCTION(aoc_spi3),
	GOOGLE_FUNCTION(aoc_cli_int_3),
	GOOGLE_FUNCTION(aoc_i3c2),
	GOOGLE_FUNCTION(aoc_spi2),
	GOOGLE_FUNCTION(aoc_cli_int_2),
	GOOGLE_FUNCTION(aoc_uart1),
	GOOGLE_FUNCTION(aoc_uart4),
	GOOGLE_FUNCTION(aoc_uart5),
	GOOGLE_FUNCTION(aoc_spi5),
	GOOGLE_FUNCTION(aoc_i2c3),
	GOOGLE_FUNCTION(aoc_cli_int_4),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_aoc = {
	.pins = google_lga_aoc,
	.num_pins = ARRAY_SIZE(google_lga_aoc),
	.groups = google_lga_aoc_groups,
	.num_groups = ARRAY_SIZE(google_lga_aoc_groups),
	.funcs = google_lga_aoc_functions,
	.num_funcs = ARRAY_SIZE(google_lga_aoc_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "aoc",
};

/* GDMC */
static const struct pinctrl_pin_desc google_lga_gdmc[] = {
	PINCTRL_PIN(0, "XGDMC_UART0_RXD"),
	PINCTRL_PIN(1, "XGDMC_UART0_TXD"),
	PINCTRL_PIN(2, "XGDMC_IN_DEBUG_1"),
	PINCTRL_PIN(3, "XGDMC_DEBUG_DEFAULT_1"),
	PINCTRL_PIN(4, "XGDMC_UART_MODEM_RXD"),
	PINCTRL_PIN(5, "XGDMC_UART_MODEM_TXD"),
	PINCTRL_PIN(6, "XGDMC_DBGSEL"),
	PINCTRL_PIN(7, "XGDMC_JTAG_TRST_n"),
	PINCTRL_PIN(8, "XGDMC_JTAG_SRST_n"),
	PINCTRL_PIN(9, "XGDMC_JTAG_TMS"),
	PINCTRL_PIN(10, "XGDMC_JTAG_TCK"),
	PINCTRL_PIN(11, "XGDMC_JTAG_TDI"),
	PINCTRL_PIN(12, "XGDMC_JTAG_TDO"),
	PINCTRL_PIN(13, "XGDMC_SOC_DEBUG_UART_RXD"),
	PINCTRL_PIN(14, "XGDMC_SOC_DEBUG_UART_TXD"),
	PINCTRL_PIN(15, "XGDMC_CTI_0"),
	PINCTRL_PIN(16, "XGDMC_UART_GSC_RXD"),
	PINCTRL_PIN(17, "XGDMC_UART_GSC_TXD"),
	PINCTRL_PIN(18, "XGDMC_IN_DEBUG_0"),
	PINCTRL_PIN(19, "XGDMC_DEBUG_DEFAULT_0"),
	PINCTRL_PIN(20, "XGDMC_CTI_1"),
};

enum google_lga_gdmc_funcs {
	google_pinmux_gdmc_int,
	google_pinmux_gdmc_uart,
	google_pinmux_gdmc_debug,
	google_pinmux_gdmc_dbgsel,
	google_pinmux_gdmc_jtag,
	google_pinmux_gdmc_cti,
};

GOOGLE_FUNCTION_GROUPS(gdmc_int, "XGDMC_UART0_RXD", "XGDMC_UART0_TXD",
		       "XGDMC_IN_DEBUG_1", "XGDMC_DEBUG_DEFAULT_1",
		       "XGDMC_UART_MODEM_RXD", "XGDMC_UART_MODEM_TXD",
		       "XGDMC_SOC_DEBUG_UART_RXD", "XGDMC_SOC_DEBUG_UART_TXD",
		       "XGDMC_CTI_0", "XGDMC_UART_GSC_RXD",
		       "XGDMC_UART_GSC_TXD", "XGDMC_IN_DEBUG_0",
		       "XGDMC_DEBUG_DEFAULT_0", "XGDMC_CTI_1");
GOOGLE_FUNCTION_GROUPS(gdmc_uart, "XGDMC_UART0_RXD", "XGDMC_UART0_TXD",
		       "XGDMC_UART_MODEM_RXD", "XGDMC_UART_MODEM_TXD",
		       "XGDMC_UART_GSC_RXD", "XGDMC_UART_GSC_TXD");
GOOGLE_FUNCTION_GROUPS(gdmc_debug, "XGDMC_IN_DEBUG_1", "XGDMC_DEBUG_DEFAULT_1",
		       "XGDMC_SOC_DEBUG_UART_RXD", "XGDMC_SOC_DEBUG_UART_TXD",
		       "XGDMC_IN_DEBUG_0", "XGDMC_DEBUG_DEFAULT_0");
GOOGLE_FUNCTION_GROUPS(gdmc_dbgsel, "XGDMC_DBGSEL");
GOOGLE_FUNCTION_GROUPS(gdmc_jtag, "XGDMC_JTAG_TRST_n", "XGDMC_JTAG_SRST_n",
		       "XGDMC_JTAG_TMS", "XGDMC_JTAG_TCK", "XGDMC_JTAG_TDI",
		       "XGDMC_JTAG_TDO");
GOOGLE_FUNCTION_GROUPS(gdmc_cti, "XGDMC_CTI_0", "XGDMC_CTI_1");

static const struct google_pingroup google_lga_gdmc_groups[] = {
	GOOGLE_PIN_GROUP(0, "XGDMC_UART0_RXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XGDMC_UART0_TXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XGDMC_IN_DEBUG_1", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XGDMC_DEBUG_DEFAULT_1", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XGDMC_UART_MODEM_RXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XGDMC_UART_MODEM_TXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XGDMC_DBGSEL", none, gdmc_dbgsel, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XGDMC_JTAG_TRST_n", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XGDMC_JTAG_SRST_n", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XGDMC_JTAG_TMS", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XGDMC_JTAG_TCK", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XGDMC_JTAG_TDI", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XGDMC_JTAG_TDO", none, gdmc_jtag, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(13, "XGDMC_SOC_DEBUG_UART_RXD", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(14, "XGDMC_SOC_DEBUG_UART_TXD", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(15, "XGDMC_CTI_0", gdmc_int, gdmc_cti, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(16, "XGDMC_UART_GSC_RXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(17, "XGDMC_UART_GSC_TXD", gdmc_int, gdmc_uart, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(18, "XGDMC_IN_DEBUG_0", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(19, "XGDMC_DEBUG_DEFAULT_0", gdmc_int, gdmc_debug, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(20, "XGDMC_CTI_1", gdmc_int, gdmc_cti, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_gdmc_functions[] = {
	GOOGLE_FUNCTION(gdmc_int),
	GOOGLE_FUNCTION(gdmc_uart),
	GOOGLE_FUNCTION(gdmc_debug),
	GOOGLE_FUNCTION(gdmc_dbgsel),
	GOOGLE_FUNCTION(gdmc_jtag),
	GOOGLE_FUNCTION(gdmc_cti),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_gdmc = {
	.pins = google_lga_gdmc,
	.num_pins = ARRAY_SIZE(google_lga_gdmc),
	.groups = google_lga_gdmc_groups,
	.num_groups = ARRAY_SIZE(google_lga_gdmc_groups),
	.funcs = google_lga_gdmc_functions,
	.num_funcs = ARRAY_SIZE(google_lga_gdmc_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "gdmc",
};

/* CPM */
static const struct pinctrl_pin_desc google_lga_cpm[] = {
	PINCTRL_PIN(0, "XCPM_CLKBUF_0"),
	PINCTRL_PIN(1, "XCPM_IN_STBY_CLK"),
	PINCTRL_PIN(2, "XCPM_PWR_REQ"),
	PINCTRL_PIN(3, "XCPM_XnRESET"),
	PINCTRL_PIN(4, "XCPM_XnWRESET"),
	PINCTRL_PIN(5, "XCPM_XnRESET_OUT"),
	PINCTRL_PIN(6, "XCPM_SPMI0_SCLK"),
	PINCTRL_PIN(7, "XCPM_SPMI0_SDATA"),
	PINCTRL_PIN(8, "XCPM_PRE_UVLO"),
	PINCTRL_PIN(9, "XCPM_PRE_OCP_TPU"),
	PINCTRL_PIN(10, "XCPM_EINT_0"),
	PINCTRL_PIN(11, "XCPM_EINT_1"),
	PINCTRL_PIN(12, "XCPM_SOFT_PRE_OCP_TPU"),
	PINCTRL_PIN(13, "XCPM_PRE_OCP_CPU1"),
	PINCTRL_PIN(14, "XCPM_PRE_OCP_CPU2"),
	PINCTRL_PIN(15, "XCPM_SOFT_PRE_OCP_CPU1"),
	PINCTRL_PIN(16, "XCPM_SOFT_PRE_OCP_CPU2"),
	PINCTRL_PIN(17, "XCPM_PRE_OCP_CPU0"),
	PINCTRL_PIN(18, "XCPM_SOFT_PRE_OCP_CPU0"),
	PINCTRL_PIN(19, "XCPM_EINT_2"),
	PINCTRL_PIN(20, "XCPM_EINT_3"),
	PINCTRL_PIN(21, "XCPM_CLKOUT0"),
	PINCTRL_PIN(22, "XCPM_CLKOUT0_EN"),
	PINCTRL_PIN(23, "XCPM_CLKOUT1"),
	PINCTRL_PIN(24, "XCPM_CLKOUT1_EN"),
	PINCTRL_PIN(25, "XCPM_EINT_4"),
	PINCTRL_PIN(26, "XCPM_EINT_5"),
	PINCTRL_PIN(27, "XCPM_EINT_6"),
	PINCTRL_PIN(28, "XCPM_M0_BOOT_SEL"),
	PINCTRL_PIN(29, "XCPM_OTP_EMU_MODE"),
	PINCTRL_PIN(30, "XCPM_BOOT_CONFIG_SPARE"),
	PINCTRL_PIN(31, "XCPM_EINT_7"),
	PINCTRL_PIN(32, "XCPM_EINT_8"),
	PINCTRL_PIN(33, "XCPM_EINT_9"),
	PINCTRL_PIN(34, "XCPM_VDROOP1"),
	PINCTRL_PIN(35, "XCPM_VDROOP2"),
	PINCTRL_PIN(36, "XCPM_I2C0_SCL"),
	PINCTRL_PIN(37, "XCPM_I2C0_SDA"),
	PINCTRL_PIN(38, "XCPM_EINT_10"),
	PINCTRL_PIN(39, "XCPM_EINT_11"),
	PINCTRL_PIN(40, "XCPM_SRC_OPT0"),
	PINCTRL_PIN(41, "XCPM_SRC_OPT1"),
	PINCTRL_PIN(42, "XCPM_SRC_OPT2"),
	PINCTRL_PIN(43, "XCPM_BP_PWROK"),
	PINCTRL_PIN(44, "XCPM_SPI0_SCLK"),
	PINCTRL_PIN(45, "XCPM_SPI0_MOSI"),
	PINCTRL_PIN(46, "XCPM_SPI0_MISO"),
	PINCTRL_PIN(47, "XCPM_SPI0_SSB"),
	PINCTRL_PIN(48, "XCPM_GPI0"),
	PINCTRL_PIN(49, "XCPM_CLKBUF_ON"),
	PINCTRL_PIN(50, "XCPM_XOM_0"),
	PINCTRL_PIN(51, "XCPM_GPI1"),
	PINCTRL_PIN(52, "XCPM_EINT_12"),
	PINCTRL_PIN(53, "XCPM_EINT_13"),
	PINCTRL_PIN(54, "XCPM_EINT_14"),
	PINCTRL_PIN(55, "XCPM_EINT_15"),
	PINCTRL_PIN(56, "XCPM_EINT_16"),
};

enum google_lga_cpm_funcs {
	google_pinmux_cpm_clkbuf,
	google_pinmux_cpm_stby,
	google_pinmux_cpm_pwr,
	google_pinmux_cpm_reset,
	google_pinmux_cpm_int,
	google_pinmux_cpm_spmi,
	google_pinmux_cpm_pre,
	google_pinmux_cpm_irq,
	google_pinmux_cpm_clkout,
	google_pinmux_cpm_xtal,
	google_pinmux_cpm_uart0,
	google_pinmux_cpm_m0_uart,
	google_pinmux_cpm_boot,
	google_pinmux_cpm_otp_emu,
	google_pinmux_cpm_xom,
	google_pinmux_cpm_vdroop,
	google_pinmux_cpm_i2c,
	google_pinmux_cpm_src,
	google_pinmux_cpm_pwrok,
	google_pinmux_cpm_spi,
};

GOOGLE_FUNCTION_GROUPS(cpm_clkbuf, "XCPM_CLKBUF_0", "XCPM_CLKBUF_ON");
GOOGLE_FUNCTION_GROUPS(cpm_stby, "XCPM_IN_STBY_CLK");
GOOGLE_FUNCTION_GROUPS(cpm_pwr, "XCPM_PWR_REQ");
GOOGLE_FUNCTION_GROUPS(cpm_reset, "XCPM_XnRESET", "XCPM_XnWRESET",
		       "XCPM_XnRESET_OUT");
GOOGLE_FUNCTION_GROUPS(cpm_int, "XCPM_SPMI0_SCLK", "XCPM_SPMI0_SDATA",
		       "XCPM_PRE_UVLO", "XCPM_PRE_OCP_TPU", "XCPM_EINT_0",
		       "XCPM_EINT_1", "XCPM_SOFT_PRE_OCP_TPU",
		       "XCPM_PRE_OCP_CPU1", "XCPM_PRE_OCP_CPU2",
		       "XCPM_SOFT_PRE_OCP_CPU1", "XCPM_SOFT_PRE_OCP_CPU2",
		       "XCPM_PRE_OCP_CPU0", "XCPM_SOFT_PRE_OCP_CPU0",
		       "XCPM_EINT_2", "XCPM_EINT_3", "XCPM_CLKOUT0",
		       "XCPM_CLKOUT0_EN", "XCPM_CLKOUT1", "XCPM_CLKOUT1_EN",
		       "XCPM_EINT_4", "XCPM_EINT_5", "XCPM_EINT_6",
		       "XCPM_M0_BOOT_SEL", "XCPM_OTP_EMU_MODE",
		       "XCPM_BOOT_CONFIG_SPARE", "XCPM_EINT_7", "XCPM_EINT_8",
		       "XCPM_EINT_9", "XCPM_VDROOP1", "XCPM_VDROOP2",
		       "XCPM_I2C0_SCL", "XCPM_I2C0_SDA", "XCPM_EINT_10",
		       "XCPM_EINT_11", "XCPM_SRC_OPT0", "XCPM_SRC_OPT1",
		       "XCPM_SRC_OPT2", "XCPM_SPI0_SCLK", "XCPM_SPI0_MOSI",
		       "XCPM_SPI0_MISO", "XCPM_SPI0_SSB", "XCPM_GPI0",
		       "XCPM_CLKBUF_ON", "XCPM_XOM_0", "XCPM_GPI1",
		       "XCPM_EINT_12", "XCPM_EINT_13", "XCPM_EINT_14",
		       "XCPM_EINT_15", "XCPM_EINT_16");
GOOGLE_FUNCTION_GROUPS(cpm_spmi, "XCPM_SPMI0_SCLK", "XCPM_SPMI0_SDATA",
		       "XCPM_I2C0_SCL", "XCPM_I2C0_SDA");
GOOGLE_FUNCTION_GROUPS(cpm_pre, "XCPM_PRE_UVLO", "XCPM_PRE_OCP_TPU",
		       "XCPM_SOFT_PRE_OCP_TPU", "XCPM_PRE_OCP_CPU1",
		       "XCPM_PRE_OCP_CPU2", "XCPM_SOFT_PRE_OCP_CPU1",
		       "XCPM_SOFT_PRE_OCP_CPU2", "XCPM_PRE_OCP_CPU0",
		       "XCPM_SOFT_PRE_OCP_CPU0");
GOOGLE_FUNCTION_GROUPS(cpm_irq, "XCPM_EINT_0", "XCPM_EINT_2");
GOOGLE_FUNCTION_GROUPS(cpm_clkout, "XCPM_CLKOUT0", "XCPM_CLKOUT0_EN",
		       "XCPM_CLKOUT1", "XCPM_CLKOUT1_EN");
GOOGLE_FUNCTION_GROUPS(cpm_xtal, "XCPM_CLKOUT1");
GOOGLE_FUNCTION_GROUPS(cpm_uart0, "XCPM_EINT_5", "XCPM_EINT_6");
GOOGLE_FUNCTION_GROUPS(cpm_m0_uart, "XCPM_EINT_6");
GOOGLE_FUNCTION_GROUPS(cpm_boot, "XCPM_M0_BOOT_SEL", "XCPM_BOOT_CONFIG_SPARE");
GOOGLE_FUNCTION_GROUPS(cpm_otp_emu, "XCPM_OTP_EMU_MODE");
GOOGLE_FUNCTION_GROUPS(cpm_xom, "XCPM_EINT_8", "XCPM_EINT_9", "XCPM_XOM_0",
		       "XCPM_EINT_15");
GOOGLE_FUNCTION_GROUPS(cpm_vdroop, "XCPM_VDROOP1", "XCPM_VDROOP2");
GOOGLE_FUNCTION_GROUPS(cpm_i2c, "XCPM_I2C0_SCL", "XCPM_I2C0_SDA");
GOOGLE_FUNCTION_GROUPS(cpm_src, "XCPM_SRC_OPT0", "XCPM_SRC_OPT1",
		       "XCPM_SRC_OPT2");
GOOGLE_FUNCTION_GROUPS(cpm_pwrok, "XCPM_BP_PWROK");
GOOGLE_FUNCTION_GROUPS(cpm_spi, "XCPM_SPI0_SCLK", "XCPM_SPI0_MOSI",
		       "XCPM_SPI0_MISO", "XCPM_SPI0_SSB");

static const struct google_pingroup google_lga_cpm_groups[] = {
	GOOGLE_PIN_GROUP(0, "XCPM_CLKBUF_0", none, cpm_clkbuf, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XCPM_IN_STBY_CLK", none, cpm_stby, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XCPM_PWR_REQ", none, cpm_pwr, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XCPM_XnRESET", none, cpm_reset, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XCPM_XnWRESET", none, cpm_reset, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XCPM_XnRESET_OUT", none, cpm_reset, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XCPM_SPMI0_SCLK", cpm_int, none, cpm_spmi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XCPM_SPMI0_SDATA", cpm_int, none, cpm_spmi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XCPM_PRE_UVLO", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XCPM_PRE_OCP_TPU", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XCPM_EINT_0", cpm_int, cpm_irq, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XCPM_EINT_1", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XCPM_SOFT_PRE_OCP_TPU", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(13, "XCPM_PRE_OCP_CPU1", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(14, "XCPM_PRE_OCP_CPU2", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(15, "XCPM_SOFT_PRE_OCP_CPU1", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(16, "XCPM_SOFT_PRE_OCP_CPU2", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(17, "XCPM_PRE_OCP_CPU0", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(18, "XCPM_SOFT_PRE_OCP_CPU0", cpm_int, cpm_pre, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(19, "XCPM_EINT_2", cpm_int, cpm_irq, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(20, "XCPM_EINT_3", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(21, "XCPM_CLKOUT0", cpm_int, cpm_clkout, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(22, "XCPM_CLKOUT0_EN", cpm_int, cpm_clkout, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(23, "XCPM_CLKOUT1", cpm_int, cpm_clkout, cpm_xtal, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(24, "XCPM_CLKOUT1_EN", cpm_int, cpm_clkout, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(25, "XCPM_EINT_4", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(26, "XCPM_EINT_5", cpm_int, none, cpm_uart0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(27, "XCPM_EINT_6", cpm_int, none, cpm_uart0, cpm_m0_uart, none, none, none, none, none),
	GOOGLE_PIN_GROUP(28, "XCPM_M0_BOOT_SEL", cpm_int, cpm_boot, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(29, "XCPM_OTP_EMU_MODE", cpm_int, cpm_otp_emu, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(30, "XCPM_BOOT_CONFIG_SPARE", cpm_int, cpm_boot, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(31, "XCPM_EINT_7", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(32, "XCPM_EINT_8", cpm_int, cpm_xom, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(33, "XCPM_EINT_9", cpm_int, cpm_xom, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(34, "XCPM_VDROOP1", cpm_int, cpm_vdroop, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(35, "XCPM_VDROOP2", cpm_int, cpm_vdroop, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(36, "XCPM_I2C0_SCL", cpm_int, cpm_i2c, cpm_spmi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(37, "XCPM_I2C0_SDA", cpm_int, cpm_i2c, cpm_spmi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(38, "XCPM_EINT_10", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(39, "XCPM_EINT_11", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(40, "XCPM_SRC_OPT0", cpm_int, cpm_src, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(41, "XCPM_SRC_OPT1", cpm_int, cpm_src, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(42, "XCPM_SRC_OPT2", cpm_int, cpm_src, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(43, "XCPM_BP_PWROK", none, cpm_pwrok, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(44, "XCPM_SPI0_SCLK", cpm_int, none, cpm_spi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(45, "XCPM_SPI0_MOSI", cpm_int, none, cpm_spi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(46, "XCPM_SPI0_MISO", cpm_int, none, cpm_spi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(47, "XCPM_SPI0_SSB", cpm_int, none, cpm_spi, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(48, "XCPM_GPI0", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(49, "XCPM_CLKBUF_ON", cpm_int, cpm_clkbuf, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(50, "XCPM_XOM_0", cpm_int, cpm_xom, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(51, "XCPM_GPI1", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(52, "XCPM_EINT_12", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(53, "XCPM_EINT_13", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(54, "XCPM_EINT_14", cpm_int, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(55, "XCPM_EINT_15", cpm_int, cpm_xom, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(56, "XCPM_EINT_16", cpm_int, none, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_cpm_functions[] = {
	GOOGLE_FUNCTION(cpm_clkbuf),
	GOOGLE_FUNCTION(cpm_stby),
	GOOGLE_FUNCTION(cpm_pwr),
	GOOGLE_FUNCTION(cpm_reset),
	GOOGLE_FUNCTION(cpm_int),
	GOOGLE_FUNCTION(cpm_spmi),
	GOOGLE_FUNCTION(cpm_pre),
	GOOGLE_FUNCTION(cpm_irq),
	GOOGLE_FUNCTION(cpm_clkout),
	GOOGLE_FUNCTION(cpm_xtal),
	GOOGLE_FUNCTION(cpm_uart0),
	GOOGLE_FUNCTION(cpm_m0_uart),
	GOOGLE_FUNCTION(cpm_boot),
	GOOGLE_FUNCTION(cpm_otp_emu),
	GOOGLE_FUNCTION(cpm_xom),
	GOOGLE_FUNCTION(cpm_vdroop),
	GOOGLE_FUNCTION(cpm_i2c),
	GOOGLE_FUNCTION(cpm_src),
	GOOGLE_FUNCTION(cpm_pwrok),
	GOOGLE_FUNCTION(cpm_spi),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_cpm = {
	.pins = google_lga_cpm,
	.num_pins = ARRAY_SIZE(google_lga_cpm),
	.groups = google_lga_cpm_groups,
	.num_groups = ARRAY_SIZE(google_lga_cpm_groups),
	.funcs = google_lga_cpm_functions,
	.num_funcs = ARRAY_SIZE(google_lga_cpm_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "cpm",
};

/* LSIOS */
static const struct pinctrl_pin_desc google_lga_lsios[] = {
	PINCTRL_PIN(0, "XLSIOS_MCLK1"),
	PINCTRL_PIN(1, "XLSIOS_MCLK2"),
	PINCTRL_PIN(2, "XLSIOS_MCLK3"),
	PINCTRL_PIN(3, "XLSIOS_MCLK4"),
	PINCTRL_PIN(4, "XLSIOS_MCLK5"),
	PINCTRL_PIN(5, "XLSIOS_MCLK6"),
	PINCTRL_PIN(6, "XLSIOS_MCLK7"),
	PINCTRL_PIN(7, "XLSIOS_CLI0_PIN0"),
	PINCTRL_PIN(8, "XLSIOS_CLI0_PIN1"),
	PINCTRL_PIN(9, "XLSIOS_CLI0_PIN2"),
	PINCTRL_PIN(10, "XLSIOS_CLI0_PIN3"),
	PINCTRL_PIN(11, "XLSIOS_CLI1_PIN0"),
	PINCTRL_PIN(12, "XLSIOS_CLI1_PIN1"),
	PINCTRL_PIN(13, "XLSIOS_CLI1_PIN2"),
	PINCTRL_PIN(14, "XLSIOS_CLI1_PIN3"),
	PINCTRL_PIN(15, "XLSIOS_CLI2_PIN0"),
	PINCTRL_PIN(16, "XLSIOS_CLI2_PIN1"),
	PINCTRL_PIN(17, "XLSIOS_CLI2_PIN2"),
	PINCTRL_PIN(18, "XLSIOS_CLI2_PIN3"),
	PINCTRL_PIN(19, "XLSIOS_CLI3_PIN0"),
	PINCTRL_PIN(20, "XLSIOS_CLI3_PIN1"),
	PINCTRL_PIN(21, "XLSIOS_CLI3_PIN2"),
	PINCTRL_PIN(22, "XLSIOS_CLI3_PIN3"),
	PINCTRL_PIN(23, "XLSIOS_PWM_0"),
	PINCTRL_PIN(24, "XLSIOS_PWM_1"),
	PINCTRL_PIN(25, "XLSIOS_VSYNC1"),
	PINCTRL_PIN(26, "XLSIOS_VSYNC2"),
	PINCTRL_PIN(27, "XLSIOS_VSYNC3"),
	PINCTRL_PIN(28, "XLSIOS_VSYNC6"),
	PINCTRL_PIN(29, "XLSIOS_VSYNC7"),
	PINCTRL_PIN(30, "XLSIOS_PWM_2"),
	PINCTRL_PIN(31, "XLSIOS_GPIO0"),
	PINCTRL_PIN(32, "XLSIOS_GPIO1"),
	PINCTRL_PIN(33, "XLSIOS_GPIO2"),
	PINCTRL_PIN(34, "XLSIOS_CLI5_PIN0"),
	PINCTRL_PIN(35, "XLSIOS_CLI5_PIN1"),
	PINCTRL_PIN(36, "XLSIOS_CLI5_PIN2"),
	PINCTRL_PIN(37, "XLSIOS_CLI5_PIN3"),
	PINCTRL_PIN(38, "XLSIOS_GPIO3"),
	PINCTRL_PIN(39, "XLSIOS_MCLK0"),
	PINCTRL_PIN(40, "XLSIOS_CLI4_PIN0"),
	PINCTRL_PIN(41, "XLSIOS_CLI4_PIN1"),
	PINCTRL_PIN(42, "XLSIOS_CLI4_PIN2"),
	PINCTRL_PIN(43, "XLSIOS_CLI4_PIN3"),
	PINCTRL_PIN(44, "XLSIOS_VSYNC0"),
	PINCTRL_PIN(45, "XLSIOS_GPIO4"),
	PINCTRL_PIN(46, "XLSIOS_GPIO5"),
	PINCTRL_PIN(47, "XLSIOS_PRE_OCP_GPU"),
	PINCTRL_PIN(48, "XLSIOS_SOFT_PRE_OCP_GPU"),
	PINCTRL_PIN(49, "XLSIOS_VSYNC5"),
	PINCTRL_PIN(50, "XLSIOS_CLI6_PIN0"),
	PINCTRL_PIN(51, "XLSIOS_CLI6_PIN1"),
	PINCTRL_PIN(52, "XLSIOS_CLI6_PIN2"),
	PINCTRL_PIN(53, "XLSIOS_CLI6_PIN3"),
	PINCTRL_PIN(54, "XLSIOS_CAMERA_MUTE"),
};

enum google_lga_lsios_funcs {
	google_pinmux_lsios_gpio,
	google_pinmux_lsios_mclk,
	google_pinmux_lsios_debug_mux,
	google_pinmux_lsios_cli0,
	google_pinmux_lsios_qspi0,
	google_pinmux_lsios_cli1,
	google_pinmux_lsios_pwm,
	google_pinmux_lsios_cli2,
	google_pinmux_lsios_cli3,
	google_pinmux_lsios_vsync,
	google_pinmux_lsios_cli5,
	google_pinmux_lsios_cli4,
	google_pinmux_lsios_pre_ocp_gpu,
	google_pinmux_lsios_soft_pre_ocp_gpu,
	google_pinmux_lsios_cli6,
	google_pinmux_lsios_camera_mute,
};

GOOGLE_FUNCTION_GROUPS(lsios_gpio, "XLSIOS_MCLK1", "XLSIOS_MCLK2",
		       "XLSIOS_MCLK3", "XLSIOS_MCLK4", "XLSIOS_MCLK5",
		       "XLSIOS_MCLK6", "XLSIOS_MCLK7", "XLSIOS_CLI0_PIN0",
		       "XLSIOS_CLI0_PIN1", "XLSIOS_CLI0_PIN2",
		       "XLSIOS_CLI0_PIN3", "XLSIOS_CLI1_PIN0",
		       "XLSIOS_CLI1_PIN1", "XLSIOS_CLI1_PIN2",
		       "XLSIOS_CLI1_PIN3", "XLSIOS_CLI2_PIN0",
		       "XLSIOS_CLI2_PIN1", "XLSIOS_CLI2_PIN2",
		       "XLSIOS_CLI2_PIN3", "XLSIOS_CLI3_PIN0",
		       "XLSIOS_CLI3_PIN1", "XLSIOS_CLI3_PIN2",
		       "XLSIOS_CLI3_PIN3", "XLSIOS_PWM_0", "XLSIOS_PWM_1",
		       "XLSIOS_VSYNC1", "XLSIOS_VSYNC2", "XLSIOS_VSYNC3",
		       "XLSIOS_VSYNC6", "XLSIOS_VSYNC7", "XLSIOS_PWM_2",
		       "XLSIOS_GPIO0", "XLSIOS_GPIO1", "XLSIOS_GPIO2",
		       "XLSIOS_CLI5_PIN0", "XLSIOS_CLI5_PIN1",
		       "XLSIOS_CLI5_PIN2", "XLSIOS_CLI5_PIN3", "XLSIOS_GPIO3",
		       "XLSIOS_MCLK0", "XLSIOS_CLI4_PIN0", "XLSIOS_CLI4_PIN1",
		       "XLSIOS_CLI4_PIN2", "XLSIOS_CLI4_PIN3", "XLSIOS_VSYNC0",
		       "XLSIOS_GPIO4", "XLSIOS_GPIO5", "XLSIOS_PRE_OCP_GPU",
		       "XLSIOS_SOFT_PRE_OCP_GPU", "XLSIOS_VSYNC5",
		       "XLSIOS_CLI6_PIN0", "XLSIOS_CLI6_PIN1",
		       "XLSIOS_CLI6_PIN2", "XLSIOS_CLI6_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_mclk, "XLSIOS_MCLK1", "XLSIOS_MCLK2",
		       "XLSIOS_MCLK3", "XLSIOS_MCLK4", "XLSIOS_MCLK5",
		       "XLSIOS_MCLK6", "XLSIOS_MCLK7", "XLSIOS_MCLK0");
GOOGLE_FUNCTION_GROUPS(lsios_debug_mux, "XLSIOS_MCLK2", "XLSIOS_MCLK3",
		       "XLSIOS_MCLK4", "XLSIOS_MCLK5", "XLSIOS_MCLK6",
		       "XLSIOS_MCLK7", "XLSIOS_CLI1_PIN2", "XLSIOS_CLI1_PIN3",
		       "XLSIOS_CLI2_PIN0", "XLSIOS_CLI2_PIN1",
		       "XLSIOS_CLI2_PIN2", "XLSIOS_CLI2_PIN3",
		       "XLSIOS_CLI3_PIN0", "XLSIOS_CLI3_PIN1",
		       "XLSIOS_CLI3_PIN2", "XLSIOS_CLI3_PIN3", "XLSIOS_PWM_1",
		       "XLSIOS_VSYNC1", "XLSIOS_VSYNC2", "XLSIOS_VSYNC3",
		       "XLSIOS_VSYNC6", "XLSIOS_VSYNC7", "XLSIOS_PWM_2",
		       "XLSIOS_GPIO0", "XLSIOS_GPIO1", "XLSIOS_GPIO2",
		       "XLSIOS_CLI4_PIN0", "XLSIOS_CLI4_PIN1",
		       "XLSIOS_CLI4_PIN2", "XLSIOS_CLI4_PIN3", "XLSIOS_GPIO4",
		       "XLSIOS_GPIO5");
GOOGLE_FUNCTION_GROUPS(lsios_cli0, "XLSIOS_CLI0_PIN0", "XLSIOS_CLI0_PIN1",
		       "XLSIOS_CLI0_PIN2", "XLSIOS_CLI0_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_qspi0, "XLSIOS_CLI0_PIN0", "XLSIOS_CLI0_PIN1",
		       "XLSIOS_CLI0_PIN2", "XLSIOS_CLI0_PIN3",
		       "XLSIOS_CLI1_PIN0", "XLSIOS_CLI1_PIN1");
GOOGLE_FUNCTION_GROUPS(lsios_cli1, "XLSIOS_CLI1_PIN0", "XLSIOS_CLI1_PIN1",
		       "XLSIOS_CLI1_PIN2", "XLSIOS_CLI1_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_pwm, "XLSIOS_CLI1_PIN2", "XLSIOS_PWM_0",
		       "XLSIOS_PWM_1", "XLSIOS_PWM_2");
GOOGLE_FUNCTION_GROUPS(lsios_cli2, "XLSIOS_CLI2_PIN0", "XLSIOS_CLI2_PIN1",
		       "XLSIOS_CLI2_PIN2", "XLSIOS_CLI2_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_cli3, "XLSIOS_CLI3_PIN0", "XLSIOS_CLI3_PIN1",
		       "XLSIOS_CLI3_PIN2", "XLSIOS_CLI3_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_vsync, "XLSIOS_VSYNC1", "XLSIOS_VSYNC2",
		       "XLSIOS_VSYNC3", "XLSIOS_VSYNC6", "XLSIOS_VSYNC7",
		       "XLSIOS_VSYNC0", "XLSIOS_VSYNC5");
GOOGLE_FUNCTION_GROUPS(lsios_cli5, "XLSIOS_CLI5_PIN0", "XLSIOS_CLI5_PIN1",
		       "XLSIOS_CLI5_PIN2", "XLSIOS_CLI5_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_cli4, "XLSIOS_CLI4_PIN0", "XLSIOS_CLI4_PIN1",
		       "XLSIOS_CLI4_PIN2", "XLSIOS_CLI4_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_pre_ocp_gpu, "XLSIOS_PRE_OCP_GPU");
GOOGLE_FUNCTION_GROUPS(lsios_soft_pre_ocp_gpu, "XLSIOS_SOFT_PRE_OCP_GPU");
GOOGLE_FUNCTION_GROUPS(lsios_cli6, "XLSIOS_CLI6_PIN0", "XLSIOS_CLI6_PIN1",
		       "XLSIOS_CLI6_PIN2", "XLSIOS_CLI6_PIN3");
GOOGLE_FUNCTION_GROUPS(lsios_camera_mute, "XLSIOS_CAMERA_MUTE");

static const struct google_pingroup google_lga_lsios_groups[] = {
	GOOGLE_PIN_GROUP(0, "XLSIOS_MCLK1", lsios_gpio, lsios_mclk, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XLSIOS_MCLK2", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XLSIOS_MCLK3", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XLSIOS_MCLK4", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XLSIOS_MCLK5", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XLSIOS_MCLK6", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XLSIOS_MCLK7", lsios_gpio, lsios_mclk, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XLSIOS_CLI0_PIN0", lsios_gpio, lsios_cli0, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XLSIOS_CLI0_PIN1", lsios_gpio, lsios_cli0, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XLSIOS_CLI0_PIN2", lsios_gpio, lsios_cli0, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XLSIOS_CLI0_PIN3", lsios_gpio, lsios_cli0, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XLSIOS_CLI1_PIN0", lsios_gpio, lsios_cli1, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XLSIOS_CLI1_PIN1", lsios_gpio, lsios_cli1, lsios_qspi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(13, "XLSIOS_CLI1_PIN2", lsios_gpio, lsios_cli1, lsios_pwm, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(14, "XLSIOS_CLI1_PIN3", lsios_gpio, lsios_cli1, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(15, "XLSIOS_CLI2_PIN0", lsios_gpio, lsios_cli2, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(16, "XLSIOS_CLI2_PIN1", lsios_gpio, lsios_cli2, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(17, "XLSIOS_CLI2_PIN2", lsios_gpio, lsios_cli2, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(18, "XLSIOS_CLI2_PIN3", lsios_gpio, lsios_cli2, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(19, "XLSIOS_CLI3_PIN0", lsios_gpio, lsios_cli3, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(20, "XLSIOS_CLI3_PIN1", lsios_gpio, lsios_cli3, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(21, "XLSIOS_CLI3_PIN2", lsios_gpio, lsios_cli3, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(22, "XLSIOS_CLI3_PIN3", lsios_gpio, lsios_cli3, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(23, "XLSIOS_PWM_0", lsios_gpio, lsios_pwm, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(24, "XLSIOS_PWM_1", lsios_gpio, lsios_pwm, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(25, "XLSIOS_VSYNC1", lsios_gpio, lsios_vsync, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(26, "XLSIOS_VSYNC2", lsios_gpio, lsios_vsync, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(27, "XLSIOS_VSYNC3", lsios_gpio, lsios_vsync, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(28, "XLSIOS_VSYNC6", lsios_gpio, lsios_vsync, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(29, "XLSIOS_VSYNC7", lsios_gpio, lsios_vsync, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(30, "XLSIOS_PWM_2", lsios_gpio, lsios_pwm, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(31, "XLSIOS_GPIO0", lsios_gpio, none, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(32, "XLSIOS_GPIO1", lsios_gpio, none, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(33, "XLSIOS_GPIO2", lsios_gpio, none, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(34, "XLSIOS_CLI5_PIN0", lsios_gpio, lsios_cli5, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(35, "XLSIOS_CLI5_PIN1", lsios_gpio, lsios_cli5, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(36, "XLSIOS_CLI5_PIN2", lsios_gpio, lsios_cli5, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(37, "XLSIOS_CLI5_PIN3", lsios_gpio, lsios_cli5, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(38, "XLSIOS_GPIO3", lsios_gpio, none, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(39, "XLSIOS_MCLK0", lsios_gpio, lsios_mclk, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(40, "XLSIOS_CLI4_PIN0", lsios_gpio, lsios_cli4, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(41, "XLSIOS_CLI4_PIN1", lsios_gpio, lsios_cli4, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(42, "XLSIOS_CLI4_PIN2", lsios_gpio, lsios_cli4, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(43, "XLSIOS_CLI4_PIN3", lsios_gpio, lsios_cli4, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(44, "XLSIOS_VSYNC0", lsios_gpio, lsios_vsync, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(45, "XLSIOS_GPIO4", lsios_gpio, none, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(46, "XLSIOS_GPIO5", lsios_gpio, none, none, lsios_debug_mux, none, none, none, none, none),
	GOOGLE_PIN_GROUP(47, "XLSIOS_PRE_OCP_GPU", lsios_gpio, lsios_pre_ocp_gpu, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(48, "XLSIOS_SOFT_PRE_OCP_GPU", lsios_gpio, lsios_soft_pre_ocp_gpu, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(49, "XLSIOS_VSYNC5", lsios_gpio, lsios_vsync, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(50, "XLSIOS_CLI6_PIN0", lsios_gpio, lsios_cli6, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(51, "XLSIOS_CLI6_PIN1", lsios_gpio, lsios_cli6, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(52, "XLSIOS_CLI6_PIN2", lsios_gpio, lsios_cli6, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(53, "XLSIOS_CLI6_PIN3", lsios_gpio, lsios_cli6, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(54, "XLSIOS_CAMERA_MUTE", none, lsios_camera_mute, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_lsios_functions[] = {
	GOOGLE_FUNCTION(lsios_gpio),
	GOOGLE_FUNCTION(lsios_mclk),
	GOOGLE_FUNCTION(lsios_debug_mux),
	GOOGLE_FUNCTION(lsios_cli0),
	GOOGLE_FUNCTION(lsios_qspi0),
	GOOGLE_FUNCTION(lsios_cli1),
	GOOGLE_FUNCTION(lsios_pwm),
	GOOGLE_FUNCTION(lsios_cli2),
	GOOGLE_FUNCTION(lsios_cli3),
	GOOGLE_FUNCTION(lsios_vsync),
	GOOGLE_FUNCTION(lsios_cli5),
	GOOGLE_FUNCTION(lsios_cli4),
	GOOGLE_FUNCTION(lsios_pre_ocp_gpu),
	GOOGLE_FUNCTION(lsios_soft_pre_ocp_gpu),
	GOOGLE_FUNCTION(lsios_cli6),
	GOOGLE_FUNCTION(lsios_camera_mute),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_lsios = {
	.pins = google_lga_lsios,
	.num_pins = ARRAY_SIZE(google_lga_lsios),
	.groups = google_lga_lsios_groups,
	.num_groups = ARRAY_SIZE(google_lga_lsios_groups),
	.funcs = google_lga_lsios_functions,
	.num_funcs = ARRAY_SIZE(google_lga_lsios_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "lsios",
};

/* LSIOE */
static const struct pinctrl_pin_desc google_lga_lsioe[] = {
	PINCTRL_PIN(0, "XLSIOE_CLI7_PIN0"),
	PINCTRL_PIN(1, "XLSIOE_CLI7_PIN1"),
	PINCTRL_PIN(2, "XLSIOE_CLI7_PIN2"),
	PINCTRL_PIN(3, "XLSIOE_CLI7_PIN3"),
	PINCTRL_PIN(4, "XLSIOE_CLI8_PIN0"),
	PINCTRL_PIN(5, "XLSIOE_CLI8_PIN1"),
	PINCTRL_PIN(6, "XLSIOE_CLI8_PIN2"),
	PINCTRL_PIN(7, "XLSIOE_CLI8_PIN3"),
	PINCTRL_PIN(8, "XLSIOE_CLI13_PIN0"),
	PINCTRL_PIN(9, "XLSIOE_CLI13_PIN1"),
	PINCTRL_PIN(10, "XLSIOE_CLI13_PIN2"),
	PINCTRL_PIN(11, "XLSIOE_CLI13_PIN3"),
	PINCTRL_PIN(12, "XLSIOE_CLI9_PIN0"),
	PINCTRL_PIN(13, "XLSIOE_CLI9_PIN1"),
	PINCTRL_PIN(14, "XLSIOE_CLI9_PIN2"),
	PINCTRL_PIN(15, "XLSIOE_CLI9_PIN3"),
	PINCTRL_PIN(16, "XLSIOE_CLI10_PIN0"),
	PINCTRL_PIN(17, "XLSIOE_CLI10_PIN1"),
	PINCTRL_PIN(18, "XLSIOE_CLI10_PIN2"),
	PINCTRL_PIN(19, "XLSIOE_CLI10_PIN3"),
	PINCTRL_PIN(20, "XLSIOE_CLI11_PIN0"),
	PINCTRL_PIN(21, "XLSIOE_CLI11_PIN1"),
	PINCTRL_PIN(22, "XLSIOE_CLI11_PIN2"),
	PINCTRL_PIN(23, "XLSIOE_CLI11_PIN3"),
	PINCTRL_PIN(24, "XLSIOE_CLI12_PIN0"),
	PINCTRL_PIN(25, "XLSIOE_CLI12_PIN1"),
	PINCTRL_PIN(26, "XLSIOE_CLI12_PIN2"),
	PINCTRL_PIN(27, "XLSIOE_CLI12_PIN3"),
	PINCTRL_PIN(28, "XLSIOE_CLI14_PIN0"),
	PINCTRL_PIN(29, "XLSIOE_CLI14_PIN1"),
	PINCTRL_PIN(30, "XLSIOE_CLI14_PIN2"),
	PINCTRL_PIN(31, "XLSIOE_CLI14_PIN3"),
};

enum google_lga_lsioe_funcs {
	google_pinmux_lsioe_gpio,
	google_pinmux_lsioe_cli7,
	google_pinmux_lsioe_cli8,
	google_pinmux_lsioe_cli13,
	google_pinmux_lsioe_cli9,
	google_pinmux_lsioe_cli10,
	google_pinmux_lsioe_cli11,
	google_pinmux_lsioe_spmi0,
	google_pinmux_lsioe_cli12,
	google_pinmux_lsioe_cli14,
};

GOOGLE_FUNCTION_GROUPS(lsioe_gpio, "XLSIOE_CLI7_PIN0", "XLSIOE_CLI7_PIN1",
		       "XLSIOE_CLI7_PIN2", "XLSIOE_CLI7_PIN3",
		       "XLSIOE_CLI8_PIN0", "XLSIOE_CLI8_PIN1",
		       "XLSIOE_CLI8_PIN2", "XLSIOE_CLI8_PIN3",
		       "XLSIOE_CLI13_PIN0", "XLSIOE_CLI13_PIN1",
		       "XLSIOE_CLI13_PIN2", "XLSIOE_CLI13_PIN3",
		       "XLSIOE_CLI9_PIN0", "XLSIOE_CLI9_PIN1",
		       "XLSIOE_CLI9_PIN2", "XLSIOE_CLI9_PIN3",
		       "XLSIOE_CLI10_PIN0", "XLSIOE_CLI10_PIN1",
		       "XLSIOE_CLI10_PIN2", "XLSIOE_CLI10_PIN3",
		       "XLSIOE_CLI11_PIN0", "XLSIOE_CLI11_PIN1",
		       "XLSIOE_CLI11_PIN2", "XLSIOE_CLI11_PIN3",
		       "XLSIOE_CLI12_PIN0", "XLSIOE_CLI12_PIN1",
		       "XLSIOE_CLI12_PIN2", "XLSIOE_CLI12_PIN3",
		       "XLSIOE_CLI14_PIN0", "XLSIOE_CLI14_PIN1",
		       "XLSIOE_CLI14_PIN2", "XLSIOE_CLI14_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli7, "XLSIOE_CLI7_PIN0", "XLSIOE_CLI7_PIN1",
		       "XLSIOE_CLI7_PIN2", "XLSIOE_CLI7_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli8, "XLSIOE_CLI8_PIN0", "XLSIOE_CLI8_PIN1",
		       "XLSIOE_CLI8_PIN2", "XLSIOE_CLI8_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli13, "XLSIOE_CLI13_PIN0", "XLSIOE_CLI13_PIN1",
		       "XLSIOE_CLI13_PIN2", "XLSIOE_CLI13_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli9, "XLSIOE_CLI9_PIN0", "XLSIOE_CLI9_PIN1",
		       "XLSIOE_CLI9_PIN2", "XLSIOE_CLI9_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli10, "XLSIOE_CLI10_PIN0", "XLSIOE_CLI10_PIN1",
		       "XLSIOE_CLI10_PIN2", "XLSIOE_CLI10_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli11, "XLSIOE_CLI11_PIN0", "XLSIOE_CLI11_PIN1",
		       "XLSIOE_CLI11_PIN2", "XLSIOE_CLI11_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_spmi0, "XLSIOE_CLI11_PIN0", "XLSIOE_CLI11_PIN1");
GOOGLE_FUNCTION_GROUPS(lsioe_cli12, "XLSIOE_CLI12_PIN0", "XLSIOE_CLI12_PIN1",
		       "XLSIOE_CLI12_PIN2", "XLSIOE_CLI12_PIN3");
GOOGLE_FUNCTION_GROUPS(lsioe_cli14, "XLSIOE_CLI14_PIN0", "XLSIOE_CLI14_PIN1",
		       "XLSIOE_CLI14_PIN2", "XLSIOE_CLI14_PIN3");

static const struct google_pingroup google_lga_lsioe_groups[] = {
	GOOGLE_PIN_GROUP(0, "XLSIOE_CLI7_PIN0", lsioe_gpio, lsioe_cli7, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XLSIOE_CLI7_PIN1", lsioe_gpio, lsioe_cli7, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XLSIOE_CLI7_PIN2", lsioe_gpio, lsioe_cli7, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XLSIOE_CLI7_PIN3", lsioe_gpio, lsioe_cli7, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XLSIOE_CLI8_PIN0", lsioe_gpio, lsioe_cli8, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XLSIOE_CLI8_PIN1", lsioe_gpio, lsioe_cli8, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XLSIOE_CLI8_PIN2", lsioe_gpio, lsioe_cli8, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XLSIOE_CLI8_PIN3", lsioe_gpio, lsioe_cli8, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(8, "XLSIOE_CLI13_PIN0", lsioe_gpio, lsioe_cli13, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(9, "XLSIOE_CLI13_PIN1", lsioe_gpio, lsioe_cli13, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(10, "XLSIOE_CLI13_PIN2", lsioe_gpio, lsioe_cli13, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(11, "XLSIOE_CLI13_PIN3", lsioe_gpio, lsioe_cli13, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(12, "XLSIOE_CLI9_PIN0", lsioe_gpio, lsioe_cli9, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(13, "XLSIOE_CLI9_PIN1", lsioe_gpio, lsioe_cli9, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(14, "XLSIOE_CLI9_PIN2", lsioe_gpio, lsioe_cli9, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(15, "XLSIOE_CLI9_PIN3", lsioe_gpio, lsioe_cli9, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(16, "XLSIOE_CLI10_PIN0", lsioe_gpio, lsioe_cli10, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(17, "XLSIOE_CLI10_PIN1", lsioe_gpio, lsioe_cli10, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(18, "XLSIOE_CLI10_PIN2", lsioe_gpio, lsioe_cli10, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(19, "XLSIOE_CLI10_PIN3", lsioe_gpio, lsioe_cli10, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(20, "XLSIOE_CLI11_PIN0", lsioe_gpio, lsioe_cli11, lsioe_spmi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(21, "XLSIOE_CLI11_PIN1", lsioe_gpio, lsioe_cli11, lsioe_spmi0, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(22, "XLSIOE_CLI11_PIN2", lsioe_gpio, lsioe_cli11, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(23, "XLSIOE_CLI11_PIN3", lsioe_gpio, lsioe_cli11, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(24, "XLSIOE_CLI12_PIN0", lsioe_gpio, lsioe_cli12, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(25, "XLSIOE_CLI12_PIN1", lsioe_gpio, lsioe_cli12, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(26, "XLSIOE_CLI12_PIN2", lsioe_gpio, lsioe_cli12, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(27, "XLSIOE_CLI12_PIN3", lsioe_gpio, lsioe_cli12, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(28, "XLSIOE_CLI14_PIN0", lsioe_gpio, lsioe_cli14, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(29, "XLSIOE_CLI14_PIN1", lsioe_gpio, lsioe_cli14, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(30, "XLSIOE_CLI14_PIN2", lsioe_gpio, lsioe_cli14, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(31, "XLSIOE_CLI14_PIN3", lsioe_gpio, lsioe_cli14, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_lsioe_functions[] = {
	GOOGLE_FUNCTION(lsioe_gpio),
	GOOGLE_FUNCTION(lsioe_cli7),
	GOOGLE_FUNCTION(lsioe_cli8),
	GOOGLE_FUNCTION(lsioe_cli13),
	GOOGLE_FUNCTION(lsioe_cli9),
	GOOGLE_FUNCTION(lsioe_cli10),
	GOOGLE_FUNCTION(lsioe_cli11),
	GOOGLE_FUNCTION(lsioe_spmi0),
	GOOGLE_FUNCTION(lsioe_cli12),
	GOOGLE_FUNCTION(lsioe_cli14),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_lsioe = {
	.pins = google_lga_lsioe,
	.num_pins = ARRAY_SIZE(google_lga_lsioe),
	.groups = google_lga_lsioe_groups,
	.num_groups = ARRAY_SIZE(google_lga_lsioe_groups),
	.funcs = google_lga_lsioe_functions,
	.num_funcs = ARRAY_SIZE(google_lga_lsioe_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "lsioe",
};

/* LSION */
static const struct pinctrl_pin_desc google_lga_lsion[] = {
	PINCTRL_PIN(0, "XLSION_CLI15_PIN0"),
	PINCTRL_PIN(1, "XLSION_CLI15_PIN1"),
	PINCTRL_PIN(2, "XLSION_CLI15_PIN2"),
	PINCTRL_PIN(3, "XLSION_CLI15_PIN3"),
	PINCTRL_PIN(4, "XLSION_CLI16_PIN0"),
	PINCTRL_PIN(5, "XLSION_CLI16_PIN1"),
	PINCTRL_PIN(6, "XLSION_CLI16_PIN2"),
	PINCTRL_PIN(7, "XLSION_CLI16_PIN3"),
};

enum google_lga_lsion_funcs {
	google_pinmux_lsion_gpio,
	google_pinmux_lsion_cli15,
	google_pinmux_lsion_cli16,
};

GOOGLE_FUNCTION_GROUPS(lsion_gpio, "XLSION_CLI15_PIN0", "XLSION_CLI15_PIN1",
		       "XLSION_CLI15_PIN2", "XLSION_CLI15_PIN3",
		       "XLSION_CLI16_PIN0", "XLSION_CLI16_PIN1",
		       "XLSION_CLI16_PIN2", "XLSION_CLI16_PIN3");
GOOGLE_FUNCTION_GROUPS(lsion_cli15, "XLSION_CLI15_PIN0", "XLSION_CLI15_PIN1",
		       "XLSION_CLI15_PIN2", "XLSION_CLI15_PIN3");
GOOGLE_FUNCTION_GROUPS(lsion_cli16, "XLSION_CLI16_PIN0", "XLSION_CLI16_PIN1",
		       "XLSION_CLI16_PIN2", "XLSION_CLI16_PIN3");

static const struct google_pingroup google_lga_lsion_groups[] = {
	GOOGLE_PIN_GROUP(0, "XLSION_CLI15_PIN0", lsion_gpio, lsion_cli15, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(1, "XLSION_CLI15_PIN1", lsion_gpio, lsion_cli15, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(2, "XLSION_CLI15_PIN2", lsion_gpio, lsion_cli15, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(3, "XLSION_CLI15_PIN3", lsion_gpio, lsion_cli15, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(4, "XLSION_CLI16_PIN0", lsion_gpio, lsion_cli16, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(5, "XLSION_CLI16_PIN1", lsion_gpio, lsion_cli16, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(6, "XLSION_CLI16_PIN2", lsion_gpio, lsion_cli16, none, none, none, none, none, none, none),
	GOOGLE_PIN_GROUP(7, "XLSION_CLI16_PIN3", lsion_gpio, lsion_cli16, none, none, none, none, none, none, none),
};

static const struct google_pin_function google_lga_lsion_functions[] = {
	GOOGLE_FUNCTION(lsion_gpio),
	GOOGLE_FUNCTION(lsion_cli15),
	GOOGLE_FUNCTION(lsion_cli16),
};

static const struct google_pinctrl_soc_info google_lga_pinctrl_lsion = {
	.pins = google_lga_lsion,
	.num_pins = ARRAY_SIZE(google_lga_lsion),
	.groups = google_lga_lsion_groups,
	.num_groups = ARRAY_SIZE(google_lga_lsion_groups),
	.funcs = google_lga_lsion_functions,
	.num_funcs = ARRAY_SIZE(google_lga_lsion_functions),
	.gpio_func = GOOGLE_GPIO_FUNC_DEFAULT,
	.pin_stride = LGA_PIN_STRIDE,
	.label = "lsion",
};

static const struct of_device_id google_lga_of_match[] = {
	{ .compatible = "google,lga-hsion-pinctrl", .data = &google_lga_pinctrl_hsion },
	{ .compatible = "google,lga-hsios-stby-pinctrl", .data = &google_lga_pinctrl_hsios_stby },
	{ .compatible = "google,lga-hsios-pinctrl", .data = &google_lga_pinctrl_hsios },
	{ .compatible = "google,lga-dpu-pinctrl", .data = &google_lga_pinctrl_dpu },
	{ .compatible = "google,lga-aoc-pinctrl", .data = &google_lga_pinctrl_aoc },
	{ .compatible = "google,lga-gdmc-pinctrl", .data = &google_lga_pinctrl_gdmc },
	{ .compatible = "google,lga-cpm-pinctrl", .data = &google_lga_pinctrl_cpm },
	{ .compatible = "google,lga-lsios-pinctrl", .data = &google_lga_pinctrl_lsios },
	{ .compatible = "google,lga-lsioe-pinctrl", .data = &google_lga_pinctrl_lsioe },
	{ .compatible = "google,lga-lsion-pinctrl", .data = &google_lga_pinctrl_lsion },
	{}
};
MODULE_DEVICE_TABLE(of, google_lga_of_match);

static int google_lga_pinctrl_probe(struct platform_device *pdev)
{
	return google_pinctrl_probe(pdev, of_device_get_match_data(&pdev->dev));
}

static struct platform_driver google_lga_pinctrl_driver = {
	.driver = {
		.name = "google_lga_pinctrl",
		.of_match_table = google_lga_of_match,
		.pm = pm_ptr(&google_pinctrl_pm_ops),
	},
	.probe = google_lga_pinctrl_probe,
};

static int __init google_lga_pinctrl_init(void)
{
	return platform_driver_register(&google_lga_pinctrl_driver);
}
arch_initcall(google_lga_pinctrl_init);

static void __exit google_lga_pinctrl_exit(void)
{
	platform_driver_unregister(&google_lga_pinctrl_driver);
}
module_exit(google_lga_pinctrl_exit);

MODULE_DESCRIPTION("Google Tensor G5 (LGA) pin controller driver");
MODULE_LICENSE("GPL");
