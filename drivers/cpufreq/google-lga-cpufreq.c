// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Laguna (Tensor G5) CPM CPUFreq and MMIO fast-switch driver.
 *
 * Discovers per-cluster OPP tables (frequency, voltage, pf_state level, and
 * fuse-binned availability) at probe time from the Central Power Manager (CPM)
 * power-block service over goog_mba_cpm_iface, and performs frequency changes
 * by writing the hardware pf_state index to the cluster MMIO CSR block.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <soc/google/goog_cpm_service_ids.h>
#include <soc/google/goog_mba_cpm_iface.h>

#define LGA_MAX_DOMAINS			8
#define LGA_MAX_CPUS			16
#define LGA_CPM_MAX_DOMAINS		48
#define LGA_CPM_MAX_OPPS		32
#define LGA_CPM_MBOX_TIMEOUT_MS		3000
#define LGA_CPUFREQ_CSR_STRIDE		8
#define LGA_CPUFREQ_TRANSITION_LAT_NS	500000

#define LGA_PWRBLK_CMD_GET_NUM_DOMAINS	0x0003
#define LGA_PWRBLK_CMD_GET_DOM_INFO	0x0103
#define LGA_PWRBLK_CMD_GET_OPPS		0x0203

#define LGA_PWRBLK_CMD_MASK		GENMASK(15, 0)
#define LGA_PWRBLK_DOM_MASK		GENMASK(23, 16)
#define LGA_PWRBLK_PAIR_MASK		GENMASK(31, 24)
#define LGA_PWRBLK_COUNT_MASK		GENMASK(7, 0)

#define LGA_OPP_FREQ_MASK		GENMASK(13, 0)
#define LGA_OPP_ENABLE_BIT		BIT(14)
#define LGA_OPP_VOLT_MASK		GENMASK(29, 16)
#define LGA_OPP_VALID_BIT		BIT(30)
#define LGA_OPP_FLAGS_MASK		(LGA_OPP_ENABLE_BIT | LGA_OPP_VALID_BIT)

struct lga_cpu_opp {
	u32 freq_khz;
	u32 u_volt;
	u8 level;
	bool disabled;
};

struct lga_cpufreq;

struct lga_cpu_domain {
	struct lga_cpufreq *drv;
	const char *name;
	u32 csr_offset;
	unsigned int nr_opps;
	unsigned int nr_enabled;
	struct lga_cpu_opp opps[LGA_CPM_MAX_OPPS];
};

struct lga_cpufreq {
	struct regmap *regmap;
	unsigned int nr_domains;
	unsigned int nr_cpus;
	u32 cpu_domain_map[LGA_MAX_CPUS];
	struct lga_cpu_domain domains[LGA_MAX_DOMAINS];
};

static struct cpufreq_driver lga_cpufreq_driver;

static int lga_cpm_send_pwrblk(struct cpm_iface_client *client, u32 cmd,
			       struct cpm_iface_payload *resp)
{
	struct cpm_iface_payload req_msg = {
		.payload = { cmd, 0, 0 },
	};
	struct cpm_iface_req req = {
		.msg_type = REQUEST_MSG,
		.req_msg = &req_msg,
		.resp_msg = resp,
		.dst_id = CPM_COMMON_PWRBLK_SERVICE,
		.tout_ms = LGA_CPM_MBOX_TIMEOUT_MS,
	};

	memset(resp, 0, sizeof(*resp));
	return cpm_send_message(client, &req);
}

static struct lga_cpu_domain *lga_cpufreq_match_domain(struct lga_cpufreq *drv,
						       const char *name)
{
	unsigned int i;

	for (i = 0; i < drv->nr_domains; i++) {
		if (!strcmp(drv->domains[i].name, name))
			return &drv->domains[i];
	}

	return NULL;
}

static int lga_cpufreq_query_domain_opps(struct device *dev,
					 struct cpm_iface_client *client,
					 unsigned int cpm_dom,
					 unsigned int num_opps,
					 struct lga_cpu_domain *dom)
{
	struct cpm_iface_payload resp;
	unsigned int pair, slot;
	int ret;

	for (pair = 0; pair < num_opps; pair += 2) {
		u32 cmd = FIELD_PREP(LGA_PWRBLK_CMD_MASK, LGA_PWRBLK_CMD_GET_OPPS) |
			  FIELD_PREP(LGA_PWRBLK_DOM_MASK, cpm_dom) |
			  FIELD_PREP(LGA_PWRBLK_PAIR_MASK, pair);

		ret = lga_cpm_send_pwrblk(client, cmd, &resp);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to query OPPs for %s\n",
					     dom->name);

		for (slot = 0; slot < 2 && pair + slot < num_opps; slot++) {
			u32 raw = resp.payload[1 + slot];
			u32 freq_mhz = FIELD_GET(LGA_OPP_FREQ_MASK, raw);
			u32 volt_mv = FIELD_GET(LGA_OPP_VOLT_MASK, raw);
			struct lga_cpu_opp *opp;

			if (!(raw & LGA_OPP_FLAGS_MASK) || !freq_mhz)
				continue;

			if (dom->nr_opps >= ARRAY_SIZE(dom->opps))
				break;

			opp = &dom->opps[dom->nr_opps++];
			opp->freq_khz = freq_mhz * 1000U;
			opp->u_volt = volt_mv * 1000U;
			opp->level = pair + slot;
			opp->disabled = (raw & LGA_OPP_FLAGS_MASK) == LGA_OPP_VALID_BIT;
			if (!opp->disabled)
				dom->nr_enabled++;
		}
	}

	return 0;
}

static int lga_cpufreq_query_opps(struct device *dev, struct lga_cpufreq *drv)
{
	struct cpm_iface_client *client;
	struct cpm_iface_payload resp;
	unsigned int cpm_dom, nr_cpm_domains, i;
	int ret;

	client = cpm_iface_request_client(dev, 0, NULL, NULL);
	if (IS_ERR(client))
		return dev_err_probe(dev, PTR_ERR(client),
				     "failed to request CPM client\n");

	ret = lga_cpm_send_pwrblk(client, LGA_PWRBLK_CMD_GET_NUM_DOMAINS, &resp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to query CPM power-block count\n");
		goto out_free_client;
	}

	nr_cpm_domains = min_t(unsigned int,
			       FIELD_GET(LGA_PWRBLK_COUNT_MASK, resp.payload[0]),
			       LGA_CPM_MAX_DOMAINS);
	if (!nr_cpm_domains) {
		ret = -ENODEV;
		goto out_free_client;
	}

	for (cpm_dom = 0; cpm_dom < nr_cpm_domains; cpm_dom++) {
		u32 cmd = FIELD_PREP(LGA_PWRBLK_CMD_MASK, LGA_PWRBLK_CMD_GET_DOM_INFO) |
			  FIELD_PREP(LGA_PWRBLK_DOM_MASK, cpm_dom);
		struct lga_cpu_domain *dom;
		unsigned int num_opps;
		char cpm_name[9];

		ret = lga_cpm_send_pwrblk(client, cmd, &resp);
		if (ret) {
			dev_err_probe(dev, ret,
				      "failed to query CPM domain %u info\n",
				      cpm_dom);
			goto out_free_client;
		}

		memcpy(cpm_name, &resp.payload[1], 8);
		cpm_name[8] = '\0';

		dom = lga_cpufreq_match_domain(drv, cpm_name);
		if (!dom)
			continue;

		num_opps = min_t(unsigned int,
				 FIELD_GET(LGA_PWRBLK_COUNT_MASK, resp.payload[0]),
				 LGA_CPM_MAX_OPPS);
		ret = lga_cpufreq_query_domain_opps(dev, client, cpm_dom,
						    num_opps, dom);
		if (ret)
			goto out_free_client;
	}

	for (i = 0; i < drv->nr_domains; i++) {
		if (!drv->domains[i].nr_enabled) {
			ret = dev_err_probe(dev, -ENODEV,
					    "CPM domain %s has no enabled OPPs\n",
					    drv->domains[i].name);
			goto out_free_client;
		}
	}

out_free_client:
	cpm_iface_free_client(client);
	return ret;
}

static unsigned int lga_cpufreq_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct cpufreq_frequency_table *pos;
	struct lga_cpu_domain *dom;
	unsigned int i;
	u32 pf_state;
	int ret;

	if (!policy)
		return 0;

	dom = policy->driver_data;
	ret = regmap_read(dom->drv->regmap, dom->csr_offset, &pf_state);
	if (ret)
		return 0;

	cpufreq_for_each_valid_entry(pos, policy->freq_table) {
		if (pos->driver_data == pf_state)
			return pos->frequency;
	}

	/*
	 * Fall back to the full cached CPM table (including binned OPPs) so
	 * policy bring-up transitions cleanly via CPUFREQ_NEED_INITIAL_FREQ_CHECK.
	 */
	for (i = 0; i < dom->nr_opps; i++) {
		if (dom->opps[i].level == pf_state)
			return dom->opps[i].freq_khz;
	}

	return policy->freq_table[0].frequency;
}

static int lga_cpufreq_target_index(struct cpufreq_policy *policy,
				    unsigned int index)
{
	struct lga_cpu_domain *dom = policy->driver_data;
	u32 pf_state = policy->freq_table[index].driver_data;

	return regmap_write(dom->drv->regmap, dom->csr_offset, pf_state);
}

static unsigned int lga_cpufreq_fast_switch(struct cpufreq_policy *policy,
					    unsigned int target_freq)
{
	unsigned int idx = policy->cached_resolved_idx;

	if (lga_cpufreq_target_index(policy, idx))
		return 0;

	return policy->freq_table[idx].frequency;
}

static int lga_cpufreq_init(struct cpufreq_policy *policy)
{
	struct lga_cpufreq *drv = cpufreq_get_driver_data();
	struct cpufreq_frequency_table *freq_table = NULL;
	struct device *cpu_dev;
	struct lga_cpu_domain *dom;
	unsigned int dom_idx, cpu, i;
	int ret;

	if (!drv || policy->cpu >= drv->nr_cpus)
		return -ENODEV;

	dom_idx = drv->cpu_domain_map[policy->cpu];
	dom = &drv->domains[dom_idx];

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev)
		return -ENODEV;

	cpumask_clear(policy->cpus);
	for_each_possible_cpu(cpu) {
		if (cpu < drv->nr_cpus && drv->cpu_domain_map[cpu] == dom_idx)
			cpumask_set_cpu(cpu, policy->cpus);
	}

	for (i = 0; i < dom->nr_opps; i++) {
		const struct lga_cpu_opp *entry = &dom->opps[i];
		struct dev_pm_opp_data opp_data = {
			.level = entry->level,
			.freq = (unsigned long)entry->freq_khz * 1000UL,
			.u_volt = entry->u_volt,
		};

		ret = dev_pm_opp_add_dynamic(cpu_dev, &opp_data);
		if (ret) {
			dev_err(cpu_dev, "failed to add OPP %u kHz (%d)\n",
				entry->freq_khz, ret);
			goto err_remove_opp;
		}

		if (entry->disabled) {
			ret = dev_pm_opp_disable(cpu_dev, opp_data.freq);
			if (ret)
				goto err_remove_opp;
		}
	}

	ret = dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);
	if (ret) {
		dev_err(cpu_dev, "failed to share OPPs (%d)\n", ret);
		goto err_remove_opp;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table (%d)\n", ret);
		goto err_remove_opp;
	}

	if (!freq_table || freq_table[0].frequency == CPUFREQ_TABLE_END) {
		ret = -ENODEV;
		goto err_free_table;
	}

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		unsigned long rate = freq_table[i].frequency * 1000UL + 999;
		struct dev_pm_opp *opp;

		opp = dev_pm_opp_find_freq_floor(cpu_dev, &rate);
		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			goto err_free_table;
		}

		freq_table[i].driver_data = dev_pm_opp_get_level(opp);
		dev_pm_opp_put(opp);
	}

	policy->freq_table = freq_table;
	policy->driver_data = dom;
	policy->cpuinfo.transition_latency = LGA_CPUFREQ_TRANSITION_LAT_NS;
	policy->fast_switch_possible = true;
	policy->dvfs_possible_from_any_cpu = true;

	return 0;

err_free_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
err_remove_opp:
	dev_pm_opp_remove_all_dynamic(cpu_dev);
	return ret;
}

static void lga_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct device *cpu_dev = get_cpu_device(policy->cpu);

	if (!cpu_dev)
		return;

	dev_pm_opp_free_cpufreq_table(cpu_dev, &policy->freq_table);
	dev_pm_opp_remove_all_dynamic(cpu_dev);
}

static struct cpufreq_driver lga_cpufreq_driver = {
	.name = "lga-cpufreq",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		 CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = lga_cpufreq_target_index,
	.fast_switch = lga_cpufreq_fast_switch,
	.get = lga_cpufreq_get,
	.init = lga_cpufreq_init,
	.exit = lga_cpufreq_exit,
	.register_em = cpufreq_register_em_with_opp,
	.suspend = cpufreq_generic_suspend,
};

static int lga_cpufreq_parse_dt(struct device *dev, struct lga_cpufreq *drv)
{
	struct device_node *np = dev->of_node;
	int count, map_count, i;

	count = of_property_count_strings(np, "google,domain-names");
	if (count <= 0 || count > LGA_MAX_DOMAINS)
		return dev_err_probe(dev, count < 0 ? count : -EINVAL,
				     "invalid google,domain-names count\n");

	drv->nr_domains = count;
	for (i = 0; i < count; i++) {
		struct lga_cpu_domain *dom = &drv->domains[i];
		int ret;

		ret = of_property_read_string_index(np, "google,domain-names",
						    i, &dom->name);
		if (ret)
			return ret;

		dom->drv = drv;
		dom->csr_offset = i * LGA_CPUFREQ_CSR_STRIDE;
	}

	map_count = of_property_count_u32_elems(np, "google,cpu-domain-map");
	if (map_count <= 0 || map_count > LGA_MAX_CPUS)
		return dev_err_probe(dev, map_count < 0 ? map_count : -EINVAL,
				     "invalid google,cpu-domain-map count\n");

	drv->nr_cpus = map_count;
	count = of_property_read_u32_array(np, "google,cpu-domain-map",
					   drv->cpu_domain_map, map_count);
	if (count)
		return count;

	for (i = 0; i < map_count; i++) {
		if (drv->cpu_domain_map[i] >= drv->nr_domains)
			return -EINVAL;
	}

	return 0;
}

static void lga_cpufreq_unregister(void *data)
{
	cpufreq_unregister_driver(&lga_cpufreq_driver);
	lga_cpufreq_driver.driver_data = NULL;
}

static const struct regmap_config lga_cpufreq_regmap_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = (LGA_MAX_DOMAINS - 1) * LGA_CPUFREQ_CSR_STRIDE,
	.fast_io = true,
	.use_raw_spinlock = true,
};

static int lga_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lga_cpufreq *drv;
	struct resource *res;
	void __iomem *base;
	int ret;

	if (lga_cpufreq_driver.driver_data)
		return -EBUSY;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	ret = lga_cpufreq_parse_dt(dev, drv);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (resource_size(res) < drv->nr_domains * LGA_CPUFREQ_CSR_STRIDE)
		return -EINVAL;

	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	drv->regmap = devm_regmap_init_mmio(dev, base, &lga_cpufreq_regmap_cfg);
	if (IS_ERR(drv->regmap))
		return PTR_ERR(drv->regmap);

	ret = lga_cpufreq_query_opps(dev, drv);
	if (ret)
		return ret;

	lga_cpufreq_driver.driver_data = drv;
	ret = cpufreq_register_driver(&lga_cpufreq_driver);
	if (ret) {
		lga_cpufreq_driver.driver_data = NULL;
		return ret;
	}

	return devm_add_action_or_reset(dev, lga_cpufreq_unregister, NULL);
}

static const struct of_device_id lga_cpufreq_of_match[] = {
	{ .compatible = "google,lga-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, lga_cpufreq_of_match);

static struct platform_driver lga_cpufreq_plat_driver = {
	.probe = lga_cpufreq_probe,
	.driver = {
		.name = "google-lga-cpufreq",
		.of_match_table = lga_cpufreq_of_match,
	},
};
module_platform_driver(lga_cpufreq_plat_driver);

MODULE_AUTHOR("Bobby Bones <saariuscrypto@gmail.com>");
MODULE_DESCRIPTION("Google Laguna (Tensor G5) CPM CPUFreq driver");
MODULE_LICENSE("GPL");
