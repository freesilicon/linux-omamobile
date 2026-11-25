/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Clean-room driver definitions for Google Tensor G5 (Laguna) EdgeTPU.
 *
 * Exposes /dev/edgetpu-soc (ABI 1.4, Litebuf VII over IKV) backed by
 * arm,smmu-v3 PASID contexts and GSA firmware authentication.
 */

#ifndef _EDGETPU_LAGUNA_H_
#define _EDGETPU_LAGUNA_H_

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/genalloc.h>
#include <linux/ioctl.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct device;
struct dma_buf;
struct dma_buf_attachment;
struct eventfd_ctx;
struct page;
struct sg_table;

/* Chip parameters for Laguna (buenos) */
#define LAGUNA_DRIVER_FW_IFACE_VERSION	1
#define LAGUNA_NUM_CORES		2
#define LAGUNA_NUM_PASIDS		16
#define LAGUNA_NUM_VCIDS		16
#define LAGUNA_NUM_VII_CREDITS		16

#define LAGUNA_IFACE_VER_MAJOR		1
#define LAGUNA_IFACE_VER_MINOR		4
#define LAGUNA_DEFAULT_FW_NAME		"google/edgetpu-buenos.fw"

#define LAGUNA_DMA_WIN_DEFAULT_START	0x84000000ULL
#define LAGUNA_DMA_WIN_DEFAULT_SIZE	0x0f7bf00000ULL

/* Offsets within "top" MMIO window (0x36500000) */
#define LAGUNA_REG_RESET_CONTROL	0x190018
#define LAGUNA_REG_AXIUSER_CORE0	0x190050
#define LAGUNA_REG_IREMAP_CONTROL	0x190070
#define LAGUNA_REG_IREMAP_LIMIT		0x190090
#define LAGUNA_REG_IREMAP_NEW_BASE	0x1900a0
#define LAGUNA_REG_IREMAP_AXIUSER_CORE0	0x1900b0

#define LAGUNA_REG_LPM_CONTROL		0x1e0028
#define LAGUNA_LPM_CTL_PWR_STATE	BIT(0)

/* CPM LPB status CSR for TPU SSWRP power check */
#define LAGUNA_LPB_TPU_DEFAULT_BASE	0x05330800ULL
#define LAGUNA_LPB_TPU_DEFAULT_SIZE	0x18
#define LAGUNA_LPB_TPU_INT_STATUS	0x04
#define LAGUNA_LPB_TPU_INT_MASK		GENMASK(1, 0)
#define LAGUNA_LPB_TPU_INT_OFF		0

/* Mailbox CSRs (index 0 = KCI @ 0xc000, index 1 = IKV @ 0xe000) */
#define LAGUNA_MBOX_BASE		0x0c000
#define LAGUNA_MBOX_STRIDE		0x2000
#define LAGUNA_KCI_MBOX_INDEX		0
#define LAGUNA_IKV_MBOX_INDEX		1

#define LAGUNA_MBOX_CTX_ENABLE		0x0000
#define LAGUNA_MBOX_CTX_PRIORITY	0x0004
#define LAGUNA_MBOX_CTX_CMD_DB_EN	0x0008
#define LAGUNA_MBOX_CTX_CMD_TAIL_DB_EN	0x000c
#define LAGUNA_MBOX_CTX_CMD_DB_CLR	0x0010
#define LAGUNA_MBOX_CTX_CMD_ADDR_LO	0x0014
#define LAGUNA_MBOX_CTX_CMD_ADDR_HI	0x0018
#define LAGUNA_MBOX_CTX_CMD_SIZE	0x001c
#define LAGUNA_MBOX_CTX_RSP_DB_EN	0x0020
#define LAGUNA_MBOX_CTX_RSP_TAIL_DB_EN	0x0024
#define LAGUNA_MBOX_CTX_RSP_ADDR_LO	0x0028
#define LAGUNA_MBOX_CTX_RSP_ADDR_HI	0x002c
#define LAGUNA_MBOX_CTX_RSP_SIZE	0x0030
#define LAGUNA_MBOX_CTX_SPARE_0		0x0034
#define LAGUNA_MBOX_CTX_SPARE_1		0x0038

#define LAGUNA_MBOX_CMDQ_DB_SET		0x1000
#define LAGUNA_MBOX_CMDQ_DB_STATUS	0x1004
#define LAGUNA_MBOX_CMDQ_HEAD		0x1008
#define LAGUNA_MBOX_CMDQ_TAIL		0x100c
#define LAGUNA_MBOX_CMDQ_CONFIG		0x1010
#define LAGUNA_MBOX_CMDQ_ERR_STATUS	0x1014

#define LAGUNA_MBOX_RSPQ_DB_SET		0x1800
#define LAGUNA_MBOX_RSPQ_DB_CLR		0x1804
#define LAGUNA_MBOX_RSPQ_DB_STATUS	0x1808
#define LAGUNA_MBOX_RSPQ_HEAD		0x180c
#define LAGUNA_MBOX_RSPQ_TAIL		0x1810
#define LAGUNA_MBOX_RSPQ_CONFIG		0x1814
#define LAGUNA_MBOX_RSPQ_ERR_STATUS	0x1818

/* Circular queue indices use BIT(10) as the wrap bit (max queue size = 1023) */
#define LAGUNA_QUEUE_WRAP_BIT		BIT(10)
#define LAGUNA_KCI_QUEUE_SIZE		512
#define LAGUNA_IKV_QUEUE_SIZE		512

static inline u32 laguna_circ_real_idx(u32 idx)
{
	return idx & (LAGUNA_QUEUE_WRAP_BIT - 1);
}

static inline u32 laguna_circ_inc(u32 idx, u32 inc, u32 size)
{
	u32 next = laguna_circ_real_idx(idx) + inc;

	if (unlikely(next >= size))
		return (idx + inc - size) ^ LAGUNA_QUEUE_WRAP_BIT;
	return idx + inc;
}

static inline u32 laguna_circ_cnt(u32 head, u32 tail, u32 size)
{
	u32 cnt;

	if ((tail ^ head) & LAGUNA_QUEUE_WRAP_BIT)
		cnt = size - laguna_circ_real_idx(head) + laguna_circ_real_idx(tail);
	else
		cnt = tail - head;

	return unlikely(cnt > size) ? 0 : cnt;
}

/* Firmware header and image configuration formats */
#define LAGUNA_FW_MAGIC			0x46555054U /* 'TPUF' */
#define LAGUNA_FW_IREMAP_BASE		0x10000000U
#define LAGUNA_FW_DEFAULT_CODE_LIMIT	SZ_1M
#define LAGUNA_FW_DEFAULT_DATA_SIZE	SZ_1M
#define LAGUNA_FW_LOG_BUF_SIZE		(16 * SZ_4K)
#define LAGUNA_FW_TRACE_BUF_SIZE	(64 * SZ_4K)

#define LAGUNA_FW_LEGACY_HDR_SIZE	0x1000
#define LAGUNA_FW_PQ_HDR_SIZE		0x5000
#define LAGUNA_FW_FMT_RDO		0x1
#define LAGUNA_FW_FMT_PQ		0x2

#define LAGUNA_FW_PRIV_GSA		0
#define LAGUNA_FW_PRIV_TZ		1
#define LAGUNA_FW_PRIV_NS		2

#define LAGUNA_IMG_MAX_IOMMU_MAPS	18
#define LAGUNA_IMG_MAX_NS_IOMMU_MAPS	5
#define LAGUNA_IMG_MAX_PROT_MEM		3

#define LAGUNA_IMG_MAP_FLAGS_MASK	(SZ_4K - 1)
#define LAGUNA_IMG_MAP_MMIO		BIT(0)
#define LAGUNA_IMG_MAP_SHARED		BIT(1)
#define LAGUNA_IMG_MAP_RO		BIT(2)
#define LAGUNA_IMG_MAP_36BIT		BIT(3)
#define LAGUNA_IMG_CFG_ADDR_SHIFT	12
#define LAGUNA_IMG_CFG_SIZE_MODE	BIT(LAGUNA_IMG_CFG_ADDR_SHIFT - 1)
#define LAGUNA_IMG_CFG_SIZE_MASK	(LAGUNA_IMG_CFG_SIZE_MODE - 1U)
#define LAGUNA_IMG_CFG_ADDR_MASK	(~(BIT(LAGUNA_IMG_CFG_ADDR_SHIFT) - 1U))

#define LAGUNA_IMG_OFF_MEM_MASK		GENMASK(3, 0)
#define LAGUNA_IMG_OFF_MMIO_MASK	GENMASK(7, 4)

struct laguna_image_config {
	u32 carveout_base;
	u32 firmware_base;
	u32 firmware_size;
	u32 firmware_versions[4];
	u32 config_version;
	u32 privilege_level;
	u32 secure_data_start;
	u32 secure_data_size;
	u32 num_iommu_mappings;
	struct {
		u32 virt_address;
		u32 image_config_value;
	} iommu_mappings[LAGUNA_IMG_MAX_IOMMU_MAPS];
	u8 mapping_offsets;
	u8 reserved[3];
	u32 shared_data_iova;
	u32 telemetry_buffer_config;
	u32 sanitizer_config;
	u32 protected_memory_regions[LAGUNA_IMG_MAX_PROT_MEM];
	u32 secure_telemetry_region_start;
	u32 shared_data_start;
	u32 shared_data_size;
	u32 num_ns_iommu_mappings;
	u32 ns_iommu_mappings[LAGUNA_IMG_MAX_NS_IOMMU_MAPS];
} __packed;

struct laguna_fw_sub_header {
	u32 magic;
	u32 rollback_info;
	u32 delegate_rollback_info;
	u32 length;
	u8 flags[40];
	u8 body_hash[64];
	u8 chip_id[20];
	u8 auth_config[256];
	struct laguna_image_config image_config;
};

struct laguna_fw_rdo_header {
	u8 root_sig[512];
	u8 root_pub[512];
	u8 delegate_pub[512];
	u8 delegate_policy[96];
	u8 delegate_sig[512];
	struct laguna_fw_sub_header sub_header;
};

struct laguna_fw_pq_header {
	u8 root_sig[512];
	u8 pq_root_sig[7856];
	u8 root_pub[512];
	u8 pq_root_pub[60];
	u8 delegate_pub[512];
	u8 pq_delegate_pub[60];
	u8 delegate_policy[96];
	u8 delegate_sig[512];
	u8 pq_delegate_sig[7856];
	struct laguna_fw_sub_header sub_header;
};

struct laguna_fw_common_header {
	u32 format_version;
	union {
		struct laguna_fw_rdo_header rdo;
		struct laguna_fw_pq_header pq;
	};
};

struct laguna_fw_legacy_header {
	u8 sig[512];
	u8 pub[512];
	struct {
		u32 magic;
		u32 generation;
		u32 rollback_info;
		u32 length;
		u8 flags[16];
	} common;
	union {
		struct {
			u8 body_hash[32];
			u8 chip_id[32];
			u8 auth_config[256];
			struct laguna_image_config image_config;
		} gen1;
		struct {
			u8 body_hash[64];
			u8 chip_id[32];
			u8 auth_config[256];
			struct laguna_image_config image_config;
		} gen2;
	};
};

/* KCI wire protocol structs */
#define LAGUNA_KCI_REVERSE_FLAG		0x8000000000000000ULL

enum laguna_kci_code {
	LAGUNA_KCI_ACK = 0,
	LAGUNA_KCI_MAP_LOG_BUFFER = 2,
	LAGUNA_KCI_MAP_TRACE_BUFFER = 5,
	LAGUNA_KCI_SHUTDOWN = 7,
	LAGUNA_KCI_OPEN_DEVICE = 9,
	LAGUNA_KCI_CLOSE_DEVICE = 10,
	LAGUNA_KCI_EXCHANGE_INFO = 11,
	LAGUNA_KCI_ALLOCATE_VMBOX = 15,
	LAGUNA_KCI_RELEASE_VMBOX = 16,
	LAGUNA_KCI_RKCI_ACK = 256,
};

enum laguna_rkci_code {
	LAGUNA_RKCI_CLIENT_FATAL_ERROR = 5,
	LAGUNA_RKCI_FIRMWARE_CRASH = 0x8000,
	LAGUNA_RKCI_JOB_LOCKUP = 0x8001,
};

struct laguna_kci_dma_desc {
	u64 address;
	u32 size;
	u32 flags;
};

struct laguna_kci_cmd_elem {
	u64 seq;
	u16 code;
	u16 reserved[3];
	struct laguna_kci_dma_desc dma;
} __packed;

struct laguna_kci_resp_elem {
	u64 seq;
	u16 code;
	u16 rkci_value1;
	union {
		u32 rkci_value2;
		u32 retval;
	};
} __packed;

struct laguna_kci_open_device_detail {
	u16 client_priv;
	u16 vcid;
	u32 flags;
};

struct laguna_kci_allocate_vmbox_detail {
	u32 client_id;
	u8 reserved_num_cores;
	u8 slice_index;
	u8 first_open;
	u8 first_party;
	u8 reserved[56];
};

struct laguna_kci_release_vmbox_detail {
	u32 client_id;
	u8 reserved[60];
};

struct laguna_fw_info {
	u64 fw_build_time;
	u32 fw_flavor;
	u32 fw_changelist;
	u32 spare[10];
};

/* Litebuf VII wire protocol structs (128B cmd / 64B resp) */
#define LAGUNA_VII_CMD_PAYLOAD_BYTES	96
#define LAGUNA_VII_RESP_PAYLOAD_BYTES	48

enum laguna_vii_cmd_type {
	LAGUNA_VII_CMD_RUNTIME = 0,
	LAGUNA_VII_CMD_LARGE_RUNTIME = 1,
	LAGUNA_VII_CMD_NULL = 3,
};

struct laguna_vii_litebuf_cmd {
	union {
		u8 runtime_command[LAGUNA_VII_CMD_PAYLOAD_BYTES];
		struct {
			u32 address;
			u32 size_bytes;
		} large_runtime_command;
	};
	u8 reserved_0[16];
	u32 seq;
	u32 client_id;
	u32 additional_info_address;
	u16 additional_info_size;
	u8 type;
	u8 reserved_1;
};

struct laguna_vii_litebuf_resp {
	u8 runtime_response[LAGUNA_VII_RESP_PAYLOAD_BYTES];
	u8 reserved_0[4];
	u32 seq;
	u32 client_id;
	u16 code;
	u8 type;
	u8 reserved_1;
};

#define LAGUNA_VII_RESP_KERNEL_BASE		BIT(15)
#define LAGUNA_VII_RESP_KERNEL_ENQUEUE_FAILED	(LAGUNA_VII_RESP_KERNEL_BASE + 1)

/* User-space UAPI (0xED ioctl ABI 1.4 subset used by libopenbuenos) */
#define EDGETPU_IOCTL_BASE		0xED

#define EDGETPU_MAP_DIR_MASK		3U
#define EDGETPU_MAP_DMA_BIDIRECTIONAL	0U
#define EDGETPU_MAP_DMA_TO_DEVICE	1U
#define EDGETPU_MAP_DMA_FROM_DEVICE	2U
#define EDGETPU_MAP_DMA_NONE		3U
#define EDGETPU_MAP_36BIT_IOVA		BIT(3)
#define EDGETPU_MAP_SKIP_CPU_SYNC	BIT(4)
#define EDGETPU_MAP_COHERENT		BIT(9)

#define EDGETPU_SYNC_FOR_DEVICE		(0U << 2)
#define EDGETPU_SYNC_FOR_CPU		(1U << 2)

#define EDGETPU_EVENT_RESPDATA		0
#define EDGETPU_EVENT_FATAL_ERROR	1

struct edgetpu_map_ioctl {
	__u64 host_address;
	__u64 size;
	__u64 device_address;
	__u32 flags;
	__u32 die_index;
};

struct edgetpu_event_register {
	__u32 event_id;
	__u32 eventfd;
};

struct edgetpu_mailbox_attr {
	__u32 cmd_queue_size;
	__u32 resp_queue_size;
	__u32 sizeof_cmd;
	__u32 sizeof_resp;
	__u32 priority            : 4;
	__u32 cmdq_tail_doorbell  : 1;
	__u32 partition_type      : 1;
	__u32 client_priv         : 1;
	__u32 partition_type_high : 1;
	__u32 reserved            : 24;
};

struct edgetpu_sync_ioctl {
	__u64 device_address;
	__u64 size;
	__u64 offset;
	__u32 die_index;
	__u32 flags;
};

struct edgetpu_map_dmabuf_ioctl {
	__u64 offset;
	__u64 size;
	__u64 device_address;
	__s32 dmabuf_fd;
	__u32 flags;
	__u32 die_index;
};

struct edgetpu_fw_version {
	__u32 major_version;
	__u32 minor_version;
	__u32 vii_version;
	__u32 kci_version;
};

struct edgetpu_vii_litebuf_command_ioctl {
	__u64 litebuf_address;
	__u32 litebuf_size;
	__u64 seq;
	__u64 in_fence_array;
	__u32 in_fence_count;
	__u64 out_fence_array;
	__u32 out_fence_count;
	__u8 reserved[4];
};

struct edgetpu_vii_litebuf_response_ioctl {
	__u64 litebuf_address;
	__u64 seq;
	__u16 code;
	__u8 reserved[6];
};

#define EDGETPU_IFACE_VER_BUILD_SIZE	64
struct edgetpu_interface_version_ioctl {
	__u16 version_major;
	__u16 version_minor;
	char version_build[EDGETPU_IFACE_VER_BUILD_SIZE];
};

#define EDGETPU_MAP_BUFFER \
	_IOWR(EDGETPU_IOCTL_BASE, 0, struct edgetpu_map_ioctl)
#define EDGETPU_UNMAP_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 4, struct edgetpu_map_ioctl)
#define EDGETPU_SET_EVENTFD \
	_IOW(EDGETPU_IOCTL_BASE, 5, struct edgetpu_event_register)
#define EDGETPU_CREATE_GROUP \
	_IOW(EDGETPU_IOCTL_BASE, 6, struct edgetpu_mailbox_attr)
#define EDGETPU_FINALIZE_GROUP \
	_IO(EDGETPU_IOCTL_BASE, 8)
#define EDGETPU_UNSET_EVENT \
	_IOW(EDGETPU_IOCTL_BASE, 14, __u32)
#define EDGETPU_SYNC_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 16, struct edgetpu_sync_ioctl)
#define EDGETPU_MAP_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 17, struct edgetpu_map_dmabuf_ioctl)
#define EDGETPU_UNMAP_DMABUF \
	_IOW(EDGETPU_IOCTL_BASE, 18, struct edgetpu_map_dmabuf_ioctl)
#define EDGETPU_RELEASE_WAKE_LOCK \
	_IO(EDGETPU_IOCTL_BASE, 25)
#define EDGETPU_ACQUIRE_WAKE_LOCK \
	_IO(EDGETPU_IOCTL_BASE, 26)
#define EDGETPU_FIRMWARE_VERSION \
	_IOR(EDGETPU_IOCTL_BASE, 27, struct edgetpu_fw_version)
#define EDGETPU_GET_FATAL_ERRORS \
	_IOR(EDGETPU_IOCTL_BASE, 32, __u32)
#define EDGETPU_ACQUIRE_WAKE_LOCK_FLAGS \
	_IOR(EDGETPU_IOCTL_BASE, 33, __u32)
#define EDGETPU_VII_LITEBUF_COMMAND \
	_IOWR(EDGETPU_IOCTL_BASE, 37, struct edgetpu_vii_litebuf_command_ioctl)
#define EDGETPU_VII_LITEBUF_RESPONSE \
	_IOWR(EDGETPU_IOCTL_BASE, 38, struct edgetpu_vii_litebuf_response_ioctl)
#define EDGETPU_GET_CREDIT_SIZE \
	_IOR(EDGETPU_IOCTL_BASE, 39, __u64)
#define EDGETPU_GET_INTERFACE_VERSION \
	_IOR(EDGETPU_IOCTL_BASE, 42, struct edgetpu_interface_version_ioctl)

/* Driver internal state */
struct laguna_iremap_mem {
	void *vaddr;
	dma_addr_t daddr;
	phys_addr_t paddr;
	size_t size;
};

struct laguna_mailbox {
	struct edgetpu_laguna_dev *etdev;
	void __iomem *csr_base;
	int irq;
	u32 index;
	u32 cmd_queue_size;
	u32 cmd_queue_tail;
	u32 resp_queue_size;
	u32 resp_queue_head;
	void (*handle_irq)(struct laguna_mailbox *mbox);
};

struct laguna_kci {
	struct edgetpu_laguna_dev *etdev;
	struct laguna_mailbox *mbox;
	struct laguna_iremap_mem cmdq_mem;
	struct laguna_iremap_mem rspq_mem;
	struct mutex cmdq_lock;
	spinlock_t rspq_lock;
	spinlock_t waiter_lock;
	struct list_head waiters;
	atomic64_t seq;
	struct workqueue_struct *rkci_wq;
	struct work_struct rkci_work;
	spinlock_t rkci_lock;
	struct laguna_kci_resp_elem rkci_ring[32];
	u32 rkci_head;
	u32 rkci_tail;
};

struct laguna_ikv {
	struct edgetpu_laguna_dev *etdev;
	struct laguna_mailbox *mbox;
	struct laguna_iremap_mem cmdq_mem;
	struct laguna_iremap_mem rspq_mem;
	struct mutex cmdq_lock;
	spinlock_t rspq_lock;
	spinlock_t inflight_lock;
	struct list_head inflight;
	atomic_t seq;
};

struct laguna_domain {
	struct edgetpu_laguna_dev *etdev;
	struct iommu_domain *domain;
	struct iova_domain iovad;
	u32 pasid;
	bool attached;
	bool shared_mapped;
};

struct laguna_img_mapping {
	struct list_head node;
	dma_addr_t daddr;
	phys_addr_t paddr;
	size_t size;
	u32 cfg_map_flags;
	struct sg_table *sgt;
	struct page **pages;
	unsigned int nr_pages;
};

struct laguna_buf_mapping {
	struct list_head node;
	dma_addr_t iova;
	size_t size;
	size_t alloc_size;
	u32 flags;
	enum dma_data_direction dir;
	bool is_dmabuf;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct page **pages;
	unsigned int nr_pages;
};

struct laguna_client {
	struct kref ref;
	struct edgetpu_laguna_dev *etdev;
	struct mutex lock;
	bool group_created;
	bool wakelock_held;
	bool activated;
	bool first_open;
	bool client_priv;
	bool vmbox_allocated;
	s16 vcid;
	struct laguna_domain *etdom;
	struct list_head mappings;
	struct mutex map_lock;
	spinlock_t resp_lock;
	struct list_head resp_ready;
	u32 inflight_count;
	struct eventfd_ctx *resp_eventfd;
	struct eventfd_ctx *fatal_eventfd;
	u32 fatal_errors;
};

struct edgetpu_laguna_dev {
	struct device *dev;
	struct device *dma_dev;
	struct device_dma_parameters dma_parms;
	struct miscdevice mdev;
	void __iomem *top_regs;
	void __iomem *lpcm_regs;
	void __iomem *lpb_regs;
	struct device *gsa_dev;
	int mbox_irqs[2];

	struct mutex pm_lock;
	unsigned int power_ref;
	bool powered;

	struct mutex fw_lock;
	bool fw_loaded;
	bool fw_is_ns;
	phys_addr_t fw_paddr;
	size_t fw_region_size;
	phys_addr_t shared_paddr;
	dma_addr_t shared_daddr;
	void *shared_vaddr;
	size_t shared_size;
	u32 fw_versions[4];
	struct laguna_fw_info fw_info;
	u32 num_telemetry;
	size_t log_size;
	size_t trace_size;
	struct gen_pool *iremap_pool;
	unsigned long iremap_vaddr;
	dma_addr_t iremap_daddr;
	phys_addr_t iremap_paddr;
	struct list_head img_mappings;
	struct mutex img_map_lock;

	struct laguna_kci kci;
	struct laguna_ikv ikv;

	dma_addr_t dma_win_start;
	dma_addr_t dma_win_end;
	u32 max_pasids;
	struct iommu_domain *default_domain;
	DECLARE_BITMAP(pasid_bitmap, LAGUNA_NUM_PASIDS);
	DECLARE_BITMAP(vcid_bitmap, LAGUNA_NUM_VCIDS);
	struct mutex mmu_lock;
	spinlock_t clients_lock;
	struct laguna_client *pasid_clients[LAGUNA_NUM_PASIDS];
};

/* Core CSR accessors */
static inline u32 laguna_read32(struct edgetpu_laguna_dev *etdev, u32 off)
{
	return readl(etdev->top_regs + off);
}

static inline void laguna_write32(struct edgetpu_laguna_dev *etdev, u32 off, u32 val)
{
	writel(val, etdev->top_regs + off);
}

/* Firmware & power management */
int edgetpu_laguna_fw_init(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_fw_exit(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_fw_boot(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_fw_restart(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_fw_stop(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_fw_shared_context_map(struct edgetpu_laguna_dev *etdev,
					 struct laguna_domain *etdom);
void edgetpu_laguna_fw_shared_context_unmap(struct edgetpu_laguna_dev *etdev,
					    struct laguna_domain *etdom);
int edgetpu_laguna_iremap_alloc(struct edgetpu_laguna_dev *etdev, size_t size,
				struct laguna_iremap_mem *mem);
void edgetpu_laguna_iremap_free(struct edgetpu_laguna_dev *etdev,
				struct laguna_iremap_mem *mem);
int edgetpu_laguna_pm_get(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_pm_put(struct edgetpu_laguna_dev *etdev);

/* Mailbox, KCI & IKV */
int edgetpu_laguna_kci_init(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_kci_reinit(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_kci_exit(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_kci_exchange_info(struct edgetpu_laguna_dev *etdev,
				     struct laguna_fw_info *info);
int edgetpu_laguna_kci_map_telemetry(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_kci_open_device(struct edgetpu_laguna_dev *etdev, u32 mailbox_map,
				   u32 client_priv, s16 vcid, bool first_open);
int edgetpu_laguna_kci_allocate_vmbox(struct edgetpu_laguna_dev *etdev, u32 client_id,
				      u8 slice_index, bool first_open, bool first_party);
int edgetpu_laguna_kci_release_vmbox(struct edgetpu_laguna_dev *etdev, u32 client_id);
int edgetpu_laguna_kci_shutdown(struct edgetpu_laguna_dev *etdev);

int edgetpu_laguna_ikv_init(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_ikv_reinit(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_ikv_exit(struct edgetpu_laguna_dev *etdev);
int edgetpu_laguna_ikv_submit(struct laguna_client *client,
			      const struct edgetpu_vii_litebuf_command_ioctl *req);
int edgetpu_laguna_ikv_pop_resp(struct laguna_client *client,
				struct edgetpu_vii_litebuf_response_ioctl *req);
void edgetpu_laguna_ikv_cancel_client(struct laguna_client *client);

/* IOMMU & buffer mapping */
int edgetpu_laguna_iommu_init(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_iommu_exit(struct edgetpu_laguna_dev *etdev);
struct laguna_domain *edgetpu_laguna_domain_alloc(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_domain_free(struct edgetpu_laguna_dev *etdev,
				struct laguna_domain *etdom);
int edgetpu_laguna_iommu_map(struct iommu_domain *domain, dma_addr_t iova,
			     phys_addr_t paddr, size_t size, u32 flags);
void edgetpu_laguna_iommu_unmap(struct iommu_domain *domain, dma_addr_t iova,
				size_t size);
int edgetpu_laguna_iommu_map_sgt(struct iommu_domain *domain, dma_addr_t iova,
				 struct sg_table *sgt, u32 flags);
int edgetpu_laguna_map_dmabuf(struct laguna_client *client,
			      struct edgetpu_map_dmabuf_ioctl *req);
int edgetpu_laguna_unmap_dmabuf(struct laguna_client *client,
				const struct edgetpu_map_dmabuf_ioctl *req);
int edgetpu_laguna_map_user(struct laguna_client *client,
			    struct edgetpu_map_ioctl *req);
int edgetpu_laguna_unmap_user(struct laguna_client *client,
			      const struct edgetpu_map_ioctl *req);
int edgetpu_laguna_sync_buffer(struct laguna_client *client,
			       const struct edgetpu_sync_ioctl *req);
void edgetpu_laguna_unmap_all_client(struct laguna_client *client);

/* Userspace device node & client helpers */
int edgetpu_laguna_fs_register(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_fs_unregister(struct edgetpu_laguna_dev *etdev);
void edgetpu_laguna_client_get(struct laguna_client *client);
void edgetpu_laguna_client_put(struct laguna_client *client);
void edgetpu_laguna_client_notify_fatal(struct laguna_client *client, u32 err_mask);
void edgetpu_laguna_broadcast_fatal(struct edgetpu_laguna_dev *etdev, u32 err_mask);

#endif /* _EDGETPU_LAGUNA_H_ */
