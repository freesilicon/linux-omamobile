// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clean-room ALSA ASoC driver for Google Tensor G5 (Laguna) AoC audio.
 *
 * Bridges standard ALSA PCM playback/capture and mixer kcontrols to the
 * Laguna Always-on Compute (AoC) firmware IPC rings and control queues via
 * the lga-aoc core driver.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/google/lga-aoc.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define LGA_AOC_AUDIO_DRV_NAME		"lga-aoc-audio"
#define LGA_AOC_AUDIO_DAI_NAME		"lga-aoc-pcm-dai"

#define LGA_AOC_AUDIO_OUT_CTRL_SVC	"audio_output_control"
#define LGA_AOC_AUDIO_IN_CTRL_SVC	"audio_input_control"

#define LGA_AOC_AUDIO_CMD_TIMEOUT_MS	1000
#define LGA_AOC_AUDIO_TX_TIMEOUT_MS	500
#define LGA_AOC_AUDIO_PUMP_POLL_MS	20
#define LGA_AOC_AUDIO_PUMP_CHUNK_MAX	4096U
#define LGA_AOC_AUDIO_PCM_BUF_MAX	(128 * 1024)
#define LGA_AOC_AUDIO_RSP_MAX		1024
#define LGA_AOC_AUDIO_DRAIN_LIMIT	16

/* AoC audio IPC container type */
#define LGA_AOC_AUDIO_TYPE_CMD		0

/* AoC audio_output_control command IDs */
#define LGA_AOC_CMD_OUT_SOURCE		201
#define LGA_AOC_CMD_OUT_EP_SETUP	206
#define LGA_AOC_CMD_OUT_SET_PARAM	209
#define LGA_AOC_CMD_OUT_GET_DSP_STATE	233
#define LGA_AOC_CMD_OUT_SINK2		270
#define LGA_AOC_CMD_OUT_BIND2		271
#define LGA_AOC_CMD_OUT_EP_SETUP2	298

/* AoC audio_input_control command IDs */
#define LGA_AOC_CMD_IN_MIC_PWR_ON	203
#define LGA_AOC_CMD_IN_MIC_PWR_OFF	204
#define LGA_AOC_CMD_IN_AP_START		208
#define LGA_AOC_CMD_IN_AP_STOP		209
#define LGA_AOC_CMD_IN_MIC_SET_PARAMS	224
#define LGA_AOC_CMD_IN_MIC_START_DATA	293
#define LGA_AOC_CMD_IN_MIC_SET_PARAMS2	359

/* AoC audio output sink identifiers */
#define LGA_AOC_SINK_SPEAKER		0
#define LGA_AOC_SINK_AUX		1
#define LGA_AOC_SINK_BT			2
#define LGA_AOC_SINK_USB		3
#define LGA_AOC_SINK_MODEM		4
#define LGA_AOC_SINK_NONE		5
#define LGA_AOC_SINK_BLOCK_BASE		16

/* AoC sink processing modes */
#define LGA_AOC_SINK_MODE_IDLE		0
#define LGA_AOC_SINK_MODE_ACTIVE	1

/* AoC entrypoint modes */
#define LGA_AOC_EP_MODE_PLAYBACK	0
#define LGA_AOC_EP_DEFAULT_WATERMARK	48000U

/* AoC audio input processor source indices */
#define LGA_AOC_IN_SRC_MODEM		0
#define LGA_AOC_IN_SRC_MIC		1
#define LGA_AOC_IN_SRC_BT		2
#define LGA_AOC_IN_SRC_USB		3

/* AoC capture PDM metadata constants */
#define LGA_AOC_PDM_MAX_MICS		3
#define LGA_AOC_PDM_SLOTS		4
#define LGA_AOC_PDM_SLOT_UNUSED		0xff
#define LGA_AOC_MIC_PROC_RAW		0

/* AoC channel metadata format encodings */
#define LGA_AOC_FMT_FIXED_POINT		1
#define LGA_AOC_BITS_16			1
#define LGA_AOC_BITS_24			2
#define LGA_AOC_BITS_32			3

/* AoC sample-rate encodings */
#define LGA_AOC_SR_8K			0
#define LGA_AOC_SR_16K			1
#define LGA_AOC_SR_24K			2
#define LGA_AOC_SR_32K			3
#define LGA_AOC_SR_44K1			4
#define LGA_AOC_SR_48K			5
#define LGA_AOC_SR_96K			6
#define LGA_AOC_SR_192K			7

static char *playback_service = "audio_playback0";
module_param(playback_service, charp, 0444);
MODULE_PARM_DESC(playback_service,
		 "AoC playback IPC ring service name (default audio_playback0)");

static char *capture_service = "audio_capture0";
module_param(capture_service, charp, 0444);
MODULE_PARM_DESC(capture_service,
		 "AoC capture IPC ring service name (default audio_capture0)");

static bool ep_setup2 = true;
module_param(ep_setup2, bool, 0644);
MODULE_PARM_DESC(ep_setup2,
		 "Send extended buffer/period setup commands (default true)");

static bool debug_cmd;
module_param(debug_cmd, bool, 0644);
MODULE_PARM_DESC(debug_cmd,
		 "Log AoC audio control command exchanges (default false)");

/* Packed wire structures for AoC audio control IPC */
struct lga_aoc_audio_hdr {
	u8 type;
	u8 cntr;
	__le16 len;
	__le16 id;
	__le16 reply;
} __packed;

struct lga_aoc_chan_meta {
	u8 chan;
	u8 bits;
	u8 sr;
	u8 format;
	u8 offset;
} __packed;

struct lga_aoc_cmd_out_source {
	struct lga_aoc_audio_hdr hdr;
	u8 source;
	u8 on;
} __packed;

struct lga_aoc_cmd_out_ep_setup {
	struct lga_aoc_audio_hdr hdr;
	__le32 address;
	__le32 watermark;
	__le32 length;
	struct lga_aoc_chan_meta meta;
	u8 channel;
	u8 wraparound;
	u8 mode;
} __packed;

struct lga_aoc_cmd_out_ep_setup2 {
	struct lga_aoc_audio_hdr hdr;
	u8 channel;
	__le32 buffer_size;
	__le32 period_size;
	u8 mode;
} __packed;

struct lga_aoc_cmd_out_sink2 {
	struct lga_aoc_audio_hdr hdr;
	u8 sink;
	u8 mode;
	u8 timestamping;
} __packed;

struct lga_aoc_cmd_out_bind2 {
	struct lga_aoc_audio_hdr hdr;
	u8 src;
	u8 dst;
	u8 bind;
} __packed;

struct lga_aoc_cmd_out_set_param {
	struct lga_aoc_audio_hdr hdr;
	u8 block;
	u8 component;
	__le32 key;
	__le32 val;
} __packed;

struct lga_aoc_cmd_out_dsp_state {
	struct lga_aoc_audio_hdr hdr;
	u8 mode;
} __packed;

struct lga_aoc_cmd_in_mic_pwr {
	struct lga_aoc_audio_hdr hdr;
	u8 mic_index;
} __packed;

struct lga_aoc_cmd_in_ap_start {
	struct lga_aoc_audio_hdr hdr;
	u8 mic_input_source;
} __packed;

struct lga_aoc_cmd_in_mic_params {
	struct lga_aoc_audio_hdr hdr;
	struct lga_aoc_chan_meta meta;
	u8 pdm_mask;
	u8 period_ms;
	u8 num_periods;
	u8 interleaving[LGA_AOC_PDM_SLOTS];
	u8 sample_rate;
	u8 mic_process_index;
} __packed;

struct lga_aoc_cmd_in_mic_params2 {
	struct lga_aoc_audio_hdr hdr;
	u8 channel;
	__le32 buffer_size;
	__le32 period_size;
} __packed;

struct lga_aoc_stream {
	struct lga_aoc_audio *audio;
	struct snd_pcm_substream *substream;
	struct lga_aoc_service_info svc;
	struct task_struct *pump_task;
	wait_queue_head_t pump_wq;
	wait_queue_head_t idle_wq;
	atomic_t running;
	atomic_t in_pump;
	atomic_t xfer_pos;
	size_t period_accum;
	u8 *scratch;
	bool available;
	bool configured;
	bool active_in_aoc;
	u8 ep_idx;
	u8 bound_sink;
	u8 powered_mics;
};

struct lga_aoc_audio {
	struct device *dev;
	struct device *aoc_dev;

	struct lga_aoc_service_info out_ctrl;
	struct lga_aoc_service_info in_ctrl;
	bool have_out_ctrl;
	bool have_in_ctrl;

	struct mutex out_cmd_lock;	/* Serializes audio_output_control IPC */
	struct mutex in_cmd_lock;	/* Serializes audio_input_control IPC */
	atomic_t cmd_seq;
	u8 out_rsp_buf[LGA_AOC_AUDIO_RSP_MAX];
	u8 in_rsp_buf[LGA_AOC_AUDIO_RSP_MAX];

	/* Mixer state protected by mixer_lock */
	struct mutex mixer_lock;
	u8 pb_route;
	u8 cap_src;
	bool pb_unmuted;
	u32 pb_volume;
	u8 mic_mask;

	struct lga_aoc_stream pb;
	struct lga_aoc_stream cap;

	struct snd_soc_dai_link_component cpu_dlc;
	struct snd_soc_dai_link_component plat_dlc;
	struct snd_soc_dai_link dai_link;
	struct snd_soc_card card;
};

/* Global notifier binding state */
static DEFINE_MUTEX(lga_aoc_audio_bind_lock);
static struct platform_device *lga_aoc_audio_pdev;
static struct work_struct lga_aoc_audio_attach_work;
static struct work_struct lga_aoc_audio_detach_work;

static const struct snd_pcm_hardware lga_aoc_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_8000 |
		 SNDRV_PCM_RATE_16000 |
		 SNDRV_PCM_RATE_24000 |
		 SNDRV_PCM_RATE_32000 |
		 SNDRV_PCM_RATE_44100 |
		 SNDRV_PCM_RATE_48000 |
		 SNDRV_PCM_RATE_96000 |
		 SNDRV_PCM_RATE_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 4,
	.buffer_bytes_max = LGA_AOC_AUDIO_PCM_BUF_MAX,
	.period_bytes_min = 480,
	.period_bytes_max = 32768,
	.periods_min = 2,
	.periods_max = 16,
};

static const char * const lga_aoc_pb_route_texts[] = {
	"Speaker",
	"Earpiece",
	"BT-SCO",
	"USB",
	"Modem",
	"None",
};

static const char * const lga_aoc_cap_src_texts[] = {
	"Internal Mic",
	"BT-SCO Mic",
	"USB Mic",
	"Modem",
};

static const char * const lga_aoc_dsp_state_texts[] = {
	"Idle",
	"Playback",
	"Voice",
};

static const struct soc_enum lga_aoc_pb_route_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lga_aoc_pb_route_texts),
			    lga_aoc_pb_route_texts);

static const struct soc_enum lga_aoc_cap_src_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lga_aoc_cap_src_texts),
			    lga_aoc_cap_src_texts);

static const struct soc_enum lga_aoc_dsp_state_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lga_aoc_dsp_state_texts),
			    lga_aoc_dsp_state_texts);

static int lga_aoc_encode_rate(unsigned int rate, u8 *out)
{
	switch (rate) {
	case 8000:
		*out = LGA_AOC_SR_8K;
		return 0;
	case 16000:
		*out = LGA_AOC_SR_16K;
		return 0;
	case 24000:
		*out = LGA_AOC_SR_24K;
		return 0;
	case 32000:
		*out = LGA_AOC_SR_32K;
		return 0;
	case 44100:
		*out = LGA_AOC_SR_44K1;
		return 0;
	case 48000:
		*out = LGA_AOC_SR_48K;
		return 0;
	case 96000:
		*out = LGA_AOC_SR_96K;
		return 0;
	case 192000:
		*out = LGA_AOC_SR_192K;
		return 0;
	default:
		return -EINVAL;
	}
}

static int lga_aoc_encode_bits(snd_pcm_format_t format, u8 *out)
{
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		*out = LGA_AOC_BITS_16;
		return 0;
	case SNDRV_PCM_FORMAT_S24_LE:
		*out = LGA_AOC_BITS_24;
		return 0;
	case SNDRV_PCM_FORMAT_S32_LE:
		*out = LGA_AOC_BITS_32;
		return 0;
	default:
		return -EINVAL;
	}
}

static u8 lga_aoc_cap_src_to_fw(u8 cap_src)
{
	switch (cap_src) {
	case 0:
		return LGA_AOC_IN_SRC_MIC;
	case 1:
		return LGA_AOC_IN_SRC_BT;
	case 2:
		return LGA_AOC_IN_SRC_USB;
	default:
		return LGA_AOC_IN_SRC_MODEM;
	}
}

static u8 lga_aoc_parse_ep_index(const char *svc_name)
{
	size_t len;
	char last;

	if (!svc_name)
		return 0;
	len = strlen(svc_name);
	if (!len)
		return 0;
	last = svc_name[len - 1];
	if (last >= '0' && last <= '5')
		return last - '0';
	return 0;
}

static int lga_aoc_audio_cmd(struct lga_aoc_audio *audio, bool is_input,
			     u16 cmd_id, void *cmd_buf, size_t cmd_len,
			     void *rsp_buf, size_t rsp_len)
{
	struct lga_aoc_audio_hdr *hdr = cmd_buf;
	struct lga_aoc_service_info *ctrl;
	struct mutex *lock;	/* Points to out_cmd_lock or in_cmd_lock */
	u8 *rx_buf;
	unsigned long deadline;
	ssize_t ret;
	int i;
	u8 seq;

	if (!lga_aoc_is_online(audio->aoc_dev))
		return -ENODEV;

	if (is_input) {
		if (!audio->have_in_ctrl)
			return -ENODEV;
		ctrl = &audio->in_ctrl;
		lock = &audio->in_cmd_lock;
		rx_buf = audio->in_rsp_buf;
	} else {
		if (!audio->have_out_ctrl)
			return -ENODEV;
		ctrl = &audio->out_ctrl;
		lock = &audio->out_cmd_lock;
		rx_buf = audio->out_rsp_buf;
	}

	if (cmd_len < sizeof(*hdr) || cmd_len > ctrl->down_size)
		return -EINVAL;

	mutex_lock(lock);

	/* Drain any stale queued responses before issuing a new command. */
	for (i = 0; i < LGA_AOC_AUDIO_DRAIN_LIMIT; i++) {
		ret = lga_aoc_ipc_read(audio->aoc_dev, ctrl->index,
				       rx_buf, LGA_AOC_AUDIO_RSP_MAX);
		if (ret <= 0)
			break;
	}

	seq = atomic_inc_return(&audio->cmd_seq) & 0xff;
	if (!seq)
		seq = atomic_inc_return(&audio->cmd_seq) & 0xff;

	hdr->type = LGA_AOC_AUDIO_TYPE_CMD;
	hdr->cntr = seq;
	hdr->len = cpu_to_le16(cmd_len);
	hdr->id = cpu_to_le16(cmd_id);
	hdr->reply = 0;

	if (debug_cmd)
		dev_info(audio->dev, "cmd tx [%s] id=%u seq=%u len=%zu\n",
			 ctrl->name, cmd_id, seq, cmd_len);

	ret = lga_aoc_ipc_write_timeout(audio->aoc_dev, ctrl->index,
					cmd_buf, cmd_len,
					msecs_to_jiffies(LGA_AOC_AUDIO_TX_TIMEOUT_MS));
	if (ret < 0) {
		dev_warn(audio->dev, "cmd tx [%s] id=%u failed: %zd\n",
			 ctrl->name, cmd_id, ret);
		goto out_unlock;
	}

	deadline = jiffies + msecs_to_jiffies(LGA_AOC_AUDIO_CMD_TIMEOUT_MS);
	for (;;) {
		struct lga_aoc_audio_hdr *rhdr;
		long rem = deadline - jiffies;
		s16 reply;

		if (rem <= 0) {
			dev_warn(audio->dev, "cmd rx [%s] id=%u timed out\n",
				 ctrl->name, cmd_id);
			ret = -ETIMEDOUT;
			goto out_unlock;
		}

		ret = lga_aoc_ipc_read_timeout(audio->aoc_dev, ctrl->index,
					       rx_buf, LGA_AOC_AUDIO_RSP_MAX,
					       rem);
		if (ret < 0) {
			if (ret != -EINTR)
				dev_warn(audio->dev,
					 "cmd rx [%s] id=%u failed: %zd\n",
					 ctrl->name, cmd_id, ret);
			goto out_unlock;
		}

		if (ret < (ssize_t)sizeof(*rhdr))
			continue;

		rhdr = (struct lga_aoc_audio_hdr *)rx_buf;
		if (le16_to_cpu(rhdr->id) != cmd_id)
			continue;

		reply = (s16)le16_to_cpu(rhdr->reply);
		if (debug_cmd)
			dev_info(audio->dev,
				 "cmd rx [%s] id=%u seq=%u reply=%d len=%zd\n",
				 ctrl->name, cmd_id, rhdr->cntr, reply, ret);

		if (rsp_buf && rsp_len)
			memcpy(rsp_buf, rx_buf,
			       min_t(size_t, ret, rsp_len));

		ret = reply < 0 ? -EREMOTEIO : 0;
		goto out_unlock;
	}

out_unlock:
	mutex_unlock(lock);
	return ret;
}

static int lga_aoc_set_volume_hw(struct lga_aoc_audio *audio, u8 ep_idx,
				 u8 sink, u32 volume)
{
	struct lga_aoc_cmd_out_set_param cmd = { };

	if (sink >= LGA_AOC_SINK_NONE)
		return 0;

	cmd.block = sink + LGA_AOC_SINK_BLOCK_BASE;
	cmd.component = 0;
	cmd.key = cpu_to_le32(ep_idx);
	cmd.val = cpu_to_le32(volume);

	return lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_SET_PARAM,
				 &cmd, sizeof(cmd), NULL, 0);
}

static int lga_aoc_set_mic_power(struct lga_aoc_audio *audio, u8 mic_mask,
				 bool on)
{
	u16 cmd_id = on ? LGA_AOC_CMD_IN_MIC_PWR_ON : LGA_AOC_CMD_IN_MIC_PWR_OFF;
	int ret = 0;
	u8 i;

	for (i = 0; i < LGA_AOC_PDM_MAX_MICS; i++) {
		struct lga_aoc_cmd_in_mic_pwr cmd = { };
		int rc;

		if (!(mic_mask & BIT(i)))
			continue;

		cmd.mic_index = i;
		rc = lga_aoc_audio_cmd(audio, true, cmd_id, &cmd, sizeof(cmd),
				       NULL, 0);
		if (rc < 0 && !ret)
			ret = rc;
	}

	return ret;
}

static void lga_aoc_playback_teardown(struct lga_aoc_stream *st)
{
	struct lga_aoc_audio *audio = st->audio;
	u8 sink;
	bool was_configured;

	if (st->active_in_aoc) {
		struct lga_aoc_cmd_out_source src_cmd = {
			.source = st->ep_idx,
			.on = 0,
		};

		lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_SOURCE,
				  &src_cmd, sizeof(src_cmd), NULL, 0);
		st->active_in_aoc = false;
	}

	mutex_lock(&audio->mixer_lock);
	was_configured = st->configured;
	sink = st->bound_sink;
	st->bound_sink = LGA_AOC_SINK_NONE;
	st->configured = false;
	mutex_unlock(&audio->mixer_lock);

	if (!was_configured)
		return;

	if (sink < LGA_AOC_SINK_NONE) {
		struct lga_aoc_cmd_out_bind2 bind_cmd = {
			.src = st->ep_idx,
			.dst = sink,
			.bind = 0,
		};
		struct lga_aoc_cmd_out_sink2 sink_cmd = {
			.sink = sink,
			.mode = LGA_AOC_SINK_MODE_IDLE,
			.timestamping = 0,
		};

		lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_BIND2,
				  &bind_cmd, sizeof(bind_cmd), NULL, 0);
		lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_SINK2,
				  &sink_cmd, sizeof(sink_cmd), NULL, 0);
	}
}

static void lga_aoc_capture_teardown(struct lga_aoc_stream *st)
{
	struct lga_aoc_audio *audio = st->audio;

	if (st->active_in_aoc) {
		struct lga_aoc_audio_hdr stop_cmd = { };

		lga_aoc_audio_cmd(audio, true, LGA_AOC_CMD_IN_AP_STOP,
				  &stop_cmd, sizeof(stop_cmd), NULL, 0);
		st->active_in_aoc = false;
	}

	if (!st->configured)
		return;

	if (st->powered_mics) {
		lga_aoc_set_mic_power(audio, st->powered_mics, false);
		st->powered_mics = 0;
	}

	st->configured = false;
}

static int lga_aoc_playback_setup(struct lga_aoc_stream *st,
				  struct snd_pcm_runtime *runtime)
{
	struct lga_aoc_audio *audio = st->audio;
	struct lga_aoc_cmd_out_ep_setup ep_cmd = { };
	u8 sr_code, bits_code, sink;
	u32 vol;
	int ret;

	ret = lga_aoc_encode_rate(runtime->rate, &sr_code);
	if (ret)
		return ret;
	ret = lga_aoc_encode_bits(runtime->format, &bits_code);
	if (ret)
		return ret;

	mutex_lock(&audio->mixer_lock);
	sink = audio->pb_route;
	vol = audio->pb_unmuted ? audio->pb_volume : 0;
	mutex_unlock(&audio->mixer_lock);

	ep_cmd.address = 0;
	ep_cmd.watermark = cpu_to_le32(LGA_AOC_EP_DEFAULT_WATERMARK);
	ep_cmd.length = 0;
	ep_cmd.meta.chan = runtime->channels;
	ep_cmd.meta.bits = bits_code;
	ep_cmd.meta.sr = sr_code;
	ep_cmd.meta.format = LGA_AOC_FMT_FIXED_POINT;
	ep_cmd.meta.offset = 0;
	ep_cmd.channel = st->ep_idx;
	ep_cmd.wraparound = 1;
	ep_cmd.mode = LGA_AOC_EP_MODE_PLAYBACK;

	ret = lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_EP_SETUP,
				&ep_cmd, sizeof(ep_cmd), NULL, 0);
	if (ret < 0)
		return ret;

	if (ep_setup2) {
		struct lga_aoc_cmd_out_ep_setup2 ep2_cmd = {
			.channel = st->ep_idx,
			.buffer_size = cpu_to_le32(snd_pcm_lib_buffer_bytes(st->substream)),
			.period_size = cpu_to_le32(snd_pcm_lib_period_bytes(st->substream)),
			.mode = LGA_AOC_EP_MODE_PLAYBACK,
		};
		int rc2;

		rc2 = lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_EP_SETUP2,
					&ep2_cmd, sizeof(ep2_cmd), NULL, 0);
		if (rc2 < 0)
			dev_info(audio->dev,
				 "optional OUT_EP_SETUP2 returned %d; continuing\n",
				 rc2);
	}

	if (sink < LGA_AOC_SINK_NONE) {
		struct lga_aoc_cmd_out_sink2 sink_cmd = {
			.sink = sink,
			.mode = LGA_AOC_SINK_MODE_ACTIVE,
			.timestamping = 0,
		};
		struct lga_aoc_cmd_out_bind2 bind_cmd = {
			.src = st->ep_idx,
			.dst = sink,
			.bind = 1,
		};

		ret = lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_SINK2,
					&sink_cmd, sizeof(sink_cmd), NULL, 0);
		if (ret < 0)
			return ret;

		ret = lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_BIND2,
					&bind_cmd, sizeof(bind_cmd), NULL, 0);
		if (ret < 0) {
			sink_cmd.mode = LGA_AOC_SINK_MODE_IDLE;
			lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_SINK2,
					  &sink_cmd, sizeof(sink_cmd), NULL, 0);
			return ret;
		}

		lga_aoc_set_volume_hw(audio, st->ep_idx, sink, vol);
	}

	mutex_lock(&audio->mixer_lock);
	st->bound_sink = sink;
	st->configured = true;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_capture_setup(struct lga_aoc_stream *st,
				 struct snd_pcm_runtime *runtime)
{
	struct lga_aoc_audio *audio = st->audio;
	struct lga_aoc_cmd_in_mic_params params_cmd = { };
	u8 sr_code, bits_code, cap_src, mic_mask;
	unsigned int period_ms;
	u8 i, slot = 0;
	int ret;

	ret = lga_aoc_encode_rate(runtime->rate, &sr_code);
	if (ret)
		return ret;
	ret = lga_aoc_encode_bits(runtime->format, &bits_code);
	if (ret)
		return ret;

	mutex_lock(&audio->mixer_lock);
	cap_src = audio->cap_src;
	mic_mask = audio->mic_mask;
	mutex_unlock(&audio->mixer_lock);

	if (cap_src == 0 && mic_mask) {
		ret = lga_aoc_set_mic_power(audio, mic_mask, true);
		if (ret < 0)
			return ret;
		st->powered_mics = mic_mask;
	}

	period_ms = (runtime->period_size * 1000U) / runtime->rate;
	if (!period_ms)
		period_ms = 10;
	if (period_ms > 255)
		period_ms = 255;

	params_cmd.meta.chan = runtime->channels;
	params_cmd.meta.bits = bits_code;
	params_cmd.meta.sr = sr_code;
	params_cmd.meta.format = LGA_AOC_FMT_FIXED_POINT;
	params_cmd.meta.offset = 0;
	params_cmd.pdm_mask = cap_src == 0 ? mic_mask : 0;
	params_cmd.period_ms = period_ms;
	params_cmd.num_periods = clamp_t(unsigned int, runtime->periods, 2, 16);
	memset(params_cmd.interleaving, LGA_AOC_PDM_SLOT_UNUSED,
	       sizeof(params_cmd.interleaving));
	for (i = 0; i < LGA_AOC_PDM_MAX_MICS && slot < LGA_AOC_PDM_SLOTS; i++) {
		if (params_cmd.pdm_mask & BIT(i))
			params_cmd.interleaving[slot++] = i;
	}
	params_cmd.sample_rate = sr_code;
	params_cmd.mic_process_index = LGA_AOC_MIC_PROC_RAW;

	ret = lga_aoc_audio_cmd(audio, true, LGA_AOC_CMD_IN_MIC_SET_PARAMS,
				&params_cmd, sizeof(params_cmd), NULL, 0);
	if (ret < 0) {
		if (st->powered_mics) {
			lga_aoc_set_mic_power(audio, st->powered_mics, false);
			st->powered_mics = 0;
		}
		return ret;
	}

	if (ep_setup2) {
		struct lga_aoc_cmd_in_mic_params2 p2_cmd = {
			.channel = 0,
			.buffer_size = cpu_to_le32(snd_pcm_lib_buffer_bytes(st->substream)),
			.period_size = cpu_to_le32(snd_pcm_lib_period_bytes(st->substream)),
		};
		int rc2;

		rc2 = lga_aoc_audio_cmd(audio, true,
					LGA_AOC_CMD_IN_MIC_SET_PARAMS2,
					&p2_cmd, sizeof(p2_cmd), NULL, 0);
		if (rc2 < 0)
			dev_info(audio->dev,
				 "optional IN_MIC_SET_PARAMS2 returned %d; continuing\n",
				 rc2);
	}

	st->configured = true;
	return 0;
}

static int lga_aoc_playback_pump(void *data)
{
	struct lga_aoc_stream *st = data;
	struct lga_aoc_audio *audio = st->audio;
	unsigned long poll = msecs_to_jiffies(LGA_AOC_AUDIO_PUMP_POLL_MS);
	size_t ring_cap;

	ring_cap = min_t(size_t, LGA_AOC_AUDIO_PUMP_CHUNK_MAX,
			 max_t(size_t, st->svc.down_size / 4, 256));

	while (!kthread_should_stop()) {
		wait_event_interruptible(st->pump_wq,
					 kthread_should_stop() ||
					 atomic_read(&st->running));

		while (!kthread_should_stop() && atomic_read(&st->running)) {
			struct snd_pcm_substream *sub = st->substream;
			struct snd_pcm_runtime *rt;
			size_t buf_bytes, period_bytes, pos, chunk;
			const void *src;
			ssize_t written;
			bool muted;

			atomic_set(&st->in_pump, 1);
			/* Pairs with smp_mb() in hw_free/prepare */
			smp_mb();
			if (!atomic_read(&st->running) || !sub || !sub->runtime ||
			    !sub->runtime->dma_area) {
				atomic_set_release(&st->in_pump, 0);
				wake_up(&st->idle_wq);
				break;
			}

			rt = sub->runtime;
			buf_bytes = snd_pcm_lib_buffer_bytes(sub);
			period_bytes = snd_pcm_lib_period_bytes(sub);
			pos = atomic_read(&st->xfer_pos);
			if (!buf_bytes || !period_bytes || pos >= buf_bytes) {
				atomic_set_release(&st->in_pump, 0);
				wake_up(&st->idle_wq);
				break;
			}

			chunk = min_t(size_t, ring_cap, buf_bytes - pos);
			chunk = min_t(size_t, chunk,
				      period_bytes - (st->period_accum % period_bytes));

			muted = !READ_ONCE(audio->pb_unmuted);
			if (muted) {
				memset(st->scratch, 0, chunk);
				src = st->scratch;
			} else {
				src = rt->dma_area + pos;
			}

			written = lga_aoc_ipc_write_timeout(audio->aoc_dev,
							    st->svc.index,
							    src, chunk, poll);
			if (written > 0) {
				pos = (pos + written) % buf_bytes;
				atomic_set(&st->xfer_pos, pos);
				st->period_accum += written;
				if (st->period_accum >= period_bytes) {
					st->period_accum -= period_bytes;
					if (atomic_read(&st->running))
						snd_pcm_period_elapsed(sub);
				}
			} else if (written == -ENODEV) {
				atomic_set(&st->running, 0);
			}

			atomic_set_release(&st->in_pump, 0);
			wake_up(&st->idle_wq);
		}
	}

	return 0;
}

static int lga_aoc_capture_pump(void *data)
{
	struct lga_aoc_stream *st = data;
	struct lga_aoc_audio *audio = st->audio;
	unsigned long poll = msecs_to_jiffies(LGA_AOC_AUDIO_PUMP_POLL_MS);
	size_t ring_cap;

	ring_cap = min_t(size_t, LGA_AOC_AUDIO_PUMP_CHUNK_MAX,
			 max_t(size_t, st->svc.up_size / 4, 256));

	while (!kthread_should_stop()) {
		wait_event_interruptible(st->pump_wq,
					 kthread_should_stop() ||
					 atomic_read(&st->running));

		while (!kthread_should_stop() && atomic_read(&st->running)) {
			struct snd_pcm_substream *sub = st->substream;
			struct snd_pcm_runtime *rt;
			size_t buf_bytes, period_bytes, pos, chunk;
			ssize_t nread;

			atomic_set(&st->in_pump, 1);
			/* Pairs with smp_mb() in hw_free/prepare */
			smp_mb();
			if (!atomic_read(&st->running) || !sub || !sub->runtime ||
			    !sub->runtime->dma_area) {
				atomic_set_release(&st->in_pump, 0);
				wake_up(&st->idle_wq);
				break;
			}

			rt = sub->runtime;
			buf_bytes = snd_pcm_lib_buffer_bytes(sub);
			period_bytes = snd_pcm_lib_period_bytes(sub);
			pos = atomic_read(&st->xfer_pos);
			if (!buf_bytes || !period_bytes || pos >= buf_bytes) {
				atomic_set_release(&st->in_pump, 0);
				wake_up(&st->idle_wq);
				break;
			}

			chunk = min_t(size_t, ring_cap, buf_bytes - pos);
			chunk = min_t(size_t, chunk,
				      period_bytes - (st->period_accum % period_bytes));

			nread = lga_aoc_ipc_read_timeout(audio->aoc_dev,
							 st->svc.index,
							 st->scratch, chunk,
							 poll);
			if (nread > 0) {
				memcpy(rt->dma_area + pos, st->scratch, nread);
				pos = (pos + nread) % buf_bytes;
				atomic_set(&st->xfer_pos, pos);
				st->period_accum += nread;
				if (st->period_accum >= period_bytes) {
					st->period_accum -= period_bytes;
					if (atomic_read(&st->running))
						snd_pcm_period_elapsed(sub);
				}
			} else if (nread == -ENODEV) {
				atomic_set(&st->running, 0);
			}

			atomic_set_release(&st->in_pump, 0);
			wake_up(&st->idle_wq);
		}
	}

	return 0;
}

static struct lga_aoc_stream *
lga_aoc_substream_to_stream(struct lga_aoc_audio *audio,
			    struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return &audio->pb;
	return &audio->cap;
}

static int lga_aoc_pcm_new(struct snd_soc_component *component,
			   struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       LGA_AOC_AUDIO_PCM_BUF_MAX,
				       LGA_AOC_AUDIO_PCM_BUF_MAX);
	return 0;
}

static int lga_aoc_pcm_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);
	struct snd_pcm_hardware hw = lga_aoc_pcm_hw;
	int (*pump_fn)(void *data);
	const char *tname;

	if (!st->available || !lga_aoc_is_online(audio->aoc_dev))
		return -ENODEV;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		hw.channels_max = 2;
		pump_fn = lga_aoc_playback_pump;
		tname = "lga-aoc-pb";
	} else {
		hw.channels_max = 4;
		pump_fn = lga_aoc_capture_pump;
		tname = "lga-aoc-cap";
	}

	snd_soc_set_runtime_hwparams(substream, &hw);

	atomic_set(&st->running, 0);
	atomic_set(&st->in_pump, 0);
	atomic_set(&st->xfer_pos, 0);
	st->period_accum = 0;
	st->substream = substream;

	st->pump_task = kthread_run(pump_fn, st, "%s", tname);
	if (IS_ERR(st->pump_task)) {
		int err = PTR_ERR(st->pump_task);

		st->pump_task = NULL;
		st->substream = NULL;
		return err;
	}

	return 0;
}

static int lga_aoc_pcm_close(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);

	atomic_set(&st->running, 0);
	wake_up(&st->pump_wq);

	if (st->pump_task) {
		kthread_stop(st->pump_task);
		st->pump_task = NULL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		lga_aoc_playback_teardown(st);
	else
		lga_aoc_capture_teardown(st);

	st->substream = NULL;
	return 0;
}

static int lga_aoc_pcm_hw_free(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);

	atomic_set(&st->running, 0);
	/* Pairs with smp_mb() in pump threads */
	smp_mb();
	wake_up(&st->pump_wq);
	wait_event(st->idle_wq, !atomic_read_acquire(&st->in_pump));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		lga_aoc_playback_teardown(st);
	else
		lga_aoc_capture_teardown(st);

	return 0;
}

static int lga_aoc_pcm_prepare(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);

	atomic_set(&st->running, 0);
	/* Pairs with smp_mb() in pump threads */
	smp_mb();
	wake_up(&st->pump_wq);
	wait_event(st->idle_wq, !atomic_read_acquire(&st->in_pump));

	atomic_set(&st->xfer_pos, 0);
	st->period_accum = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		lga_aoc_playback_teardown(st);
		return lga_aoc_playback_setup(st, substream->runtime);
	}

	lga_aoc_capture_teardown(st);
	return lga_aoc_capture_setup(st, substream->runtime);
}

static int lga_aoc_pcm_trigger(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream, int cmd)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			struct lga_aoc_cmd_out_source src_cmd = {
				.source = st->ep_idx,
				.on = 1,
			};

			ret = lga_aoc_audio_cmd(audio, false,
						LGA_AOC_CMD_OUT_SOURCE,
						&src_cmd, sizeof(src_cmd),
						NULL, 0);
		} else {
			struct lga_aoc_cmd_in_ap_start start_cmd = { };
			u8 cap_src;

			mutex_lock(&audio->mixer_lock);
			cap_src = audio->cap_src;
			mutex_unlock(&audio->mixer_lock);

			start_cmd.mic_input_source = lga_aoc_cap_src_to_fw(cap_src);
			ret = lga_aoc_audio_cmd(audio, true,
						LGA_AOC_CMD_IN_AP_START,
						&start_cmd, sizeof(start_cmd),
						NULL, 0);
			if (!ret && cap_src == 0) {
				struct lga_aoc_audio_hdr data_cmd = { };

				ret = lga_aoc_audio_cmd(audio, true,
							LGA_AOC_CMD_IN_MIC_START_DATA,
							&data_cmd,
							sizeof(data_cmd),
							NULL, 0);
			}
		}
		if (ret < 0)
			return ret;

		st->active_in_aoc = true;
		atomic_set(&st->running, 1);
		wake_up(&st->pump_wq);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * Never block on pump kthread exit inside trigger(): under
		 * dai_link.nonatomic = true, trigger() holds the PCM stream
		 * mutex which snd_pcm_period_elapsed() also acquires.
		 */
		atomic_set(&st->running, 0);
		if (st->active_in_aoc) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				struct lga_aoc_cmd_out_source src_cmd = {
					.source = st->ep_idx,
					.on = 0,
				};

				lga_aoc_audio_cmd(audio, false,
						  LGA_AOC_CMD_OUT_SOURCE,
						  &src_cmd, sizeof(src_cmd),
						  NULL, 0);
			} else {
				struct lga_aoc_audio_hdr stop_cmd = { };

				lga_aoc_audio_cmd(audio, true,
						  LGA_AOC_CMD_IN_AP_STOP,
						  &stop_cmd, sizeof(stop_cmd),
						  NULL, 0);
			}
			st->active_in_aoc = false;
		}
		return 0;

	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t lga_aoc_pcm_pointer(struct snd_soc_component *component,
					     struct snd_pcm_substream *substream)
{
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(component);
	struct lga_aoc_stream *st = lga_aoc_substream_to_stream(audio, substream);

	return bytes_to_frames(substream->runtime, atomic_read(&st->xfer_pos));
}

/* Mixer kcontrols */
static int lga_aoc_pb_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);

	mutex_lock(&audio->mixer_lock);
	ucontrol->value.enumerated.item[0] = audio->pb_route;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_pb_route_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	unsigned int val = ucontrol->value.enumerated.item[0];

	if (val >= ARRAY_SIZE(lga_aoc_pb_route_texts))
		return -EINVAL;

	mutex_lock(&audio->mixer_lock);
	if (audio->pb_route == val) {
		mutex_unlock(&audio->mixer_lock);
		return 0;
	}
	audio->pb_route = val;
	mutex_unlock(&audio->mixer_lock);
	return 1;
}

static int lga_aoc_cap_src_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);

	mutex_lock(&audio->mixer_lock);
	ucontrol->value.enumerated.item[0] = audio->cap_src;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_cap_src_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	unsigned int val = ucontrol->value.enumerated.item[0];

	if (val >= ARRAY_SIZE(lga_aoc_cap_src_texts))
		return -EINVAL;

	mutex_lock(&audio->mixer_lock);
	if (audio->cap_src == val) {
		mutex_unlock(&audio->mixer_lock);
		return 0;
	}
	audio->cap_src = val;
	mutex_unlock(&audio->mixer_lock);
	return 1;
}

static int lga_aoc_pb_switch_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);

	mutex_lock(&audio->mixer_lock);
	ucontrol->value.integer.value[0] = audio->pb_unmuted ? 1 : 0;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_pb_switch_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	bool unmuted = !!ucontrol->value.integer.value[0];
	u32 vol;
	u8 sink;

	mutex_lock(&audio->mixer_lock);
	if (audio->pb_unmuted == unmuted) {
		mutex_unlock(&audio->mixer_lock);
		return 0;
	}
	WRITE_ONCE(audio->pb_unmuted, unmuted);
	vol = unmuted ? audio->pb_volume : 0;
	sink = audio->pb.configured ? audio->pb.bound_sink : LGA_AOC_SINK_NONE;
	mutex_unlock(&audio->mixer_lock);

	if (sink < LGA_AOC_SINK_NONE)
		lga_aoc_set_volume_hw(audio, audio->pb.ep_idx, sink, vol);

	return 1;
}

static int lga_aoc_pb_vol_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);

	mutex_lock(&audio->mixer_lock);
	ucontrol->value.integer.value[0] = audio->pb_volume;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_pb_vol_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	long val = ucontrol->value.integer.value[0];
	bool apply;
	u8 sink;

	if (val < 0 || val > 100)
		return -EINVAL;

	mutex_lock(&audio->mixer_lock);
	if (audio->pb_volume == (u32)val) {
		mutex_unlock(&audio->mixer_lock);
		return 0;
	}
	audio->pb_volume = val;
	apply = audio->pb_unmuted && audio->pb.configured;
	sink = audio->pb.bound_sink;
	mutex_unlock(&audio->mixer_lock);

	if (apply && sink < LGA_AOC_SINK_NONE)
		lga_aoc_set_volume_hw(audio, audio->pb.ep_idx, sink, val);

	return 1;
}

static int lga_aoc_mic_mask_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);

	mutex_lock(&audio->mixer_lock);
	ucontrol->value.integer.value[0] = audio->mic_mask;
	mutex_unlock(&audio->mixer_lock);
	return 0;
}

static int lga_aoc_mic_mask_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	long val = ucontrol->value.integer.value[0];

	if (val < 0 || val > GENMASK(LGA_AOC_PDM_MAX_MICS - 1, 0))
		return -EINVAL;

	mutex_lock(&audio->mixer_lock);
	if (audio->mic_mask == (u8)val) {
		mutex_unlock(&audio->mixer_lock);
		return 0;
	}
	audio->mic_mask = val;
	mutex_unlock(&audio->mixer_lock);
	return 1;
}

static int lga_aoc_dsp_state_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct lga_aoc_audio *audio = snd_soc_component_get_drvdata(comp);
	struct lga_aoc_cmd_out_dsp_state cmd = { };
	int ret;

	ret = lga_aoc_audio_cmd(audio, false, LGA_AOC_CMD_OUT_GET_DSP_STATE,
				&cmd, sizeof(cmd), &cmd, sizeof(cmd));
	if (ret < 0)
		return ret;

	if (cmd.mode >= ARRAY_SIZE(lga_aoc_dsp_state_texts))
		ucontrol->value.enumerated.item[0] = 0;
	else
		ucontrol->value.enumerated.item[0] = cmd.mode;

	return 0;
}

static const struct snd_kcontrol_new lga_aoc_controls[] = {
	SOC_ENUM_EXT("Playback Route", lga_aoc_pb_route_enum,
		     lga_aoc_pb_route_get, lga_aoc_pb_route_put),
	SOC_ENUM_EXT("Capture Source", lga_aoc_cap_src_enum,
		     lga_aoc_cap_src_get, lga_aoc_cap_src_put),
	SOC_SINGLE_BOOL_EXT("Playback Switch", 0,
			    lga_aoc_pb_switch_get, lga_aoc_pb_switch_put),
	SOC_SINGLE_EXT("Playback Volume", 0, 0, 100, 0,
		       lga_aoc_pb_vol_get, lga_aoc_pb_vol_put),
	SOC_SINGLE_EXT("Mic Mask", 0, 0, GENMASK(LGA_AOC_PDM_MAX_MICS - 1, 0),
		       0, lga_aoc_mic_mask_get, lga_aoc_mic_mask_put),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "AoC DSP State",
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = snd_soc_info_enum_double,
		.get = lga_aoc_dsp_state_get,
		.private_value = (unsigned long)&lga_aoc_dsp_state_enum,
	},
};

static struct snd_soc_dai_driver lga_aoc_dai_drv = {
	.name = LGA_AOC_AUDIO_DAI_NAME,
	.playback = {
		.stream_name = "AoC Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			 SNDRV_PCM_RATE_24000 | SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "AoC Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			 SNDRV_PCM_RATE_24000 | SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
};

static const struct snd_soc_component_driver lga_aoc_component_drv = {
	.name = LGA_AOC_AUDIO_DRV_NAME,
	.controls = lga_aoc_controls,
	.num_controls = ARRAY_SIZE(lga_aoc_controls),
	.pcm_new = lga_aoc_pcm_new,
	.open = lga_aoc_pcm_open,
	.close = lga_aoc_pcm_close,
	.hw_free = lga_aoc_pcm_hw_free,
	.prepare = lga_aoc_pcm_prepare,
	.trigger = lga_aoc_pcm_trigger,
	.pointer = lga_aoc_pcm_pointer,
};

SND_SOC_DAILINK_DEF(lga_aoc_dummy_codec,
		    DAILINK_COMP_ARRAY(COMP_DUMMY()));

static int lga_aoc_audio_probe(struct platform_device *pdev)
{
	struct device *aoc_dev = pdev->dev.parent;
	struct lga_aoc_audio *audio;
	int ret;

	if (!aoc_dev || !lga_aoc_is_online(aoc_dev))
		return -ENODEV;

	audio = devm_kzalloc(&pdev->dev, sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return -ENOMEM;

	audio->dev = &pdev->dev;
	audio->aoc_dev = aoc_dev;
	mutex_init(&audio->out_cmd_lock);
	mutex_init(&audio->in_cmd_lock);
	mutex_init(&audio->mixer_lock);
	atomic_set(&audio->cmd_seq, 0);

	/* Default mixer policy: Speaker route, Internal Mic, unmuted, 80% */
	audio->pb_route = LGA_AOC_SINK_SPEAKER;
	audio->cap_src = 0;
	audio->pb_unmuted = true;
	audio->pb_volume = 80;
	audio->mic_mask = GENMASK(LGA_AOC_PDM_MAX_MICS - 1, 0);

	audio->pb.audio = audio;
	audio->pb.bound_sink = LGA_AOC_SINK_NONE;
	init_waitqueue_head(&audio->pb.pump_wq);
	init_waitqueue_head(&audio->pb.idle_wq);
	audio->pb.scratch = devm_kmalloc(&pdev->dev,
					 LGA_AOC_AUDIO_PUMP_CHUNK_MAX,
					 GFP_KERNEL);
	if (!audio->pb.scratch)
		return -ENOMEM;

	audio->cap.audio = audio;
	audio->cap.bound_sink = LGA_AOC_SINK_NONE;
	init_waitqueue_head(&audio->cap.pump_wq);
	init_waitqueue_head(&audio->cap.idle_wq);
	audio->cap.scratch = devm_kmalloc(&pdev->dev,
					  LGA_AOC_AUDIO_PUMP_CHUNK_MAX,
					  GFP_KERNEL);
	if (!audio->cap.scratch)
		return -ENOMEM;

	if (!lga_aoc_find_service(aoc_dev, LGA_AOC_AUDIO_OUT_CTRL_SVC,
				  &audio->out_ctrl) &&
	    audio->out_ctrl.type == LGA_AOC_SVC_QUEUE)
		audio->have_out_ctrl = true;

	if (!lga_aoc_find_service(aoc_dev, LGA_AOC_AUDIO_IN_CTRL_SVC,
				  &audio->in_ctrl) &&
	    audio->in_ctrl.type == LGA_AOC_SVC_QUEUE)
		audio->have_in_ctrl = true;

	if (audio->have_out_ctrl &&
	    !lga_aoc_find_service(aoc_dev, playback_service, &audio->pb.svc) &&
	    audio->pb.svc.type == LGA_AOC_SVC_RING &&
	    audio->pb.svc.down_size > 0) {
		audio->pb.ep_idx = lga_aoc_parse_ep_index(playback_service);
		audio->pb.available = true;
	}

	if (audio->have_in_ctrl &&
	    !lga_aoc_find_service(aoc_dev, capture_service, &audio->cap.svc) &&
	    audio->cap.svc.type == LGA_AOC_SVC_RING &&
	    audio->cap.svc.up_size > 0)
		audio->cap.available = true;

	if (!audio->pb.available && !audio->cap.available) {
		dev_err(&pdev->dev,
			"no usable AoC audio IPC services found (%s / %s)\n",
			playback_service, capture_service);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, audio);

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &lga_aoc_component_drv,
					      &lga_aoc_dai_drv, 1);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register ASoC component\n");

	audio->cpu_dlc.dai_name = LGA_AOC_AUDIO_DAI_NAME;
	audio->plat_dlc.name = dev_name(&pdev->dev);

	audio->dai_link.name = "AoC Primary PCM";
	audio->dai_link.stream_name = "AoC Primary PCM";
	audio->dai_link.cpus = &audio->cpu_dlc;
	audio->dai_link.num_cpus = 1;
	audio->dai_link.codecs = lga_aoc_dummy_codec;
	audio->dai_link.num_codecs = ARRAY_SIZE(lga_aoc_dummy_codec);
	audio->dai_link.platforms = &audio->plat_dlc;
	audio->dai_link.num_platforms = 1;
	audio->dai_link.nonatomic = true;
	audio->dai_link.playback_only = audio->pb.available && !audio->cap.available;
	audio->dai_link.capture_only = !audio->pb.available && audio->cap.available;

	audio->card.dev = &pdev->dev;
	audio->card.owner = THIS_MODULE;
	audio->card.name = LGA_AOC_AUDIO_DRV_NAME;
	audio->card.long_name = "Google Laguna AoC Audio";
	audio->card.driver_name = LGA_AOC_AUDIO_DRV_NAME;
	audio->card.dai_link = &audio->dai_link;
	audio->card.num_links = 1;

	ret = devm_snd_soc_register_card(&pdev->dev, &audio->card);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register ASoC card\n");

	dev_info(&pdev->dev,
		 "registered AoC ALSA card (playback=%s[%u] ep=%u, capture=%s[%u])\n",
		 audio->pb.available ? audio->pb.svc.name : "none",
		 audio->pb.available ? audio->pb.svc.down_size : 0,
		 audio->pb.ep_idx,
		 audio->cap.available ? audio->cap.svc.name : "none",
		 audio->cap.available ? audio->cap.svc.up_size : 0);

	return 0;
}

static struct platform_driver lga_aoc_audio_pdrv = {
	.probe = lga_aoc_audio_probe,
	.driver = {
		.name = LGA_AOC_AUDIO_DRV_NAME,
	},
};

static void lga_aoc_audio_attach_fn(struct work_struct *work)
{
	struct platform_device_info pinfo = { };
	struct platform_device *pdev;
	struct device *aoc_dev = NULL;

	mutex_lock(&lga_aoc_audio_bind_lock);
	if (lga_aoc_audio_pdev)
		goto out_unlock;

	aoc_dev = lga_aoc_get_device();
	if (!aoc_dev || !lga_aoc_is_online(aoc_dev))
		goto out_put;

	pinfo.parent = aoc_dev;
	pinfo.name = LGA_AOC_AUDIO_DRV_NAME;
	pinfo.id = PLATFORM_DEVID_NONE;

	pdev = platform_device_register_full(&pinfo);
	if (IS_ERR(pdev)) {
		dev_err(aoc_dev, "failed to create %s device: %ld\n",
			LGA_AOC_AUDIO_DRV_NAME, PTR_ERR(pdev));
		goto out_put;
	}
	lga_aoc_audio_pdev = pdev;

out_put:
	put_device(aoc_dev);
out_unlock:
	mutex_unlock(&lga_aoc_audio_bind_lock);
}

static void lga_aoc_audio_detach_fn(struct work_struct *work)
{
	struct platform_device *pdev;

	mutex_lock(&lga_aoc_audio_bind_lock);
	pdev = lga_aoc_audio_pdev;
	lga_aoc_audio_pdev = NULL;
	if (pdev)
		platform_device_unregister(pdev);
	mutex_unlock(&lga_aoc_audio_bind_lock);
}

static int lga_aoc_audio_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	if (event == LGA_AOC_EVENT_ONLINE)
		schedule_work(&lga_aoc_audio_attach_work);
	else if (event == LGA_AOC_EVENT_OFFLINE)
		schedule_work(&lga_aoc_audio_detach_work);

	return NOTIFY_DONE;
}

static struct notifier_block lga_aoc_audio_nb = {
	.notifier_call = lga_aoc_audio_notify,
};

static int __init lga_aoc_audio_init(void)
{
	struct device *aoc_dev;
	int ret;

	INIT_WORK(&lga_aoc_audio_attach_work, lga_aoc_audio_attach_fn);
	INIT_WORK(&lga_aoc_audio_detach_work, lga_aoc_audio_detach_fn);

	ret = platform_driver_register(&lga_aoc_audio_pdrv);
	if (ret)
		return ret;

	ret = lga_aoc_register_notifier(&lga_aoc_audio_nb);
	if (ret) {
		platform_driver_unregister(&lga_aoc_audio_pdrv);
		return ret;
	}

	aoc_dev = lga_aoc_get_device();
	if (aoc_dev) {
		if (lga_aoc_is_online(aoc_dev))
			schedule_work(&lga_aoc_audio_attach_work);
		put_device(aoc_dev);
	}

	return 0;
}

static void __exit lga_aoc_audio_exit(void)
{
	lga_aoc_unregister_notifier(&lga_aoc_audio_nb);
	cancel_work_sync(&lga_aoc_audio_attach_work);
	cancel_work_sync(&lga_aoc_audio_detach_work);
	lga_aoc_audio_detach_fn(NULL);
	platform_driver_unregister(&lga_aoc_audio_pdrv);
}

module_init(lga_aoc_audio_init);
module_exit(lga_aoc_audio_exit);

MODULE_DESCRIPTION("Google Tensor G5 (Laguna) AoC ALSA ASoC audio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" LGA_AOC_AUDIO_DRV_NAME);
