/*
 * ALSA SoC VT1613 AC97 codec driver
 *
 * Copyright 2010-2012 Seco s.r.l.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include <sound/tlv.h>

#include <sound/ac97_codec.h>
#include "vt1613.h"
#include <linux/delay.h>

#define WM9712_DAI_AC97_HIFI    0
#define WM9712_DAI_AC97_AUX     1

#define VT1613_CACHE_REG_NUM	30

static const u16 vt1613_ac97_reg[VT1613_CACHE_REG_NUM] = {
	0x0000, /* 0x00  this register not used		*/
	0x0C0C, /* 0x02  Master Volume */
	0x0C0C, /* 0x04  Headphone Volume (optional) */
	0x0C0C, /* 0x06  Master Volume Mono (optional) */
	0x0808, /* 0x08  Master Tone (Bass & Treble) (optional) */
	0x0000, /* 0x0A  PC Beep Volume (optinal) */
	0x0000, /* 0x0C  Phone Volume (optional) */
	0x000C, /* 0x0E  MIC Volume */
	0x0C0C, /* 0x10  Line In Volume */
	0x0404, /* 0x12  CD Volume */
	0x0404, /* 0x14  Video Volume (optional) */
	0x0404, /* 0x16  AUX Volume (optional) */
	0x0C0C, /* 0x18  PCM Volume */
	0x0000, /* 0x1A   Record Select */
	0x0404, /* 0x1C   Record Gain */
	0x0000, /* 0x1E   Record Gain MIC (optional) */
	0x0000, /* 0x20   General Purpose (optional) */
	0x0000, /* 0x22   3D Control (optional) */
	0x0000, /* 0x24   Audio Interrupt & Paging (AC'97 2.3) */
	0x0000, /* 0x26   Powerdown control / status */
/* range 0x28-0x3a - AUDIO AC'97 2.0 extensions */
	0x0000, /* 0x28   Extended Audio ID */
	0x0000, /* 0x2A   Extended Audio Status and Control */
	0xBB80, /* 0x2C   PCM Front DAC Rate */
	0x0000, /* 0x2E   PCM Surround DAC Rate */
	0x0000, /* 0x30   PCM LFE DAC Rate */
	0xBB80, /* 0x32   PCM LR ADC Rate */
	0x0000, /* 0x34   PCM MIC ADC Rate */
	0x0000, /* 0x36   Center + LFE Master Volume */
	0x0C0C, /* 0x38   Surround (Rear) Master Volume */
	0x0000, /* 0x3A   S/PDIF control */
};

/* codec private data */
struct vt1613_priv {
	struct snd_soc_codec codec;
	int master;
	int fmt;
	int rev;
	int lrclk;
	int capture_channels;
	int playback_active;
	int capture_active;
	int vddio;		/* voltage of VDDIO (mv) */
	int vdda;		/* voltage of vdda (mv) */
	int vddd;		/* voltage of vddd (mv), 0 if not connected */

	unsigned int codec_powered;

	/* reference counts of AIF/APLL users */
	unsigned int apll_enabled;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;

	unsigned int configured;
	unsigned int rate;
	unsigned int sample_bits;
	unsigned int channels;

	unsigned int sysclk;

	/* Output (with associated amp) states */
	u8 earpiece_enabled;
	u8 cd_in_enabled, video_in_enabled, aux_in_enabled;
	u8 hsl_enabled, hsr_enabled;
	u8 predrivel_enabled, predriver_enabled;
	u8 carkitl_enabled, carkitr_enabled;
	u8 aux_out_enabled, mono_out_enabled, pcm_out_enabled;
};


static unsigned int ac97_read(struct snd_soc_codec *codec, unsigned int reg)
{
        u16 *cache = codec->reg_cache;

        if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
                reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
                reg == AC97_REC_GAIN) {
                return soc_ac97_ops.read(codec->ac97, reg);
        } else {
                reg = reg >> 1;

                if (reg >= (ARRAY_SIZE(vt1613_ac97_reg)))
                        return -EIO;

                return cache[reg];
        }
}

static int ac97_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
        u16 *cache = codec->reg_cache;

        if (reg < 0x7c)
                soc_ac97_ops.write(codec->ac97, reg, val);
        reg = reg >> 1;
        if (reg < (ARRAY_SIZE(vt1613_ac97_reg)))
                cache[reg] = val;

        return 0;
}

static void vt1613_codec_enable(struct snd_soc_codec *codec, int enable)
{
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        vt1613->codec_powered = enable;
        return;
}

static void vt1613_init_chip(struct snd_soc_codec *codec)
{
//        u16 *cache = codec->reg_cache;
//        u16 vendor_id, prod_id;
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        /* clear CODECPDZ prior to setting register defaults */
        vt1613_codec_enable(codec, 0);

        /* enable the oputput prior to setting register defaults */

        vt1613->pcm_out_enabled = 1;
        vt1613->mono_out_enabled = 0;
        vt1613->aux_out_enabled = 0;
        codec->reg_cache = kmemdup(vt1613_ac97_reg, sizeof(vt1613_ac97_reg), GFP_KERNEL);

        msleep(10);
}



/*
 * Output PGA builder:
 * Handle the muting and unmuting of the given output (turning off the
 * amplifier associated with the output pin)
 * On mute bypass the reg_cache and write 0 to the register
 * On unmute: restore the register content from the reg_cache
 * Outputs handled in this way:  Earpiece, PreDrivL/R, CarkitL/R
 */
#define VT1613_OUTPUT_PGA(pin_name, reg, mask)                          \
static int pin_name##pga_event(struct snd_soc_dapm_widget *w,           \
                struct snd_kcontrol *kcontrol, int event)               \
{                                                                       \
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(w->codec); \
                                                                        \
        switch (event) {                                                \
        case SND_SOC_DAPM_POST_PMU:                                     \
                vt1613->pin_name##_enabled = 1;                 \
                snd_soc_write(w->codec, reg,                            \
                        vt1613_cpld_read_u16(w->codec, reg));           \
                break;                                                  \
        case SND_SOC_DAPM_POST_PMD:                                     \
                vt1613->pin_name##_enabled = 0;                 \
                snd_soc_write(w->codec,                 \
                                        reg, 0);                        \
                break;                                                  \
        }                                                               \
        return 0;                                                       \
}

/*
 * Stereo Volume gain
 * -46.5 dB to 0 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(stereo_tvl, -4650, 150, 0);

/*
 * PCM Volume gain
 * -34.5 dB to 12 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(pcm_tvl, -3450, 150, 0);

/*
 * MIC Volume gain
 * -34.5 dB to 12 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_tvl, -3450, 150, 0);

/*
 * LINE Volume gain
 * -34.5 dB to 12 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(line_tvl, -3450, 150, 0);

/*
 * Mono Volume gain
 * -46.5 dB to 0 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(mono_tlv, -4650, 150, 0);

static const struct snd_kcontrol_new vt1613_snd_controls[] = {

        SOC_DOUBLE_TLV("Stereo Master Volume",
                AC97_MASTER, 0, 8, 0x1f, 1, stereo_tvl),

        SOC_DOUBLE_TLV("PCM Playback Volume",
                AC97_PCM, 0, 8, 0x1f, 1, pcm_tvl),

        SOC_SINGLE_TLV("MIC Recording Volume",
                AC97_MIC, 0, 0x1f, 1, mic_tvl),

        SOC_DOUBLE_TLV("LINE Recording Volume",
                AC97_LINE, 0, 8, 0x1f, 1, line_tvl),

        SOC_SINGLE_TLV("Mono Master Volume",
                AC97_MASTER_MONO, 0, 0x1f, 1, mono_tlv),

        SOC_DOUBLE_TLV("AUX Recording Volume",
                AC97_AUX, 0, 8, 0x1f, 1, line_tvl),
};

static const struct snd_soc_dapm_route intercon[] = {
        {"Digital L1 Playback Mixer", NULL, "DAC Left1"},
        {"Digital R1 Playback Mixer", NULL, "DAC Right1"},
        {"Digital L2 Playback Mixer", NULL, "DAC Left2"},
        {"Digital R2 Playback Mixer", NULL, "DAC Right2"},
        {"Digital Voice Playback Mixer", NULL, "DAC Voice"},

        /* Supply for the digital part (APLL) */
        {"Digital Voice Playback Mixer", NULL, "APLL Enable"},

        {"Digital R1 Playback Mixer", NULL, "AIF Enable"},
        {"Digital L1 Playback Mixer", NULL, "AIF Enable"},
        {"Digital R2 Playback Mixer", NULL, "AIF Enable"},
        {"Digital L2 Playback Mixer", NULL, "AIF Enable"},

        {"Analog L1 Playback Mixer", NULL, "Digital L1 Playback Mixer"},
        {"Analog R1 Playback Mixer", NULL, "Digital R1 Playback Mixer"},
        {"Analog L2 Playback Mixer", NULL, "Digital L2 Playback Mixer"},
        {"Analog R2 Playback Mixer", NULL, "Digital R2 Playback Mixer"},
        {"Analog Voice Playback Mixer", NULL, "Digital Voice Playback Mixer"},

        /* Internal playback routings */
        /* Earpiece */
        {"Earpiece Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"Earpiece Mixer", "AudioL1", "Analog L1 Playback Mixer"},
        {"Earpiece Mixer", "AudioL2", "Analog L2 Playback Mixer"},
        {"Earpiece Mixer", "AudioR1", "Analog R1 Playback Mixer"},
        {"Earpiece PGA", NULL, "Earpiece Mixer"},
        /* PreDrivL */
        {"PredriveL Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"PredriveL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
        {"PredriveL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
        {"PredriveL Mixer", "AudioR2", "Analog R2 Playback Mixer"},
        {"PredriveL PGA", NULL, "PredriveL Mixer"},
        /* PreDrivR */
        {"PredriveR Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"PredriveR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
        {"PredriveR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
        {"PredriveR Mixer", "AudioL2", "Analog L2 Playback Mixer"},
        {"PredriveR PGA", NULL, "PredriveR Mixer"},
        /* HeadsetL */
        {"HeadsetL Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"HeadsetL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
        {"HeadsetL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
        {"HeadsetL PGA", NULL, "HeadsetL Mixer"},
        /* HeadsetR */
        {"HeadsetR Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"HeadsetR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
        {"HeadsetR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
        {"HeadsetR PGA", NULL, "HeadsetR Mixer"},
        /* CarkitL */
        {"CarkitL Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"CarkitL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
        {"CarkitL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
        {"CarkitL PGA", NULL, "CarkitL Mixer"},
        /* CarkitR */
        {"CarkitR Mixer", "Voice", "Analog Voice Playback Mixer"},
        {"CarkitR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
        {"CarkitR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
        {"CarkitR PGA", NULL, "CarkitR Mixer"},
        /* HandsfreeL */
        {"HandsfreeL Mux", "Voice", "Analog Voice Playback Mixer"},
        {"HandsfreeL Mux", "AudioL1", "Analog L1 Playback Mixer"},
        {"HandsfreeL Mux", "AudioL2", "Analog L2 Playback Mixer"},
        {"HandsfreeL Mux", "AudioR2", "Analog R2 Playback Mixer"},
        {"HandsfreeL", "Switch", "HandsfreeL Mux"},
        {"HandsfreeL PGA", NULL, "HandsfreeL"},
        /* HandsfreeR */
        {"HandsfreeR Mux", "Voice", "Analog Voice Playback Mixer"},
        {"HandsfreeR Mux", "AudioR1", "Analog R1 Playback Mixer"},
        {"HandsfreeR Mux", "AudioR2", "Analog R2 Playback Mixer"},
        {"HandsfreeR Mux", "AudioL2", "Analog L2 Playback Mixer"},
        {"HandsfreeR", "Switch", "HandsfreeR Mux"},
        {"HandsfreeR PGA", NULL, "HandsfreeR"},
        /* Vibra */
        {"Vibra Mux", "AudioL1", "DAC Left1"},
        {"Vibra Mux", "AudioR1", "DAC Right1"},
        {"Vibra Mux", "AudioL2", "DAC Left2"},
        {"Vibra Mux", "AudioR2", "DAC Right2"},

        /* outputs */
        /* Must be always connected (for AIF and APLL) */
        {"Virtual HiFi OUT", NULL, "Digital L1 Playback Mixer"},
        {"Virtual HiFi OUT", NULL, "Digital R1 Playback Mixer"},
        {"Virtual HiFi OUT", NULL, "Digital L2 Playback Mixer"},
        {"Virtual HiFi OUT", NULL, "Digital R2 Playback Mixer"},
        /* Must be always connected (for APLL) */
        {"Virtual Voice OUT", NULL, "Digital Voice Playback Mixer"},
        /* Physical outputs */
        {"EARPIECE", NULL, "Earpiece PGA"},
        {"PREDRIVEL", NULL, "PredriveL PGA"},
        {"PREDRIVER", NULL, "PredriveR PGA"},
        {"HSOL", NULL, "HeadsetL PGA"},
        {"HSOR", NULL, "HeadsetR PGA"},
        {"CARKITL", NULL, "CarkitL PGA"},
        {"CARKITR", NULL, "CarkitR PGA"},
        {"HFL", NULL, "HandsfreeL PGA"},
        {"HFR", NULL, "HandsfreeR PGA"},
        {"Vibra Route", "Audio", "Vibra Mux"},
        {"VIBRA", NULL, "Vibra Route"},

        /* Capture path */
        /* Must be always connected (for AIF and APLL) */
        {"ADC Virtual Left1", NULL, "Virtual HiFi IN"},
        {"ADC Virtual Right1", NULL, "Virtual HiFi IN"},
        {"ADC Virtual Left2", NULL, "Virtual HiFi IN"},
        {"ADC Virtual Right2", NULL, "Virtual HiFi IN"},
        /* Physical inputs */
        {"Analog Left", "Main Mic Capture Switch", "MAINMIC"},
        {"Analog Left", "Headset Mic Capture Switch", "HSMIC"},
        {"Analog Left", "AUXL Capture Switch", "AUXL"},
        {"Analog Left", "Carkit Mic Capture Switch", "CARKITMIC"},

        {"Analog Right", "Sub Mic Capture Switch", "SUBMIC"},
        {"Analog Right", "AUXR Capture Switch", "AUXR"},

        {"ADC Physical Left", NULL, "Analog Left"},
        {"ADC Physical Right", NULL, "Analog Right"},

        {"Digimic0 Enable", NULL, "DIGIMIC0"},
        {"Digimic1 Enable", NULL, "DIGIMIC1"},

        /* TX1 Left capture path */
        {"TX1 Capture Route", "Analog", "ADC Physical Left"},
        {"TX1 Capture Route", "Digimic0", "Digimic0 Enable"},

        /* TX1 Right capture path */
        {"TX1 Capture Route", "Analog", "ADC Physical Right"},
        {"TX1 Capture Route", "Digimic0", "Digimic0 Enable"},
        /* TX2 Left capture path */
        {"TX2 Capture Route", "Analog", "ADC Physical Left"},
        {"TX2 Capture Route", "Digimic1", "Digimic1 Enable"},
        /* TX2 Right capture path */
        {"TX2 Capture Route", "Analog", "ADC Physical Right"},
        {"TX2 Capture Route", "Digimic1", "Digimic1 Enable"},

        {"ADC Virtual Left1", NULL, "TX1 Capture Route"},
        {"ADC Virtual Right1", NULL, "TX1 Capture Route"},
        {"ADC Virtual Left2", NULL, "TX2 Capture Route"},
        {"ADC Virtual Right2", NULL, "TX2 Capture Route"},

        {"ADC Virtual Left1", NULL, "AIF Enable"},
        {"ADC Virtual Right1", NULL, "AIF Enable"},
        {"ADC Virtual Left2", NULL, "AIF Enable"},
        {"ADC Virtual Right2", NULL, "AIF Enable"},

        /* Analog bypass routes */
        {"Right1 Analog Loopback", "Switch", "Analog Right"},
        {"Left1 Analog Loopback", "Switch", "Analog Left"},
        {"Right2 Analog Loopback", "Switch", "Analog Right"},
        {"Left2 Analog Loopback", "Switch", "Analog Left"},
        {"Voice Analog Loopback", "Switch", "Analog Left"},

        /* Supply for the Analog loopbacks */
        {"Right1 Analog Loopback", NULL, "FM Loop Enable"},
        {"Left1 Analog Loopback", NULL, "FM Loop Enable"},
        {"Right2 Analog Loopback", NULL, "FM Loop Enable"},
        {"Left2 Analog Loopback", NULL, "FM Loop Enable"},
        {"Voice Analog Loopback", NULL, "FM Loop Enable"},

        {"Analog R1 Playback Mixer", NULL, "Right1 Analog Loopback"},
        {"Analog L1 Playback Mixer", NULL, "Left1 Analog Loopback"},
        {"Analog R2 Playback Mixer", NULL, "Right2 Analog Loopback"},
        {"Analog L2 Playback Mixer", NULL, "Left2 Analog Loopback"},
        {"Analog Voice Playback Mixer", NULL, "Voice Analog Loopback"},

        /* Digital bypass routes */
        {"Right Digital Loopback", "Volume", "TX1 Capture Route"},
        {"Left Digital Loopback", "Volume", "TX1 Capture Route"},
        {"Voice Digital Loopback", "Volume", "TX2 Capture Route"},

        {"Digital R2 Playback Mixer", NULL, "Right Digital Loopback"},
        {"Digital L2 Playback Mixer", NULL, "Left Digital Loopback"},
        {"Digital Voice Playback Mixer", NULL, "Voice Digital Loopback"},

};

static int __vt1613_digital_mute(struct snd_soc_codec *codec, int mute)
{
	int ret;
	u16 adcdac_ctrl = 0x9f1f;

	if (mute)
		ret = snd_soc_update_bits(codec, AC97_MASTER, adcdac_ctrl, adcdac_ctrl);
	else
		ret = snd_soc_update_bits(codec, AC97_MASTER, adcdac_ctrl, 0);

	return ret;
}

static int vt1613_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	return __vt1613_digital_mute(codec, mute);
}

static int vt1613_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        u8 old_format, format;

        /* get format */
        old_format = snd_soc_read(codec, VT1613_REG_AUDIO_IF);
        format = old_format;

        /* set master/slave audio interface */
        switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBM_CFM:
                format &= ~(VT1613_AIF_SLAVE_EN);
                format &= ~(VT1613_CLK256FS_EN);
                break;
        case SND_SOC_DAIFMT_CBS_CFS:
                format |= VT1613_AIF_SLAVE_EN;
                format |= VT1613_CLK256FS_EN;
                break;
        default:
                return -EINVAL;
        }
        /* interface format */
        format &= ~VT1613_AIF_FORMAT;
        switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S:
                format |= VT1613_AIF_FORMAT_CODEC;
                break;
        case SND_SOC_DAIFMT_DSP_A:
                format |= VT1613_AIF_FORMAT_TDM;
                break;
        default:
                return -EINVAL;
        }

        if (format != old_format) {

                /* clear CODECPDZ before changing format (codec requirement) */
                vt1613_codec_enable(codec, 0);

                /* change format */
                snd_soc_write(codec, VT1613_REG_AUDIO_IF, format);

                /* set CODECPDZ afterwards */
                vt1613_codec_enable(codec, 1);
        }

	return 0;
}

static int vt1613_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        switch (freq) {
        case 19200000:
        case 26000000:
        case 38400000:
        case 44100000:
        case 48000000:
                break;
        case 19200:
        case 26000:
        case 38400:
        case 44100:
        case 48000:
                freq = freq * 1000;
                break;
        default:
                dev_err(codec->dev, "Unsupported APLL mclk: %u\n", freq);
                return -EINVAL;
        }

        if ((freq / 1000) != vt1613->sysclk) {
                dev_err(codec->dev,
                        "Mismatch in APLL mclk: %u (configured: %u)\n",
                        freq, vt1613->sysclk * 1000);
                return -EINVAL;
        }

	return 0;
}

static int ac97_prepare(struct snd_pcm_substream *substream,
                        struct snd_soc_dai *dai)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->codec;
        int reg;
        u16 vra;

        vra = snd_soc_read(codec, AC97_EXTENDED_STATUS);
        snd_soc_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                reg = AC97_PCM_FRONT_DAC_RATE;
        else
                reg = AC97_PCM_LR_ADC_RATE;

        return snd_soc_write(codec, reg, runtime->rate);
}

static void vt1613_constraints(struct vt1613_priv *vt1613,
                                struct snd_pcm_substream *mst_substream)
{
        struct snd_pcm_substream *slv_substream;

        /* Pick the stream, which need to be constrained */
        if (mst_substream == vt1613->master_substream)
                slv_substream = vt1613->slave_substream;
        else if (mst_substream == vt1613->slave_substream)
                slv_substream = vt1613->master_substream;
        else /* This should not happen.. */
                return;

        /* Set the constraints according to the already configured stream */
        snd_pcm_hw_constraint_minmax(slv_substream->runtime,
                                SNDRV_PCM_HW_PARAM_RATE,
                                vt1613->rate,
                                vt1613->rate);

        snd_pcm_hw_constraint_minmax(slv_substream->runtime,
                                SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
                                vt1613->sample_bits,
                                vt1613->sample_bits);

        snd_pcm_hw_constraint_minmax(slv_substream->runtime,
                                SNDRV_PCM_HW_PARAM_CHANNELS,
                                vt1613->channels,
                                vt1613->channels);
}

static int vt1613_startup(struct snd_pcm_substream *substream,
                           struct snd_soc_dai *dai)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->codec;
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        if (vt1613->master_substream) {
                vt1613->slave_substream = substream;
                /* The DAI has one configuration for playback and capture, so
                 * if the DAI has been already configured then constrain this
                 * substream to match it. */

// ERROR: we do not need to do this here. Seco patch.
// If we don't commentout this operation we can't use full duplex audio on imx soc. [gp000q7]
//                if (vt1613->configured)
//                        vt1613_constraints(vt1613, vt1613->master_substream);

        } else {
                vt1613->master_substream = substream;
        }

        return 0;
}

static void vt1613_shutdown(struct snd_pcm_substream *substream,
                             struct snd_soc_dai *dai)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->codec;
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        if (vt1613->master_substream == substream)
                vt1613->master_substream = vt1613->slave_substream;

        vt1613->slave_substream = NULL;

        /* If all streams are closed, or the remaining stream has not yet
         * been configured than set the DAI as not configured. */
        if (!vt1613->master_substream)
                vt1613->configured = 0;
         else if (!vt1613->master_substream->runtime->channels)
                vt1613->configured = 0;

}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int vt1613_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
        u16 mode, old_mode;
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->codec;
        struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);

        if (vt1613->configured)
                /* Ignoring hw_params for already configured DAI */
                return 0;
        /* bit rate */
        old_mode = snd_soc_read(codec, AC97_PCM_FRONT_DAC_RATE);
        mode = old_mode;

        switch (params_rate(params)) {
        case 24000:
                mode = VT1613_APLL_RATE_24000;
                break;
        case 32000:
                mode = VT1613_APLL_RATE_32000;
                break;
        case 44100:
                mode = VT1613_APLL_RATE_44100;
                break;
        case 48000:
                mode = VT1613_APLL_RATE_48000;
                break;
        default:
                printk(KERN_ERR "vt1613 hw params: unknown rate %d\n",
                        params_rate(params));
                return -EINVAL;
        }
        if (mode != old_mode) {
                /* change rate and set CODECPDZ */
                vt1613_codec_enable(codec, 0);
                snd_soc_write(codec, AC97_PCM_FRONT_DAC_RATE, params_rate(params));
                vt1613_codec_enable(codec, 1);
        }
        vt1613->configured = 1;
        vt1613->rate = params_rate(params);

        return 0;
}

static int vt1613_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	u16 reg, ana_pwr;
	int delay = 0;
	u16 *cache = codec->reg_cache;

	if (codec->dapm.bias_level == level)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
		snd_soc_update_bits(codec, VT1613_CHIP_MIC_CTRL,
				VT1613_BIAS_R_MASK, VT1613_BIAS_R_MASK);

		snd_soc_update_bits(codec, VT1613_CHIP_ANA_POWER,
			VT1613_VAG_POWERUP, VT1613_VAG_POWERUP);
		msleep(400);

		break;

	case SND_SOC_BIAS_PREPARE:	/* partial On */
		snd_soc_update_bits(codec, VT1613_CHIP_MIC_CTRL,
				VT1613_BIAS_R_MASK, 0);

		/* must power up hp/line out before vag & dac to
		   avoid pops. */
		reg = snd_soc_read(codec, VT1613_CHIP_ANA_POWER);
		if (reg & VT1613_VAG_POWERUP)
			delay = 400;
		reg &= ~VT1613_VAG_POWERUP;
		reg |= VT1613_DAC_POWERUP;
		reg |= VT1613_HP_POWERUP;
		reg |= VT1613_LINE_OUT_POWERUP;
		snd_soc_write(codec, VT1613_CHIP_ANA_POWER, reg);
		if (delay)
			msleep(delay);

		snd_soc_update_bits(codec, VT1613_CHIP_DIG_POWER,
			VT1613_DAC_EN, VT1613_DAC_EN);

		break;

	case SND_SOC_BIAS_STANDBY:
		/* soc calls digital_mute to unmute before record but doesn't
		   call digital_mute to mute after record. */
		__vt1613_digital_mute(codec, 1);

		snd_soc_update_bits(codec, VT1613_CHIP_MIC_CTRL,
				VT1613_BIAS_R_MASK, 0);

		reg = snd_soc_read(codec, VT1613_CHIP_ANA_POWER);
		if (reg & VT1613_VAG_POWERUP) {
			reg &= ~VT1613_VAG_POWERUP;
			snd_soc_write(codec, VT1613_CHIP_ANA_POWER, reg);
			msleep(400);
		}
		reg &= ~VT1613_DAC_POWERUP;
		reg &= ~VT1613_HP_POWERUP;
		reg &= ~VT1613_LINE_OUT_POWERUP;
		snd_soc_write(codec, VT1613_CHIP_ANA_POWER, reg);

		reg = snd_soc_read(codec, VT1613_CHIP_DIG_POWER);
		reg &= ~VT1613_DAC_EN;
		snd_soc_write(codec, VT1613_CHIP_DIG_POWER, reg);

		break;

	case SND_SOC_BIAS_OFF:	/* Off, without power */
		/* must power down hp/line out after vag & dac to
		   avoid pops. */
		reg = snd_soc_read(codec, VT1613_CHIP_ANA_POWER);
		ana_pwr = reg;
		reg &= ~VT1613_VAG_POWERUP;

		snd_soc_write(codec, VT1613_CHIP_ANA_POWER, reg);
		msleep(600);

		reg &= ~VT1613_HP_POWERUP;
		reg &= ~VT1613_LINE_OUT_POWERUP;
		reg &= ~VT1613_DAC_POWERUP;
		reg &= ~VT1613_ADC_POWERUP;
		snd_soc_write(codec, VT1613_CHIP_ANA_POWER, reg);

		/* save ANA POWER register value for resume */
		cache[VT1613_CHIP_ANA_POWER >> 1] = ana_pwr;
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static int vt1613_set_tristate(struct snd_soc_dai *dai, int tristate)
{
        struct snd_soc_codec *codec = dai->codec;
        u8 reg = snd_soc_read(codec, VT1613_REG_AUDIO_IF);

        if (tristate)
                reg |= VT1613_AIF_TRI_EN;
        else
                reg &= ~VT1613_AIF_TRI_EN;

        return snd_soc_write(codec, VT1613_REG_AUDIO_IF, reg);
}

#define VT1613_RATES     (SNDRV_PCM_RATE_8000_48000)
#define VT1613_FORMATS   (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FORMAT_S24_LE)


struct snd_soc_dai_ops vt1613_dai_ops = {
	.prepare = ac97_prepare,
//	.digital_mute = vt1613_digital_mute,
        .startup        = vt1613_startup,
        .shutdown       = vt1613_shutdown,
	.hw_params 	= vt1613_hw_params,
	.set_sysclk 	= vt1613_set_dai_sysclk,
	.set_fmt 	= vt1613_set_dai_fmt,
	.set_tristate   = vt1613_set_tristate,
};

struct snd_soc_dai_driver vt1613_dai[] = {
{
	.name = "vt1613",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 4,
		.rates = VT1613_RATES,
		.formats = VT1613_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = VT1613_RATES,
		.formats = VT1613_FORMATS,
	},
	.ops = &vt1613_dai_ops,
	.symmetric_rates = 1,
},
};

static int vt1613_volatile_register(unsigned int reg)
{
	if (reg == VT1613_CHIP_ID ||
	    reg == VT1613_CHIP_ADCDAC_CTRL ||
	    reg == VT1613_CHIP_ANA_STATUS)
		return 1;
	return 0;
}

static int vt1613_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	vt1613_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int vt1613_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	return snd_soc_write(codec, reg, cache[reg >> 1]);
}

static int vt1613_resume(struct snd_soc_codec *codec)
{
	int i;

	/* Restore refs first in same order as in vt1613_init */
	vt1613_restore_reg(codec, VT1613_CHIP_LINREG_CTRL);
	vt1613_restore_reg(codec, VT1613_CHIP_ANA_POWER);
	msleep(10);
	vt1613_restore_reg(codec, VT1613_CHIP_REF_CTRL);
	vt1613_restore_reg(codec, VT1613_CHIP_LINE_OUT_CTRL);

	/* Restore everythine else */
	for (i = 0; i < ARRAY_SIZE(vt1613_ac97_reg); i++)
		vt1613_restore_reg(codec, i<<1);

	snd_soc_write(codec, VT1613_DAP_CTRL, 0);

	/* Bring the codec back up to standby first to minimise pop/clicks */
	vt1613_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (codec->dapm.suspend_bias_level == SND_SOC_BIAS_ON)
		vt1613_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	vt1613_set_bias_level(codec, codec->dapm.suspend_bias_level);

	return 0;
}

static int vt1613_driver_probe(struct snd_soc_codec *codec)
{
	struct vt1613_priv *vt1613 = snd_soc_codec_get_drvdata(codec);
	u16 reg, ana_pwr, lreg_ctrl;
	int vag;
	int ret;
	int i;

	ret = snd_soc_new_ac97_codec(codec, &soc_ac97_ops, 0);

	soc_ac97_ops.reset(codec->ac97);
        msleep(20);
	soc_ac97_ops.warm_reset(codec->ac97);
        msleep(20);

	vt1613_init_chip(codec);

	for (i = 0; i < ARRAY_SIZE(vt1613_ac97_reg); i++)
		vt1613_restore_reg(codec, i<<1);

	printk("Found Audio codec AC97 vt1613, [ID = %04x - %04x]\n", snd_soc_read(codec, AC97_VENDOR_ID1), snd_soc_read(codec, AC97_VENDOR_ID2));

        snd_soc_write(codec, AC97_LINE, 0x0A0A);
	snd_soc_write(codec, 0x5A, 0x0400); // <-- [gp000q7] Noise reducing during mic recording on vt1613
	vt1613->sysclk = VT1613_APLL_RATE_48000;
	
	snd_soc_add_controls(codec, vt1613_snd_controls, ARRAY_SIZE(vt1613_snd_controls));
        msleep(10);
/*
for (i = 0; i < ARRAY_SIZE(vt1613_ac97_reg); i++)
 	printk("REGISTRO IDX 0x%02x = 0x%04x \n", i<<1, snd_soc_read(codec, i<<1));

printk("REGISTER IDX 0x%02x = 0x%04x \n", 0x7C, snd_soc_read(codec, 0x7C));
printk("REGISTER IDX 0x%02x = 0x%04x \n", 0x7E, snd_soc_read(codec, 0x7E));
*/

	return 0;
}

static int vt1613_driver_remove(struct snd_soc_codec *codec)
{
	if (codec->control_data)
		vt1613_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

struct snd_soc_codec_driver vt1613_driver = {
	.probe = vt1613_driver_probe,
	.remove = vt1613_driver_remove,
	.suspend = vt1613_suspend,
	.resume = vt1613_resume,
	.read = ac97_read,
	.write = ac97_write,
	.set_bias_level = vt1613_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(vt1613_ac97_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = vt1613_ac97_reg,
	.volatile_register = vt1613_volatile_register,
};

static __devinit int vt1613_codec_probe(struct platform_device *client)
{
	struct vt1613_priv *vt1613;
	int ret;

	vt1613 = kzalloc(sizeof(struct vt1613_priv), GFP_KERNEL);
	if (!vt1613)
		return -ENOMEM;

	vt1613->vdda = 3300;
	vt1613->vddio = 3300;

	dev_set_drvdata(&client->dev, vt1613);

	ret = snd_soc_register_codec(&client->dev, &vt1613_driver, &vt1613_dai[0], 1);
	if (ret) {
		dev_err(&client->dev, "Failed to register codec: %d\n", ret);
		kfree(vt1613);
		return ret;
	}

	return 0;
}

static __devexit int vt1613_remove(struct platform_device *pdev)
{
	struct vt1613_priv *vt1613 = platform_get_drvdata(pdev);

        snd_soc_unregister_dais(&vt1613_dai[0], ARRAY_SIZE(vt1613_dai));
        snd_soc_unregister_codec(&vt1613->codec);
        kfree(vt1613->codec.reg_cache);
        kfree(vt1613);

	return 0;
}

static struct platform_driver vt1613_codec_driver = {
	.driver = {
		   .name = "vt1613-ac97",
		   .owner = THIS_MODULE,
	},
	.probe = vt1613_codec_probe,
	.remove = __devexit_p(vt1613_remove),
};

static int __init vt1613_modinit(void)
{
	return platform_driver_register(&vt1613_codec_driver);
}
module_init(vt1613_modinit);

static void __exit vt1613_exit(void)
{
	platform_driver_unregister(&vt1613_codec_driver);
}

module_exit(vt1613_exit);

MODULE_DESCRIPTION("ASoC VT1613 codec driver");
MODULE_AUTHOR("Seco s.r.l.");
MODULE_LICENSE("GPL");
