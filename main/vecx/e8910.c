#include "defines.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "../libayemu/ayemu.h"
ayemu_ay_t ay;

/***************************************************************************

  ay8910.c


  Emulation of the AY-3-8910 / YM2149 sound chip.

  Based on various code snippets by Ville Hallik, Michael Cuddy,
  Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria.

***************************************************************************/

#define MAX_OUTPUT 0x0fff
//#define MAX_OUTPUT 0x7f

#define STEP3 1
#define STEP2 length
#define STEP  2


typedef int           INT32;
typedef unsigned int  UINT32;
typedef signed char          INT8;
typedef unsigned char UINT8;

DRAM_ATTR struct AY8910 {
	int index;
	int ready;
	int lastEnable;
	int PeriodA,PeriodB,PeriodC,PeriodN,PeriodE;
	int CountA,CountB,CountC,CountN,CountE;
	int RNG;
	unsigned VolA,VolB,VolC,VolE;
	signed char CountEnv;
	unsigned char EnvelopeA,EnvelopeB,EnvelopeC;
	unsigned char OutputA,OutputB,OutputC,OutputN;
	unsigned char Hold,Alternate,Attack,Holding;
	unsigned VolTable[32];
} PSG;


/* register id's */
#define AY_AFINE	(0)
#define AY_ACOARSE	(1)
#define AY_BFINE	(2)
#define AY_BCOARSE	(3)
#define AY_CFINE	(4)
#define AY_CCOARSE	(5)
#define AY_NOISEPER	(6)
#define AY_ENABLE	(7)
#define AY_AVOL		(8)
#define AY_BVOL		(9)
#define AY_CVOL		(10)
#define AY_EFINE	(11)
#define AY_ECOARSE	(12)
#define AY_ESHAPE	(13)

#define AY_PORTA	(14)
#define AY_PORTB	(15)

DRAM_ATTR int reg14In = 0xff;
DRAM_ATTR int reg14Out = 0xff;
DRAM_ATTR static int SAMPLE_THRESHOLD = 25;
DRAM_ATTR static int digitByteCounter =0;       

#define MAX_DIGIT_BUFFER 2000
extern long cyclesRunning;

//
EXT_RAM_BSS_ATTR static int digitByte[MAX_DIGIT_BUFFER];

DRAM_ATTR int tickpos=0;
DRAM_ATTR int tickdiv=68;
DRAM_ATTR int curdac=0;
DRAM_ATTR int dacto;

void initAY(int freq, int chans, int bits)
{
	memset (&ay, 0, sizeof(ay));

	ayemu_init(&ay);
//	ayemu_set_chip_type(&ay, AYEMU_YM, NULL);
	ayemu_set_sound_format(&ay, freq, chans, bits);
	ayemu_set_chip_freq(&ay, 1500000);  // Vectrex PSG clock
	if (chans==2) 
		ayemu_set_stereo(&ay, AYEMU_ABC, NULL);
	else
		ayemu_set_stereo(&ay, AYEMU_MONO, NULL);
//	ayemu_set_stereo(&ay, AYEMU_MONO, NULL);
}

void ayemu_set_regs(ayemu_ay_t *ay, unsigned *regs);

// length in byte!
IRAM_ATTR void callbackAY(void *userdata, uint8_t *stream, int length)
{
	(void) userdata;

	/* hack to prevent us from hanging when starting filtered outputs */
	if (!PSG.ready)
	{
		memset(stream, 0, length * sizeof(*stream));
		return;
	}

	// small sample counts are ignored!
	if ((digitByteCounter>SAMPLE_THRESHOLD) ) // are there any samples?
	{

		// 30000 cycles = 1/50 second
		// we are called every 1/50th second 
/*
		int sampleCounter = 0;

		// AY_FREQUENCY * AY_CHANNEL * AY_BITS / 50 Hz / 4 bytes  -> 882 "frames" per call
		// that means 882 frames equal 300000cycles equals 1/50th of a second
		// len is in byte which mewans it is 3528 = 4*882 -> each frame is 2 words (2 channels a 16bit)

		// that means each sample "length is 882/cyclesPerSample"
*/
		if (digitByteCounter > SAMPLE_THRESHOLD)
		{
			int out_samples = length / (AY_CHANNEL * (AY_BITS >> 3));

			for (int i = 0; i < out_samples; i++) {
				int idx = (int)(i * (double)digitByteCounter / out_samples);
				if (idx >= digitByteCounter) idx = digitByteCounter - 1;
				int8_t raw = (int8_t)digitByte[idx];

		#if AY_BITS == 16
				int16_t sample = (int16_t)raw * 64;
				int16_t *out   = (int16_t *)stream;
		#  if AY_CHANNEL == 2
				out[i * 2]     = sample;
				out[i * 2 + 1] = sample;
		#  else
				out[i] = sample;
		#  endif

		#else  /* AY_BITS == 8 */
				uint8_t sample = (uint8_t)(raw + 128);
				uint8_t *out   = (uint8_t *)stream;
		#  if AY_CHANNEL == 2
				out[i * 2]     = sample;
				out[i * 2 + 1] = sample;
		#  else
				out[i] = sample;
		#  endif
		#endif
			}

			digitByteCounter = -1;
			return;
		}
	}
	else
	{
		// the set reg as a batch - is buggy
		// some noises do not fade if  regs are set with the batch!
		//	ayemu_set_regs(&ay, snd_regs);
		ayemu_gen_sound(&ay, stream, length / (AY_CHANNEL *(AY_BITS>>3)));
	}
	digitByteCounter = -1;

}










IRAM_ATTR einline 
int e8910_read(int reg)
{
	return snd_regs[reg];
	/* TODO 
	if (reg != 14) return snd_regs[reg];
	// readDataToPSGFromPortA();

	if  ((snd_regs[AY_ENABLE] & 0x40 ) ==  0x00)
	{
		// input mode
		return reg14In;//snd_regs[14];
	}
	// output mode
    return reg14In;//snd_regs[14];
	*/
}

IRAM_ATTR einline 
void e8910_write(int r, int v)
{
    int old;

	// 255 dummy register for "path thru data to sound direct
	if ((r != 255) && (digitByteCounter != 0))
	{
		digitByteCounter = 0; // counter 0 == NO digitizing active
	}
	if (r == 255)
	{
		// sound sample is active
		if (digitByteCounter>=MAX_DIGIT_BUFFER) return;
		if (digitByteCounter==-1) digitByteCounter =0; // -1 means digitizing active, but buffer was reset
		digitByte[digitByteCounter] = v;
		digitByteCounter++;
		return;
	}

    snd_regs[r] = v;
 	ayemu_set_reg(&ay, r, v);

	/* A note about the period of tones, noise and envelope: for speed reasons,*/
	/* we count down from the period to 0, but careful studies of the chip     */
	/* output prove that it instead counts up from 0 until the counter becomes */
	/* greater or equal to the period. This is an important difference when the*/
	/* program is rapidly changing the period to modulate the sound.           */
	/* To compensate for the difference, when the period is changed we adjust  */
	/* our internal counter.                                                   */
	/* Also, note that period = 0 is the same as period = 1. This is mentioned */
	/* in the YM2203 data sheets. However, this does NOT apply to the Envelope */
	/* period. In that case, period = 0 is half as period = 1. */
	switch( r )
	{
	case AY_AFINE:
	case AY_ACOARSE:
		snd_regs[AY_ACOARSE] &= 0x0f;
		old = PSG.PeriodA;
		PSG.PeriodA = (snd_regs[AY_AFINE] + 256 * snd_regs[AY_ACOARSE]) * STEP3;
		if (PSG.PeriodA == 0) PSG.PeriodA = STEP3;
		PSG.CountA += PSG.PeriodA - old;
		if (PSG.CountA <= 0) PSG.CountA = 1;
		break;
	case AY_BFINE:
	case AY_BCOARSE:
		snd_regs[AY_BCOARSE] &= 0x0f;
		old = PSG.PeriodB;
		PSG.PeriodB = (snd_regs[AY_BFINE] + 256 * snd_regs[AY_BCOARSE]) * STEP3;
		if (PSG.PeriodB == 0) PSG.PeriodB = STEP3;
		PSG.CountB += PSG.PeriodB - old;
		if (PSG.CountB <= 0) PSG.CountB = 1;
		break;
	case AY_CFINE:
	case AY_CCOARSE:
		snd_regs[AY_CCOARSE] &= 0x0f;
		old = PSG.PeriodC;
		PSG.PeriodC = (snd_regs[AY_CFINE] + 256 * snd_regs[AY_CCOARSE]) * STEP3;
		if (PSG.PeriodC == 0) PSG.PeriodC = STEP3;
		PSG.CountC += PSG.PeriodC - old;
		if (PSG.CountC <= 0) PSG.CountC = 1;
		break;
	case AY_NOISEPER:
		snd_regs[AY_NOISEPER] &= 0x1f;
		old = PSG.PeriodN;
		PSG.PeriodN = snd_regs[AY_NOISEPER] * STEP3;
		if (PSG.PeriodN == 0) PSG.PeriodN = STEP3;
		PSG.CountN += PSG.PeriodN - old;
		if (PSG.CountN <= 0) PSG.CountN = 1;
		break;
	case AY_ENABLE:
		PSG.lastEnable = snd_regs[AY_ENABLE];
		break;
	case AY_AVOL:
		snd_regs[AY_AVOL] &= 0x1f;
		PSG.EnvelopeA = snd_regs[AY_AVOL] & 0x10;
		PSG.VolA = PSG.EnvelopeA ? PSG.VolE : PSG.VolTable[snd_regs[AY_AVOL] ? snd_regs[AY_AVOL]*2+1 : 0];
		break;
	case AY_BVOL:
		snd_regs[AY_BVOL] &= 0x1f;
		PSG.EnvelopeB = snd_regs[AY_BVOL] & 0x10;
		PSG.VolB = PSG.EnvelopeB ? PSG.VolE : PSG.VolTable[snd_regs[AY_BVOL] ? snd_regs[AY_BVOL]*2+1 : 0];
		break;
	case AY_CVOL:
		snd_regs[AY_CVOL] &= 0x1f;
		PSG.EnvelopeC = snd_regs[AY_CVOL] & 0x10;
		PSG.VolC = PSG.EnvelopeC ? PSG.VolE : PSG.VolTable[snd_regs[AY_CVOL] ? snd_regs[AY_CVOL]*2+1 : 0];
		break;
	case AY_EFINE:
	case AY_ECOARSE:
		old = PSG.PeriodE;
		PSG.PeriodE = ((snd_regs[AY_EFINE] + 256 * snd_regs[AY_ECOARSE])) * STEP3;
		//if (PSG.PeriodE == 0) PSG.PeriodE = STEP3 / 2;
		if (PSG.PeriodE == 0) PSG.PeriodE = STEP3;
		PSG.CountE += PSG.PeriodE - old;
		if (PSG.CountE <= 0) PSG.CountE = 1;
		break;
	case AY_ESHAPE:
		/* envelope shapes:
        C AtAlH
        0 0 x x  \___

        0 1 x x  /___

        1 0 0 0  \\\\

        1 0 0 1  \___

        1 0 1 0  \/\/
                  ___
        1 0 1 1  \

        1 1 0 0  ////
                  ___
        1 1 0 1  /

        1 1 1 0  /\/\

        1 1 1 1  /___

        The envelope counter on the AY-3-8910 has 16 steps. On the YM2149 it
        has twice the steps, happening twice as fast. Since the end result is
        just a smoother curve, we always use the YM2149 behaviour.
        */
		snd_regs[AY_ESHAPE] &= 0x0f;
		PSG.Attack = (snd_regs[AY_ESHAPE] & 0x04) ? 0x1f : 0x00;
		if ((snd_regs[AY_ESHAPE] & 0x08) == 0)
		{
			/* if Continue = 0, map the shape to the equivalent one which has Continue = 1 */
			PSG.Hold = 1;
			PSG.Alternate = PSG.Attack;
		}
		else
		{
			PSG.Hold = snd_regs[AY_ESHAPE] & 0x01;
			PSG.Alternate = snd_regs[AY_ESHAPE] & 0x02;
		}
		PSG.CountE = PSG.PeriodE;
		PSG.CountEnv = 0x1f;
		PSG.Holding = 0;
		PSG.VolE = PSG.VolTable[PSG.CountEnv ^ PSG.Attack];
		if (PSG.EnvelopeA) PSG.VolA = PSG.VolE;
		if (PSG.EnvelopeB) PSG.VolB = PSG.VolE;
		if (PSG.EnvelopeC) PSG.VolC = PSG.VolE;
		break;
	case AY_PORTA:
		break;
	case AY_PORTB:
		break;
	}
}

// NOT USED!
// length is the "frame length"
// meaning one "unit" of sound samples 16bit stereo - > 1 frame (4 byte!)
IRAM_ATTR 
void e8910_callback(void *userdata, uint8_t *stream, int length)
{

	int outn;
    int lengthOrg = length;
	uint8_t* buf1 = stream;

	(void) userdata;

	/* hack to prevent us from hanging when starting filtered outputs */
	if (!PSG.ready)
	{
		memset(stream, 0, length*4 * sizeof(*stream));
		return;
	}

  length = length * 2;

	/* The 8910 has three outputs, each output is the mix of one of the three */
	/* tone generators and of the (single) noise generator. The two are mixed */
	/* BEFORE going into the DAC. The formula to mix each channel is: */
	/* (ToneOn | ToneDisable) & (NoiseOn | NoiseDisable). */
	/* Note that this means that if both tone and noise are disabled, the output */
	/* is 1, not 0, and can be modulated changing the volume. */


	/* If the channels are disabled, set their output to 1, and increase the */
	/* counter, if necessary, so they will not be inverted during this update. */
	/* Setting the output to 1 is necessary because a disabled channel is locked */
	/* into the ON state (see above); and it has no effect if the volume is 0. */
	/* If the volume is 0, increase the counter, but don't touch the output. */
	if (snd_regs[AY_ENABLE] & 0x01)
	{
		if (PSG.CountA <= STEP2) PSG.CountA += STEP2;
		PSG.OutputA = 1;
	}
	else if (snd_regs[AY_AVOL] == 0)
	{
		/* note that I do count += length, NOT count = length + 1. You might think */
		/* it's the same since the volume is 0, but doing the latter could cause */
		/* interferencies when the program is rapidly modulating the volume. */
		if (PSG.CountA <= STEP2) PSG.CountA += STEP2;
	}
	if (snd_regs[AY_ENABLE] & 0x02)
	{
		if (PSG.CountB <= STEP2) PSG.CountB += STEP2;
		PSG.OutputB = 1;
	}
	else if (snd_regs[AY_BVOL] == 0)
	{
		if (PSG.CountB <= STEP2) PSG.CountB += STEP2;
	}
	if (snd_regs[AY_ENABLE] & 0x04)
	{
		if (PSG.CountC <= STEP2) PSG.CountC += STEP2;
		PSG.OutputC = 1;
	}
	else if (snd_regs[AY_CVOL] == 0)
	{
		if (PSG.CountC <= STEP2) PSG.CountC += STEP2;
	}

	/* for the noise channel we must not touch OutputN - it's also not necessary */
	/* since we use outn. */
	if ((snd_regs[AY_ENABLE] & 0x38) == 0x38)	/* all off */
		if (PSG.CountN <= STEP2) PSG.CountN += STEP2;

	outn = (PSG.OutputN | snd_regs[AY_ENABLE]);

	/* buffering loop */
	while (length > 0)
	{
        unsigned vol;
	    int left  = 2;
		/* vola, volb and volc keep track of how long each square wave stays */
		/* in the 1 position during the sample period. */

		int vola,volb,volc;
		vola = volb = volc = 0;

		do
		{
			int nextevent;

			if (PSG.CountN < left) nextevent = PSG.CountN;
			else nextevent = left;

			if (outn & 0x08)
			{
				if (PSG.OutputA) vola += PSG.CountA;
				PSG.CountA -= nextevent;
				/* PeriodA is the half period of the square wave. Here, in each */
				/* loop I add PeriodA twice, so that at the end of the loop the */
				/* square wave is in the same status (0 or 1) it was at the start. */
				/* vola is also incremented by PeriodA, since the wave has been 1 */
				/* exactly half of the time, regardless of the initial position. */
				/* If we exit the loop in the middle, OutputA has to be inverted */
				/* and vola incremented only if the exit status of the square */
				/* wave is 1. */
				while (PSG.CountA <= 0)
				{
					PSG.CountA += PSG.PeriodA;
					if (PSG.CountA > 0)
					{
						PSG.OutputA ^= 1;
						if (PSG.OutputA) vola += PSG.PeriodA;
						break;
					}
					PSG.CountA += PSG.PeriodA;
					vola += PSG.PeriodA;
				}
				if (PSG.OutputA) vola -= PSG.CountA;
			}
			else
			{
				PSG.CountA -= nextevent;
				while (PSG.CountA <= 0)
				{
					PSG.CountA += PSG.PeriodA;
					if (PSG.CountA > 0)
					{
						PSG.OutputA ^= 1;
						break;
					}
					PSG.CountA += PSG.PeriodA;
				}
			}

			if (outn & 0x10)
			{
				if (PSG.OutputB) volb += PSG.CountB;
				PSG.CountB -= nextevent;
				while (PSG.CountB <= 0)
				{
					PSG.CountB += PSG.PeriodB;
					if (PSG.CountB > 0)
					{
						PSG.OutputB ^= 1;
						if (PSG.OutputB) volb += PSG.PeriodB;
						break;
					}
					PSG.CountB += PSG.PeriodB;
					volb += PSG.PeriodB;
				}
				if (PSG.OutputB) volb -= PSG.CountB;
			}
			else
			{
				PSG.CountB -= nextevent;
				while (PSG.CountB <= 0)
				{
					PSG.CountB += PSG.PeriodB;
					if (PSG.CountB > 0)
					{
						PSG.OutputB ^= 1;
						break;
					}
					PSG.CountB += PSG.PeriodB;
				}
			}

			if (outn & 0x20)
			{
				if (PSG.OutputC) volc += PSG.CountC;
				PSG.CountC -= nextevent;
				while (PSG.CountC <= 0)
				{
					PSG.CountC += PSG.PeriodC;
					if (PSG.CountC > 0)
					{
						PSG.OutputC ^= 1;
						if (PSG.OutputC) volc += PSG.PeriodC;
						break;
					}
					PSG.CountC += PSG.PeriodC;
					volc += PSG.PeriodC;
				}
				if (PSG.OutputC) volc -= PSG.CountC;
			}
			else
			{
				PSG.CountC -= nextevent;
				while (PSG.CountC <= 0)
				{
					PSG.CountC += PSG.PeriodC;
					if (PSG.CountC > 0)
					{
						PSG.OutputC ^= 1;
						break;
					}
					PSG.CountC += PSG.PeriodC;
				}
			}

			PSG.CountN -= nextevent;
			if (PSG.CountN <= 0)
			{
				/* Is noise output going to change? */
				if ((PSG.RNG + 1) & 2)	/* (bit0^bit1)? */
				{
					PSG.OutputN = ~PSG.OutputN;
					outn = (PSG.OutputN | snd_regs[AY_ENABLE]);
				}

				/* The Random Number Generator of the 8910 is a 17-bit shift */
				/* register. The input to the shift register is bit0 XOR bit3 */
				/* (bit0 is the output). This was verified on AY-3-8910 and YM2149 chips. */

				/* The following is a fast way to compute bit17 = bit0^bit3. */
				/* Instead of doing all the logic operations, we only check */
				/* bit0, relying on the fact that after three shifts of the */
				/* register, what now is bit3 will become bit0, and will */
				/* invert, if necessary, bit14, which previously was bit17. */
				if (PSG.RNG & 1) PSG.RNG ^= 0x24000; /* This version is called the "Galois configuration". */
				PSG.RNG >>= 1;
				PSG.CountN += PSG.PeriodN;
			}

			left -= nextevent;
		} while (left > 0);

		/* update envelope */
		if (PSG.Holding == 0)
		{
			PSG.CountE -= STEP;
			if (PSG.CountE <= 0)
			{
				do
				{
					PSG.CountEnv--;
					PSG.CountE += PSG.PeriodE;
				} while (PSG.CountE <= 0);

				/* check envelope current position */
				if (PSG.CountEnv < 0)
				{
					if (PSG.Hold)
					{
						if (PSG.Alternate)
							PSG.Attack ^= 0x1f;
						PSG.Holding = 1;
						PSG.CountEnv = 0;
					}
					else
					{
						/* if CountEnv has looped an odd number of times (usually 1), */
						/* invert the output. */
						if (PSG.Alternate && (PSG.CountEnv & 0x20))
 							PSG.Attack ^= 0x1f;

						PSG.CountEnv &= 0x1f;
					}
				}

				PSG.VolE = PSG.VolTable[PSG.CountEnv ^ PSG.Attack];
				/* reload volume */
				if (PSG.EnvelopeA) PSG.VolA = PSG.VolE;
				if (PSG.EnvelopeB) PSG.VolB = PSG.VolE;
				if (PSG.EnvelopeC) PSG.VolC = PSG.VolE;
			}
		}

		vol = (vola * PSG.VolA + volb * PSG.VolB + volc * PSG.VolC) / (3 * STEP);
		if (--length & 1) 
			*(buf1++) = vol >> 8;
	}

	// small sample counts are ignored!
	if ((digitByteCounter>SAMPLE_THRESHOLD) ) // are there any samples?
	{
		double sampleScale = ((double)digitByteCounter) / ((double) lengthOrg);
		int i=0;
		// we fill the needed sample buffer
		// with as many samples as we have
		// each sample may be "stretched" to fill the buffer
		// it is NOT considered how long (in cycles) a sample "stayed" for digital output
		// all samples are considered to have the same "length"
		double sampleCounter = 0;

		// samples come from the DAC (more or less)
		// therefor samples are signed 8bit samples, -128 - +127
		// output line is done in signed 8bit samples (PSG has signed output) [although the output is allways positive]
		// and our data here is ORer with
		// PSG out
		double volDigital = 0.1;
		while (i<lengthOrg) 
		{
			double signed8BitSampleVolumne = digitByte[(int)sampleCounter]*volDigital;
			unsigned char sampleValueVolumne8BitSigned = (unsigned char)(( ((unsigned char ) signed8BitSampleVolumne)) & 0xff); // UNSIGNED
			sampleCounter += sampleScale;
			
			// for now a sample just overwrites PSG
			stream[i] = sampleValueVolumne8BitSigned;
			i++;
		}
	}
	digitByteCounter = -1;

}
/*
static void e8910_build_mixer_table(void)
{
	int i;
	double out;

	// calculate the volume->voltage conversion table 
	// The AY-3-8910 has 16 levels, in a logarithmic scale (3dB per STEP) 
	// The YM2149 still has 16 levels for the tone generators, but 32 for 
	// the envelope generator (1.5dB per STEP). 
	out = MAX_OUTPUT;
	for (i = 31;i > 0;i--)
	{
		PSG.VolTable[i] = (unsigned)(out + 0.5);	// round to nearest 
		out /= 1.188502227;	// = 10 ^ (1.5/20) = 1.5dB 
	}
	PSG.VolTable[0] = 0;
}
*/
void e8910_init_sound(void)
{
	PSG.RNG     = 1;
	PSG.OutputA = 0;
	PSG.OutputB = 0;
	PSG.OutputC = 0;
	PSG.OutputN = 0xff;
//	e8910_build_mixer_table();
	PSG.ready   = 1;
}


void e8910_done_sound()
{
}

