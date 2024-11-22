/*
 * QuadratureEncoderCCL.cpp
 *
 *  Created on: 13 Nov 2024
 *      Author: Simon Kueppers
 *
 *
 * This class realizes a quadrature decoder using CCL (configurable logic) and a TCC instance.
 * The basic idea follows DS00002434B: AN2434 Interfacing Quadrature Encoder using CCL with TCA and TCB from Microchip
 * In this application note however is a bug, especially with the generation of the CNT signal for the counter. It is
 * derived from the RS-FF (that samples on what signal the last edge occured). The shown implementation swallows one count
 * pulse during direction reversal.
 * This implementation is an adaptation that does not have this issue, because the count signal is generated from the sum
 * (OR gate) of A-Pulse and B-Pulse.
 * A': A-Pulse that is high for one CCL clock cycle when any edge on A signal occured (using EIC or EVSYS)
 * B': B-Pulse that is high for one CCL clock cycle when any edge on B signal occured (using EIC or EVSYS)
 * A: Asynchronous state of A signal
 * B: Asynchronous state of B signal
 * DIR: Direction signal for counter
 * CNT: Count event signal for counter
 *
 *          LUT3
 *          +---+
 * A >------+ X +----------------> DIR
 * B >------+ O |
 *      +---+ R |
 *      |   +---+
 *      +----------------------------+ L
 *          LUT0         SEQ0        | I
 *          +---+        +------+    | N
 * A' >-+---+ 1 +--------+ D  Q +----+ K
 *      |   +---+        |      |
 *      |            +---+ G    |
 *      |   LUT1     |   +------+
 *      |   +---+    |
 *      +---+ O + ---+-----------> CNT
 *          | R |
 *      +---+   |
 *      |   +---+
 *      +------------+ L
 *          LUT2     | I
 *          +---+    | N
 * B' >-----+ 1 +----+ K
 *          +---+
 */

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP && SAME5x && SUPPORT_CCL_ENCODER

#include "QuadratureEncoderCCL.h"
#include <hri_mclk_e54.h>
#include <hri_eic_e54.h>
#include <cmath>

#if DIFFERENTIAL_STEPPER_OUTPUTS
# error CCL_ENCODER and DIFFERENTIAL_STEPPER_OUTPUTS cannot be enabled simultaneously
#endif

// Overridden virtual functions

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult QuadratureEncoderCCL::Init(const StringRef& reply) noexcept
{
	/* Enable APB clocks and enable GCLKs on CCL and TCC and make them run synchronously */
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL | MCLK_APBCMASK_TCC3;
    hri_gclk_write_PCHCTRL_reg(GCLK, CCL_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC3_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);

	/* Setup EIC to route A and B signals into the event system with both-edge event triggering */
	for (Pin p : PositionDecoderEicPins)
	{
		/* Set pins to EIC function */
		SetPinFunction(p, GpioPinFunction::A); /* EIC */
	}

	EIC->CTRLA.bit.ENABLE = 0;
	while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) { }

	/* Configure sense configuration of EIC channels */
	EIC->CONFIG[0].reg |=  (PositionDecoderEicChannels[0]  < 8) ? (EIC_CONFIG_SENSE0_BOTH_Val << (PositionDecoderEicChannels[0] * (EIC_CONFIG_SENSE1_Pos-EIC_CONFIG_SENSE0_Pos))) : 0
						|  (PositionDecoderEicChannels[1]  < 8) ? (EIC_CONFIG_SENSE0_BOTH_Val << (PositionDecoderEicChannels[1] * (EIC_CONFIG_SENSE1_Pos-EIC_CONFIG_SENSE0_Pos))) : 0;
	EIC->CONFIG[1].reg |=  (PositionDecoderEicChannels[0] >= 8) ? (EIC_CONFIG_SENSE0_BOTH_Val << ((PositionDecoderEicChannels[0] - 8) * (EIC_CONFIG_SENSE1_Pos-EIC_CONFIG_SENSE0_Pos))) : 0
						|  (PositionDecoderEicChannels[1] >= 8) ? (EIC_CONFIG_SENSE0_BOTH_Val << ((PositionDecoderEicChannels[1] - 8) * (EIC_CONFIG_SENSE1_Pos-EIC_CONFIG_SENSE0_Pos))) : 0;
	EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO(1<<PositionDecoderEicChannels[0]) | EIC_EVCTRL_EXTINTEO(1<<PositionDecoderEicChannels[1]);

	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) { }

	/* Setup event system channels on the external interrupt pins */
	EVSYS->USER[EVSYS_ID_USER_CCL_LUTIN_0].reg = EVSYS_USER_CHANNEL(PositionDecoderAsyncEventChannels[0]+1); /* PULSE_A to LUT0 */
	EVSYS->USER[EVSYS_ID_USER_CCL_LUTIN_1].reg = EVSYS_USER_CHANNEL(PositionDecoderAsyncEventChannels[1]+1); /* PULSE_A to LUT1 */
	EVSYS->USER[EVSYS_ID_USER_CCL_LUTIN_2].reg = EVSYS_USER_CHANNEL(PositionDecoderAsyncEventChannels[2]+1); /* PULSE_B to LUT2 */
	EVSYS->Channel[PositionDecoderAsyncEventChannels[0]].CHANNEL.reg =
	EVSYS->Channel[PositionDecoderAsyncEventChannels[1]].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_0 + PositionDecoderEicChannels[0]); /* PULSE_A */
	EVSYS->Channel[PositionDecoderAsyncEventChannels[2]].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_0 + PositionDecoderEicChannels[1]); /* PULSE_B */

	/* Set up CCL LUTs.
	 * For the CCL, A and B IOs are a second set of pins, different from EIC pins above, routed to the CCL peripheral. */
	for (Pin p : PositionDecoderCclPins)
	{
		/* Set pins to CCL function */
		SetPinFunction(p, GpioPinFunction::N); /* CCL */
	}

	/* Reset CCL */
	CCL->CTRL.reg = 0;
	CCL->SEQCTRL[0].reg = CCL_SEQCTRL_SEQSEL_DISABLE;
	CCL->SEQCTRL[1].reg = CCL_SEQCTRL_SEQSEL_DISABLE;
	CCL->LUTCTRL[0].bit.ENABLE = 0;
	CCL->LUTCTRL[1].bit.ENABLE = 0;
	CCL->LUTCTRL[2].bit.ENABLE = 0;
	CCL->LUTCTRL[3].bit.ENABLE = 0;

	/* LUT0/LUT1 as gated D-FF, LUT2 as passthru
	 * D(LUT0) = [A'(EVSYS)],
	 * G(LUT1) = [A'(EVSYS) or [B'(EVSYS)](LINK)]
	 * CNT = G
	 *
	 * LUT3 as DIR decoder using output from D-FF.
	 * DIR = [A(IN9) xor B(IN10) xor D-FF(LINK)] */
	CCL->SEQCTRL[0].reg = CCL_SEQCTRL_SEQSEL_DFF;
	CCL->LUTCTRL[0].reg = CCL_LUTCTRL_TRUTH(0b00000010) | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_INSEL1_MASK | CCL_LUTCTRL_INSEL0_EVENT
			| CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_ENABLE;
	CCL->LUTCTRL[1].reg = CCL_LUTCTRL_TRUTH(0b00001110) | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_INSEL1_LINK | CCL_LUTCTRL_INSEL0_EVENT
			| CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_ENABLE;
	CCL->LUTCTRL[2].reg = CCL_LUTCTRL_TRUTH(0b00000010) | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_INSEL1_MASK | CCL_LUTCTRL_INSEL0_EVENT
			| CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_ENABLE;
	CCL->LUTCTRL[3].reg = CCL_LUTCTRL_TRUTH(0b10010110) | CCL_LUTCTRL_INSEL2_LINK | CCL_LUTCTRL_INSEL1_IO | CCL_LUTCTRL_INSEL0_IO
						| CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_ENABLE;

	/* Enable CCL */
	CCL->CTRL.reg = CCL_CTRL_ENABLE;

	/* Configure event system to feed CNT/DIR signals into TCC3 */
    hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_0, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
    hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_1, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
	EVSYS->USER[EVSYS_ID_USER_TCC3_EV_0].reg = EVSYS_USER_CHANNEL(PositionDecoderSyncEventChannels[0]+1);
	EVSYS->USER[EVSYS_ID_USER_TCC3_EV_1].reg = EVSYS_USER_CHANNEL(PositionDecoderSyncEventChannels[1]+1);
	EVSYS->Channel[PositionDecoderSyncEventChannels[0]].CHANNEL.reg = EVSYS_CHANNEL_PATH_RESYNCHRONIZED | EVSYS_CHANNEL_EDGSEL_RISING_EDGE | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CCL_LUTOUT_1); /* CNT */
	EVSYS->Channel[PositionDecoderSyncEventChannels[1]].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CCL_LUTOUT_3); /* DIR  */

    /* Configure TCC3 as counter with two events serving as direction and count input */
	TCC3->CTRLA.bit.ENABLE = 0;
	while (TCC3->SYNCBUSY.bit.ENABLE) { }
	TCC3->CTRLA.reg = TCC_CTRLA_SWRST;
	while (TCC3->SYNCBUSY.bit.SWRST) { }

	TCC3->EVCTRL.reg = TCC_EVCTRL_EVACT1_DIR | TCC_EVCTRL_EVACT0_COUNTEV | TCC_EVCTRL_TCEI1 | TCC_EVCTRL_TCEI0;

	Enable();
	return GCodeResult::ok;
}

void QuadratureEncoderCCL::Enable() noexcept
{
	SetPosition(0);

	TCC3->CTRLA.bit.ENABLE = 1;
	while (TCC3->SYNCBUSY.bit.ENABLE) { };
}

void QuadratureEncoderCCL::Disable() noexcept
{
	TCC3->CTRLA.bit.ENABLE = 0;
	while (TCC3->SYNCBUSY.bit.ENABLE) { };
}

void QuadratureEncoderCCL::ClearFullRevs() noexcept
{
	counterHigh = (lastCount & 0x8000) ? 0xFFFF : 0;
	(void)TakeReading();
}

void QuadratureEncoderCCL::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf("Encoder reverse polarity: %s", (IsBackwards()) ? "yes" : "no");

#if 1	//debug
	TCC3->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
	while (TCC3->SYNCBUSY.reg & (TCC_SYNCBUSY_CTRLB | TCC_SYNCBUSY_COUNT)) { }
	const uint16_t count = TCC3->COUNT.reg;
	reply.catf(", raw count %u", count);
#endif
}

void QuadratureEncoderCCL::AppendStatus(const StringRef& reply) noexcept
{
	reply.lcatf("Quadrature encoder pulses/rev: %.2f", (double)(countsPerRev / 4));
}

// Get the current position relative to the starting position
int32_t QuadratureEncoderCCL::GetRelativePosition(bool& error) noexcept
{
	TCC3->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
	while (TCC3->SYNCBUSY.reg & (TCC_SYNCBUSY_CTRLB | TCC_SYNCBUSY_COUNT)) { }

	const uint16_t count = TCC3->COUNT.reg;

	// Handle wrap around of the high position bits
	const uint16_t currentHighBits = count >> 14;
	const uint16_t lastHighBits = lastCount >> 14;
	if (currentHighBits == 3 && lastHighBits == 0)
	{
		--counterHigh;
	}
	else if (currentHighBits == 0 && lastHighBits == 3)
	{
		++counterHigh;
	}

	lastCount = count;

	error = false;
	return (int32_t)((counterHigh << 16) | count);
}

// End of overridden virtual functions

// Set the position to the 32 bit signed value 'position'
void QuadratureEncoderCCL::SetPosition(int32_t position) noexcept
{
	while (TCC3->SYNCBUSY.bit.STATUS) { }
	const bool stopped = TCC3->STATUS.bit.STOP;

	if (!stopped)
	{
		/* "When a stop is detected while the counter is running, the counter will maintain its current
		 * value." */
		TCC3->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
		while (TCC3->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB) { }
		while (TCC3->CTRLBSET.bit.CMD != 0) { }
	}

	TCC3->COUNT.reg = lastCount = (uint16_t)position;
	while (TCC3->SYNCBUSY.bit.COUNT != 0) { }
	counterHigh = (uint32_t)position >> 16;

	if (!stopped)
	{
		/* "If the re-trigger command is detected when the counter is stopped, the counter will
		 * resume counting operation from the value in COUNT." */
		TCC3->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
		while (TCC3->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB) { }
		while (TCC3->CTRLBSET.bit.CMD != 0) { }
	}
}

#endif	// SUPPORT_CLOSED_LOOP
