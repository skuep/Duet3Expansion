/*
 * PositionDecoder.cpp
 *
 *  Created on: 13 Nov 2024
 *      Author: Simon Kueppers
 *
 *
 * This class realizes a quadrature decoder using CCL (configurable logic) and a TCC instance.
 * The basic idea follows DS00002434B: AN2434 Interfacing Quadrature Encoder using CCL with TCA and TCB from Microchip
 */

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP && SAME5x && SUPPORT_CCL_ENCODER

#include "QuadratureEncoderCCL.h"
#include <hri_mclk_e54.h>
#include <hri_eic_e54.h>
#include <cmath>

// Overridden virtual functions

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult QuadratureEncoderCCL::Init(const StringRef& reply) noexcept
{
#if DIFFERENTIAL_STEPPER_OUTPUTS
	reply.printf('Encoder not available due to DIFFERENTIAL_STEPPER_OUTPUTS');
	return;
#endif

	/* Enable APB clocks and enable GCLKs on CCL and TCC and make them run synchronously */
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL | MCLK_APBCMASK_TCC3;
    hri_gclk_write_PCHCTRL_reg(GCLK, CCL_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC3_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);

	/* Setup EIC to route A and B signals into the event system with both-edge event triggering */
	constexpr Pin PositionDecoderEICPins[] = { PortAPin(7), PortBPin(10) };

	for (Pin p : PositionDecoderEICPins)
	{
		/* Set pins to EIC function */
		SetPinFunction(p, GpioPinFunction::A); /* EIC */
	}

	EIC->CTRLA.bit.ENABLE = 0;
	while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) { }

	EIC->CONFIG[0].bit.SENSE7 = EIC_CONFIG_SENSE7_BOTH_Val; /* EXTINT7 */
	EIC->CONFIG[1].bit.SENSE2 = EIC_CONFIG_SENSE2_BOTH_Val; /* EXTINT10 */
	EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO(1<<7) | EIC_EVCTRL_EXTINTEO(1<<10);

	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) { }

	/* Setup event system channels on the external interrupt pins */
	EVSYS->USER[EVSYS_ID_USER_CCL_LUTIN_0].reg = EVSYS_USER_CHANNEL(30+1); /* PULSE_A to LUT0 */
	EVSYS->USER[EVSYS_ID_USER_CCL_LUTIN_1].reg = EVSYS_USER_CHANNEL(31+1); /* PULSE_B to LUT1 */
	EVSYS->Channel[30].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_7); /* PULSE_A */
	EVSYS->Channel[31].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_10); /* PULSE_B */

	/* Set up CCL LUTs.
	 * For the CCL, A and B IOs are a second set of pins, different from EIC pins above, routed to the CCL peripheral. */
	constexpr Pin PositionDecoderCCLPins[] = { PortBPin(14), PortBPin(15) };

	for (Pin p : PositionDecoderCCLPins)
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

	/* LUT0/LUT1 as S-R Latch, PULSE_A connects to S, PULSE_B connects to R, CNT output is on LUT0 */
	CCL->SEQCTRL[0].reg = CCL_SEQCTRL_SEQSEL_RS;
	CCL->LUTCTRL[0].reg = CCL_LUTCTRL_TRUTH(0b10101010) | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_INSEL1_MASK | CCL_LUTCTRL_INSEL0_EVENT
			| CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_ENABLE;
	CCL->LUTCTRL[1].reg = CCL_LUTCTRL_TRUTH(0b10101010) | CCL_LUTCTRL_INSEL2_MASK | CCL_LUTCTRL_INSEL1_MASK | CCL_LUTCTRL_INSEL0_EVENT
			| CCL_LUTCTRL_LUTEI | CCL_LUTCTRL_ENABLE;

	/* LUT3 as DIR decoder.
	 * DIR = A(IN9) xor B(IN10) xor CNT(LINK) */
	CCL->LUTCTRL[3].reg = CCL_LUTCTRL_TRUTH(0b10010110) | CCL_LUTCTRL_INSEL2_LINK | CCL_LUTCTRL_INSEL1_IO | CCL_LUTCTRL_INSEL0_IO
						| CCL_LUTCTRL_LUTEO | CCL_LUTCTRL_ENABLE;

	/* Enable CCL */
	CCL->CTRL.reg = CCL_CTRL_ENABLE;

	/* Configure event system to feed CNT/DIR signals into TCC3 */
    hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_0, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
    hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_1, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);
	EVSYS->USER[EVSYS_ID_USER_TCC3_EV_0].reg = EVSYS_USER_CHANNEL(0+1);
	EVSYS->USER[EVSYS_ID_USER_TCC3_EV_1].reg = EVSYS_USER_CHANNEL(1+1);
	EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_PATH_RESYNCHRONIZED | EVSYS_CHANNEL_EDGSEL_BOTH_EDGES | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CCL_LUTOUT_0); /* CNT */
	EVSYS->Channel[1].CHANNEL.reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_CCL_LUTOUT_3); /* DIR  */

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
		TCC3->CTRLA.bit.ENABLE = 0;
		while (TCC3->SYNCBUSY.bit.ENABLE != 0) { }
	}

	TCC3->COUNT.reg = lastCount = (uint16_t)position;
	while (TCC3->SYNCBUSY.bit.COUNT != 0) { }
	counterHigh = (uint32_t)position >> 16;

	if (!stopped)
	{
		TCC3->CTRLA.bit.ENABLE = 1;
		while (TCC3->SYNCBUSY.bit.ENABLE != 0) { }
	}
}

#endif	// SUPPORT_CLOSED_LOOP
