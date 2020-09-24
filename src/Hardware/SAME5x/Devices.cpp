/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <AnalogIn.h>
#include <AnalogOut.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

Uart uart0(3, 3, 512, 512);

extern "C" void SERCOM3_0_Handler()
{
	uart0.Interrupt0();
}

extern "C" void SERCOM3_2_Handler()
{
	uart0.Interrupt2();
}

extern "C" void SERCOM3_3_Handler()
{
	uart0.Interrupt3();
}

void DeviceInit() noexcept
{
	SetPinFunction(PortBPin(20), GpioPinFunction::C);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	SetPinFunction(PortBPin(21), GpioPinFunction::C);		// RxD
# endif

	AnalogIn::Init(DmacChanAdc0Tx, DmacPrioAdcTx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
