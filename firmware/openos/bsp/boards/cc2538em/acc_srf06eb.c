//*****************************************************************************
//! @file       acc_srf06eb.c
//! @brief      SmartRF06EB device driver implementation for BMA250
//!             accelerometer. Accelerometer pin INT2 is not connected on
//!             SmartRF06EB revision < 1.2.0.
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef ACC_EXCLUDE


/**************************************************************************//**
* @addtogroup acc_api
* @{
******************************************************************************/

#if SENSOR_ACCEL
/******************************************************************************
* INCLUDES
*/
#include "board.h"
#include "hw_memmap.h"
//#include "io_pin_int.h"
#include "acc_bma250.h"

#include "ioc.h"                    // Access to driverlib io control fns
#include "gpio.h"                   // Access to driverlib gpio fns
#include "ssi.h"                    // Access to driverlib ssi fns
#include "sys_ctrl.h"               // Access to driverlib sys ctrl fns
#include "interrupt.h"              // Access to driverlib interrupt fns


/******************************************************************************
* DEFINES
*/
// Base address of SSI module used by implementation
#define ACC_SSI_BASE            BSP_SPI_SSI_BASE

// Macro for asserting accelerometer CSn (set low)
#define ACC_SPI_BEGIN()         GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, 0);

// Macro for deasserting accelerometer CSn (set high)
#define ACC_SPI_END()           GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, BSP_ACC_CS);


/******************************************************************************
* VARIABLES AND LOCAL FUNCTIONS
*/


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    This function initializes the accelerometer. This must be run
*           before you can use the accelerometer. The function assumes that the
*           SPI interface has already been initialized using, for example,
*           bspSpiInit().
*
* @return   None
******************************************************************************/
void
accInit(void)
{
    uint8_t ui8RegVal, ui8Pins;

    //
    // Configure CS and PWR as output high (both on the same port)
    //
    ui8Pins = (BSP_ACC_CS | BSP_ACC_PWR);
    GPIOPinTypeGPIOOutput(BSP_ACC_PWR_BASE, ui8Pins);
    GPIOPinWrite(BSP_ACC_PWR_BASE, ui8Pins, ui8Pins);

    //
    // Configure interrupt pin as input tristate
    //
    // GPIOPinTypeGPIOInput(BSP_ACC_INT_BASE, BSP_ACC_INT);

    //
    // Delay ~2ms for ACC to be powered up (3 CPU cycles per arg. value)
    //
    SysCtrlDelay(SysCtrlClockGet() / 500 / 3);

    //
    // Disable interrupts
    //
    ui8RegVal = 0;
    accWriteReg(ACC_INT_EN0, &ui8RegVal, 1);
    accWriteReg(ACC_INT_EN1, &ui8RegVal, 1);

    //
    // Set 2G range
    //
    ui8RegVal = ACC_RANGE_2G;
    accWriteReg(ACC_RANGE, &ui8RegVal, 1);

    //
    // Set filter detection bandwidth
    //
    // ui8RegVal = ACC_BW_250HZ;
    // accWriteReg(ACC_BW, &ui8RegVal, 1);
}


/**************************************************************************//**
* @brief    This function disables the accelerometer by turning off its power.
*           This function assumes the accelerometer PWR pin is already
*           configured as output by, for example, accInit().
*
* @return   None
******************************************************************************/
void
accDisable(void)
{
    //
    // Set PWR pin low
    //
    GPIOPinWrite(BSP_ACC_PWR_BASE, BSP_ACC_PWR, 0);
}


/*
 * RFF
 * Read e Write SPI register
 */

uint8_t  bsp_spi_read_frame(uint8_t *pui8Buf, uint8_t size)
{
	uint8_t  ui8Len=size;
	uint32_t ui32Data;

	//
    // Set CSn active
    //
    // ACC_SPI_BEGIN();

    //
    // Send address byte to SSI FIFO and read dummy data
    //
    //  SSIDataPut(ACC_SSI_BASE, (ui8Addr | ACC_READ_M));
    //  SSIDataGet(ACC_SSI_BASE, &ui32Data);

    while(ui8Len--)
    {
        //
        // Send dummy byte and read returned data from SPI FIFO
        //
        SSIDataPut(ACC_SSI_BASE, 0x00);
        SSIDataGet(ACC_SSI_BASE, &ui32Data);

        //
        // Store read data to buffer
        //
        *pui8Buf++ = (ui32Data & 0xFF);
    }

    //
    // Clear CSn
    //
    ACC_SPI_END();

    return 1;
}

uint8_t bsp_spi_write_frame(uint8_t *frame, uint8_t size)
{
	uint8_t  count=size;
	uint32_t ui32Data;

	while(SSIBusy(ACC_SSI_BASE))
	{
	}

	while(SSIDataGetNonBlocking(ACC_SSI_BASE, &ui32Data));
	{
	}

	ACC_SPI_BEGIN();  // Set CSn active

	while (count > 0)
	{
		SSIDataPut(ACC_SSI_BASE, (*frame | ACC_READ_M));
		SSIDataGet(ACC_SSI_BASE, &ui32Data);

		frame++;
		count--;
	}

    //ACC_SPI_END();	// Clear CSn

#if 0
    uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index=0;
    uint32_t ui32Data = (uint32_t) 0x12345633;

    SSIDataPut(SSI0_BASE, ui32Data);

    while(SSIBusy(SSI0_BASE))
    {
    }

    for(ui32Index = 0; ui32Index < (ui32Index) size; ui32Index++)
    {
        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        SSIDataGet(SSI0_BASE, &pui32DataRx[ui32Index]);

        //
        // Since we are using 8-bit data, mask off the MSB.
        //
        pui32DataRx[ui32Index] &= 0x00FF;
    }
#endif

    return size;
}



/**************************************************************************//**
* @brief    This function reads one or more accelerometer registers.
*
* @param    ui8Addr     is the register start address.
* @param    pui8Buf     is a pointer to the destination buffer.
* @param    ui8Len      is the number of registers to read.
*
* @return   None
******************************************************************************/
void
accReadReg(uint8_t ui8Addr, uint8_t *pui8Buf, uint8_t ui8Len)
{
    uint32_t ui32Data;

    //
    // Wait for ongoing transfers to complete and then clear in fifo before
    // pulling CSn low. This makes sure that accelerometer only retrieves data
    // intended for it.
    //
    while(SSIBusy(ACC_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(ACC_SSI_BASE, &ui32Data));
    {
    }

    //
    // Set CSn active
    //
    ACC_SPI_BEGIN();

    //
    // Send address byte to SSI FIFO and read dummy data
    //
    SSIDataPut(ACC_SSI_BASE, (ui8Addr | ACC_READ_M));
    SSIDataGet(ACC_SSI_BASE, &ui32Data);

    while(ui8Len--)
    {
        //
        // Send dummy byte and read returned data from SPI FIFO
        //
        SSIDataPut(ACC_SSI_BASE, 0x00);
        SSIDataGet(ACC_SSI_BASE, &ui32Data);

        //
        // Store read data to buffer
        //
        *pui8Buf++ = (ui32Data & 0xFF);
    }

    //
    // Clear CSn
    //
    ACC_SPI_END();
}


/**************************************************************************//**
* @brief    This function writes one or more accelerometer registers. The
*           function implements burst-like functionality. The BMA250
*           accelerometer does not support burst write (multiple writes with
*           CSn low) thus CSn is pulled high between each address-data pair.
*
* @param    ui8Addr     is the register start address.
* @param    pui8Buf     is the pointer to source buffer.
* @param    ui8Len      is the number of registers to write.
*
* @return   None
******************************************************************************/
void
accWriteReg(uint8_t ui8Addr, const uint8_t *pui8Buf, uint8_t ui8Len)
{
    uint32_t ui32Dummy;

    //
    // Wait for ongoing transfers to complete before pulling CSn low
    //
    while(SSIBusy(ACC_SSI_BASE))
    {
    }

    while(ui8Len--)
    {
        //
        // Set CSn active
        //
        ACC_SPI_BEGIN();

        //
        // Send address byte to SSI FIFO and increment address
        //
        SSIDataPut(ACC_SSI_BASE, ui8Addr++);

        //
        // Send value byte to SSI FIFO and increment buffer pointer
        //
        SSIDataPut(ACC_SSI_BASE, *pui8Buf++);

        //
        // Wait for data to be clocked out before pulling CSn high
        //
        while(SSIBusy(ACC_SSI_BASE))
        {
        }

        //
        // Clear CSn
        //
        ACC_SPI_END();
    }

    //
    // Empty SSI IN FIFO
    //
    while(SSIDataGetNonBlocking(ACC_SSI_BASE, &ui32Dummy));
}


/**************************************************************************//**
* @brief    This function reads present acceleration data. the function assumes
*           the SPI in FIFO of the device to be empty.
*
* @param    *pi16XVal   is a pointer to where the x-axis value is stored.
* @param    *pi16YVal   is a pointer to where the y-axis value is stored.
* @param    *pi16ZVal   is a pointer to where the z-axis value is stored.
*
* @return   None
******************************************************************************/
void
accReadData(int16_t *pi16XVal, int16_t *pi16YVal, int16_t *pi16ZVal)
{
    int8_t i8Data[6] = {0};

    //
    // Read axis bytes from accelerometer
    //
    accReadReg(ACC_X_LSB, (uint8_t *)i8Data, 6);

    //
    // Convert LSB and MSB parts into a int16_t
    //
    *pi16XVal = (((int16_t)(i8Data[1]) << 2) | ((i8Data[0] >> 6) & 0x03));
    *pi16YVal = (((int16_t)(i8Data[3]) << 2) | ((i8Data[2] >> 6) & 0x03));
    *pi16ZVal = (((int16_t)(i8Data[5]) << 2) | ((i8Data[4] >> 6) & 0x03));
}

#if 0
/**************************************************************************//**
* @brief    This function registers a custom interrupt handler to the GPIO pins
*           specified by \e ui8Pins.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*                       \li \b BSP_ACC_INT1
*                       \li \b BSP_ACC_INT2
* @param    pfnHandler  is a pointer to the interrupt handler function.
*
* @return   None
******************************************************************************/
void
accIntRegister(uint8_t ui8Pins, void (*pfnHandler)(void))
{
    //
    // Register handler in the I/O interrupt handler
    //
    ioPinIntRegister(BSP_ACC_INT_BASE, ui8Pins, pfnHandler);
}


/**************************************************************************//**
* @brief    This function unregisters the custom interrupt handler from the
*           GPIO pins specified by \e ui8Pins.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*                       \li \b BSP_ACC_INT1
*                       \li \b BSP_ACC_INT2
*
* @return   None
******************************************************************************/
void
accIntUnregister(uint8_t ui8Pins)
{
    //
    // Unregister handler in the I/O interrupt handler
    //
    ioPinIntRegister(BSP_ACC_INT_BASE, ui8Pins, 0);
}


/**************************************************************************//**
* @brief    This function enables interrupts on GPIO pins connected to
*           accelerometer.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*                       \li \b BSP_ACC_INT1
*                       \li \b BSP_ACC_INT2
*
* @return   None
******************************************************************************/
void
accIntEnable(uint8_t ui8Pins)
{
    GPIOPinIntEnable(BSP_ACC_INT_BASE, ui8Pins);
}


/**************************************************************************//**
* @brief    This function disables interrupts on GPIO pins connected to
*           accelerometer.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*                       \li \b BSP_ACC_INT1
*                       \li \b BSP_ACC_INT2
*
* @return   None
******************************************************************************/
void
accIntDisable(uint8_t ui8Pins)
{
    GPIOPinIntDisable(BSP_ACC_INT_BASE, ui8Pins);
}


/**************************************************************************//**
* @brief    This function clears interrupt flag on GPIO pins connected to
*           accelerometer.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*           \li \b BSP_ACC_INT1
*           \li \b BSP_ACC_INT2
*
* @return   None
******************************************************************************/
void
accIntClear(uint8_t ui8Pins)
{
    GPIOPinIntClear(BSP_ACC_INT_BASE, ui8Pins);
}


/**************************************************************************//**
* @brief    This function sets the interrupt type for the GPIO pins connected
*           to the accelerometer interrupt pin.
*
* @param    ui8Pins     is a bitpacked bitmask of accelerometer interrupt pins;
*                       it can be an ORed combination of the following values:
*                       \li \b BSP_ACC_INT1
*                       \li \b BSP_ACC_INT2
* @param    ui32IntType is an enumerated data type that must be one of the
*                       following values:
*                       \li \b GPIO_FALLING_EDGE
*                       \li \b GPIO_RISING_EDGE
*                       \li \b GPIO_BOTH_EDGES
*                       \li \b GPIO_LOW_LEVEL
*                       \li \b GPIO_HIGH_LEVEL
******************************************************************************/
void
accIntTypeSet(uint8_t ui8Pins, uint32_t ui32IntType)
{
    GPIOIntTypeSet(BSP_ACC_INT_BASE, ui8Pins, ui32IntType);
}

#endif

/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef ACC_EXCLUDE
#endif
