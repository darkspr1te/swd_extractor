/*
 * Copyright (C) 2017 Obermaier Johannes
 *
 * This Source Code Form is subject to the terms of the MIT License.
 * If a copy of the MIT License was not distributed with this file,
 * you can obtain one at https://opensource.org/licenses/MIT
 */

#include "main.h"
#include "swd.h"
#include "clk.h"
#include "target.h"

#define MWAIT __asm__ __volatile__( \
		 ".syntax unified 		\n" \
		 "	movs r0, #0x30 		\n" \
		 "1: 	subs r0, #1 		\n" \
		 "	bne 1b 			\n" \
		 ".syntax divided" : : : 	    \
		 "cc", "r0")

#define N_READ_TURN (3u)


static uint8_t swdParity( uint8_t const * data, uint8_t const len );
static void swdDatasend( uint8_t const * data, uint8_t const len );
static void swdDataIdle( void );
static void swdDataPP( void );
static void swdTurnaround( void );
static void swdReset( void );
static void swdDataRead( uint8_t * const data, uint8_t const len );
static void swdBuildHeader( swdAccessDirection_t const adir, swdPortSelect_t const portSel, uint8_t const A32, uint8_t * const header);
static swdStatus_t swdReadPacket( swdPortSelect_t const portSel, uint8_t const A32, uint32_t * const data );
static swdStatus_t swdWritePacket( swdPortSelect_t const portSel, uint8_t const A32, uint32_t const data );
static swdStatus_t swdReadAP0( uint32_t * const data );

#ifdef UNUSED_EXPERIMENTAL
static swdStatus_t swdReadDPCtrl( uint32_t * const data );
static swdStatus_t swdReadAPCtrl( uint32_t * const data );
static swdStatus_t swdReadWIREMODE( uint32_t * const data );
static swdStatus_t swdReadDHCSR( uint32_t * const data );
static swdStatus_t swdWriteAHBAddr( uint32_t const addr, uint32_t const data );
static swdStatus_t swdCoreHalt( void );
static swdStatus_t swdGetRegister( uint8_t const regId, uint32_t * const data );
#endif


void swdCtrlInit( void )
{
    #ifdef stm32f0xx
	RCC->AHBENR |= RCC_AHBENR_GPIO_SWDIO;
	RCC->AHBENR |= RCC_AHBENR_GPIO_SWCLK;

	GPIO_SWDIO->MODER |= (0x01u << (PIN_SWDIO << 1u));
	GPIO_SWCLK->MODER |= (0x01u << (PIN_SWCLK << 1u));

	GPIO_SWDIO->OSPEEDR |= (0x03 << (PIN_SWDIO << 1u));
	GPIO_SWCLK->OSPEEDR |= (0x03 << (PIN_SWCLK << 1u));

	/* pulldown for clk, pullup for swdio */
	GPIO_SWDIO->PUPDR |= (0x01u << (PIN_SWDIO << 1u));
	GPIO_SWCLK->PUPDR |= (0x02u << (PIN_SWCLK << 1u));
    #endif;
    pinMode(SWDIO_PIN,OUTPUT);
    pinMode(SWCLK_PIN,OUTPUT);
    digitalWrite(SWDIO_PIN,HIGH);
    digitalWrite(SWCLK_PIN,HIGH);
    
	return ;
}


static uint8_t swdParity( uint8_t const * data, uint8_t const len )
{
	uint8_t par = 0u;
	uint8_t cdata = 0u;
	uint8_t i = 0u;

	for (i=0u; i<len; ++i)
	{
		if ((i & 0x07u) == 0u)
		{
			cdata = *data;
			++data;
		}

		par ^= (cdata & 0x01u);
		cdata >>= 1u;
	}

	return par;
}


static void swdDatasend( uint8_t const * data, uint8_t const len )
{
	uint8_t cdata = 0u;
	uint8_t i = 0u;

	for (i=0u; i<len; ++i)
	{
		if ((i & 0x07u) == 0x00u)
		{
			cdata = *data;
			++data;
		}

		if ((cdata & 0x01u) == 0x01u)
		{
			//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_SET));
            digitalWrite(SWDIO_PIN,HIGH);
		}
		else
		{
			//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_CLEAR));
            digitalWrite(SWDIO_PIN,LOW);
		}
		MWAIT;

		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
		digitalWrite(SWCLK_PIN,HIGH);
        MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
        digitalWrite(SWCLK_PIN,LOW);
		cdata >>= 1u;
		MWAIT;
	}

	return ;
}


static void swdDataIdle( void )
{
	//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO));
    digitalWrite(SWDIO_PIN,HIGH);
    
	MWAIT;
	//GPIO_SWDIO->MODER &= ~(0x03u << (PIN_SWDIO << 1u));
	//pinMode(SWDIO_PIN,OUTPUT);
    pinMode(SWDIO_PIN,INPUT_PULLUP);
    //pinMode(SWDIO_PIN,INPUT_ANALOG);
    MWAIT;

	return ;
}


static void swdDataPP( void )
{
	MWAIT;
	//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_CLEAR));
    digitalWrite(SWDIO_PIN,LOW);
	//GPIO_SWDIO->MODER |= (0x01u << (PIN_SWDIO << 1u));
	pinMode(SWDIO_PIN,OUTPUT);
	//pinMode(SWDIO_PIN,OUTPUT_OPEN_DRAIN);
    //pinMode(SWDIO_PIN,INPUT_PULLUP);
    MWAIT;

	return ;
}


static void swdTurnaround( void )
{
	//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
    digitalWrite(SWCLK_PIN,HIGH);
	MWAIT;
	//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
    digitalWrite(SWCLK_PIN,LOW);
	MWAIT;

	return ;
}


static void swdDataRead( uint8_t * const data, uint8_t const len )
{
	uint8_t i = 0u;
	uint8_t cdata = 0u;

	MWAIT;
	swdDataIdle();
	MWAIT;

	for (i=0u; i<len; ++i)
	{

		cdata >>= 1u;
		//cdata |= (GPIO_SWDIO->IDR & (0x01u << (PIN_SWDIO))) ? 0x80u : 0x00u;
        cdata |= digitalRead(SWDIO_PIN) ? 0x80u :0x00u;
       // Serial.print(cdata,HEX);
		//Serial.print(",");
		data[(((len + 7u) >> 3u) - (i >> 3u)) - 1u] = cdata;

		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
        digitalWrite(SWCLK_PIN,HIGH);
		MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
        digitalWrite(SWCLK_PIN,LOW);
		MWAIT;

		/* clear buffer after reading 8 bytes */
		if ((i & 0x07u) == 0x07u)
		{
			cdata = 0u;
		}
	}

	return ;
}


static void swdReset( void )
{
	uint8_t i = 0u;

	MWAIT;
	GPIO_SWDIO->ODR |= 0x01u << PIN_SWDIO;
	GPIO_SWCLK->ODR |= 0x01u << PIN_SWCLK;
	MWAIT;

/* Switch from JTAG to SWD mode. Not required for SWD-only devices (STM32F0x). */
#ifdef DO_JTAG_RESET

	/* 50 clk+x */
	for (i=0u; i < (50u + 10u); ++i)
	{
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
        digitalWrite(SWCLK_PIN,HIGH);
		MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
		 digitalWrite(SWCLK_PIN,LOW);
        MWAIT;
	}

	uint8_t send1[] = {0u, 1u, 1u, 1u, 1u, 0u, 0u, 1u, 1u, 1u, 1u, 0u, 0u, 1u, 1u, 1u};
	/* send 0111 1001 1110 0111 */

	for (i = 0u; i < 16u; ++i)
	{
		if (send1[i])
			//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_SET));
             digitalWrite(SWDIO_PIN,HIGH);
		else
			//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_CLEAR));
             digitalWrite(SWDIO_PIN,LOW);

		MWAIT;

		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
        digitalWrite(SWCLK_PIN,HIGH);
		MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
        digitalWrite(SWCLK_PIN,LOW);
		MWAIT;

	}
#endif

	/* 50 clk+x */
	for (i = 0u; i < (50u + 10u); ++i)
	{
        
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
		digitalWrite(SWCLK_PIN,HIGH);
        MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
        digitalWrite(SWCLK_PIN,LOW);
		MWAIT;
	}


	//GPIO_SWDIO->BSRR = (0x01u << (PIN_SWDIO + BSRR_CLEAR));
    digitalWrite(SWDIO_PIN,LOW);
	for (i = 0u; i < 3u; ++i)
	{
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_SET));
        digitalWrite(SWCLK_PIN,HIGH);
		MWAIT;
		//GPIO_SWCLK->BSRR = (0x01u << (PIN_SWCLK + BSRR_CLEAR));
		digitalWrite(SWCLK_PIN,LOW);
        MWAIT;
	}

	return ;
}


static void swdBuildHeader( swdAccessDirection_t const adir, swdPortSelect_t const portSel, uint8_t const A32, uint8_t * const header)
{

	if (portSel == swdPortSelectAP)
	{
		*header |= 0x02u; /* Access AP */
	}

	if (adir == swdAccessDirectionRead)
	{
		*header |= 0x04u; /* read access */
	}

	switch (A32)
	{
		case 0x01u:
			*header |= 0x08u;
		break;

		case 0x02u:
			*header |= 0x10u;
		break;

		case 0x03u:
			*header |= 0x18u;
		break;

		default:
		case 0x00u:

		break;
	}

	*header |= swdParity(header, 7u) << 5u;
	*header |= 0x01u; /* startbit */
	*header |= 0x80u;
}


static swdStatus_t swdReadPacket( swdPortSelect_t const portSel, uint8_t const A32, uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t header = 0x00u;
	uint8_t rp[1] = {0x00u};
	uint8_t resp[5] = {0u};
	uint8_t i = 0u;

	swdBuildHeader( swdAccessDirectionRead, portSel, A32, &header );

	swdDatasend( &header, 8u );
	swdDataIdle();
	swdTurnaround();
	swdDataRead( rp, 3u );

	swdDataRead( resp, 33u );

	swdDataPP();

	for (i=0u; i < N_READ_TURN; ++i)
	{
		swdTurnaround();
	}

	*data = resp[4] | (resp[3] << 8u) | (resp[2] << 16u) | (resp[1] << 24u);

	ret = (swdStatus_t) rp[0];

	return ret;
}


static swdStatus_t swdWritePacket( swdPortSelect_t const portSel, uint8_t const A32, uint32_t const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t header = 0x00u;
	uint8_t rp[1] = {0x00u};
	uint8_t data1[5] = {0u};
	uint8_t i = 0u;

	swdBuildHeader( swdAccessDirectionWrite, portSel, A32, &header );

	swdDatasend( &header, 8u );
	MWAIT;

	swdDataIdle();
	MWAIT;

	swdTurnaround();

	swdDataRead( rp, 3u );

	swdDataIdle();

	swdTurnaround();
	swdDataPP();

	data1[0] = data & 0xFFu;
	data1[1] = (data >> 8u) & 0xFFu;
	data1[2] = (data >> 16u) & 0xFFu;
	data1[3] = (data >> 24u) & 0xFFu;
	data1[4] = swdParity(data1, 8u * 4u);

	swdDatasend( data1, 33u );

	swdDataPP();

	for (i=0u; i < 20u; ++i)
	{
		swdTurnaround();
	}

	ret = (swdStatus_t)rp[0];

	return ret;
}


swdStatus_t swdReadIdcode( uint32_t * const idCode )
{
	uint32_t ret = 0u;

	ret = swdReadPacket(swdPortSelectDP, 0x00u, idCode);

	return (swdStatus_t)ret;
}


swdStatus_t swdSelectAPnBank(uint8_t const ap, uint8_t const bank)
{
	swdStatus_t ret = swdStatusNone;
	uint32_t data = 0x00000000u;
    uint8_t ret_8 =0;
	data |= (uint32_t) (ap & 0xFFu) << 24u;
	data |= (uint32_t) (bank & 0x0Fu) << 0u;

	/* write to select register */
	//ret |= (swdStatus_t)swdWritePacket(swdPortSelectDP, 0x02u, data);
    ret_8|= swdWritePacket(swdPortSelectDP, 0x02u, data);
    ret =(swdStatus_t)ret_8;
	return ret;
}


static swdStatus_t swdReadAP0( uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	ret_8=swdReadPacket(swdPortSelectAP, 0x00u, data);
	ret=(swdStatus_t)ret_8;
	return ret;
}


swdStatus_t swdSetAP32BitMode( uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
    uint8_t ret_8 =0;
	swdSelectAPnBank( 0x00u, 0x00u );

	uint32_t d = 0u;

	ret_8 |= swdReadAP0( &d );
    ret =(swdStatus_t)ret_8;

	ret_8 |= swdReadPacket(swdPortSelectDP, 0x03u, &d);
    ret =(swdStatus_t)ret_8;
	d &= ~(0x07u);
	d |= 0x02u;

	ret_8 |= swdWritePacket(swdPortSelectAP, 0x00u, d);
    ret =(swdStatus_t)ret_8;
	ret_8 |= swdReadAP0( &d );
	ret_8 |= swdReadPacket(swdPortSelectDP, 0x03u, &d);

	if (data != NULL)
	{
		*data = d;
	}
    ret =(swdStatus_t)ret_8;
	return ret;
}


swdStatus_t swdSelectAHBAP( void )
{
	swdStatus_t ret = swdSelectAPnBank(0x00u, 0x00u);

	return ret;
}


swdStatus_t swdReadAHBAddr( uint32_t const addr, uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
    uint8_t ret_8 =0;
	uint32_t d = 0u;

	ret_8 |= swdWritePacket(swdPortSelectAP, 0x01u, addr);

	ret_8 |= swdReadPacket(swdPortSelectAP, 0x03u, &d);
	ret_8 |= swdReadPacket(swdPortSelectDP, 0x03u, &d);

	*data = d;
    ret =(swdStatus_t)ret_8;
	return ret;
}


swdStatus_t swdEnableDebugIF( void )
{
	swdStatus_t ret = swdStatusNone;
    uint8_t ret_8 =0;

	ret_8 |= swdWritePacket(swdPortSelectDP, 0x01u, 0x50000000u);
    ret =(swdStatus_t)ret_8;
	return ret;
}


swdStatus_t swdInit( uint32_t * const idcode )
{
	swdStatus_t ret = swdStatusNone;
    uint8_t ret_8 =0;
	swdReset();
	ret_8 |= swdReadIdcode( idcode );
    ret =(swdStatus_t)ret_8;

	return ret;
}

/**
 * Below not implemented within main.c 
 */



#ifdef UNUSED_EXPERIMENTAL
static swdStatus_t swdReadDPCtrl( uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	ret_8 |= swdSelectAPnBank(0x00u, 0x00u);
	ret_8 |= swdReadPacket(swdPortSelectAP, 0x01u, data);
	ret=(swdStatus_t)	ret_8;
	return ret;
}


static swdStatus_t swdReadAPCtrl( uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	ret_8 |= swdReadPacket(swdPortSelectDP, 0x01u, data);
	ret=(swdStatus_t)	ret_8;
	return ret;
}


static swdStatus_t swdReadWIREMODE( uint32_t * const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	uint32_t ret_32=0;
	ret_8 |= swdWritePacket(swdPortSelectDP, 0x02u, 0x00000001u);
	ret_8 |= swdReadPacket(swdPortSelectDP, 0x01u, data);
	ret_32 = *data;
	ret=(swdStatus_t)ret_32;
	return ret;
}


static swdStatus_t swdReadDHCSR( uint32_t * const data )
{
	swdStatus_t ret = swdReadAHBAddr(0xE000EDF0u, data);

	return ret;
}


static swdStatus_t swdWriteAHBAddr( uint32_t const addr, uint32_t const data )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	ret_8 |= swdWritePacket(swdPortSelectAP, 0x01u, addr);
	ret_8 |= swdWritePacket(swdPortSelectAP, 0x03u, data);
	ret=(swdStatus_t)ret_8;
	return ret;
}


static swdStatus_t swdCoreHalt( void )
{
	swdStatus_t ret = swdStatusNone;
	uint8_t ret_8 =0;
	ret_8 |= swdWriteAHBAddr(0xE000EDF0u, 0xA05F0003u);
	ret=(swdStatus_t)ret_8;
	return ret;
}


static swdStatus_t swdGetRegister( uint8_t const regId, uint32_t * const data )
{
	swdWriteAHBAddr(0xE000EDF4u, regId & 0x1Fu);

	swdStatus_t ret = swdReadAHBAddr(0xE000EDF8u, data);

	return ret;
}

#endif
