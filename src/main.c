#include "S32K144.h"
#include "../src/Driver/PCC/Include/s32k144_pcc_driver.h"
#include "../src/Driver/PORT/Include/s32k144_port_driver.h"
#include "../src/Driver/UART/Include/s32k144_lpuart_driver.h"

#define MAX_NUMBER_OF_MB    (32U)

#define DEFAULT_MB_SIZE     (4U)

#define NODE_TX

typedef enum
{
    SOSC_CLK = 8000000U,
    SYS_CLK = 80000000U
} Clock_Rate_t;

typedef enum
{
    FLEX_CAN_0,
    FLEX_CAN_1,
    FLEX_CAN_2
} FlexCAN_Instance_t;

typedef enum
{
    FLEX_CAN_CLK_SRC_SOSCDIV2,
    FLEX_CAN_CLK_SRC_SYS_CLK
} FlexCAN_Clock_Source_t;

typedef enum
{
    HIGHEST_PRIORITY_BUFFER_TRANSMITTED_FIRST,
    LOWEST_BUFFER_TRANSMITTED_FIRST
} FlexCAN_Internal_Arbitration_t;

typedef enum
{
    RxFifoPriorityLow,
    RxFifoPriorityHigh
} FlexCAN_FIFO_Priority_t;

typedef enum
{
    RX_MB_INACTIVE  = 0U,
    RX_MB_EMPTY     = 4U,
    RX_MB_FULL      = 2U,
    RX_MB_OVERRUN   = 6U,
    RX_MB_RANSWER   = 10U,
    RX_MB_BUSY      = 1U
} FlexCAN_MB_Rx_Code_t;

typedef enum
{
    TX_MB_INACTIVE  = 8U,
    TX_MB_ABORT     = 9U,
    TX_MB_DATA      = 12U,
    TX_MB_REMOTE    = 12U,
    TX_MB_TANSWER   = 14U
} FlexCAN_MB_Tx_Code_t;

typedef enum
{
    CAN20,
    CANFD
} FlexCAN_Type_t;

typedef enum
{
    MAX_8_BYTES,
	MAX_16_BYTES,
	MAX_32_BYTES,
	MAX_64_BYTES
} FlexCAN_Max_Data_Length_t;

typedef enum
{
    NO_DATA,
	DATA_BYTE_0,
	DATA_BYTE_0_TO_1,
	DATA_BYTE_0_TO_2,
	DATA_BYTE_0_TO_3,
	DATA_BYTE_0_TO_4,
	DATA_BYTE_0_TO_5,
	DATA_BYTE_0_TO_6,
	DATA_BYTE_0_TO_7,
	DATA_BYTE_0_TO_11,
	DATA_BYTE_0_TO_15,
	DATA_BYTE_0_TO_19,
	DATA_BYTE_0_TO_23,
	DATA_BYTE_0_TO_31,
	DATA_BYTE_0_TO_47,
	DATA_BYTE_0_TO_63
} FlexCAN_Data_Length_Code_t;
/*****************************************************************************************************************
 *                                                  STRUCTURES                                                   *
 *****************************************************************************************************************/
typedef struct
{
    uint16_t u16Prescaler;
    uint8_t u8ResyncJumpWidth;
    uint8_t u8PropSeg;
    uint8_t u8PhaseSeg1;
    uint8_t u8PhaseSeg2;
} FlexCAN_Bit_Segment_t;

typedef struct
{
    uint16_t u16Prescaler;
    uint8_t u8ResyncJumpWidth;
    uint8_t u8PropSeg;
    uint8_t u8PhaseSeg1;
    uint8_t u8PhaseSeg2;
    uint8_t u8TDCOffset;
} FlexCAN_FD_Bit_Segment_t;

typedef struct
{
	FlexCAN_Bit_Segment_t sNormalBitRateConfig;
	FlexCAN_FD_Bit_Segment_t sDataBitRateConfig;
} FlexCAN_Timing_Config_t;

typedef struct
{
    FlexCAN_Clock_Source_t eClkSrc;
    FlexCAN_Type_t eCANType;
    uint8_t u8EnableIndividualMask;
    uint8_t u8LoopBackMode;
    FlexCAN_Timing_Config_t sTimingConfig;
    FlexCAN_Internal_Arbitration_t u8InternalArbitration;
    FlexCAN_Max_Data_Length_t eMaxDataLength;
} FlexCAN_Config_t;

typedef struct
{
    uint32_t u32ExtendedID;
    uint16_t u16StandardID;
    uint8_t u8Priority;
}FlexCAN_ID_t;

typedef struct
{
    FlexCAN_ID_t sID;
    uint16_t u16TimeStamp;
    uint8_t u8ExtendedDataLength;
    uint8_t u8BitRateSwitch;
    uint8_t u8ErrorStateIndicator;
    uint8_t u8SubstituteRemoteRequest;
    uint8_t u8ExtendedFormat;
    uint8_t u8RemoteTransmitRequest;
    uint8_t u8DataLengthCode;
    uint8_t u8PayloadLength;
    uint8_t* pu8PayloadBuffer;
}FlexCAN_Tx_Rx_Frame_t;

static uint8_t u8MBSize = DEFAULT_MB_SIZE;

static uint8_t u8UART_Buffer[13];
static uint8_t u8UART_BufferIdx = 0;

#ifdef NODE_TX
    FlexCAN_Tx_Rx_Frame_t sTxFrame;
#else
    FlexCAN_Tx_Rx_Frame_t sRxFrame;
    uint8_t u8RxData[13] = {0};
    uint8_t u8MBIndex;
#endif

void CAN_Transmit(FlexCAN_Instance_t eInstance, FlexCAN_Tx_Rx_Frame_t* DataFrame);

#ifdef NODE_TX
static void UART1_InterruptConfig(void)
{
    INT_SYS_SetPriority(LPUART1_RxTx_IRQn, 1);
    INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);
}

static void LPUART1_RxAppIRQHandler(void)
{
    uint8_t tmpBuff = 0;

    if (LPUART1->STAT & LPUART_STAT_RDRF_MASK) /* check RX flag*/
    {
        /* Get the data and enqueue it, parse data when receive '\n' charactor */
        if(!LPUART_RxFrame(LPUART1, &tmpBuff))
        {
            if(tmpBuff == '\n')
            {
            	CAN_Transmit(FLEX_CAN_0, &sTxFrame);
            }
            else
            {
				u8UART_Buffer[u8UART_BufferIdx] = tmpBuff;
				u8UART_BufferIdx = (13 == u8UART_BufferIdx) ? (u8UART_BufferIdx = 0) : (u8UART_BufferIdx + 1);
            }
        }

    }
}
#endif

static void UART1Config(void)
{
	PCC_Clock_Source_Select(PCC_LPUART1_INDEX, 0x6U);

	PCC_Init(PCC_PORTC_INDEX);
	PORT_Config_t LPUART1PortConfig;
	LPUART1PortConfig.pin = 6;
	LPUART1PortConfig.mux = PORT_MUX_ALTERNATIVE2;
	PORT_Init(PORTC, &LPUART1PortConfig);
	LPUART1PortConfig.pin = 7;
	PORT_Init(PORTC, &LPUART1PortConfig);

	PCC_Init(PCC_LPUART1_INDEX);

	LPUART_Handle_Type LPUART1Config = {0};
	LPUART1Config.pLPUARTx = LPUART1;
	LPUART1Config.LPUART_Config.frameLenth = LPUART_FRAME_8BITS;
	LPUART1Config.LPUART_Config.nOfStopBits = LPUART_STOP_BITS_1;
	LPUART1Config.LPUART_Config.rxInterrupt = LPUART_RX_INTERRUPT_ENABLE;
	LPUART1Config.LPUART_Config.baudRate = 115200;
	LPUART_Init(&LPUART1Config);

#ifdef NODE_TX
	LPUART_CallbackRegister(LPUART1, LPUART1_RxAppIRQHandler);

	UART1_InterruptConfig();
#endif
}

void SOSC_init_8MHz(void) {
	SCG->SOSCDIV=0x00000101; /* SOSCDIV1 & SOSCDIV2 =1: divide by 1 */
	SCG->SOSCCFG=0x00000024; /* Range=2: Medium freq (SOSC between 1MHz-8MHz)*/
	/* HGO=0: Config xtal osc for low power */
	/* EREFS=1: Input is external XTAL */
	while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK); /* Ensure SOSCCSR unlocked */
	SCG->SOSCCSR=0x00000001; /* LK=0: SOSCCSR can be written */
	/* SOSCCMRE=0: OSC CLK monitor IRQ if enabled */
	/* SOSCCM=0: OSC CLK monitor disabled */
	/* SOSCERCLKEN=0: Sys OSC 3V ERCLK output clk disabled */
	/* SOSCLPEN=0: Sys OSC disabled in VLP modes */
	/* SOSCSTEN=0: Sys OSC disabled in Stop modes */
	/* SOSCEN=1: Enable oscillator */
	while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)); /* Wait for sys OSC clk valid */
}

void SPLL_init_160MHz(void) {
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
	SCG->SPLLCSR = 0x00000000; /* SPLLEN=0: SPLL is disabled (default) */
	SCG->SPLLDIV = 0x00000302; /* SPLLDIV1 divide by 2; SPLLDIV2 divide by 4 */
	SCG->SPLLCFG = 0x00180000; /* PREDIV=0: Divide SOSC_CLK by 0+1=1 */
	/* MULT=24: Multiply sys pll by 4+24=40 */
	/* SPLL_CLK = 8MHz / 1 * 40 / 2 = 160 MHz */
	SCG->SPLLDIV |= SCG_SPLLDIV_SPLLDIV2(0x1U);
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
	SCG->SPLLCSR = 0x00000001; /* LK=0: SPLLCSR can be written */
	/* SPLLCMRE=0: SPLL CLK monitor IRQ if enabled */
	/* SPLLCM=0: SPLL CLK monitor disabled */
	/* SPLLSTEN=0: SPLL disabled in Stop modes */
	/* SPLLEN=1: Enable SPLL */
	while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)); /* Wait for SPLL valid */
}

void NormalRUNmode_80MHz (void) { /* Change to normal RUN mode with 8MHz SOSC, 80 MHz PLL*/
	SCG->RCCR=SCG_RCCR_SCS(6) /* PLL as clock source*/
	|SCG_RCCR_DIVCORE(1) /* DIVCORE=1, div. by 2: Core clock = 160/2 MHz = 80 MHz*/
	|SCG_RCCR_DIVBUS(1) /* DIVBUS=1, div. by 2: bus clock = 40 MHz*/
	|SCG_RCCR_DIVSLOW(2); /* DIVSLOW=2, div. by 3: SCG slow, flash clock= 26 2/3 MHz*/
	while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT ) != 6) {}
	/* Wait for sys clk src = SPLL */
}

static void ClockPortConfig(void)
{
    PCC_Init(PCC_PORTE_INDEX);
    PCC_Init(PCC_FlexCAN0_INDEX);

    PORT_Config_t CAN0_PortConfig;
    CAN0_PortConfig.mux = PORT_MUX_ALTERNATIVE5;
    CAN0_PortConfig.pin = 5;
    PORT_Init(PORTE, &CAN0_PortConfig);
    CAN0_PortConfig.pin = 4;
    PORT_Init(PORTE, &CAN0_PortConfig);
}

static CAN_Type* getInstance(FlexCAN_Instance_t eInstance)
{
    CAN_Type *psCANInstance = 0;

    switch (eInstance)
    {
        case FLEX_CAN_0:
            psCANInstance = CAN0;
            break;
        case FLEX_CAN_1:
            psCANInstance = CAN1;
            break;
        case FLEX_CAN_2:
            psCANInstance = CAN2;
            break;
        default:
            break;
    }

    return psCANInstance;
}

static void bitTimeConfig(FlexCAN_Instance_t eInstance, FlexCAN_Timing_Config_t* psTimingConfig, FlexCAN_Type_t eCANType)
{
	CAN_Type* psCANInstance = getInstance(eInstance);

	if(CANFD == eCANType)
	{
        psCANInstance->FDCBT =  CAN_FDCBT_FPRESDIV(psTimingConfig->sDataBitRateConfig.u16Prescaler - 1)
                                | CAN_FDCBT_FPSEG1(psTimingConfig->sDataBitRateConfig.u8PhaseSeg1 - 1)
                                | CAN_FDCBT_FPSEG2(psTimingConfig->sDataBitRateConfig.u8PhaseSeg2 - 1)
                                | CAN_FDCBT_FPROPSEG(psTimingConfig->sDataBitRateConfig.u8PropSeg - 1)
                                | CAN_FDCBT_FRJW(psTimingConfig->sDataBitRateConfig.u8ResyncJumpWidth - 1);

        psCANInstance->FDCTRL = CAN_FDCTRL_FDRATE_MASK
                                | CAN_FDCTRL_TDCEN_MASK
                                | CAN_FDCTRL_TDCOFF(psTimingConfig->sDataBitRateConfig.u8TDCOffset);
	}
    else
    {

    }

    psCANInstance->CBT =    CAN_CBT_EPRESDIV(psTimingConfig->sNormalBitRateConfig.u16Prescaler - 1)
                            | CAN_CBT_EPROPSEG(psTimingConfig->sNormalBitRateConfig.u8PropSeg - 1)
                            | CAN_CBT_EPSEG1(psTimingConfig->sNormalBitRateConfig.u8PhaseSeg1 - 1)
                            | CAN_CBT_EPSEG2(psTimingConfig->sNormalBitRateConfig.u8PhaseSeg2 - 1)
                            | CAN_CBT_ERJW(psTimingConfig->sNormalBitRateConfig.u8ResyncJumpWidth - 1)
                            | CAN_CBT_BTF_MASK;
}

static void updateMBSize(uint8_t u8NewMBSize)
{
    u8MBSize = u8NewMBSize;
}

uint8_t enterFreezeMode(FlexCAN_Instance_t eInstance)
{
    CAN_Type *psCANInstance = getInstance(eInstance);
    uint8_t status = 0;

    if (psCANInstance)
    {
        status = 1;
        psCANInstance->MCR |= CAN_MCR_HALT_MASK;
        psCANInstance->MCR |= CAN_MCR_FRZ_MASK;

        if(psCANInstance->MCR & CAN_MCR_MDIS_MASK)
        {
            psCANInstance->MCR &= ~CAN_MCR_MDIS_MASK;
        }
        else
        {
            /* Do nothing */
        }

        while(!(psCANInstance->MCR & CAN_MCR_FRZACK_MASK)) /* !!!!! add timeout later !!!!! */
        {
            /* Wait for freeze mode */
        }

        if(!(psCANInstance->MCR & CAN_MCR_FRZACK_MASK))
        {
            psCANInstance->MCR |= CAN_MCR_SOFTRST_MASK;
        }
        else
        {
            /* Do nothing */
        }

        while(psCANInstance->MCR & CAN_MCR_SOFTRST_MASK)
        {
            /* Wait for soft reset */
        }

        /* Reconfigure the Module Control register and all Interrupt Mask registers */
    }
    else
    {
        /* Do nothing */
    }

    return status;
}

uint8_t exitFreezeMode(FlexCAN_Instance_t eInstance)
{
    CAN_Type *psCANInstance = getInstance(eInstance);
    uint8_t status = 0;

    if (psCANInstance)
    {
        status = 1;
        psCANInstance->MCR &= ~CAN_MCR_HALT_MASK;
        psCANInstance->MCR &= ~CAN_MCR_FRZ_MASK;
    }
    else
    {
        /* Do nothing */
    }

    return status;
}

uint8_t CAN_Init(FlexCAN_Instance_t eInstance, FlexCAN_Config_t *psFlexCANConfig)
{
    CAN_Type* psCANInstance = getInstance(eInstance);
    uint8_t status = 0;
    uint8_t u8MBIndex;

    if (psCANInstance)
    {
        status = 1;
        enterFreezeMode(eInstance);

        psCANInstance->MCR &= ~CAN_MCR_SUPV_MASK;
        bitTimeConfig(eInstance, &psFlexCANConfig->sTimingConfig, psFlexCANConfig->eCANType);

		psCANInstance->MCR |= CAN_MCR_MDIS(1);
		while(!(psCANInstance->MCR & CAN_MCR_LPMACK_MASK));

        psCANInstance->CTRL1 &= ~CAN_CTRL1_CLKSRC(1U);
        psCANInstance->CTRL1 |= CAN_CTRL1_CLKSRC(psFlexCANConfig->eClkSrc);

        psCANInstance->MCR &= ~CAN_MCR_MDIS(1);
        while(psCANInstance->MCR & CAN_MCR_LPMACK_MASK);

        psCANInstance->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;

        if(CANFD == psFlexCANConfig->eCANType)
        {
            psCANInstance->MCR |= CAN_MCR_FDEN_MASK;
            switch (psFlexCANConfig->eMaxDataLength)
            {
                case MAX_8_BYTES:
                    psCANInstance->FDCTRL |= CAN_FDCTRL_MBDSR0(0U);
                    updateMBSize(4U);
                    break;
                case MAX_16_BYTES:
                    psCANInstance->FDCTRL |= CAN_FDCTRL_MBDSR0(1U);
                    updateMBSize(6U);
                    break;
                case MAX_32_BYTES:
                    psCANInstance->FDCTRL |= CAN_FDCTRL_MBDSR0(2U);
                    updateMBSize(10U);
                    break;
                case MAX_64_BYTES:
                    psCANInstance->FDCTRL |= CAN_FDCTRL_MBDSR0(3U);
                    updateMBSize(18U);
                    break;
                default:
                    break;
            }
        }
        else
        {
            /* Do nothing */
        }

        /* Initialize the message buffers. */
        psCANInstance->RXMGMASK = 0x1FFFFFFF; /* Global acceptance mask: check all ID bits */

        for(u8MBIndex = 0; u8MBIndex < 0x80U; u8MBIndex++ )
        {
            psCANInstance->RAMn[u8MBIndex] = 0; /* Clear msg buf word */
        }

        if(1U == psFlexCANConfig->u8LoopBackMode)
        {
			psCANInstance->CTRL1 |= CAN_CTRL1_LPB_MASK;
			psCANInstance->FDCTRL &= ~CAN_FDCTRL_TDCEN_MASK;
        }
        else
        {

        }

    	for(int i=0; i<128; i++ ) { /* CAN0: clear 128 words RAM in FlexCAN 0 */
    		CAN0->RAMn[i] = 0; /* Clear msg buf words. All buffers CODE=0 (inactive) */
    	}
    	for(int i=0; i<16; i++ ) { /* In FRZ mode, init CAN0 16 msg buf filters */
    		CAN0->RXIMR[i] = 0xFFFFFFFF; /* Check all ID bits for incoming messages */
    	}
    	CAN0->RXMGMASK = 0x1FFFFFFF;
        /* Negate MCR[HALT]. */
        exitFreezeMode(eInstance);
    }
    else
    {
        /* Do nothing */
    }

	return status;
}

uint8_t CAN_ReceiveMBRegister(FlexCAN_Instance_t eInstance, FlexCAN_Tx_Rx_Frame_t* sFrame)
{
    CAN_Type *psCANInstance = getInstance(eInstance);
    uint8_t u8MBIndex = 0;

    while
    (
        (RX_MB_INACTIVE != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (TX_MB_INACTIVE != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (TX_MB_ABORT != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (u8MBIndex < 32)
    )
    {
        u8MBIndex++;
    }

    if(32U > u8MBIndex)
    {
        psCANInstance->RAMn[u8MBIndex * u8MBSize + 1U] =    (sFrame->sID.u8Priority << 29)
                                                            | (sFrame->sID.u16StandardID << 18U)
                                                            | sFrame->sID.u32ExtendedID;
        psCANInstance->RAMn[u8MBIndex * u8MBSize] = (sFrame->u8ExtendedDataLength << 31U)
                                                    | (sFrame->u8BitRateSwitch << 30U)
                                                    | (sFrame->u8ErrorStateIndicator << 29U)
													| 4U << 24U
                                                    | (sFrame->u8SubstituteRemoteRequest << 22U)
                                                    | (sFrame->u8ExtendedFormat << 21U)
                                                    | (sFrame->u8RemoteTransmitRequest << 20U);
    }
    else
    {

    }

    psCANInstance->RAMn[u8MBIndex * u8MBSize] |= (RX_MB_EMPTY << 24U);

    return u8MBIndex;
}

void CAN_Transmit(FlexCAN_Instance_t eInstance, FlexCAN_Tx_Rx_Frame_t* DataFrame)
{
    CAN_Type *psCANInstance = getInstance(eInstance);
    uint8_t u8MBIndex = 0;
    uint8_t u8PayloadIndex;

    while
    (
        (RX_MB_INACTIVE != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (TX_MB_INACTIVE != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (TX_MB_ABORT != ((psCANInstance->RAMn[u8MBIndex * u8MBSize]) & 0x0F000000))
        && (u8MBIndex < 32)
    )
    {
        u8MBIndex++;
    }

    if(u8MBIndex < 32U)
    {

        psCANInstance->RAMn[u8MBIndex * u8MBSize + 1U] =    (DataFrame->sID.u8Priority << 29U)
                                                            | (DataFrame->sID.u16StandardID << 18U)
                                                            | (DataFrame->sID.u32ExtendedID);

        for(u8PayloadIndex = 0; u8PayloadIndex < DataFrame->u8PayloadLength; u8PayloadIndex++)
        {
            psCANInstance->RAMn[u8MBIndex * u8MBSize + (u8PayloadIndex >> 2) + 2] |= (DataFrame->pu8PayloadBuffer[u8PayloadIndex] << ((3 - (u8PayloadIndex % 4)) * 8));
        }

        psCANInstance->RAMn[u8MBIndex * u8MBSize + 0U]  |=  (DataFrame->u8ExtendedDataLength << 31U)
                                                            | (DataFrame->u8BitRateSwitch << 30U)
                                                            | (DataFrame->u8ErrorStateIndicator << 29U)
                                                            | (DataFrame->u8SubstituteRemoteRequest << 22U)
                                                            | (DataFrame->u8ExtendedFormat << 21U)
                                                            | (DataFrame->u8RemoteTransmitRequest << 20U)
                                                            | (DataFrame->u8DataLengthCode << 16U)
                                                            | (DataFrame->u16TimeStamp);

        psCANInstance->RAMn[u8MBIndex * u8MBSize + 0U] |= (0xCU << 24U);
    }
    else
    {
        /* Do nothing */
    }
}

void CAN_Receive(FlexCAN_Instance_t eInstance, FlexCAN_Tx_Rx_Frame_t* sRxFrame, uint8_t u8MBIndex)
{
    CAN_Type *psCANInstance = getInstance(eInstance);
    uint8_t u8PayloadIndex;

    sRxFrame->sID.u32ExtendedID = (psCANInstance->RAMn[u8MBIndex * u8MBSize + 1U] & 0x3FFFFU);
    sRxFrame->sID.u16StandardID = (psCANInstance->RAMn[u8MBIndex * u8MBSize + 1U] & 0x1FFC0000U) >> 18;
    sRxFrame->u8DataLengthCode = (psCANInstance->RAMn[u8MBIndex * u8MBSize] & 0xF0000U) >> 16;

    for(u8PayloadIndex = 0; u8PayloadIndex < sRxFrame->u8PayloadLength; u8PayloadIndex++)
	{
		 sRxFrame->pu8PayloadBuffer[u8PayloadIndex] = (psCANInstance->RAMn[u8MBIndex * u8MBSize + (u8PayloadIndex >> 2) + 2] >> ((3 - (u8PayloadIndex % 4)) * 8)) & 0xFF;
	}

    psCANInstance->IFLAG1 = (1 << u8MBIndex);
}

int main(void)
{
    FlexCAN_Config_t psFlexCan0Config;

	SOSC_init_8MHz();
	SPLL_init_160MHz();
	NormalRUNmode_80MHz();
    ClockPortConfig();
    UART1Config();

    psFlexCan0Config.eCANType = CANFD;
    psFlexCan0Config.eClkSrc = FLEX_CAN_CLK_SRC_SYS_CLK;

    psFlexCan0Config.sTimingConfig.sNormalBitRateConfig.u16Prescaler = 2;
    psFlexCan0Config.sTimingConfig.sNormalBitRateConfig.u8PhaseSeg1 = 16;
	psFlexCan0Config.sTimingConfig.sNormalBitRateConfig.u8PhaseSeg2 = 16;
	psFlexCan0Config.sTimingConfig.sNormalBitRateConfig.u8PropSeg = 47;
	psFlexCan0Config.sTimingConfig.sNormalBitRateConfig.u8ResyncJumpWidth = 16;

	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u16Prescaler = 2;
	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u8PhaseSeg1 = 8;
	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u8PhaseSeg2 = 4;
	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u8PropSeg = 8;
	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u8ResyncJumpWidth = 4;
	psFlexCan0Config.sTimingConfig.sDataBitRateConfig.u8TDCOffset = 5;

    psFlexCan0Config.u8InternalArbitration = LOWEST_BUFFER_TRANSMITTED_FIRST;
    psFlexCan0Config.eMaxDataLength = MAX_16_BYTES;
    psFlexCan0Config.u8LoopBackMode = 0;
    CAN_Init(FLEX_CAN_0, &psFlexCan0Config);
//    FLEXCAN0_init();

#ifdef NODE_TX
    sTxFrame.sID.u32ExtendedID = 0x00000000;
    sTxFrame.sID.u16StandardID = 0x123;
    sTxFrame.sID.u8Priority = 1;
    sTxFrame.u16TimeStamp = 0x0000;
    sTxFrame.u8ExtendedDataLength = 1;
    sTxFrame.u8BitRateSwitch = 1;
    sTxFrame.u8ErrorStateIndicator = 0;
    sTxFrame.u8SubstituteRemoteRequest = 1;
    sTxFrame.u8ExtendedFormat = 0;
    sTxFrame.u8RemoteTransmitRequest = 0;
    sTxFrame.u8DataLengthCode = DATA_BYTE_0_TO_15;
    sTxFrame.pu8PayloadBuffer = u8UART_Buffer;
    sTxFrame.u8PayloadLength = 13;
#else
    sRxFrame.sID.u32ExtendedID = 0x00000000;
    sRxFrame.sID.u16StandardID = 0x123;
    sRxFrame.sID.u8Priority = 0;
    sRxFrame.u16TimeStamp = 0x0000;
    sRxFrame.u8ExtendedDataLength = 1;
    sRxFrame.u8BitRateSwitch = 1;
    sRxFrame.u8ErrorStateIndicator = 0;
    sRxFrame.u8SubstituteRemoteRequest = 0;
    sRxFrame.u8ExtendedFormat = 0;
    sRxFrame.u8RemoteTransmitRequest = 0;
    sRxFrame.pu8PayloadBuffer = u8UART_Buffer;
    sRxFrame.u8PayloadLength = 13;
    u8MBIndex = CAN_ReceiveMBRegister(FLEX_CAN_0, &sRxFrame);

    while(0 == ((CAN0->IFLAG1 >> u8MBIndex) & 1U))
    {

    }

	CAN_Receive(FLEX_CAN_0, &sRxFrame, u8MBIndex);

	LPUART_TxMultiFrame7_8(LPUART1, u8UART_Buffer, 13);
#endif
    return 0;
}
