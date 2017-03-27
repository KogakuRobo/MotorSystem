/************************************************************************
*
* Device     : RX/RX600/RX62T
*
* File Name  : vect.h
*
* Abstract   : Definition of Vector.
*
* History    : 1.00  (2010-06-08) [Hardware Manual Revision : 0.20]
*            : 1.01  (2011-06-20) [Hardware Manual Revision : 1.00]
*            : 1.10  (2012-06-18) [Hardware Manual Revision : 1.31]
*            : 1.20  (2013-02-18) [Hardware Manual Revision : 1.31]
*            : 1.20a (2013-02-18) [Hardware Manual Revision : 1.31]
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2013 (2010 -2012) Renesas Electronics Corporation. and
* Renesas Solutions Corp. All rights reserved.
*
************************************************************************/

// Exception(Supervisor Instruction)
#pragma interrupt (Excep_SuperVisorInst)
void Excep_SuperVisorInst(void);

// Exception(Access Instruction)
#pragma interrupt (Excep_AccessInst)
void Excep_AccessInst(void);

// Exception(Undefined Instruction)
#pragma interrupt (Excep_UndefinedInst)
void Excep_UndefinedInst(void);

// Exception(Floating Point)
#pragma interrupt (Excep_FloatingPoint)
void Excep_FloatingPoint(void);

// NMI
#pragma interrupt (NonMaskableInterrupt)
void NonMaskableInterrupt(void);

// Dummy
#pragma interrupt (Dummy)
void Dummy(void);

// BRK
#pragma interrupt (Excep_BRK(vect=0))
void Excep_BRK(void);

// vector  1 reserved
// vector  2 reserved
// vector  3 reserved
// vector  4 reserved
// vector  5 reserved
// vector  6 reserved
// vector  7 reserved
// vector  8 reserved
// vector  9 reserved
// vector 10 reserved
// vector 11 reserved
// vector 12 reserved
// vector 13 reserved
// vector 14 reserved
// vector 15 reserved

// BSC BUSERR
#pragma interrupt (Excep_BSC_BUSERR(vect=16))
void Excep_BSC_BUSERR(void);

// FCUIF FIFERR
#pragma interrupt (Excep_FCUIF_FIFERR(vect=21))
void Excep_FCUIF_FIFERR(void);

// FCUIF FRDYI
#pragma interrupt (Excep_FCUIF_FRDYI(vect=23))
void Excep_FCUIF_FRDYI(void);

// ICU SWINT
#pragma interrupt (Excep_ICU_SWINT(vect=27))
void Excep_ICU_SWINT(void);

// CMT0 CMI0
//#pragma interrupt (Excep_CMT0_CMI0(vect=28))
void Excep_CMT0_CMI0(void);

// CMT1 CMI1
#pragma interrupt (Excep_CMT1_CMI1(vect=29))
void Excep_CMT1_CMI1(void);

// CMT2 CMI2
#pragma interrupt (Excep_CMT2_CMI2(vect=30))
void Excep_CMT2_CMI2(void);

// CMT3 CMI3
#pragma interrupt (Excep_CMT3_CMI3(vect=31))
void Excep_CMT3_CMI3(void);

// RSPI0 SPEI0
#pragma interrupt (Excep_RSPI0_SPEI0(vect=44))
void Excep_RSPI0_SPEI0(void);

// RSPI0 SPRI0
#pragma interrupt (Excep_RSPI0_SPRI0(vect=45))
void Excep_RSPI0_SPRI0(void);

// RSPI0 SPTI0
#pragma interrupt (Excep_RSPI0_SPTI0(vect=46))
void Excep_RSPI0_SPTI0(void);

// RSPI0 SPII0
#pragma interrupt (Excep_RSPI0_SPII0(vect=47))
void Excep_RSPI0_SPII0(void);

// CAN0 ERS0
#pragma interrupt (Excep_CAN0_ERS0(vect=56))
void Excep_CAN0_ERS0(void);

// CAN0 RXF0
#pragma interrupt (Excep_CAN0_RXF0(vect=57))
void Excep_CAN0_RXF0(void);

// CAN0 TXF0
#pragma interrupt (Excep_CAN0_TXF0(vect=58))
void Excep_CAN0_TXF0(void);

// CAN0 RXM0
//#pragma interrupt (Excep_CAN0_RXM0(vect=59))
void Excep_CAN0_RXM0(void);

// CAN0 TXM0
//#pragma interrupt (Excep_CAN0_TXM0(vect=60))
void Excep_CAN0_TXM0(void);

// ICU IRQ0
#pragma interrupt (Excep_ICU_IRQ0(vect=64))
void Excep_ICU_IRQ0(void);

// ICU IRQ1
#pragma interrupt (Excep_ICU_IRQ1(vect=65))
void Excep_ICU_IRQ1(void);

// ICU IRQ2
#pragma interrupt (Excep_ICU_IRQ2(vect=66))
void Excep_ICU_IRQ2(void);

// ICU IRQ3
#pragma interrupt (Excep_ICU_IRQ3(vect=67))
void Excep_ICU_IRQ3(void);

// ICU IRQ4
#pragma interrupt (Excep_ICU_IRQ4(vect=68))
void Excep_ICU_IRQ4(void);

// ICU IRQ5
#pragma interrupt (Excep_ICU_IRQ5(vect=69))
void Excep_ICU_IRQ5(void);

// ICU IRQ6
#pragma interrupt (Excep_ICU_IRQ6(vect=70))
void Excep_ICU_IRQ6(void);

// ICU IRQ7
#pragma interrupt (Excep_ICU_IRQ7(vect=71))
void Excep_ICU_IRQ7(void);

// WDT WOVI
#pragma interrupt (Excep_WDT_WOVI(vect=96))
void Excep_WDT_WOVI(void);

// AD0 ADI0
#pragma interrupt (Excep_AD0_ADI0(vect=98))
void Excep_AD0_ADI0(void);

// S12AD0 S12ADI0
//#pragma interrupt (Excep_S12AD0_S12ADI0(vect=102))
void Excep_S12AD0_S12ADI0(void);

// S12AD1 S12ADI1
#pragma interrupt (Excep_S12AD1_S12ADI1(vect=103))
void Excep_S12AD1_S12ADI1(void);

// CMPB CMPI
#pragma interrupt (Excep_CMPB_CMPI(vect=106))
void Excep_CMPB_CMPI(void);

// MTU0 TGIA0
//#pragma interrupt (Excep_MTU0_TGIA0(vect=114))
void Excep_MTU0_TGIA0(void);

// MTU0 TGIB0
#pragma interrupt (Excep_MTU0_TGIB0(vect=115))
void Excep_MTU0_TGIB0(void);

// MTU0 TGIC0
//#pragma interrupt (Excep_MTU0_TGIC0(vect=116))
void Excep_MTU0_TGIC0(void);

// MTU0 TGID0
#pragma interrupt (Excep_MTU0_TGID0(vect=117))
void Excep_MTU0_TGID0(void);

// MTU0 TCIV0
#pragma interrupt (Excep_MTU0_TCIV0(vect=118))
void Excep_MTU0_TCIV0(void);

// MTU0 TGIE0
#pragma interrupt (Excep_MTU0_TGIE0(vect=119))
void Excep_MTU0_TGIE0(void);

// MTU0 TGIF0
#pragma interrupt (Excep_MTU0_TGIF0(vect=120))
void Excep_MTU0_TGIF0(void);

// MTU1 TGIA1
#pragma interrupt (Excep_MTU1_TGIA1(vect=121))
void Excep_MTU1_TGIA1(void);

// MTU1 TGIB1
#pragma interrupt (Excep_MTU1_TGIB1(vect=122))
void Excep_MTU1_TGIB1(void);

// MTU1 TCIV1
//#pragma interrupt (Excep_MTU1_TCIV1(vect=123))
void Excep_MTU1_TCIV1(void);

// MTU1 TCIU1
//#pragma interrupt (Excep_MTU1_TCIU1(vect=124))
void Excep_MTU1_TCIU1(void);

// MTU2 TGIA2
//#pragma interrupt (Excep_MTU2_TGIA2(vect=125))
void Excep_MTU2_TGIA2(void);

// MTU2 TGIB2
#pragma interrupt (Excep_MTU2_TGIB2(vect=126))
void Excep_MTU2_TGIB2(void);

// MTU2 TCIV2
#pragma interrupt (Excep_MTU2_TCIV2(vect=127))
void Excep_MTU2_TCIV2(void);

// MTU2 TCIU2
#pragma interrupt (Excep_MTU2_TCIU2(vect=128))
void Excep_MTU2_TCIU2(void);

// MTU3 TGIA3
#pragma interrupt (Excep_MTU3_TGIA3(vect=129))
void Excep_MTU3_TGIA3(void);

// MTU3 TGIB3
#pragma interrupt (Excep_MTU3_TGIB3(vect=130))
void Excep_MTU3_TGIB3(void);

// MTU3 TGIC3
#pragma interrupt (Excep_MTU3_TGIC3(vect=131))
void Excep_MTU3_TGIC3(void);

// MTU3 TGID3
#pragma interrupt (Excep_MTU3_TGID3(vect=132))
void Excep_MTU3_TGID3(void);

// MTU3 TCIV3
#pragma interrupt (Excep_MTU3_TCIV3(vect=133))
void Excep_MTU3_TCIV3(void);

// MTU4 TGIA4
#pragma interrupt (Excep_MTU4_TGIA4(vect=134))
void Excep_MTU4_TGIA4(void);

// MTU4 TGIB4
#pragma interrupt (Excep_MTU4_TGIB4(vect=135))
void Excep_MTU4_TGIB4(void);

// MTU4 TGIC4
#pragma interrupt (Excep_MTU4_TGIC4(vect=136))
void Excep_MTU4_TGIC4(void);

// MTU4 TGID4
#pragma interrupt (Excep_MTU4_TGID4(vect=137))
void Excep_MTU4_TGID4(void);

// MTU4 TCIV4
#pragma interrupt (Excep_MTU4_TCIV4(vect=138))
void Excep_MTU4_TCIV4(void);

// MTU5 TGIU5
#pragma interrupt (Excep_MTU5_TGIU5(vect=139))
void Excep_MTU5_TGIU5(void);

// MTU5 TGIV5
#pragma interrupt (Excep_MTU5_TGIV5(vect=140))
void Excep_MTU5_TGIV5(void);

// MTU5 TGIW5
#pragma interrupt (Excep_MTU5_TGIW5(vect=141))
void Excep_MTU5_TGIW5(void);

// MTU6 TGIA6
#pragma interrupt (Excep_MTU6_TGIA6(vect=142))
void Excep_MTU6_TGIA6(void);

// MTU6 TGIB6
#pragma interrupt (Excep_MTU6_TGIB6(vect=143))
void Excep_MTU6_TGIB6(void);

// MTU6 TGIC6
#pragma interrupt (Excep_MTU6_TGIC6(vect=144))
void Excep_MTU6_TGIC6(void);

// MTU6 TGID6
#pragma interrupt (Excep_MTU6_TGID6(vect=145))
void Excep_MTU6_TGID6(void);

// MTU6 TCIV6
#pragma interrupt (Excep_MTU6_TCIV6(vect=146))
void Excep_MTU6_TCIV6(void);

// MTU7 TGIA7
#pragma interrupt (Excep_MTU7_TGIA7(vect=149))
void Excep_MTU7_TGIA7(void);

// MTU7 TGIB7
#pragma interrupt (Excep_MTU7_TGIB7(vect=150))
void Excep_MTU7_TGIB7(void);

// MTU7 TGIC7
#pragma interrupt (Excep_MTU7_TGIC7(vect=151))
void Excep_MTU7_TGIC7(void);

// MTU7 TGID7
#pragma interrupt (Excep_MTU7_TGID7(vect=152))
void Excep_MTU7_TGID7(void);

// MTU7 TCIV7
#pragma interrupt (Excep_MTU7_TCIV7(vect=153))
void Excep_MTU7_TCIV7(void);

// POE OEI1
#pragma interrupt (Excep_POE_OEI1(vect=170))
void Excep_POE_OEI1(void);

// POE OEI2
#pragma interrupt (Excep_POE_OEI2(vect=171))
void Excep_POE_OEI2(void);

// POE OEI3
#pragma interrupt (Excep_POE_OEI3(vect=172))
void Excep_POE_OEI3(void);

// POE OEI4
#pragma interrupt (Excep_POE_OEI4(vect=173))
void Excep_POE_OEI4(void);

// GPT0 GTCIA0
#pragma interrupt (Excep_GPT0_GTCIA0(vect=174))
void Excep_GPT0_GTCIA0(void);

// GPT0 GTCIB0
#pragma interrupt (Excep_GPT0_GTCIB0(vect=175))
void Excep_GPT0_GTCIB0(void);

// GPT0 GTCIC0
#pragma interrupt (Excep_GPT0_GTCIC0(vect=176))
void Excep_GPT0_GTCIC0(void);

// GPT0 GTCIE0
#pragma interrupt (Excep_GPT0_GTCIE0(vect=177))
void Excep_GPT0_GTCIE0(void);

// GPT0 GTCIV0
#pragma interrupt (Excep_GPT0_GTCIV0(vect=178))
void Excep_GPT0_GTCIV0(void);

// GPT0 LOCO1
#pragma interrupt (Excep_GPT0_LOCO1(vect=179))
void Excep_GPT0_LOCO1(void);

// GPT1 GTCIA1
#pragma interrupt (Excep_GPT1_GTCIA1(vect=180))
void Excep_GPT1_GTCIA1(void);

// GPT1 GTCIB1
#pragma interrupt (Excep_GPT1_GTCIB1(vect=181))
void Excep_GPT1_GTCIB1(void);

// GPT1 GTCIC1
#pragma interrupt (Excep_GPT1_GTCIC1(vect=182))
void Excep_GPT1_GTCIC1(void);

// GPT1 GTCIE1
#pragma interrupt (Excep_GPT1_GTCIE1(vect=183))
void Excep_GPT1_GTCIE1(void);

// GPT1 GTCIV1
#pragma interrupt (Excep_GPT1_GTCIV1(vect=184))
void Excep_GPT1_GTCIV1(void);

// GPT2 GTCIA2
#pragma interrupt (Excep_GPT2_GTCIA2(vect=186))
void Excep_GPT2_GTCIA2(void);

// GPT2 GTCIB2
#pragma interrupt (Excep_GPT2_GTCIB2(vect=187))
void Excep_GPT2_GTCIB2(void);

// GPT2 GTCIC2
#pragma interrupt (Excep_GPT2_GTCIC2(vect=188))
void Excep_GPT2_GTCIC2(void);

// GPT2 GTCIE2
#pragma interrupt (Excep_GPT2_GTCIE2(vect=189))
void Excep_GPT2_GTCIE2(void);

// GPT2 GTCIV2
#pragma interrupt (Excep_GPT2_GTCIV2(vect=190))
void Excep_GPT2_GTCIV2(void);

// GPT3 GTCIA3
#pragma interrupt (Excep_GPT3_GTCIA3(vect=192))
void Excep_GPT3_GTCIA3(void);

// GPT3 GTCIB3
#pragma interrupt (Excep_GPT3_GTCIB3(vect=193))
void Excep_GPT3_GTCIB3(void);

// GPT3 GTCIC3
#pragma interrupt (Excep_GPT3_GTCIC3(vect=194))
void Excep_GPT3_GTCIC3(void);

// GPT3 GTCIE3
#pragma interrupt (Excep_GPT3_GTCIE3(vect=195))
void Excep_GPT3_GTCIE3(void);

// GPT3 GTCIV3
#pragma interrupt (Excep_GPT3_GTCIV3(vect=196))
void Excep_GPT3_GTCIV3(void);

// SCI0 ERI0
#pragma interrupt (Excep_SCI0_ERI0(vect=214))
void Excep_SCI0_ERI0(void);

// SCI0 RXI0
#pragma interrupt (Excep_SCI0_RXI0(vect=215))
void Excep_SCI0_RXI0(void);

// SCI0 TXI0
#pragma interrupt (Excep_SCI0_TXI0(vect=216))
void Excep_SCI0_TXI0(void);

// SCI0 TEI0
#pragma interrupt (Excep_SCI0_TEI0(vect=217))
void Excep_SCI0_TEI0(void);

// SCI1 ERI1
#pragma interrupt (Excep_SCI1_ERI1(vect=218))
void Excep_SCI1_ERI1(void);

// SCI1 RXI1
#pragma interrupt (Excep_SCI1_RXI1(vect=219))
void Excep_SCI1_RXI1(void);

// SCI1 TXI1
#pragma interrupt (Excep_SCI1_TXI1(vect=220))
void Excep_SCI1_TXI1(void);

// SCI1 TEI1
#pragma interrupt (Excep_SCI1_TEI1(vect=221))
void Excep_SCI1_TEI1(void);

// SCI2 ERI2
#pragma interrupt (Excep_SCI2_ERI2(vect=222))
void Excep_SCI2_ERI2(void);

// SCI2 RXI2
#pragma interrupt (Excep_SCI2_RXI2(vect=223))
void Excep_SCI2_RXI2(void);

// SCI2 TXI2
#pragma interrupt (Excep_SCI2_TXI2(vect=224))
void Excep_SCI2_TXI2(void);

// SCI2 TEI2
#pragma interrupt (Excep_SCI2_TEI2(vect=225))
void Excep_SCI2_TEI2(void);

// RIIC0 ICEEI0
#pragma interrupt (Excep_RIIC0_ICEEI0(vect=246))
void Excep_RIIC0_ICEEI0(void);

// RIIC0 ICRXI0
#pragma interrupt (Excep_RIIC0_ICRXI0(vect=247))
void Excep_RIIC0_ICRXI0(void);

// RIIC0 ICTXI0
#pragma interrupt (Excep_RIIC0_ICTXI0(vect=248))
void Excep_RIIC0_ICTXI0(void);

// RIIC0 ICTEI0
#pragma interrupt (Excep_RIIC0_ICTEI0(vect=249))
void Excep_RIIC0_ICTEI0(void);

// LIN0 LIN0
#pragma interrupt (Excep_LIN0_LIN0(vect=254))
void Excep_LIN0_LIN0(void);


//;<<VECTOR DATA START (POWER ON RESET)>>
//;Power On Reset PC
extern void PowerON_Reset_PC(void);                                                                                                                
//;<<VECTOR DATA END (POWER ON RESET)>>

