/*
// TI File $Revision: /main/3 $
// Checkin $Date: Agu 1, 2017   13:45:43 $
//
// FILE:    F280049_flash_CPU.cmd
//
// TITLE:   Linker Command File For F280049 examples that run out of Flash
//
//
//          Keep in mind that L0,L1,L2,L3 and L4 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
*/

/*========================================================= */
/* Define the memory block start/length for the F2806x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F280049 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to Flash" bootloader mode   */
   BEGIN           	: origin = 0x080000, length = 0x000002
   RESET            : origin = 0x3FFFC0, length = 0x000002

   patch_EST_Angle_run_patchable_address           : origin = 0x009000, length = 0x00000e
   patch_EST_Dir_run_patchable_address             : origin = 0x00900e, length = 0x00000e
   patch_EST_Eab_run_patchable_address             : origin = 0x00901c, length = 0x00000e
   patch_EST_Flux_ab_estFluxDot_patchable_address  : origin = 0x00902a, length = 0x00000e
   patch_EST_Flux_dq_run_patchable_address         : origin = 0x009038, length = 0x00000e
   patch_EST_Flux_run_patchable_address            : origin = 0x009046, length = 0x00000e
   patch_EST_Freq_run_patchable_address            : origin = 0x009054, length = 0x00000e
   patch_EST_Iab_run_patchable_address             : origin = 0x009062, length = 0x00000e
   patch_EST_Idq_run_patchable_address             : origin = 0x009070, length = 0x00000e
   patch_EST_Ls_run_patchable_address              : origin = 0x00907e, length = 0x00000e
   patch_EST_OneOverDcBus_run_patchable_address    : origin = 0x00908c, length = 0x00000e
   patch_EST_Rr_run_patchable_address              : origin = 0x00909a, length = 0x00000e
   patch_EST_RsOnLine_run_patchable_address        : origin = 0x0090a8, length = 0x00000e
   patch_EST_Rs_run_patchable_address              : origin = 0x0090b6, length = 0x00000e
   patch_EST_Vab_run_patchable_address             : origin = 0x0090c4, length = 0x00000e
   patch_EST_Vdq_run_patchable_address             : origin = 0x0090d2, length = 0x00000e
   patch_EST_runEst_patchable_address              : origin = 0x0090e0, length = 0x00000e

   RAMGS1_3         : origin = 0x00E000, length = 0x006000
   RAMLS4_7         : origin = 0x00A000, length = 0x002000

   FLASHB0_SA		: origin = 0x080002, length = 0x00FFFE		/* on-chip Flash */
   FLASHB1_SA   	: origin = 0x090000, length = 0x010000		/* on-chip Flash */


PAGE 1 :
   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3      /* Part of M0, BOOT rom will use this for stack */
   RAMM0           	: origin = 0x0000F5, length = 0x00030B
   RAMM1           	: origin = 0x000400, length = 0x000400

/* CLA1             : origin = 0x001400, length = 0x000080 */  /* Defined in header cmd file */

   RAMGS0_A         : origin = 0x00C000, length = 0x002000

   RAMLS0_1         : origin = 0x008000, length = 0x001000		/* Can't be used, reserved for FAST object */
   RAMLS2_3         : origin = 0x009100, length = 0x000F00		/* */

   CLA1MSGRAMLOW    : origin = 0x001480, length = 0x000080
   CLA1MSGRAMHIGH   : origin = 0x001500, length = 0x000080

}

SECTIONS
{
   .TI.ramfunc      : LOAD = FLASHB0_SA,
                      RUN = RAMLS4_7,
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_SIZE(_RamfuncsLoadSize),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      RUN_SIZE(_RamfuncsRunSize),
                      RUN_END(_RamfuncsRunEnd),
                      PAGE = 0, ALIGN(4)


   codestart        : > BEGIN,     		 PAGE = 0, ALIGN(4)
   .text            : > FLASHB0_SA,      PAGE = 0, ALIGN(4)
   .cinit           : > FLASHB0_SA,      PAGE = 0, ALIGN(4)
   .pinit           : > FLASHB0_SA,      PAGE = 0, ALIGN(4)
   .switch          : > FLASHB0_SA,      PAGE = 0, ALIGN(4)
   .econst          : > FLASHB0_SA,      PAGE = 0, ALIGN(4)

   .reset           : > RESET,           PAGE = 0, TYPE = DSECT
   .cio             : > RAMLS4_7,        PAGE = 0
   .stack           : > RAMM0,           PAGE = 1
   .ebss            : > RAMGS0_A,        PAGE = 1
   .esysmem         : > RAMGS0_A,        PAGE = 1

/* Cla1RegsFile     : > CLA1,            PAGE = 1	*/ /* Defined in header .cmd file*/

   .bss_cla         : > RAMLS2_3,        PAGE = 1

   Cla1Prog         : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      LOAD_START(_Cla1ProgLoadStart),
                      RUN_START(_Cla1ProgRunStart),
                      LOAD_SIZE(_Cla1ProgLoadSize),
                      PAGE = 0, ALIGN(4)

   Cla1Prog2        : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      LOAD_START(_Cla1Prog2LoadStart),
                      RUN_START(_Cla1Prog2RunStart),
                      LOAD_SIZE(_Cla1Prog2LoadSize),
                      PAGE = 0, ALIGN(4)

   Cla1ToCpuMsgRAM  : > CLA1MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1MSGRAMHIGH,  PAGE = 1

   .const_cla       : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      RUN_START(_Cla1ConstRunStart),
                      LOAD_START(_Cla1ConstLoadStart),
                      LOAD_SIZE(_Cla1ConstLoadSize),
                      PAGE = 0

   .scratchpad      : > RAMLS2_3,        PAGE = 1

   patch_EST_Angle_run_patchable_section           : >  patch_EST_Angle_run_patchable_address,           PAGE = 0
   patch_EST_Dir_run_patchable_section             : >  patch_EST_Dir_run_patchable_address,             PAGE = 0
   patch_EST_Eab_run_patchable_section             : >  patch_EST_Eab_run_patchable_address,             PAGE = 0
   patch_EST_Flux_ab_estFluxDot_patchable_section  : >  patch_EST_Flux_ab_estFluxDot_patchable_address,  PAGE = 0
   patch_EST_Flux_dq_run_patchable_section         : >  patch_EST_Flux_dq_run_patchable_address,         PAGE = 0
   patch_EST_Flux_run_patchable_section            : >  patch_EST_Flux_run_patchable_address,            PAGE = 0
   patch_EST_Freq_run_patchable_section            : >  patch_EST_Freq_run_patchable_address,            PAGE = 0
   patch_EST_Iab_run_patchable_section             : >  patch_EST_Iab_run_patchable_address,             PAGE = 0
   patch_EST_Idq_run_patchable_section             : >  patch_EST_Idq_run_patchable_address,             PAGE = 0
   patch_EST_Ls_run_patchable_section              : >  patch_EST_Ls_run_patchable_address,              PAGE = 0
   patch_EST_OneOverDcBus_run_patchable_section    : >  patch_EST_OneOverDcBus_run_patchable_address,    PAGE = 0
   patch_EST_Rr_run_patchable_section              : >  patch_EST_Rr_run_patchable_address,              PAGE = 0
   patch_EST_RsOnLine_run_patchable_section        : >  patch_EST_RsOnLine_run_patchable_address,        PAGE = 0
   patch_EST_Rs_run_patchable_section              : >  patch_EST_Rs_run_patchable_address,              PAGE = 0
   patch_EST_Vab_run_patchable_section             : >  patch_EST_Vab_run_patchable_address,             PAGE = 0
   patch_EST_Vdq_run_patchable_section             : >  patch_EST_Vdq_run_patchable_address,             PAGE = 0
   patch_EST_runEst_patchable_section              : >  patch_EST_runEst_patchable_address,              PAGE = 0
}

SECTIONS
{
	sysctrl_data	: > RAMM1 | RAMLS2_3,   PAGE = 1
	ctrl_data		: > RAMM1 | RAMLS2_3,   PAGE = 1
	est_data		: > RAMGS0_A,           PAGE = 1

}

SECTIONS
{
	datalog_data	: > RAMGS0_A,           PAGE = 1
	graph_data		: > RAMGS0_A,           PAGE = 1
}
