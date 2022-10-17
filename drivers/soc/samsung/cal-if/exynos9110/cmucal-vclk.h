#ifndef __CMUCAL_VCLK_H__
#define __CMUCAL_VCLK_H__

#include "../cmucal.h"

/*=================CMUCAL version: S5E9110================================*/

enum vclk_id {
/* DVFS TYPE */
	VCLK_VDD_ALIVE = DFS_VCLK_TYPE,
	VCLK_VDD_INT,
	end_of_dfs_vclk,
	num_of_dfs_vclk = end_of_dfs_vclk - DFS_VCLK_TYPE,

/* SPECIAL TYPE */
	VCLK_CLKCMU_CHUB_BUS = (MASK_OF_ID & end_of_dfs_vclk) | VCLK_TYPE,
	VCLK_DIV_CLK_CHUB_I2C0,
	VCLK_CLK_CHUB_TIMER_FCLK,
	VCLK_DIV_CLK_CHUB_USI0,
	VCLK_DIV_CLK_CHUB_I2C1,
	VCLK_MUX_CLK_CMGP_USI3,
	VCLK_MUX_CLK_CMGP_ADC,
	VCLK_MUX_CLK_CMGP_I2C0,
	VCLK_MUX_CLK_CMGP_I2C2,
	VCLK_MUX_CLK_CMGP_USI0,
	VCLK_MUX_CLK_CMGP_USI2,
	VCLK_MUX_CLK_CMGP_I2C1,
	VCLK_DIV_CLK_CMGP_USI1,
	VCLK_MUX_CLK_CMGP_I2C3,
	VCLK_CLKCMU_CIS_CLK0,
	VCLK_CLKCMU_FSYS_USB20DRD,
	VCLK_CLKCMU_PERI_UART,
	VCLK_DIV_CLK_CMU_CMUREF,
	VCLK_CLKCMU_PERI_IP,
	VCLK_CLKCMU_APM_BUS,
	VCLK_CLKCMU_CIS_CLK1,
	VCLK_CLKCMU_MIF_BUSP,
	VCLK_AP2CP_SHARED0_PLL_CLK,
	VCLK_DIV_CLK_CPUCL0_CMUREF,
	VCLK_DIV_CLK_CLUSTER0_CNTCLK,
	VCLK_DIV_CLK_AUD_FM,
	VCLK_DIV_CLK_AUD_UAIF1,
	VCLK_DIV_CLK_AUD_CPU_PCLKDBG,
	VCLK_DIV_CLK_AUD_DMIC,
	VCLK_DIV_CLK_AUD_UAIF0,
	VCLK_DIV_CLK_AUD_UAIF2,
	VCLK_DIV_CLK_AUD_MCLK,
	VCLK_MUX_MIF_CMUREF,
	VCLK_DIV_CLK_PERI_USI00_I2C,
	VCLK_DIV_CLK_PERI_HSI2C,
	VCLK_DIV_CLK_PERI_USI00_USI,
	VCLK_MUX_CLK_PERI_SPI,
	VCLK_DIV_CLK_VTS_DMIC,
	VCLK_DIV_CLK_VTS_DMIC_IF_DIV2,
	end_of_vclk,
	num_of_vclk = end_of_vclk - ((MASK_OF_ID & end_of_dfs_vclk) | VCLK_TYPE),

/* COMMON TYPE */
	VCLK_BLK_APM = (MASK_OF_ID & end_of_vclk) | COMMON_VCLK_TYPE,
	VCLK_BLK_CHUB,
	VCLK_BLK_CMU,
	VCLK_BLK_CORE,
	VCLK_BLK_CPUCL0,
	VCLK_BLK_DISPAUD,
	VCLK_BLK_G3D,
	VCLK_BLK_IS,
	VCLK_BLK_MFCMSCL,
	VCLK_BLK_VTS,
	end_of_common_vclk,
	num_of_common_vclk = end_of_common_vclk - ((MASK_OF_ID & end_of_vclk) | COMMON_VCLK_TYPE),

/* GATING TYPE */
	VCLK_IP_APBIF_GPIO_ALIVE = (MASK_OF_ID & end_of_common_vclk) | GATE_VCLK_TYPE,
	VCLK_IP_APBIF_GPIO_CMGPALV,
	VCLK_IP_APBIF_PMU_ALIVE,
	VCLK_IP_APBIF_PMU_INTR_GEN,
	VCLK_IP_APBIF_RTC,
	VCLK_IP_APBIF_TOP_RTC,
	VCLK_IP_APM_CMU_APM,
	VCLK_IP_DTZPC_APM,
	VCLK_IP_GREBEINTEGRATION,
	VCLK_IP_INTMEM,
	VCLK_IP_LHM_AXI_C_CHUB,
	VCLK_IP_LHM_AXI_C_GNSS,
	VCLK_IP_LHM_AXI_C_MODEM,
	VCLK_IP_LHM_AXI_C_VTS,
	VCLK_IP_LHM_AXI_C_WLBT,
	VCLK_IP_LHM_AXI_P_APM,
	VCLK_IP_LHS_AXI_D_APM,
	VCLK_IP_LHS_AXI_LP_CHUB,
	VCLK_IP_LHS_AXI_LP_VTS,
	VCLK_IP_MAILBOX_APM_AP,
	VCLK_IP_MAILBOX_APM_CHUB,
	VCLK_IP_MAILBOX_APM_CP,
	VCLK_IP_MAILBOX_APM_GNSS,
	VCLK_IP_MAILBOX_APM_VTS,
	VCLK_IP_MAILBOX_APM_WLBT,
	VCLK_IP_MAILBOX_AP_CHUB,
	VCLK_IP_MAILBOX_AP_CP,
	VCLK_IP_MAILBOX_AP_CP_S,
	VCLK_IP_MAILBOX_AP_GNSS,
	VCLK_IP_MAILBOX_AP_WLBT,
	VCLK_IP_MAILBOX_CP_CHUB,
	VCLK_IP_MAILBOX_CP_GNSS,
	VCLK_IP_MAILBOX_CP_WLBT,
	VCLK_IP_MAILBOX_GNSS_CHUB,
	VCLK_IP_MAILBOX_GNSS_WLBT,
	VCLK_IP_MAILBOX_WLBT_ABOX,
	VCLK_IP_MAILBOX_WLBT_CHUB,
	VCLK_IP_PEM,
	VCLK_IP_SPEEDY_APM,
	VCLK_IP_SYSREG_APM,
	VCLK_IP_WDT_APM,
	VCLK_IP_XIU_DP_APM,
	VCLK_IP_AHB_BUSMATRIX_CHUB,
	VCLK_IP_BAAW_D_CHUB,
	VCLK_IP_BAAW_P_APM_CHUB,
	VCLK_IP_BPS_AXI_LP_CHUB,
	VCLK_IP_BPS_AXI_P_CHUB,
	VCLK_IP_CHUB_CMU_CHUB,
	VCLK_IP_CHUB_RTC_APBIF,
	VCLK_IP_CM4_CHUB,
	VCLK_IP_D_TZPC_CHUB,
	VCLK_IP_GPIO_CMGPALV_CHUB_APBIF,
	VCLK_IP_I2C_CHUB00,
	VCLK_IP_I2C_CHUB01,
	VCLK_IP_LHM_AXI_LP_CHUB,
	VCLK_IP_LHM_AXI_P_CHUB,
	VCLK_IP_LHS_AXI_C_CHUB,
	VCLK_IP_LHS_AXI_D_CHUB,
	VCLK_IP_PDMA_CHUB,
	VCLK_IP_PWM_CHUB,
	VCLK_IP_SWEEPER_D_CHUB,
	VCLK_IP_SWEEPER_P_APM_CHUB,
	VCLK_IP_SYSREG_CHUB,
	VCLK_IP_TIMER_CHUB,
	VCLK_IP_USI_CHUB00,
	VCLK_IP_WDT_CHUB,
	VCLK_IP_ADC_CMGP,
	VCLK_IP_CMGP_CMU_CMGP,
	VCLK_IP_DTZPC_CMGP,
	VCLK_IP_GPIO_CMGP,
	VCLK_IP_I2C_CMGP0,
	VCLK_IP_I2C_CMGP1,
	VCLK_IP_I2C_CMGP2,
	VCLK_IP_I2C_CMGP3,
	VCLK_IP_I2C_CMGP4,
	VCLK_IP_I2C_CMGP5,
	VCLK_IP_I2C_CMGP6,
	VCLK_IP_SYSREG_CMGP,
	VCLK_IP_SYSREG_CMGP2CHUB,
	VCLK_IP_SYSREG_CMGP2CP,
	VCLK_IP_SYSREG_CMGP2GNSS,
	VCLK_IP_SYSREG_CMGP2PMU_AP,
	VCLK_IP_SYSREG_CMGP2PMU_CHUB,
	VCLK_IP_SYSREG_CMGP2WLBT,
	VCLK_IP_USI_CMGP0,
	VCLK_IP_USI_CMGP1,
	VCLK_IP_USI_CMGP2,
	VCLK_IP_USI_CMGP3,
	VCLK_IP_OTP,
	VCLK_IP_ADM_AHB_SSS,
	VCLK_IP_AD_APB_DIT,
	VCLK_IP_AD_APB_PDMA0,
	VCLK_IP_AD_APB_SPDMA,
	VCLK_IP_AD_AXI_GIC,
	VCLK_IP_AD_AXI_SSS,
	VCLK_IP_BAAW_P_CHUB,
	VCLK_IP_BAAW_P_GNSS,
	VCLK_IP_BAAW_P_MODEM,
	VCLK_IP_BAAW_P_VTS,
	VCLK_IP_BAAW_P_WLBT,
	VCLK_IP_CORE_CMU_CORE,
	VCLK_IP_DIT,
	VCLK_IP_D_TZPC_CORE,
	VCLK_IP_GIC400_AIHWACG,
	VCLK_IP_LHM_AXI_D0_MODEM,
	VCLK_IP_LHM_AXI_D1_MODEM,
	VCLK_IP_LHM_AXI_D_ABOX,
	VCLK_IP_LHM_AXI_D_APM,
	VCLK_IP_LHM_AXI_D_CHUB,
	VCLK_IP_LHM_AXI_D_CPUCL0,
	VCLK_IP_LHM_AXI_D_CSSYS,
	VCLK_IP_LHM_AXI_D_DPU,
	VCLK_IP_LHM_AXI_D_FSYS,
	VCLK_IP_LHM_AXI_D_G3D,
	VCLK_IP_LHM_AXI_D_GNSS,
	VCLK_IP_LHM_AXI_D_IS,
	VCLK_IP_LHM_AXI_D_MFCMSCL,
	VCLK_IP_LHM_AXI_D_VTS,
	VCLK_IP_LHM_AXI_D_WLBT,
	VCLK_IP_LHS_AXI_P_APM,
	VCLK_IP_LHS_AXI_P_CHUB,
	VCLK_IP_LHS_AXI_P_CPUCL0,
	VCLK_IP_LHS_AXI_P_DISPAUD,
	VCLK_IP_LHS_AXI_P_FSYS,
	VCLK_IP_LHS_AXI_P_G3D,
	VCLK_IP_LHS_AXI_P_GNSS,
	VCLK_IP_LHS_AXI_P_IS,
	VCLK_IP_LHS_AXI_P_MFCMSCL,
	VCLK_IP_LHS_AXI_P_MODEM,
	VCLK_IP_LHS_AXI_P_PERI,
	VCLK_IP_LHS_AXI_P_VTS,
	VCLK_IP_LHS_AXI_P_WLBT,
	VCLK_IP_PDMA_CORE,
	VCLK_IP_RTIC,
	VCLK_IP_SFR_APBIF_CMU_TOPC,
	VCLK_IP_SIREX,
	VCLK_IP_SPDMA_CORE,
	VCLK_IP_SSS,
	VCLK_IP_SYSREG_CORE,
	VCLK_IP_TREX_D_CORE,
	VCLK_IP_TREX_D_NRT,
	VCLK_IP_TREX_P_CORE,
	VCLK_IP_XIU_D_CORE,
	VCLK_IP_ADM_APB_G_CSSYS_MIF,
	VCLK_IP_ADM_APB_G_DUMP_PC_CPUCL0,
	VCLK_IP_ADS_AHB_G_CSSYS_SSS,
	VCLK_IP_ADS_APB_G_DUMP_PC_CPUCL0,
	VCLK_IP_AD_APB_P_DUMP_PC_CPUCL0,
	VCLK_IP_CPUCL0_CMU_CPUCL0,
	VCLK_IP_CSSYS_DBG,
	VCLK_IP_DUMP_PC_CPUCL0,
	VCLK_IP_D_TZPC_CPUCL0,
	VCLK_IP_LHM_AXI_P_CPUCL0,
	VCLK_IP_LHS_AXI_D_CPUCL0,
	VCLK_IP_LHS_AXI_D_CSSYS,
	VCLK_IP_SECJTAG,
	VCLK_IP_SYSREG_CPUCL0,
	VCLK_IP_ABOX,
	VCLK_IP_AD_APB_DECON0,
	VCLK_IP_AD_APB_SMMU_ABOX,
	VCLK_IP_AXI_US_32to128,
	VCLK_IP_DFTMUX_DISPAUD,
	VCLK_IP_DISPAUD_CMU_DISPAUD,
	VCLK_IP_DMIC,
	VCLK_IP_DPU,
	VCLK_IP_D_TZPC_DISPAUD,
	VCLK_IP_GPIO_DISPAUD,
	VCLK_IP_LHM_AXI_P_DISPAUD,
	VCLK_IP_LHS_AXI_D_ABOX,
	VCLK_IP_LHS_AXI_D_DPU,
	VCLK_IP_PERI_AXI_ASB,
	VCLK_IP_PPMU_ABOX,
	VCLK_IP_PPMU_DPU,
	VCLK_IP_SMMU_ABOX,
	VCLK_IP_SMMU_DPU,
	VCLK_IP_SYSREG_DISPAUD,
	VCLK_IP_WDT_AUD,
	VCLK_IP_D_TZPC_FSYS,
	VCLK_IP_FSYS_CMU_FSYS,
	VCLK_IP_GPIO_FSYS,
	VCLK_IP_LHM_AXI_P_FSYS,
	VCLK_IP_LHS_AXI_D_FSYS,
	VCLK_IP_MMC_CARD,
	VCLK_IP_MMC_EMBD,
	VCLK_IP_PPMU_FSYS,
	VCLK_IP_SYSREG_FSYS,
	VCLK_IP_USB20DRD_TOP,
	VCLK_IP_US_64to128_FSYS,
	VCLK_IP_XIU_D_FSYS,
	VCLK_IP_ASYNC_G3D_P,
	VCLK_IP_D_TZPC_G3D,
	VCLK_IP_G3D,
	VCLK_IP_G3D_CMU_G3D,
	VCLK_IP_GRAY2BIN_G3D,
	VCLK_IP_LHM_AXI_P_G3D,
	VCLK_IP_LHS_AXI_D_G3D,
	VCLK_IP_SYSREG_G3D,
	VCLK_IP_D_TZPC_IS,
	VCLK_IP_IS_CMU_IS,
	VCLK_IP_LHM_AXI_P_IS,
	VCLK_IP_LHS_AXI_D_IS,
	VCLK_IP_SYSREG_IS,
	VCLK_IP_is3p21p0_IS,
	VCLK_IP_AS_APB_JPEG,
	VCLK_IP_AS_APB_M2M,
	VCLK_IP_AS_APB_MCSC,
	VCLK_IP_AS_APB_MFC,
	VCLK_IP_AS_APB_SYSMMU_NS_MFCMSCL,
	VCLK_IP_AS_APB_SYSMMU_S_MFCMSCL,
	VCLK_IP_AS_AXI_M2M,
	VCLK_IP_AXI2APB_MFCMSCL,
	VCLK_IP_D_TZPC_MFCMSCL,
	VCLK_IP_JPEG,
	VCLK_IP_LHM_AXI_P_MFCMSCL,
	VCLK_IP_LHS_AXI_D_MFCMSCL,
	VCLK_IP_M2M,
	VCLK_IP_MCSC,
	VCLK_IP_MFC,
	VCLK_IP_MFCMSCL_CMU_MFCMSCL,
	VCLK_IP_PPMU_MFCMSCL,
	VCLK_IP_SYSMMU_MFCMSCL,
	VCLK_IP_SYSREG_MFCMSCL,
	VCLK_IP_XIU_D_MFCMSCL,
	VCLK_IP_DMC,
	VCLK_IP_D_TZPC_MIF,
	VCLK_IP_MIF_CMU_MIF,
	VCLK_IP_PPMU_DMC_CPU,
	VCLK_IP_QE_DMC_CPU,
	VCLK_IP_SYSREG_MIF,
	VCLK_IP_MODEM_CMU_MODEM,
	VCLK_IP_BUSIF_TMU,
	VCLK_IP_D_TZPC_PERI,
	VCLK_IP_GPIO_PERI,
	VCLK_IP_I2C_0,
	VCLK_IP_I2C_1,
	VCLK_IP_I2C_2,
	VCLK_IP_LHM_AXI_P_PERI,
	VCLK_IP_MCT,
	VCLK_IP_OTP_CON_TOP,
	VCLK_IP_PERI_CMU_PERI,
	VCLK_IP_PWM_MOTOR,
	VCLK_IP_SYSREG_PERI,
	VCLK_IP_USI00_I2C,
	VCLK_IP_USI00_USI,
	VCLK_IP_USI_I2C_0,
	VCLK_IP_USI_SPI,
	VCLK_IP_USI_UART,
	VCLK_IP_WDT_CLUSTER0,
	VCLK_IP_AHB_BUSMATRIX,
	VCLK_IP_BAAW_C_VTS,
	VCLK_IP_BAAW_D_VTS,
	VCLK_IP_BPS_LP_VTS,
	VCLK_IP_BPS_P_VTS,
	VCLK_IP_CORTEXM4INTEGRATION,
	VCLK_IP_DMIC_AHB0,
	VCLK_IP_DMIC_AHB1,
	VCLK_IP_DMIC_IF,
	VCLK_IP_D_TZPC_VTS,
	VCLK_IP_GPIO_VTS,
	VCLK_IP_HWACG_SYS_DMIC0,
	VCLK_IP_HWACG_SYS_DMIC1,
	VCLK_IP_LHM_AXI_LP_VTS,
	VCLK_IP_LHM_AXI_P_VTS,
	VCLK_IP_LHS_AXI_C_VTS,
	VCLK_IP_LHS_AXI_D_VTS,
	VCLK_IP_MAILBOX_ABOX_VTS,
	VCLK_IP_MAILBOX_AP_VTS,
	VCLK_IP_SWEEPER_C_VTS,
	VCLK_IP_SWEEPER_D_VTS,
	VCLK_IP_SYSREG_VTS,
	VCLK_IP_TIMER,
	VCLK_IP_VTS_CMU_VTS,
	VCLK_IP_WDT_VTS,
	VCLK_IP_XHB_LP_VTS,
	VCLK_IP_XHB_P_VTS,
	VCLK_IP_u_DMIC_CLK_MUX,
	end_of_gating_vclk,
	num_of_gating_vclk = end_of_gating_vclk - ((MASK_OF_ID & end_of_common_vclk) | GATE_VCLK_TYPE),

};
#endif
