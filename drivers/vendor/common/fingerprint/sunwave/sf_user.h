#ifndef __SF_USER_H__
#define __SF_USER_H__

#include "sf_def.h"

//-----------------------------------------------------------------------------
// platform&&compatible select
#ifdef CONFIG_VENDOR_SOC_SPRD_COMPILE
#define SF_PLATFORM_SEL             SF_TEE_TRUSTY
#define SF_COMPATIBLE_SEL           SF_COMPATIBLE_TRUSTY
#define  USE_PLATFORM_BUS	1
#include <linux/platform_device.h>
typedef struct platform_device sf_device_t;
typedef struct platform_driver sf_driver_t;
#define SW_BUS_NAME "platform bus"
#endif

#ifdef CONFIG_VENDOR_SOC_MTK_COMPILE
#define SF_PLATFORM_SEL             SF_TEE_OPTEE
#define SF_COMPATIBLE_SEL           SF_COMPATIBLE_OPTEE
#define  USE_SPI_BUS   1
#include <linux/spi/spidev.h>
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
typedef struct spi_device sf_device_t;
typedef struct spi_driver sf_driver_t;
#define SW_BUS_NAME "spi bus"
#endif

// power mode select
#ifdef CONFIG_PLATFORM_FINGERPRINT_SUNWAVE_PWR_MODE_REGULATOR
#define SF_POWER_MODE_SEL           PWR_MODE_REGULATOR
/* regulator name */
#define SF_VDD_NAME                 "vdd"
#else
#define SF_POWER_MODE_SEL           PWR_MODE_GPIO
#endif

// debug log select
//#define SF_LOG_ENABLE               1

// xiaomi fast unblank
//#define XIAOMI_DRM_INTERFACE_WA     1
//-----------------------------------------------------------------------------

/* Dts node. */
#define COMPATIBLE_SW_FP            "sunwave,fingerprint"

// for not mediatek
#define COMPATIBLE_RESET_GPIO       "sunwave,gpio_reset"
#define COMPATIBLE_IRQ_GPIO         "sunwave,gpio_irq"
#define COMPATIBLE_PWR_GPIO         "sunwave,gpio_pwr"

//for mediatek pinctl system
#define FINGER_POWER_ON             "finger_power_high"
#define FINGER_POWER_OFF            "finger_power_low"
#define FINGER_RESET_LOW            "finger_rst_low"
#define FINGER_RESET_HIGH           "finger_rst_high"
#define FINGER_INT_SET              "eint_as_int"
// spi dts config enable
#define SF_SPI_DTS_CS               0
#define SF_SPI_DTS_Ck               0
#define SF_SPI_DTS_MI               0
#define SF_SPI_DTS_MO               0
#define SF_SPI_DTS_MI_PU            0
#define SF_SPI_DTS_MI_PD            0
#define SF_SPI_DTS_MO_PU            0
#define SF_SPI_DTS_MO_PD            0
#define FINGER_CS_SET               "finger_mode_as_cs"
#define FINGER_CK_SET               "finger_mode_as_ck"
#define FINGER_MI_SET               "finger_mode_as_mi"
#define FINGER_MO_SET               "finger_mode_as_mo"
#define FINGER_MI_PU                "miso_pull_up"
#define FINGER_MI_PD                "miso_pull_down"
#define FINGER_MO_PU                "mosi_pull_up"
#define FINGER_MO_PD                "mosi_pull_down"

//-----------------------------------------------------------------------------

//#define SF_INT_TRIG_HIGH            0
//-----------------------------------------------------------------------------

// default set 0, 0: register platform device by dts file, 1: register platform device by driver
#define SF_REG_DEVICE_BY_DRIVER     0
//-----------------------------------------------------------------------------

//for qualcomm ree spi deassert wait time
#define QUALCOMM_REE_DEASSERT       0

//-----------------------------------------------------------------------------

#if (SF_PLATFORM_SEL == SF_REE_MTK_L5_X)
//android mtk androidL 5.x none dts config file
#define MTK_L5_X_POWER_ON           1
#define MTK_L5_X_IRQ_SET            0

//power GPIO
#if MTK_L5_X_POWER_ON
#define  GPIO_SW_PWR_PIN            GPIO_FIGERPRINT_PWR_EN_PIN
#define  GPIO_SW_PWR_M_GPIO         GPIO_MODE_00
#endif

//interrupt pin
#define  GPIO_SW_IRQ_NUM            CUST_EINT_FIGERPRINT_INT_NUM
#define  GPIO_SW_INT_PIN            GPIO_FIGERPRINT_INT

//reset pin
#define  GPIO_SW_RST_PIN            GPIO_FIGERPRINT_RST
#define  GPIO_SW_RST_PIN_M_GPIO     GPIO_MODE_00

#define  GPIO_SW_RST_M_DAIPCMOUT    GPIO_MODE_01

//interrupt mode
#if MTK_L5_X_IRQ_SET
#define  GPIO_SW_EINT_PIN_M_GPIO    GPIO_FIGERPRINT_INT_M_GPIO //GPIO_MODE_00
#define  GPIO_SW_EINT_PIN_M_EINT    GPIO_FIGERPRINT_INT_M_EINT //GPIO_MODE_00
#endif

#endif
//-----------------------------------------------------------------------------
#include "sf_auto.h"

#endif

