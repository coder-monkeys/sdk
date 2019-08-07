/***************************************************************************** 
* 
* File Name : main.c
* 
* Description: main 
* 
* Copyright (c) 2014 Winner Micro Electronic Design Co., Ltd. 
* All rights reserved. 
* 
* Author : dave
* 
* Date : 2014-6-14
*****************************************************************************/ 
#include "wm_include.h"
#include "wm_pwm.h"
#include "wm_cpu.h"
#include "wm_dma.h"
#include "wm_gpio_afsel.h"
#include "wm_timer.h"
#include <string.h>

#define COUNTER_FREQ    10000
#define POWER_ON        1
#define POWER_OFF       0
#define PWM_CAP_CHANNEL 0
#define LAMP_K2_R_IO    WM_IO_PB_06
#define LAMP_K1_G_IO    WM_IO_PB_07
#define LAMP_K3_B_IO    WM_IO_PB_08
#define FXDG_IO         WM_IO_PB_09
#define ZDY_IO          WM_IO_PB_10
#define LAMP_Y_IO       WM_IO_PB_16 //cold
#define LAMP_W_IO       WM_IO_PB_17 //warm
#define ZERO_DETEC_IO	WM_IO_PB_18
#define TEST_AUTO_DUTY  1
#define TASK_SRC_SIZE   512
#define WIND_LEVEL_0    (0)
#define WIND_LEVEL_1    (1/3)
#define WIND_LEVEL_2    (2/3)
#define WIND_LEVEL_3    (3/3)
#define MODE_SAVE_ADDR   (0xFC000)

typedef struct _frequency_info 
{
	volatile u32 pwmDmaCap;
	volatile u32 frequecy;
	volatile u32 half_period_ms;
	volatile u32 fcount;
	volatile u32 rcount;
	volatile u32 enter_int;
} frequency_info;

typedef struct _device_control 
{
	u32 k1_g_time; //cold
	u32 k2_r_time; //warm
	u32 k3_b_time;
	u32 fxdg_time; //fengshan	
	u8 k1_timer_id;
	u8 k2_timer_id;
	u8 k3_timer_id;
	u8 fxdg_timer_id;
	u8 dev_mode; //mode0-mode7
} device_control;

u32 SrcTasktStk[TASK_SRC_SIZE];

static frequency_info f_info;
static device_control device = 
{
.k1_g_time = 5*1000,
.k2_r_time = 5*1000,
.k3_b_time = 5*1000,
.fxdg_time = 5*1000,
.dev_mode = 0
};

static void k1_timer_isr_callback(void *arg)
{
	tls_gpio_write(LAMP_K1_G_IO, POWER_OFF);
}

static void k2_timer_isr_callback(void *arg)
{
	tls_gpio_write(LAMP_K2_R_IO, POWER_OFF);
}

static void k3_timer_isr_callback(void *arg)
{
	tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
}

static void fxdg_timer_isr_callback(void *arg)
{
	tls_gpio_write(FXDG_IO, POWER_OFF);
}

static int detect_zero_point(void)
{
	tls_gpio_write(LAMP_K2_R_IO, POWER_ON);
	tls_gpio_write(LAMP_K1_G_IO, POWER_ON);
	tls_gpio_write(LAMP_K3_B_IO, POWER_ON);
	tls_gpio_write(FXDG_IO, POWER_ON);

	tls_timer_change(device.k1_timer_id, device.k1_g_time);
	tls_timer_change(device.k2_timer_id, device.k2_r_time);
	tls_timer_change(device.k3_timer_id, device.k3_b_time);
	tls_timer_change(device.fxdg_timer_id, device.fxdg_time);

	tls_timer_start(device.k1_timer_id);
	tls_timer_start(device.k2_timer_id);
	tls_timer_start(device.k3_timer_id);
	tls_timer_start(device.fxdg_timer_id);

	return 0;
}

static void pwm_isr_callback0(void)
{
    volatile int fcount = 0, rcount = 0;
    int status = tls_reg_read32(HR_PWM_INTSTS);

    if (status & 0x200)
    {
        tls_reg_write32(HR_PWM_INTSTS, status | 0x60);
    }
    else
    {
        if (status & 0x40) //falling edge capture
        {
            if ((status & 0x300) == 0)
            {
                fcount = ((tls_reg_read32(HR_PWM_CAPDAT) & 0xFFFF0000) >> 16);
				if( fcount>5 ) 
				{
					f_info.fcount = fcount;
					detect_zero_point();
				}
            }
			
            tls_reg_write32(HR_PWM_INTSTS, status | 0x40);
        }
        if (status & 0x20) //rasing edge capture
        {
            if ((status & 0x00000280) == 0)
            {
                rcount = (tls_reg_read32(HR_PWM_CAPDAT) & 0x0000FFFF);
				if( rcount>5 ) 
				{
					f_info.rcount = rcount;
					detect_zero_point();
				}
            }
            tls_reg_write32(HR_PWM_INTSTS, status | 0x20);
        }
    }
	f_info.enter_int ++;
}

static int pwm_capture_int(u8 channel, u32 freq)
{
    tls_sys_clk sysclk;

    tls_sys_clk_get(&sysclk);
	tls_gpio_cfg(WM_IO_PB_18, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_FLOATING);
	wm_pwm1_config(WM_IO_PB_18);
	memset( &f_info, 0, sizeof(f_info) );
    if (channel == 0) {
        tls_pwm_isr_register(pwm_isr_callback0);
    }
    tls_pwm_cap_init(channel, sysclk.apbclk * UNIT_MHZ / freq, DISABLE, WM_PWM_CAP_RISING_FALLING_EDGE_INT);
    tls_pwm_start(channel);
		
	return 0;
}

static int gpio_initial(void)
{
	tls_gpio_cfg(LAMP_K2_R_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(LAMP_K1_G_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(LAMP_K3_B_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(FXDG_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	
    return WM_SUCCESS;
}

static int timer_init(unsigned int timeout_us, tls_timer_irq_callback callback_func)
{
    u8 timer_id;
    struct tls_timer_cfg timer_cfg;

    timer_cfg.unit = TLS_TIMER_UNIT_US;
    timer_cfg.timeout = timeout_us;
    timer_cfg.is_repeat = 0;
    timer_cfg.callback = callback_func;
    timer_cfg.arg = NULL;
    timer_id = tls_timer_create(&timer_cfg);
    return timer_id;
}

static void dev_mode_change(device_control *dev, frequency_info *freq)
{
	switch(dev->dev_mode)
	{
		case 0: //mode 1
			dev->k1_g_time = freq->half_period_ms*1000;
			dev->k2_r_time = 0;
			dev->fxdg_time = WIND_LEVEL_0;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		case 1: //mode 2
			dev->k1_g_time = 0;
			dev->k2_r_time = freq->half_period_ms*1000;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_1;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		case 2: //mode 3
			dev->k1_g_time = 0;
			dev->k2_r_time = 0;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_2;
			tls_gpio_write(LAMP_K3_B_IO, POWER_ON);
			break;
		case 3: //mode 4
			dev->k1_g_time = freq->half_period_ms*1000/2;
			dev->k2_r_time = freq->half_period_ms*1000/2;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_3;
			tls_gpio_write(LAMP_K3_B_IO, POWER_ON);
			break;
		case 4: //mode 5
			dev->k1_g_time = freq->half_period_ms*1000/2;
			dev->k2_r_time = freq->half_period_ms*1000/2;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_2;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		case 5: //mode 6
			dev->k1_g_time = freq->half_period_ms*1000/20;
			dev->k2_r_time = freq->half_period_ms*1000/20;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_1;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		case 6: //mode 7
			dev->k1_g_time = 0;
			dev->k2_r_time = freq->half_period_ms*1000;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_0;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		case 7: //mode 8
			dev->k1_g_time = 0;
			dev->k2_r_time = 0;
			dev->fxdg_time = freq->half_period_ms*1000*WIND_LEVEL_1;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
		default: //mode 1
			dev->k1_g_time = freq->half_period_ms*1000;
			dev->k2_r_time = 0;
			dev->fxdg_time = WIND_LEVEL_0;
			tls_gpio_write(LAMP_K3_B_IO, POWER_OFF);
			break;
	}
}

void src_control_task (void *data)
{
	printf("task start.\n");
	tls_fls_read(MODE_SAVE_ADDR, &(device.dev_mode), 1);
	if( device.dev_mode>7 )
	{
		device.dev_mode = 0;
		tls_fls_write(MODE_SAVE_ADDR, &(device.dev_mode), 1);
	}
	printf("mode: %d\n", device.dev_mode);
	u32 enter_int_b = 0;
	u32 enter_int_a = 0;
	u32 interval = 0;
	u32 off_time = 0;
	u32 on_time = 0;
	
	while(1)
	{
		(f_info.enter_int > 10000)?(f_info.enter_int = 0):(f_info.enter_int = f_info.enter_int);
		enter_int_b = f_info.enter_int;
		tls_os_time_delay(HZ/10);
		enter_int_a = f_info.enter_int;
		interval = enter_int_a-enter_int_b;
		if( interval<5 )
		{
			off_time ++;
		}

		if( off_time>0 && interval>5 )
		{
			on_time ++;
		}

		if( on_time>0 && off_time>0 )
		{
			on_time = 0;
			off_time = 0;
			device.dev_mode ++;
			( device.dev_mode>7 )?(device.dev_mode = 0):(device.dev_mode = device.dev_mode);
			f_info.frequecy = COUNTER_FREQ/(f_info.fcount + f_info.rcount);
			f_info.half_period_ms = 1000/f_info.frequecy;
			printf("freq: %dHz,fcount %d,rcount %d\n", f_info.frequecy, f_info.fcount, f_info.rcount);
			printf("mode: %d\n", device.dev_mode);
			tls_fls_write(MODE_SAVE_ADDR, &(device.dev_mode), 1);
			dev_mode_change(&device, &f_info);
		}
		
	}
}

static int src_lamp_intial(void)
{
	gpio_initial();
	device.k1_timer_id = timer_init(device.k1_g_time, k1_timer_isr_callback);
	device.k2_timer_id = timer_init(device.k2_r_time, k2_timer_isr_callback);
	device.k3_timer_id = timer_init(device.k3_b_time, k3_timer_isr_callback);
	device.fxdg_timer_id = timer_init(device.fxdg_time, fxdg_timer_isr_callback);
	f_info.frequecy = pwm_capture_int(PWM_CAP_CHANNEL, COUNTER_FREQ);
	
	tls_os_task_create(NULL, NULL,
	                    src_control_task,
	                    (void *)0,
	                    (void *)&SrcTasktStk[0],          /* task's stack start address */
	                    TASK_SRC_SIZE * sizeof(u32), /* task's stack size, unit:byte */
	                    1,
	                    0);

	return 0;
}

void UserMain(void)
{	
	printf("application start.\n");
	src_lamp_intial();
}

