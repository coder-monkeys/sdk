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
#define FENG_K1_G_IO    WM_IO_PB_06	
#define FENG_K2_R_IO    WM_IO_PB_07
#define FENG_K3_B_IO    WM_IO_PB_09
#define FXDG_IO         WM_IO_PB_16
#define ZDY_IO          WM_IO_PA_01
#define LAMP_Y_IO       WM_IO_PB_13 //cold
#define LAMP_W_IO       WM_IO_PB_08 //warm
#define ZERO_DETEC_IO	WM_IO_PB_18
#define ASISTANT_IO     WM_IO_PB_17
#define BEEP_IO         WM_IO_PB_15

#define LAMP_Y_CHANNEL  (1) //cold
#define LAMP_W_CHANNEL  (4) //warm
#define LAMP_PWM_FREQ   (1000)
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
	u8 dev_mode; //mode0-mode7
	u8 lamp_y_duty;
	u8 lamp_w_duty;
} device_control;

u32 SrcTasktStk[TASK_SRC_SIZE];

static frequency_info f_info;
static device_control device = 
{
.dev_mode = 0
};

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

/*
**********************************************
channel: 0-4
freq   : Hz
duty   : 3-100
**********************************************
*/
static int pwm_output(int channel, int freq, int duty)
{
	duty = duty*255/100;
	tls_pwm_init(channel, freq, duty, 0);
    tls_pwm_start(channel);
	
	return 0;
}

static int gpio_initial(void)
{
	tls_gpio_cfg(ZDY_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(ASISTANT_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(BEEP_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	
	tls_gpio_cfg(FENG_K2_R_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(FENG_K1_G_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(FENG_K3_B_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
	tls_gpio_cfg(FXDG_IO, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);

	wm_pwm5_config(LAMP_W_IO);
	wm_pwm2_config(LAMP_Y_IO);
	
    return WM_SUCCESS;
}

static int wind_level_control(int level)
{
	tls_gpio_write(FENG_K1_G_IO, POWER_OFF);
	tls_gpio_write(FENG_K2_R_IO, POWER_OFF);
	tls_gpio_write(FENG_K3_B_IO, POWER_OFF);
	tls_os_time_delay(HZ/5);
	
	switch(level)
	{
		case 0:
			tls_gpio_write(FENG_K1_G_IO, POWER_OFF);
			tls_gpio_write(FENG_K2_R_IO, POWER_OFF);
			tls_gpio_write(FENG_K3_B_IO, POWER_OFF);
			break;
		case 1:
			tls_gpio_write(FENG_K1_G_IO, POWER_ON);
			tls_gpio_write(FENG_K2_R_IO, POWER_OFF);
			tls_gpio_write(FENG_K3_B_IO, POWER_OFF);
			break;
		case 2:
			tls_gpio_write(FENG_K1_G_IO, POWER_OFF);
			tls_gpio_write(FENG_K2_R_IO, POWER_ON);
			tls_gpio_write(FENG_K3_B_IO, POWER_OFF);
			break;
		case 3:
			tls_gpio_write(FENG_K1_G_IO, POWER_OFF);
			tls_gpio_write(FENG_K2_R_IO, POWER_OFF);
			tls_gpio_write(FENG_K3_B_IO, POWER_ON);
			break;
		default:
			break;
	}
	return 0;
}

static void dev_mode_start(int mode)
{
	switch(mode)
	{
		case 0: //mode 1
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 100);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 0);
			wind_level_control(0);
			break;
		case 1: //mode 2
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 0);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 100);
			wind_level_control(1);
			break;
		case 2: //mode 3
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 0);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 0);
			wind_level_control(2);
			break;
		case 3: //mode 4
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 50);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 50);
			wind_level_control(3);
			break;
		case 4: //mode 5
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 50);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 50);
			wind_level_control(0);
			break;
		case 5: //mode 6
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 5);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 5);
			wind_level_control(1);
			break;
		case 6: //mode 7
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 50);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 50);
			wind_level_control(2);
			break;
		case 7: //mode 8
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 0);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 0);
			wind_level_control(3);
			break;
		default: //mode 1
			pwm_output(LAMP_Y_CHANNEL, LAMP_PWM_FREQ, 100);
			pwm_output(LAMP_W_CHANNEL, LAMP_PWM_FREQ, 0);
			wind_level_control(0);
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

	tls_gpio_write(ZDY_IO, POWER_ON);
	tls_gpio_write(BEEP_IO, POWER_OFF);
	tls_os_time_delay(HZ/10);
	dev_mode_start(device.dev_mode);
	
	while(1)
	{
		(f_info.enter_int > 10000)?(f_info.enter_int = 0):(f_info.enter_int = f_info.enter_int);
		enter_int_b = f_info.enter_int;
		tls_os_time_delay(HZ/10);
		enter_int_a = f_info.enter_int;
		interval = enter_int_a-enter_int_b;

		//power off detected
		if( interval<5 )
		{
			off_time ++;
		}

		//power on now
		if( off_time>0 && interval>5 )
		{
			on_time ++;
		}

		//a short power off and a power on has been detected
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
			dev_mode_start( device.dev_mode );
		}
	}
}

static int src_lamp_intial(void)
{
	gpio_initial();
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

