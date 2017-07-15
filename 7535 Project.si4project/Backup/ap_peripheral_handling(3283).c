
#include "ap_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"

//#include "drv_l1_system.h"
#include "driver_l1.h"
#include "drv_l1_cdsp.h"

#define LED_STATUS_FLASH		1
#define LED_STATUS_BLINK		2

#define CRAZY_KEY_TEST			0		// Send key events faster than human finger can do
#define LED_ON					1
#define LED_OFF 				0

#define C_PHOTO 				0x10 //拍照模式
#define C_VIDEO 				0x20 //录像模式
#define C_MOTION				0x30 //移动侦测模式

static INT8U	led_status; //0: nothing  1: flash	2: blink
static INT8U	led_cnt;

static INT32U	led_mode;
static INT8U	g_led_count;
static INT8U	g_led_r_state; //0 = OFF;	1=ON;	2=Flicker
static INT8U	g_led_g_state;
static INT8U	g_led_b_state;
static INT8U	g_led_flicker_state; //0=同时闪烁	1=交替闪烁
static INT8U	led_red_flag;
static INT8U	led_green_flag;
static INT8U	led_blue_flag;
static INT8U	g_led_flicker_group; //=0;默认RB, =1;RG, =2,BG  
static INT8U	charge_state_flag = 0;
static INT8U	savefile_led_flag = 0; //保存录像文件时闪灯的标志, =1,闪灯

extern INT8S	video_record_sts;
extern volatile INT8U pic_down_flag;
extern volatile INT8U video_down_flag;
volatile INT8U	AutoTakePic_Mode = 0;

#if TV_DET_ENABLE
INT8U			tv_plug_in_flag;
INT8U			tv_debounce_cnt = 0;

#endif

//static	INT8U tv = !TV_DET_ACTIVE;
//static INT8U backlight_tmr = 0;
#if C_SCREEN_SAVER				== CUSTOM_ON
INT8U			auto_off_force_disable = 0;
void ap_peripheral_auto_off_force_disable_set(INT8U);

#endif

static INT8U	led_flash_timerid;
static INT16U	config_cnt;

//----------------------------
typedef struct 
{
INT8U			byRealVal;
INT8U			byCalVal;
} AD_MAP_t;


//----------------------------
extern void avi_adc_gsensor_data_register(void * *msgq_id, INT32U * msg_id);
INT8U			gsensor_data[2][32] =
{
	0
};


//static void	*gsensor_msgQId0 = 0;
//static INT32U gsensor_msgId0 = 0;
//static INT8U	ad_line_select = 0;
//static INT16U adc_battery_value_new, adc_battery_value_old;
//static INT32U battery_stable_cnt = 0;
#define C_BATTERY_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

#if C_BATTERY_DETECT			== CUSTOM_ON
static INT16U	low_voltage_cnt;
static INT32U	battery_value_sum = 0;
static INT8U	bat_ck_cnt = 0;

#endif



#if USE_ADKEY_NO
static INT8U	ad_detect_timerid;
static INT16U	ad_value;
static KEYSTATUS ad_key_map[USE_ADKEY_NO + 1];

//static INT16U ad_key_cnt = 0;
//static INT16U adc_key_release_value_old, adc_key_release_value_new; 
static INT16U	adc_key_release_value_stable;

//static INT32U key_release_stable_cnt = 0;//lx
#define C_RESISTOR_ACCURACY 	5//josephhsieh@140418 3			// 2% accuracy
#define C_KEY_PRESS_WATERSHED	600//josephhsieh@140418 175
#define C_KEY_STABLE_THRESHOLD	4//josephhsieh@140418 3			// Defines threshold number that AD value of key is deemed stable
#define C_KEY_FAST_JUDGE_THRESHOLD 40			// Defines threshold number that key is should be judge before it is release. 0=Disable
#define C_KEY_RELEASE_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

INT16U			adc_key_value;

//static INT8U	ad_value_cnt ;
INT32U			key_pressed_cnt;
INT8U			fast_key_exec_flag;
INT8U			normal_key_exec_flag;
INT8U			long_key_exec_flag;

#endif

static INT32U	key_active_cnt;

//static INT8U lcd_bl_sts;
static INT8U	power_off_timerid;
static INT8U	usbd_detect_io_timerid;
static KEYSTATUS key_map[USE_IOKEY_NO];
static INT8U	key_detect_timerid;
static INT16U	adp_out_cnt;
static INT16U	usbd_cnt;
static INT8U	up_firmware_flag = 0;
static INT8U	flash_flag = 0;

#if USB_PHY_SUSPEND 			== 1
static INT16U	phy_cnt = 0;

#endif

static INT16U	adp_cnt;
INT8U			adp_status;
static INT8U	battery_low_flag = 0;
INT8U			usbd_exit;
volatile INT8U	s_usbd_pin;
volatile INT8U	sence;

//extern INT8U MODE_KEY_flag;
//	prototypes
void ap_peripheral_key_init(void);
void ap_peripheral_rec_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr);

#if KEY_FUNTION_TYPE			== SAMPLE2
void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr);

#endif

void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr);

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*bat_detect_isr) (INT16U data));
void ap_peripheral_ad_check_isr(INT16U value);

#endif

void ap_peripheral_power_on_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_video_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_motion_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_ir_switch_exe(INT16U * tick_cnt_ptr);
INT8U ap_peripheral_power_off_flag_get(void);
extern void task_peripheral_handling_idle_count_set(INT32U cnt);
static INT16U	First_PowerOn_Flag = 1;
static INT8U	PowerOff_Flag = 0;
static INT8U	Led_Work_Flag = 0;
INT16U			KeyScan_Wait_Flag = 1;
static INT8U	ChaAndEnc_USB_Flag = 0;
static INT8U	Usb_PowerOn_State = 0;
static INT8U	UsbMode_Tf_flag = 0;
static INT8U	Low_Voltage_Flag = 0;


INT8U ap_peripheral_sence_get(void)
{
	return sence;
}


void ap_peripheral_UsbMode_tf_set(INT8U flag)
{
	if (UsbMode_Tf_flag != flag)
		flag = UsbMode_Tf_flag;
}


INT8U ap_peripheral_UsbMode_tf_get(void)
{
	return UsbMode_Tf_flag;
}


void ap_peripheral_handling_usb_state_set(INT8U flag)
{
	ChaAndEnc_USB_Flag	= flag;
}


INT8U ap_peripheral_handling_usb_state_get(void)
{
	return ChaAndEnc_USB_Flag;
}


void LDO12_Switch(INT8U OnOff)
{
	INT32U			ret;

	ret 				= R_SYSTEM_POWER_CTRL0;

	if (OnOff)
		ret |= 0x01; //enable LDO12 and LDO33
	else 
		ret &= (~0x01);

	R_SYSTEM_POWER_CTRL0 = ret;
}


INT8U SaveFile_Led_Flag_Get(void)
{
	return savefile_led_flag;
}


void SaveFile_Led_Flag_Set(INT8U flag)
{
	savefile_led_flag	= flag;
}


void power_pin_set(INT8U onoff)
{

	gpio_init_io(POWER_ENABLE_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(POWER_ENABLE_PIN, ATTRIBUTE_HIGH);
	gpio_write_io(POWER_ENABLE_PIN, onoff);
}


INT8U ap_peripheral_usb_poweron_get(void)
{
	return Usb_PowerOn_State;
}


INT8U ap_peripheral_power_key_read(int pin)
{
	int 			status;


#if (								KEY_TYPE == KEY_TYPE5)
	status				= gpio_read_io(pin);

#else

	switch (pin)
	{
		case PWR_KEY0:
			status = sys_pwr_key0_read();
			break;

		case PWR_KEY1:
			status = sys_pwr_key1_read();
			break;
	}

#endif

	if (status != 0)
		return 1;

	else 
		return 0;
}


void ap_peripheral_init(void)
{
#if TV_DET_ENABLE
	INT32U i;
#endif

	First_PowerOn_Flag	= 1;
	PowerOff_Flag		= 0;
	power_off_timerid	= usbd_detect_io_timerid = led_flash_timerid = 0xFF;
	key_detect_timerid	= 0xFF;

	//LED IO init
	//gpio_init_io(LED, GPIO_OUTPUT);
	//gpio_set_port_attribute(LED, ATTRIBUTE_HIGH);
	//gpio_write_io(LED, DATA_LOW);
	//led_status = 0;
	//led_cnt = 0;
	power_pin_set(1);								//enable external power
	LDO12_Switch(0);								//disable internal power

	//DBG_PRINT("R_SYSTEM_POWER_CTRL0 = 0x%X \r\n",R_SYSTEM_POWER_CTRL0); 
	//Test_ExtLDO_Status();
	LED_pin_init();

	gpio_init_io(CHARGE_DETECTION_PIN, GPIO_INPUT);
	gpio_set_port_attribute(CHARGE_DETECTION_PIN, ATTRIBUTE_LOW);
	gpio_write_io(CHARGE_DETECTION_PIN, DATA_HIGH); //pull high

	//sence = C_PHOTO;

	/*
		gpio_init_io(IR_CTRL,GPIO_OUTPUT);
		gpio_set_port_attribute(IR_CTRL, ATTRIBUTE_HIGH);
		gpio_write_io(IR_CTRL, 0);

		gpio_init_io(AV_IN_DET,GPIO_INPUT);
		gpio_set_port_attribute(AV_IN_DET, ATTRIBUTE_LOW);
		gpio_write_io(AV_IN_DET, !TV_DET_ACTIVE);	//pull high or low
	*/
#if TV_DET_ENABLE
	tv_plug_in_flag 	= 0;

	for (i = 0; i < 5; i++)
	{
		if (gpio_read_io(AV_IN_DET) == !TV_DET_ACTIVE)
		{
			break;
		}

		OSTimeDly(1);
	}

	if (i == 5)
	{
		tv					= TV_DET_ACTIVE;
		tv_plug_in_flag 	= 1;
	}

#endif

	gpio_init_io(HDMI_IN_DET, GPIO_INPUT);
	gpio_set_port_attribute(HDMI_IN_DET, ATTRIBUTE_LOW);
	gpio_write_io(HDMI_IN_DET, 0);					//pull low

	gpio_init_io(SPEAKER_EN, GPIO_OUTPUT);
	gpio_set_port_attribute(SPEAKER_EN, ATTRIBUTE_HIGH);

#if TV_DET_ENABLE

	if (tv_plug_in_flag)
	{
		gpio_write_io(SPEAKER_EN, 0);				//mute local speaker
	}
	else 
#endif

	{
		gpio_write_io(SPEAKER_EN, 1);				//enable local speaker
	}

	ap_peripheral_key_init();

	Usb_PowerOn_State	= ap_peripheral_power_key_read(C_USBDEVICE_PIN);


#if USE_ADKEY_NO
	ad_detect_timerid	= 0xFF;

	//ap_peripheral_ad_detect_init(AD_KEY_DETECT_PIN, ap_peripheral_ad_check_isr);
	ap_peripheral_ad_detect_init(AD_BAT_DETECT_PIN, ap_peripheral_ad_check_isr);

#else

	adc_init();
#endif

	config_cnt			= 0;

	//MODE_KEY_flag = 2;
}


#ifdef PWM_CTR_LED
void ap_peripheral_PWM_OFF(void)
{
	INT8U byPole		= 0;
	INT16U wPeriod		= 0;
	INT16U wPreload 	= 0;
	INT8U byEnable		= 0;
	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable); 
	DBG_PRINT("PWM0/1 off!\r\n");

}


void ap_peripheral_PWM_LED_high(void)
{
	INT8U byPole		= 1;
	INT16U wPeriod		= 0x6000;
	INT16U wPreload 	= 0x5fff;
	INT8U byEnable		= 0;
	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable); 
	DBG_PRINT("PWM0/1 OUT PUT HIGH 750ms, low 68us\r\n");

}


void ap_peripheral_PWM_LED_low(void)
{
	INT8U byPole		= 1;
	INT16U wPeriod		= 0x6000;
	INT16U wPreload 	= 0x1;
	INT8U byEnable		= TRUE;
	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	  ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ; 
	//	  DBG_PRINT("PWM0/1 OUT PUT LOW 750ms, high 68us\r\n"); 
}


#endif

void ap_peripheral_led_set(INT8U type)
{
#ifdef PWM_CTR_LED
	INT8U byPole;
	INT16U wPeriod		= 0;
	INT16U wPreload 	= 0;
	INT8U byEnable;

	if (type)
	{ //high
		ap_peripheral_PWM_LED_high();
	}
	else 
	{ //low
		ap_peripheral_PWM_LED_low();
	}

#else

	gpio_write_io(LED, type);
	led_status			= 0;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_led_flash_set(void)
{
#ifdef PWM_CTR_LED
	ap_peripheral_PWM_LED_high();
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_led_blink_set(void)
{

#ifdef PWM_CTR_LED
	INT8U byPole		= 1;
	INT16U wPeriod		= 0x6000;
	INT16U wPreload 	= 0x2fff;
	INT8U byEnable		= TRUE;
	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	  ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ; 
	DBG_PRINT("PWM0/1 blink on	750ms,380ms \r\n");

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_BLINK;
	led_cnt 			= 0;
#endif
}


void LED_pin_init(void)
{
	INT32U type;

	//led init as ouput pull-low
	gpio_init_io(LED1, GPIO_OUTPUT);
	gpio_set_port_attribute(LED1, ATTRIBUTE_HIGH);
	gpio_write_io(LED1, LED1_ACTIVE ^ 1);

	gpio_init_io(LED2, GPIO_OUTPUT);
	gpio_set_port_attribute(LED2, ATTRIBUTE_HIGH);
	gpio_write_io(LED2, LED2_ACTIVE ^ 1);

	gpio_init_io(LED3, GPIO_OUTPUT);
	gpio_set_port_attribute(LED3, ATTRIBUTE_HIGH);
	gpio_write_io(LED3, LED3_ACTIVE ^ 1);

	led_red_flag		= LED_OFF;
	led_green_flag		= LED_OFF;
	led_blue_flag		= LED_OFF;

	//LED init放到其他地方去
#if 1
	type				= LED_INIT;
	msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
#endif

	sys_registe_timer_isr(LED_blanking_isr);		//timer base c to start adc convert
}


extern INT8U card_space_less_flag;
extern volatile INT8U pic_down_flag;
static INT8U ir_power_flag = 0;
void set_led_mode(LED_MODE_ENUM mode)
{
	INT8U i;
	static INT8U prev_mode = 0xaa;

	// INT32U type;
	static INT8U led_r_b_flag = 0;
	INT8U led_red_cnt_set;

	led_mode			= mode;
	g_led_g_state		= 0;						//3oE??÷oiAAA
	g_led_r_state		= 0;
	g_led_b_state		= 0;
	g_led_flicker_state = 0;
	g_led_flicker_group = 0;

	//目前做法是容量小于设置的最低容量后，灯不在给出任何响应
	//if(card_space_less_flag)
	//return;
	Led_Work_Flag		= 1;

	if (Low_Voltage_Flag != 0)
		mode = LED_LVD_POWER_OFF;

	switch ((INT32U)mode)
	{
		case LED_INIT: //初始化
			//sys_release_timer_isr(LED_blanking_isr);
//			led_red_on();
//			led_blue_on();

			//sys_registe_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_INIT\r\n");
			break;

		case LED_UPDATE_PROGRAM: //开始升级
			led_all_off();
			g_led_r_state = 1;
			DBG_PRINT("led_type = LED_UPDATE_PROGRAM\r\n");
			break;

		case LED_UPDATE_FINISH: //升级完成
			led_all_off();
			led_red_on();
			DBG_PRINT("led_type = LED_UPDATE_FINISH\r\n");
			break;

		case LED_UPDATE_FAIL: //升级失败
			sys_release_timer_isr(LED_blanking_isr);

			for (i = 0; i < 2; i++)
			{
				led_all_off();
				OSTimeDly(15);
				led_red_on();
				OSTimeDly(15);
				led_all_off();
			}

			Led_Work_Flag = 0;
			DBG_PRINT("led_type = LED_UPDATE_FAIL\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
			ap_peripheral_handling_power_off(); //升级失败关机
			break;

		case LED_USB_CONNECT: //连接USB	
//			if (prev_mode != mode)
//			{
//				g_led_count 		= 0;
//			}

//			led_red_on();
//			g_led_b_state = 2;
//			g_led_flicker_state = 0;
//			DBG_PRINT("led_type = LED_USB_CONNECT\r\n");
			break;

		case LED_RECORD: //录像
//			led_all_off();

			//g_led_flicker_state = 1;
//			DBG_PRINT("led_type = LED_RECORD\r\n");
			break;

		case LED_SDC_FULL: //卡满
//			if (sys_pwr_key1_read())
//			{
//				Led_Work_Flag		= 0;
//				break;
//			}

//			g_led_r_state = 2;
//			g_led_b_state = 2;
//			g_led_flicker_state = 1;
//			DBG_PRINT("led_type = LED_SDC_FULL\r\n");
			break;

		case LED_WAITING_RECORD: //录像待机
			//sys_release_timer_isr(LED_blanking_isr);
			//sys_registe_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_WAITING_RECORD\r\n");

			//break;
		case LED_POWER_OFF: //关机
//			DBG_PRINT("led_type = LED_POWER_OFF\r\n");
//			sys_release_timer_isr(LED_blanking_isr);
//			led_all_off();

//			for (i = 0; i < 3; i++)
//			{
//				OSTimeDly(20);
//				led_red_on();
//				OSTimeDly(20);
//				led_red_off();
//			}

//			OSTimeDly(50);
//			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_AUDIO_RECORD: //录音
//			g_led_r_state = 0;
//			g_led_flicker_state = 0;
//			led_all_off();
//			DBG_PRINT("led_type = LED_AUDIO_RECORD\r\n");
			break;

		case LED_WAITING_AUDIO_RECORD: //录音待机		
//			led_all_off();
//			DBG_PRINT("led_type = LED_WAITING_AUDIO_RECORD\r\n");
			break;

		case LED_CAPTURE: //拍照
			//sys_release_timer_isr(LED_blanking_isr);
//			led_blue_on();
//			led_red_off();

			//sys_registe_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_CAPTURE\r\n");
			break;

		case LED_CARD_DETE_SUC: //检测到卡
//			if (storage_sd_upgrade_file_flag_get() == 2)
//			{
//				Led_Work_Flag		= 0;
//				break;
//			}

//			DBG_PRINT("led_type = LED_CARD_DETE_SUC\r\n");
			break;

		case LED_CAPTURE_FAIL: //拍照失败
//			for (i = 0; i < 2; i++)
//			{
//				led_all_off();
//				OSTimeDly(50);
//				led_red_on();
//				OSTimeDly(50);
//			}
			break;
		case LED_WAITING_CAPTURE: //拍照待机
//			if (AutoTakePic_Mode)
//				led_all_off();
//			else 
//			{
//				led_red_off();
//				led_blue_on();
//			}

//			DBG_PRINT("led_type = LED_WAITING_CAPTURE\r\n");
			break;

		case LED_MOTION_DETECTION: //移动侦测		
//			led_red_off();
//			led_blue_off();
//			DBG_PRINT("led_type = LED_MOTION_DETECTION\r\n");
			break;

		case LED_MOTION_MODE:
//			if (sence == C_PHOTO)
//			{
//				led_red_off();
//				led_blue_on();
//			}
//			else if (sence == C_VIDEO)
//			{
//				led_red_on();
//				led_blue_off();
//			}
//			else 
//			{
//				led_red_on();
//				led_blue_on();
//			}

//			DBG_PRINT("led_type = LED_MOTION_MODE\r\n");
			break;

		case LED_MOTION_WAITING:
//			g_led_b_state = 2;
//			g_led_r_state = 2;
//			g_led_flicker_state = 0;
//			DBG_PRINT("led_type = LED_MOTION_WAITING\r\n");
			break;

		case LED_NO_SDC: //无卡
//			if (sys_pwr_key1_read())
//			{
//				Led_Work_Flag		= 0;
//				break;
//			}

//			led_all_off();
//			g_led_r_state = 2;
//			g_led_flicker_state = 0;
//			DBG_PRINT("led_type = LED_NO_SDC\r\n");
			break;

		case LED_TELL_CARD: //无卡时按键闪灯状态
			//sys_release_timer_isr(LED_blanking_isr);
			//DBG_PRINT("led_type = LED_TELL_CARD\r\n");
			//sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_CARD_NO_SPACE: //卡内剩余空间不够
			//led_all_off();
//			if (storage_sd_upgrade_file_flag_get() == 2)
//				break;

//			g_led_r_state = 2;
//			g_led_b_state = 2;
//			g_led_flicker_state = 1;
//			DBG_PRINT("led_type = LED_CARD_NO_SPACE\r\n");
			break;

		case LED_LVD_POWER_OFF: //低电压关机
			//sys_release_timer_isr(LED_blanking_isr);
//			led_all_off();
//			DBG_PRINT("led_type = LED_LVD_POWER_OFF\r\n");

			//sys_registe_timer_isr(LED_blanking_isr);
			//msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL); //低电压关机 
			break;

		case LED_CHARGE_FULL: //电已充满
//			led_red_off();
//			led_blue_on();
//			charge_state_flag = 0;
//			DBG_PRINT("led_type = LED_CHARGE_FULL\r\n");
			break;

		case LED_CHARGEING: //正在充电
//			led_red_off();

			//ir_off();
//			g_led_b_state = 2;

			//g_led_flicker_state = 1;
//			charge_state_flag = 1;
//			DBG_PRINT("led_type = LED_CHARGEING\r\n");
			break;

		case LED_RECORD_READY: //录像准备
//			sys_release_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_RECORD_READY\r\n");
//			led_all_off();
//			OSTimeDly(20);

//			for (i = 0; i < 3; i++)
//			{
//				led_blue_on();

//				//ir_on();
//				OSTimeDly(20);
//				led_blue_off();

//				//ir_off();
//				OSTimeDly(20);
//			}

//			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_STATUS_INDICATORS: //状态提示
//			sys_release_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_STATUS_INDICATORS\r\n");

//			for (i = 0; i < 3; i++)
//			{
//				led_red_on();
//				OSTimeDly(20);
//				led_red_off();
//				OSTimeDly(20);
//			}

//			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_IR_STATUS:
//			sys_release_timer_isr(LED_blanking_isr);
//			DBG_PRINT("led_type = LED_IR_STATUS\r\n");

			//---------------------------------------
			//保存指示灯状态
//			if (led_red_flag)
//			{
//				//led_red_cnt_set = 1;
//				led_r_b_flag		|= 0x01;
//			}
//			else 
//			{
//				//led_red_cnt_set = 1;
//				led_r_b_flag		&= ~0x01;
//			}

//			if (led_blue_flag)
//				led_r_b_flag |= 0x02;
//			else 
//				led_r_b_flag &= ~0x02;

//			if (ir_power_flag)
//				led_red_cnt_set = 1; //关IR时, 闪1次
//			else 
//				led_red_cnt_set = 2; //关IR时, 闪2次

			//---------------------------------------
//			led_blue_off();

//			for (i = 0; i < led_red_cnt_set; i++)
//			{
//				if (led_r_b_flag & 0x01)
//					led_red_off();
//				else 
//					led_red_on();

//				OSTimeDly(20);

//				if (led_r_b_flag & 0x01)
//					led_red_on();
//				else 
//					led_red_off();

//				OSTimeDly(20);
//			}

//			ir_power_flag ^= 1;

//			if (ir_power_flag)
//				led_green_on(); //红外灯开
//			else 
//				led_green_off(); //红外灯关

			//--------------------------------------------
			//恢复指示灯状态
//			if (led_r_b_flag & 0x01)
//				led_red_on();
//			else 
//				led_red_off();

//			if (led_r_b_flag & 0x02)
//				led_blue_on();
//			else 
//				led_blue_off();

			//--------------------------------------------
//			sys_registe_timer_isr(LED_blanking_isr);

			/*
			if (video_record_sts&0x2) 
			{//如果是在录像, 恢复之前的录像闪灯
				led_red_off();
				led_blue_off();
			}
			*/
			break;
	}

	Led_Work_Flag		= 0;
	prev_mode			= mode;
}


void led_red_on(void)
{
	if (led_red_flag != LED_ON)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 0);
		led_red_flag		= LED_ON;
	}
}


void led_green_on(void)
{
	if (led_green_flag != LED_ON)
	{
		gpio_write_io(LED3, LED3_ACTIVE ^ 0);
		led_green_flag		= LED_ON;
	}
}


void ir_on(void)
{
	if (led_green_flag != LED_ON)
	{
		gpio_write_io(LED3, LED3_ACTIVE ^ 0);
		led_green_flag		= LED_ON;
	}
}


void led_blue_on(void)
{
	if (led_blue_flag != LED_ON)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 0);
		led_blue_flag		= LED_ON;
	}
}


void led_all_off(void)
{
	if (led_blue_flag != LED_OFF)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 1);
		led_blue_flag		= LED_OFF;
	}

	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 1);
		led_red_flag		= LED_OFF;
	}

	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED3, LED3_ACTIVE ^ 1);
		led_green_flag		= LED_OFF;
	}
}


void led_green_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED3, LED3_ACTIVE ^ 1);
		led_green_flag		= LED_OFF;
	}
}


void ir_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED3, LED3_ACTIVE ^ 1);
		led_green_flag		= LED_OFF;
	}
}


void led_blue_off(void)
{
	if (led_blue_flag != LED_OFF)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 1);
		led_blue_flag		= LED_OFF;
	}
}


void led_red_off(void)
{
	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 1);
		led_red_flag		= LED_OFF;
	}
}


extern INT8U video_stop_flag;

void LED_blanking_isr(void)
{
	// INT8U type=NULL;
	static INT8U charge_flag = 0;
	static INT16U charge_led_cnt = 0;

	//if(card_space_less_flag)
	//	return;
	if (g_led_count++ == 105)
	{
		g_led_count 		= 0;
	}

	if (video_stop_flag)
		return;

	if (g_led_r_state == 1)
	{ //=0;默认RB, =1;RG, =2,BG

		if (g_led_count % 10 == 0)
		{
			if (up_firmware_flag == 1)
			{
				led_red_off();
				up_firmware_flag	= 0;
			}
			else 
			{
				led_red_on();
				up_firmware_flag	= 1;
			}
		}
	}
	else if (g_led_r_state == 2)
	{
		if (g_led_flicker_group != 2)
		{
			if (charge_state_flag == 0)
			{
				if (g_led_count / 53 == g_led_flicker_state)
					led_red_on();
				else 
					led_red_off();
			}

			/*
			else				
			{
				if (g_led_count == 0)
				{
					if (++charge_led_cnt >= 2)
					{
						charge_flag ^= 1;
						charge_led_cnt = 0;
					}
				}
				if (charge_flag)
					led_red_on();
				else
					led_red_off();
			}
			*/
		}
		else 
		{
			if (g_led_count < 53)
				led_red_on();
			else 
				led_red_off();
		}
	}
	else if (g_led_r_state == 3)
	{
		if (g_led_count % 27 == 0)
		{
			if (flash_flag == 0)
			{
				led_red_on();

				if (g_led_flicker_group == 3)
					flash_flag = 1;
			}
			else 
			{
				led_red_off();

				if (g_led_flicker_group == 3)
					flash_flag = 0;
			}
		}
	}

	if (g_led_b_state == 2)
	{ //=0;默认RB, =1;RG, =2,BG

		if (g_led_flicker_group == 2)
		{
			if (g_led_count / 53 == g_led_flicker_state)
				led_blue_on();
			else 
				led_blue_off();
		}
		else 
		{
			if (charge_state_flag == 0)
			{
				if (g_led_count < 53)
					led_blue_on();
				else 
					led_blue_off();
			}
			else 
			{
				if (g_led_count == 0)
				{
					if (++charge_led_cnt >= 2)
					{
						charge_flag 		^= 1;
						charge_led_cnt		= 0;
					}
				}

				if (charge_flag)
					led_blue_on();
				else 
					led_blue_off();
			}
		}
	}
	else if (g_led_b_state == 3)
	{
		if (g_led_count % 27 == 0)
		{
			if (flash_flag == 0)
			{
				led_blue_on();

				if (g_led_flicker_group == 2)
					flash_flag = 1;
			}
			else 
			{
				led_blue_off();

				if (g_led_flicker_group == 2)
					flash_flag = 0;
			}
		}
	}

	if (g_led_g_state == 2)
	{
		if (g_led_count < 53)
			led_green_on();
		else 
			led_green_off();
	}
	else if (g_led_g_state == 3)
	{
		if (g_led_count % 27 == 0)
		{
			if (flash_flag == 0)
				led_green_on();
			else 
				led_green_off();
		}
	}

	//g_led_flicker_group
	//	ap_peripheral_key_judge();
	// msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SINGLE_JUGE, &type, sizeof(INT8U), MSG_PRI_NORMAL);
}




#if C_MOTION_DETECTION			== CUSTOM_ON

void ap_peripheral_motion_detect_judge(void)
{
	INT32U result;

	//DBG_PRINT("-\r\n");//fan
	if (video_down_flag)
		return;

	result				= hwCdsp_MD_get_result();
	DBG_PRINT("MD_result = 0x%x\r\n", result);

	//if(result>0x40){MD_SENSITIVE
	if (result > MD_SENSITIVE)
	{ //lx 2015-09-14
		msgQSend(ApQ, MSG_APQ_MOTION_DETECT_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
	}
}


void ap_peripheral_motion_detect_start(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_START);
}


void ap_peripheral_motion_detect_stop(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_STOP);
}


#endif

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*ad_detect_isr) (INT16U data))
{
#if C_BATTERY_DETECT				== CUSTOM_ON
	battery_value_sum	= 0;
	bat_ck_cnt			= 0;
#endif

	//	ad_value_cnt = 0;
	adc_init();
	adc_vref_enable_set(TRUE);

	//adc_conv_time_sel(1);//lx
	adc_conv_time_sel(4);
	adc_manual_ch_set(adc_channel);
	adc_manual_callback_set(ad_detect_isr);

	if (ad_detect_timerid == 0xFF)
	{
		ad_detect_timerid	= AD_DETECT_TIMER_ID;
		sys_set_timer((void *) msgQSend, (void *) PeripheralTaskQ, MSG_PERIPHERAL_TASK_AD_DETECT_CHECK, ad_detect_timerid, PERI_TIME_INTERVAL_AD_DETECT);
	}
}


void ap_peripheral_ad_check_isr(INT16U value)
{
	ad_value			= value;
}


INT16U adc_key_release_calibration(INT16U value)
{
	return value;
}


void ap_peripheral_clr_screen_saver_timer(void)
{
	key_active_cnt		= 0;
}


#if 0 //(KEY_TYPE == KEY_TYPE1)||(KEY_TYPE==KEY_TYPE2)||(KEY_TYPE==KEY_TYPE3)||(KEY_TYPE==KEY_TYPE4)||(KEY_TYPE==KEY_TYPE5)


#else

/*
	0.41v => 495
	0.39v =>
	0.38v => 460
	0.37v => 
	0.36v => 440
	0.35v =>
	0.34v =>
*/
/*
enum {
	BATTERY_CNT = 8,
	BATTERY_Lv3 = 495*BATTERY_CNT,
	BATTERY_Lv2 = 460*BATTERY_CNT,
	BATTERY_Lv1 = 440*BATTERY_CNT
};
*/
enum 
{
BATTERY_CNT = 8, 
	BATTERY_Lv3 = 2515 * BATTERY_CNT,				//4V	
BATTERY_Lv2 = 2319 * BATTERY_CNT,					//3.75V
BATTERY_Lv1 = 2175 * BATTERY_CNT					//3.55V 
};


#if USE_ADKEY_NO				== 6
static INT32U adc_key_factor_table[USE_ADKEY_NO] =
{ // x1000

	// 6 AD-keys
	//680K, 300K, 150K, 68K, 39K, 22K
	1969, 2933, 4182, 5924, 7104, 8102
};


#else

static INT32U adc_key_factor_table[USE_ADKEY_NO] =
{ // x1000

	// 1 AD-keys
	//680K
	1969
};


#endif

//static INT32U ad_time_stamp;
INT32U adc_key_judge(INT32U adc_value)
{
	INT32U candidate_key;
	INT32U candidate_diff;
	INT32U i, temp1, temp2, temp3, diff;

	candidate_key		= USE_ADKEY_NO;
	candidate_diff		= 0xFFFFFFFF;
	temp1				= 1000 * adc_value; 		// to avoid "decimal point"

	temp2				= adc_key_release_calibration(adc_key_release_value_stable); // adc_battery_value_stable = stable adc value got when no key press

	// temp2: adc theoretical value 
	for (i = 0; i < USE_ADKEY_NO; i++)
	{
		temp3				= temp2 * adc_key_factor_table[i]; // temp3: the calculated delimiter	

		if (temp1 >= temp3)
		{
			diff				= temp1 - temp3;
		}
		else 
		{
			diff				= temp3 - temp1;
		}

		// DBG_PRINT("adc:[%d], bat:[%d], diff:[%d]\r\n", temp1, temp3, diff);
		if (diff > candidate_diff)
		{
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);

			break;
		}

		candidate_key		= i;
		candidate_diff		= diff;
	}

	if (candidate_key < USE_ADKEY_NO)
	{
		//DBG_PRINT("\r\nKey %d", candidate_key+1);
		//		power_off_time_beep_1 = 0;
		//		power_off_time_beep_2 = 0;
		//		power_off_time_beep_3 = 0;
#if C_SCREEN_SAVER						== CUSTOM_ON
		key_active_cnt		= 0;
		ap_peripheral_lcd_backlight_set(BL_ON);
#endif
	}

	return candidate_key;
}


//#define SA_TIME	50	//seconds, for screen saver time. Temporary use "define" before set in "STATE_SETTING".
#define SPI_SAVE_FILE_THRESHOLD 0x0888	//0x0863	//低电压自动保存文件
#define SPI_LVD_THRESHOLD		0x0860	//3.47V , 0x0830	//0x080C	//0x081F	//自动关机门槛电压
#define LVD_SAVE_FILE_TIME_LIMIT 128/PERI_TIME_INTERVAL_AD_DETECT*60*1	 //低电压保存时, 至少录像1分钟才会执行	  

void ap_peripheral_ad_key_judge(void)
{
	static INT32U adc_val_cnt = 0;
	static INT32U adc_val_sum = 0;
	static INT32U low_voltage_cnt_1 = 0;
	static INT32U low_voltage_cnt_2 = 0;
	static INT8U adc_save_file_flag = 1;
	static INT8U Start_LowVol_PowOff_Flag = 0;
	INT8U type;

	adc_val_cnt++;
	adc_val_sum 		+= (ad_value >> 4);

	if (adc_val_cnt >= 64)
	{
		adc_val_sum 		= (adc_val_sum >> 6);	//

#if C_BATTERY_DETECT					== CUSTOM_ON

		if (adc_val_sum >= 0x10000)
		{
			adc_val_sum 		= 0xFF00;
		}

		DBG_PRINT("adc_val_sum= 0x%4X!\r\n", adc_val_sum);

		if (Start_LowVol_PowOff_Flag)
			return;

		if ((adc_val_sum <= SPI_SAVE_FILE_THRESHOLD) && (adc_val_sum > SPI_LVD_THRESHOLD))
		{
			if (++low_voltage_cnt_1 >= 3)
			{
				if (adc_save_file_flag == 1)
				{
					if (task_peripheral_lvd_save_cnt_get() > LVD_SAVE_FILE_TIME_LIMIT)
					{
						if ((!pic_down_flag) && (!video_down_flag))
						{ //检测是否在录像, 录像保存或拍照保存状态

							if (video_record_sts & 0x2)
							{ //先保存视频文件,再重新开始录像
								DBG_PRINT("LOW VOLTAGE SAVE 1!\r\n");
								msgQSend(ApQ, MSG_APQ_VDO_REC_RESTART, NULL, NULL, MSG_PRI_NORMAL);
							}
						}

						adc_save_file_flag	= 0;
					}
				}
			}
		}
		else 
			low_voltage_cnt_1 = 0;

		if (adc_val_sum <= SPI_LVD_THRESHOLD)
		{
			DBG_PRINT("LOW VOLTAGE CNT = %d!\r\n", low_voltage_cnt_2);

			if (++low_voltage_cnt_2 >= 3)
			{
				if (!video_down_flag)
				{ //检测是否在录像, 录像保存或拍照保存状态

					if (video_record_sts & 0x2)
					{ //先保存视频文件
						DBG_PRINT("LOW VOLTAGE SAVE 2!\r\n");
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
					else 
					{ //等待保存完成

						//msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
						//type = FALSE;
						//msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U), MSG_PRI_NORMAL);
						DBG_PRINT("LOW VOLTAGE POWER OFF!\r\n");
						type				= BETTERY_LOW_STATUS_KEY;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
						type				= LED_LVD_POWER_OFF; //低压关机指示及关机控制 
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						Start_LowVol_PowOff_Flag = 1;
					}
				}

				Low_Voltage_Flag	= 1;
			}
		}
		else 
		{
			//EncordVideo_Unlock_Flag = 1;
			low_voltage_cnt_2	= 0;
		}

		//ap_peripheral_battery_check_calculate(adc_val_sum);
#endif

		adc_val_cnt 		= 0;
		adc_val_sum 		= 0;
	}

	adc_manual_sample_start();
}


#endif // AD-Key

#endif

#if C_BATTERY_DETECT			== CUSTOM_ON

INT32U previous_direction = 0;
extern void ap_state_handling_led_off(void);
extern INT8U display_str_battery_low;


#define BATTERY_GAP 			10*BATTERY_CNT
static INT8U ap_peripheral_smith_trigger_battery_level(INT32U direction)
{
	static INT8U bat_lvl_cal_bak = (INT8U)
	BATTERY_Lv3;
	INT8U bat_lvl_cal;

	DBG_PRINT("battery_value_sum(%d)\r\n", battery_value_sum);

	if (battery_value_sum >= BATTERY_Lv3)
	{
		bat_lvl_cal 		= 3;
	}
	else if ((battery_value_sum < BATTERY_Lv3) && (battery_value_sum >= BATTERY_Lv2))
	{
		bat_lvl_cal 		= 2;
	}
	else if ((battery_value_sum < BATTERY_Lv2) && (battery_value_sum >= BATTERY_Lv1))
	{
		bat_lvl_cal 		= 1;
	}
	else if (battery_value_sum < BATTERY_Lv1)
	{
		bat_lvl_cal 		= 0;
	}

	if ((direction == 0) && (bat_lvl_cal > bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 + BATTERY_GAP)
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv3 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv2 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv1 + BATTERY_GAP)
		{
			bat_lvl_cal 		= 0;
		}
	}


	if ((direction == 1) && (bat_lvl_cal < bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 - BATTERY_GAP)
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv3 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv2 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv1 - BATTERY_GAP)
		{
			bat_lvl_cal 		= 0;
		}
	}

	bat_lvl_cal_bak 	= bat_lvl_cal;
	return bat_lvl_cal;

}


void ap_peripheral_battery_check_calculate(INT32U adc_val)
{
	INT8U bat_lvl_cal;
	INT32U direction	= 0;
	INT8U type;

	if (adp_status == 0)
	{ //unkown state
		return;
	}
	else if (adp_status == 1)
	{ //adaptor in state
		direction			= 1;					//low voltage to high voltage

		if (previous_direction != direction)
		{
			//msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_SHOW, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}
	else 
	{ //adaptor out state
		direction			= 0;					//high voltage to low voltage

		if (previous_direction != direction)
		{
			//msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_CLEAR, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}

	battery_value_sum	+= adc_val;

	// DBG_PRINT("%d, ",adc_val);
	bat_ck_cnt++;

	if (bat_ck_cnt >= BATTERY_CNT)
	{

		bat_lvl_cal 		= ap_peripheral_smith_trigger_battery_level(direction);

		//DBG_PRINT("%d,", bat_lvl_cal);
		//if(!battery_low_flag)
		//{
		//	msgQSend(ApQ, MSG_APQ_BATTERY_LVL_SHOW, &bat_lvl_cal, sizeof(INT8U), MSG_PRI_NORMAL);
		//}
		if (bat_lvl_cal == 0 && direction == 0)
		{
			DBG_PRINT("bat_lvl_cal(%d)\r\n", bat_lvl_cal);
			DBG_PRINT("battery_low_flag(%d)\r\n", battery_low_flag);

#if C_BATTERY_LOW_POWER_OFF 				== CUSTOM_ON

			if ((video_record_sts & 0x2) || (!pic_down_flag) || (!video_down_flag))
			{ //检测是否在录像, 录像保存或拍照保存状态

				if (video_record_sts & 0x2)
				{ //先保存视频文件
					msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
				}
			}
			else if (!battery_low_flag)
			{
				battery_low_flag	= 1;
				msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
				type				= FALSE;
				msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U), MSG_PRI_NORMAL);
				type				= BETTERY_LOW_STATUS_KEY;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			}

			//display_str_battery_low = 1;
			type				= LED_LVD_POWER_OFF; //低压关机指示及关机控制 
			msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
#endif
		}
		else 
		{
			if (battery_low_flag)
			{
				INT8U type;
				battery_low_flag	= 0;
				msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
				type				= GENERAL_KEY;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			}

			low_voltage_cnt 	= 0;
		}

		bat_ck_cnt			= 0;
		battery_value_sum	= 0;
	}
}


#endif




#if C_SCREEN_SAVER				== CUSTOM_ON

void ap_peripheral_auto_off_force_disable_set(INT8U auto_off_disable)
{
	auto_off_force_disable = auto_off_disable;
}


void ap_peripheral_lcd_backlight_set(INT8U type)
{
	if (type == BL_ON)
	{
		if (lcd_bl_sts)
		{
			lcd_bl_sts			= 0;

#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(TRUE);
#endif

			DBG_PRINT("LCD ON\r\n");
		}
	}
	else 
	{
		if (!lcd_bl_sts)
		{
			lcd_bl_sts			= 1;

#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(FALSE);
#endif

			DBG_PRINT("LCD OFF\r\n");
		}
	}
}


#endif

void ap_peripheral_night_mode_set(INT8U type)
{
	if (type)
	{
		gpio_write_io(IR_CTRL, 1);
	}
	else 
	{
		gpio_write_io(IR_CTRL, 0);
	}
}


void ap_peripheral_key_init(void)
{
	INT32U i;

	gp_memset((INT8S *) &key_map, NULL, sizeof(KEYSTATUS));
	ap_peripheral_key_register(GENERAL_KEY);

	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_io)
		{
			key_map[i].key_cnt	= 0;
			gpio_init_io(key_map[i].key_io, GPIO_INPUT);
			gpio_set_port_attribute(key_map[i].key_io, ATTRIBUTE_LOW);

			// gpio_write_io(key_map[i].key_io, KEY_ACTIVE^1);
			gpio_write_io(key_map[i].key_io, key_map[i].key_active ^ 1); //Liuxi modified in 2015-01-15
			DBG_PRINT("INIT\r\n");
		}
	}
}


void ap_peripheral_key_register(INT8U type)
{
	INT32U i;

	if (type == GENERAL_KEY)
	{
		key_map[0].key_io	= VIDEO_KEY;
		key_map[0].key_function = (KEYFUNC)ap_peripheral_video_key_exe;
		key_map[0].key_active = VIDEO_KEY_ACTIVE;	//1;
		key_map[0].long_key_flag = 0;
		key_map[0].double_key_flag = 0;

		/*
		key_map[1].key_io = VIDEO_KEY;
		key_map[1].key_function = (KEYFUNC) ap_peripheral_video_key_exe;
		key_map[1].key_active=VIDEO_KEY_ACTIVE;//1;
		key_map[1].long_key_flag=0;
		key_map[1].double_key_flag = 0;

		key_map[2].key_io = MOTION_KEY;
		key_map[2].key_function = (KEYFUNC) ap_peripheral_motion_key_exe;
		key_map[2].key_active=MOTION_KEY_ACTIVE;//1;
		key_map[2].long_key_flag=0;
		key_map[2].double_key_flag = 0;

		key_map[2].key_io = RECORD_KEY;
		key_map[2].key_function = (KEYFUNC) ap_peripheral_rec_key_exe;
		key_map[2].key_active=RECORD_KEY_ACTIVE;//1;
		key_map[2].long_key_flag=0;
		key_map[2].double_key_flag = 0;
		*/
		ad_key_map[0].key_io = FUNCTION_KEY;
		ad_key_map[0].key_function = (KEYFUNC)
		ap_peripheral_null_key_exe;
	}
	else if (type == USBD_DETECT)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			if (key_map[i].key_io != VIDEO_KEY)
				key_map[i].key_io = NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == DISABLE_KEY)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			key_map[i].key_io	= NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == BETTERY_LOW_STATUS_KEY)
	{
		key_map[0].key_io	= PW_KEY;
		key_map[0].key_function = (KEYFUNC)
		ap_peripheral_pw_key_exe;

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}

}


extern INT8U ap_state_config_auto_off_get(void);

INT8U long_pw_key_pressed = 0;

#if CRAZY_KEY_TEST				== 1
INT8U crazy_key_enable = 0;
INT32U crazy_key_cnt = 0;
#endif

void ap_peripheral_charge_det(void)
{
	INT16U pin_state	= 0;
	INT16U led_type;
	static INT8U loop_cnt = 0;
	pin_state			= gpio_read_io(CHARGE_DETECTION_PIN);

	//DBG_PRINT("pin_state=%d",pin_state);
	if (++loop_cnt >= 2)
	{
		loop_cnt			= 0;

		if (pin_state)
			led_type = LED_CHARGE_FULL;
		else 
			led_type = LED_CHARGEING;

		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
	}
}


INT8U read_pw_key0(void)
{
	if (sys_pwr_key0_read() != 0)
		return 1;

	else 
		return 0;
}


#define DOUBLE_PRESS_KEY_DELAY	48//750ms
void ap_peripheral_key_judge(void)
{
	INT32U i;										//key_press = 0;
	INT16U key_down 	= 0;
	static INT32U double_key_ovtime_cnt = 0;

	//static INT32U type = NULL;
	if (KeyScan_Wait_Flag)
	{
		//OSTimeDly(10);
		return;
	}

	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_active)
			key_down = gpio_read_io(key_map[i].key_io);
		else 
			key_down = !gpio_read_io(key_map[i].key_io);

		if (key_down)
		{
			if (! (key_map[i].long_key_flag))
			{
				key_map[i].key_cnt++;

#if SUPPORT_LONGKEY 							== CUSTOM_ON

				if (key_map[i].key_cnt >= Long_Single_width)
				{
					key_map[i].long_key_flag = 1;
					key_map[i].key_function(& (key_map[i].key_cnt));
				}

#endif
			}

			if (key_map[i].key_cnt == 65535)
			{
				key_map[i].key_cnt	= Long_Single_width + 10;
			}

			task_peripheral_handling_idle_count_set(0); //清除自动关机计数
		}
		else 
		{
			if (key_map[i].long_key_flag == 1)
			{
				key_map[i].long_key_flag = 0;
				key_map[i].key_cnt	= 0;
			}
			else if (key_map[i].key_cnt >= Short_Single_width)
			{ //短按
				key_map[i].key_function(& (key_map[i].key_cnt));
			}
			else 
				key_map[i].key_cnt = 0; //未达到短按时间时, 计数清零
		}
	}
}


void ap_peripheral_handling_power_off(void)
{
	led_all_off();

	if (s_usbd_pin)
		return;

	while (Led_Work_Flag || video_down_flag)
	{
		OSTimeDly(1);
	}

	/*
	while(read_pw_key0());
	{//等待松开关机键
		OSTimeDly(1);
	}
	*/
	LDO12_Switch(0);								//disable internal power
	OSTimeDly(1);
	power_pin_set(0);
	led_all_off();
	DBG_PRINT("[POWER OFF]\r\n");

	while (1)
		;

	OSTimeDly(100);
}


/*	已移动到上面
static int ap_peripheral_power_key_read(int pin)
{
	int status;


	#if  (KEY_TYPE == KEY_TYPE1)||(KEY_TYPE == KEY_TYPE2)||(KEY_TYPE == KEY_TYPE3)||(KEY_TYPE == KEY_TYPE4)||(KEY_TYPE
	 == KEY_TYPE5)||(KEY_TYPE == KEY_TYPE10)
		status = gpio_read_io(pin);
	#else
	switch(pin)
	{
		case PWR_KEY0:
			status = sys_pwr_key0_read();
			break;
		case PWR_KEY1:
			status = sys_pwr_key1_read();
			break;
	}
	#endif

	if (status!=0)
		 return 1;
	else return 0;
}
*/
void ap_peripheral_adaptor_out_judge(void)
{
	adp_out_cnt++;

	switch (adp_status)
	{
		case 0: //unkown state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				adp_cnt++;

				if (adp_cnt > 16)
				{
					adp_out_cnt 		= 0;
					adp_cnt 			= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);

#if C_BATTERY_DETECT								== CUSTOM_ON && USE_ADKEY_NO

					//battery_lvl = 1;
#endif
				}
			}
			else 
			{
				adp_cnt 			= 0;
			}

			if (adp_out_cnt > 24)
			{
				adp_out_cnt 		= 0;
				adp_status			= 3;

#if C_BATTERY_DETECT							== CUSTOM_ON && USE_ADKEY_NO

				//battery_lvl = 2;
				low_voltage_cnt 	= 0;
#endif
			}

			break;

		case 1: //adaptor in state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 8)
				{
					adp_status			= 2;

#if C_BATTERY_DETECT								== CUSTOM_ON
					low_voltage_cnt 	= 0;
#endif

					// 璝棵辊玂臔秨璶翴獹璉
					if (screen_saver_enable)
					{
						screen_saver_enable = 0;

#if C_SCREEN_SAVER										== CUSTOM_ON

						//ap_state_handling_lcd_backlight_switch(1);
#endif
					}
				}
			}
			else 
			{
				adp_out_cnt 		= 0;
			}

			task_peripheral_handling_idle_count_set(0); //清除自动关机计数
			break;

		case 2: //adaptor out state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if ((adp_out_cnt > PERI_ADP_OUT_PWR_OFF_TIME))
				{
					//ap_peripheral_pw_key_exe(&adp_out_cnt);
					//ap_peripheral_handling_power_off();
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
				}

				adp_cnt 			= 0;
			}
			else 
			{
				adp_cnt++;

				if (adp_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					usbd_exit			= 0;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}

			break;

		case 3: //adaptor initial out state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}
			else 
			{
				adp_out_cnt 		= 0;
			}

			break;

		default:
			break;
	}

	///DBG_PRINT("USB_PIN=%d\r\n",s_usbd_pin);
	if (s_usbd_pin == 1)
	{
		usbd_cnt++;

		if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (usbd_cnt > 3)
			{
				ap_peripheral_usbd_plug_out_exe(&usbd_cnt);
			}
		}
		else 
		{
			usbd_cnt			= 0;
		}
	}

#if USB_PHY_SUSPEND 				== 1

	if (s_usbd_pin == 0)
	{
		if (ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (phy_cnt == PERI_USB_PHY_SUSPEND_TIME)
			{
				// disable USB PHY CLK for saving power
				DBG_PRINT("Turn Off USB PHY clk (TODO)\r\n");
				phy_cnt++;							// ヘ琌 Turn Off 暗Ω
			}
			else if (phy_cnt < PERI_USB_PHY_SUSPEND_TIME)
			{
				phy_cnt++;
			}
		}
		else 
		{
			phy_cnt 			= 0;
		}
	}

#endif

}


INT8U ap_peripheral_power_off_flag_get(void)
{
	return PowerOff_Flag;
}


void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MODE, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		DBG_PRINT("function_Key\r\n");

		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			DBG_PRINT("MODE_ACTIVE\r\n");
			DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
			msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U data			= 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_DOWN, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_FORWARD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_NEXT_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U data			= 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_UP, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_BACKWORD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);

		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_PREV_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			//for test
			msgQSend(ApQ, MSG_APQ_INIT_THUMBNAIL, NULL, NULL, MSG_PRI_NORMAL);

			//			msgQSend(ApQ, MSG_APQ_FILE_LOCK_DURING_RECORDING, NULL, NULL, MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_FUNCTION_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


static INT8U pc_camera_flag = 0;

#if C_MOTION_DETECTION			== CUSTOM_ON	
extern void ap_video_record_md_disable(void);
#endif

void ap_peripheral_power_on_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type 	= NULL;

	//INT32U i;
	if (First_PowerOn_Flag == 1)
	{
		*tick_cnt_ptr		= 0;
		return; //开机时POWER键没有松开就返回
	}

	if (PowerOff_Flag == 1)
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	DBG_PRINT("***power_on_key*****\r\n");

	if (!s_usbd_pin)
	{
		if ((ap_state_handling_storage_id_get() != NO_STORAGE))
		{
			if (!card_space_less_flag)
			{ //

#if SUPPORT_LONGKEY 							== CUSTOM_ON

				if (*tick_cnt_ptr >= Long_Single_width)
				{ //关机
					led_type			= LED_POWER_OFF;
					msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					DBG_PRINT("[POWER_KEY ACTIVE...]");
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					PowerOff_Flag		= 1;
					*tick_cnt_ptr		= 0;
					return;
				}
				else 
#endif

				{
					DBG_PRINT("***sence = %d\r\n", sence);

					switch (sence)
					{
						case C_PHOTO:
							if (AutoTakePic_Mode != 0)
							{
								AutoTakePic_Mode	= 0;
								led_type			= LED_WAITING_CAPTURE;
								msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
							}
							else 
							{
								AutoTakePic_Mode	= 1;
								led_red_off();
								led_blue_off();
								OSTimeDly(20);
							}

							break;

						case C_VIDEO:
							if (video_down_flag == 0)
							{
								sence				= C_PHOTO; //切换为拍照模式

								if (video_record_sts & 0x2)
								{ //保存视频
									msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
								}

								led_type			= LED_WAITING_CAPTURE;
								msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
							}

							break;

						case C_MOTION:
							sence = C_PHOTO; //切换为拍照模式

							if (video_record_sts & 0x4)
							{
								if (video_record_sts & 0x2)
								{ //移动侦测待机

									if (video_down_flag == 0)
									{
										msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
									}
								}
								else 
								{ //移动侦测待机

									if (ap_state_config_md_get())
									{
										ap_video_record_md_disable();
										ap_state_config_md_set(0);
									}
								}
							}

							led_type = LED_MOTION_MODE;
							msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
							break;
					}
				}
			}
		}
	}
	else 
	{
#if SUPPORT_LONGKEY 					== CUSTOM_ON

		if (*tick_cnt_ptr >= 64)
		{
			if (++pc_camera_flag >= 2)
			{
				pc_camera_flag		= 0;
			}
		}
		else 
#endif

		{
			if (ap_state_handling_storage_id_get() != NO_STORAGE)
			{
				OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
			}
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_video_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;

#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	DBG_PRINT("***video_key*****\r\n");

	if (s_usbd_pin)
	{
#if SUPPORT_LONGKEY 					== CUSTOM_ON

		if (*tick_cnt_ptr >= Long_Single_width)
		{
			if (++pc_camera_flag >= 2)
			{
				pc_camera_flag		= 0;
			}
		}
		else 
#endif

		{
			if (ap_state_handling_storage_id_get() != NO_STORAGE)
			{
				OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
			}
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_motion_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;

#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	DBG_PRINT("***motion_key*****\r\n");

	if (!s_usbd_pin)
	{
		while ((pic_down_flag != 0) || (video_down_flag != 0))
			;

		{
			OSTimeDly(1);
		}
		DBG_PRINT("***sence = %d\r\n", sence);

		switch (sence)
		{
			case C_PHOTO:
				if (pic_down_flag == 0)
				{
					AutoTakePic_Mode	= 0;
					sence				= C_MOTION; //切换为移动侦测模式
					msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					led_type			= LED_MOTION_MODE;
					msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
				}

				break;

			case C_VIDEO:
				if (video_down_flag == 0)
				{
					sence				= C_MOTION; //切换为移动侦测模式

					if (video_record_sts & 0x2)
					{ //保存视频
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}

					led_type			= LED_MOTION_MODE;
					msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
				}

				break;

			case C_MOTION:
				DBG_PRINT("***video_record_sts = %d\r\n", video_record_sts);

				if ((pic_down_flag == 0) && (video_down_flag == 0))
				{
					if (video_record_sts & 0x4) //(ap_state_config_md_get()) //移动侦测状态
					{
						if (video_record_sts & 0x2)
						{ //移动侦测待机
							msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
						}
						else 
						{ //移动侦测待机

							if (ap_state_config_md_get())
							{
								ap_video_record_md_disable();
								ap_state_config_md_set(0);
							}
						}

						led_type			= LED_MOTION_MODE;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else 
					{ //移动侦测模式未检测
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_MOTION_DETECT_DELAY, NULL, NULL, MSG_PRI_NORMAL);
													//移动侦测延时检测
						msgQSend(ApQ, MSG_APQ_MD_SET, NULL, NULL, MSG_PRI_NORMAL);
					}
				}

				break;
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_ir_switch_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;

#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	if (!s_usbd_pin)
	{
		led_type			= LED_IR_STATUS;
		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
	}
	else 
	{
		led_type			= LED_IR_STATUS;
		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_rec_key_exe(INT16U * tick_cnt_ptr)
{
	// INT32U led_type;
#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	if (!s_usbd_pin)
	{
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
			if ((!pic_down_flag) && (!video_down_flag) && (!card_space_less_flag))
			{
#if SUPPORT_LONGKEY 							== CUSTOM_ON

				if (*tick_cnt_ptr >= Long_Single_width)
				{
					//led_type = LED_IR_STATUS; //打开或关闭红外灯
					//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
				}
				else 
#endif

				{
					if ((video_record_sts & 0x2) == 0)
					{
					}
				}
			}
		}
	}
	else 
	{
#if SUPPORT_LONGKEY 					== CUSTOM_ON

		if (*tick_cnt_ptr >= 64)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
		else 
#endif

		{
			//if(!pic_down_flag)
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}

	*tick_cnt_ptr		= 0;
}


#if KEY_FUNTION_TYPE			== SAMPLE2
void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;								//Sence_Mode

#if !								CHARGING_WHILE_ENCORD_VIDEO

	if (ap_peripheral_handling_usb_state_get())
	{
		*tick_cnt_ptr		= 0;
		return; //关机标志
	}

#endif

	if (!s_usbd_pin)
	{
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
			if ((!pic_down_flag) && (!card_space_less_flag) && (!video_down_flag))
			{
#if SUPPORT_LONGKEY 							== CUSTOM_ON

				if (*tick_cnt_ptr >= Long_Single_width)
				{

				}
				else 
#endif

				{
					if (video_record_sts & 0x2)
					{
						led_type			= LED_STATUS_INDICATORS;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else 
					{
						ir_on();
						OSTimeDly(20);
						ir_off();
						DBG_PRINT("[CAPTUER_ACTIVE...]\r\n");
						msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
				}
			}
		}
	}
	else 
	{
#if SUPPORT_LONGKEY 					== CUSTOM_ON

		if (*tick_cnt_ptr >= 64)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
		else 
#endif

		{
			//if(!pic_down_flag)
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}

	*tick_cnt_ptr		= 0;
}


#endif

INT16U ap_peripheral_uvc_flag(void)
{
	return pc_camera_flag;
}


void ap_peripheral_uvc_flag_set(INT8U flag)
{
	if (pc_camera_flag != flag)
		pc_camera_flag = flag;
}


void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr)
{
#if CRAZY_KEY_TEST					== 1	

	if (!crazy_key_enable)
	{
		crazy_key_enable	= 1;
	}
	else 
	{
		crazy_key_enable	= 0;
	}

	*tick_cnt_ptr		= 0;
	return;

#endif

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
	*tick_cnt_ptr		= 0;
}


void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U led_type;

	if (ap_state_handling_storage_id_get() == NO_STORAGE)
	{

		led_type			= LED_TELL_CARD;
		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
	}

	DBG_PRINT("PIC_FLAG=%d\r\n", pic_down_flag);

	if ((!s_usbd_pin) && (!pic_down_flag) && (!card_space_less_flag) && (!video_down_flag))
	{
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
#if SUPPORT_LONGKEY 						== CUSTOM_ON

			if (*tick_cnt_ptr > 24)
			{

			}
			else 
#endif

			{
				DBG_PRINT("[VIDEO_RECORD_...]\r\n");
				msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr)
{
#if KEY_FUNTION_TYPE				== C6_KEY
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MENU, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

#endif

	* tick_cnt_ptr		= 0;
}


void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr)
{

}


#if GPDV_BOARD_VERSION			!= GPCV1237A_Aerial_Photo
void ap_TFT_backlight_tmr_check(void)
{
	if (backlight_tmr)
	{
		backlight_tmr--;

		if ((backlight_tmr == 0) && (tv == !TV_DET_ACTIVE))
		{
			//gpio_write_io(TFT_BL, DATA_HIGH);	//turn on LCD backlight
#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(1);
#endif
		}
	}
}


#endif

//+++ TV_OUT_D1
#if TV_DET_ENABLE
INT8U tv_plug_status_get(void)
{
	return tv_plug_in_flag;
}


#endif

//---
#if GPDV_BOARD_VERSION			!= GPCV1237A_Aerial_Photo
void ap_peripheral_tv_detect(void)
{
#if TV_DET_ENABLE
	INT8U temp;

	temp				= gpio_read_io(AV_IN_DET);

	if (temp != tv)
	{
		tv_debounce_cnt++;

		if (tv_debounce_cnt > 4)
		{
			tv_debounce_cnt 	= 0;
			tv					= temp;

			if (tv == !TV_DET_ACTIVE)
			{ //display use TFT

				//backlight_tmr = PERI_TIME_BACKLIGHT_DELAY;	//delay some time to enable LCD backlight so that no noise shown on LCD
				gpio_write_io(SPEAKER_EN, DATA_HIGH); //open local speaker

				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 0;
				msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
			else 
			{ //display use TV
				gpio_write_io(SPEAKER_EN, DATA_LOW); //mute local speaker

				//gpio_write_io(TFT_BL, DATA_LOW);		//turn off LCD backlight
				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 1;
				msgQSend(ApQ, MSG_APQ_TV_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
		}
	}
	else 
	{
		tv_debounce_cnt 	= 0;
	}

#endif
}


void ap_peripheral_gsensor_data_register(void)
{
	avi_adc_gsensor_data_register(&gsensor_msgQId0, (INT32U *) (&gsensor_msgId0));
}


void ap_peripheral_read_gsensor(void)
{
	static INT16U g_idx = 0;
	INT16U temp;

	temp				= G_sensor_get_int_active();

	if ((temp != 0xff) && (temp & 0x04)) //active int flag
	{
		G_sensor_clear_int_flag();

		if (ap_state_config_G_sensor_get())
		{
			msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}

		DBG_PRINT("gsensor int actived\r\n");
	}

	if (gsensor_msgQId0 != NULL)
	{
		G_Sensor_gps_data_get(gsensor_data[g_idx]);
		OSQPost((OS_EVENT *) gsensor_msgQId0, (void *) (gsensor_msgId0 | g_idx));
		g_idx				^= 0x1;
	}

	//DBG_PRINT("gsensor chipid = 0x%x\r\n", temp);
}


void ap_peripheral_config_store(void)
{
	if (config_cnt++ == PERI_COFING_STORE_INTERVAL)
	{
		config_cnt			= 0;
		msgQSend(ApQ, MSG_APQ_USER_CONFIG_STORE, NULL, NULL, MSG_PRI_NORMAL);
	}
}


void ap_peripheral_hdmi_detect(void)
{
	static BOOLEAN HDMI_StatusBak = 0;
	static BOOLEAN HDMI_StateBak = 0;				// HDMI_REMOVE
	static unsigned char HDMI_DetCount = 0;
	BOOLEAN cur_status;

	cur_status			= gpio_read_io(HDMI_IN_DET);

	// debounce
	if (HDMI_StatusBak != cur_status)
	{
		HDMI_DetCount		= 0;
	}
	else 
	{
		HDMI_DetCount++;
	}

	if (HDMI_DetCount == 0x10)
	{
		if (cur_status != HDMI_StateBak)
		{
			HDMI_DetCount		= 0;

			if (cur_status) // HDM_IN_DET
			{
				msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
				gpio_write_io(SPEAKER_EN, DATA_LOW); //mute local speaker
				DBG_PRINT("HDMI Insert\r\n");		// HDMI Insert
			}
			else 
			{
				msgQSend(ApQ, MSG_APQ_HDMI_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
				gpio_write_io(SPEAKER_EN, DATA_HIGH);
				DBG_PRINT("HDMI Remove\r\n");		// HDMI Remove
			}
		}

		HDMI_StateBak		= cur_status;
	}

	HDMI_StatusBak		= cur_status;
}


#endif

#ifdef SDC_DETECT_PIN
void ap_peripheral_SDC_detect_init(void)
{

	gpio_init_io(SDC_DETECT_PIN, GPIO_INPUT);
	gpio_set_port_attribute(SDC_DETECT_PIN, ATTRIBUTE_LOW);
	gpio_write_io(SDC_DETECT_PIN, 1);				//pull high
}


INT32S ap_peripheral_SDC_at_plug_OUT_detect()
{
	INT32S ret;
	BOOLEAN cur_status;
	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN_=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in	
		ret 				= 0;
	}

	return ret;
}



INT32S ap_peripheral_SDC_at_plug_IN_detect()
{
	INT32S ret;
	BOOLEAN cur_status;
	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in
		ret 				= 0;
	}

	return ret;
}


#endif

