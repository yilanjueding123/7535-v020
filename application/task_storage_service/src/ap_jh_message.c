#include "ap_storage_service.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"
#include "fs_driver.h"
#include "avi_encoder_app.h"
#include "stdio.h"

extern void save_VERSION_NUMBER_to_disk(void);
extern void save_COPYRIGHT_MESSAGE_to_disk(void);
extern void save_SENSOR_TYPE_to_disk(void);

const INT8U  R_CopyrightMSG_Buf[]= "版权声明: 本产品由深圳市乐信兴业科技有限公司设计,版权所有,仿冒必究"; //
const INT8U   sensor_type_name[9][13]={"OV7670","OV9712","SOI_H22","BF3703","SOI_H22_MIPI","OV3640","OV5642",
	                                   "GC1004","GC1004_MIPI"
};

void save_COPYRIGHT_MESSAGE_to_disk(void)
{
    INT16S fd;
	INT32U addr;
	fd = open("C:\\CopyrightMSG.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(66+1);
	if(!addr)
	{
		close(fd);
		return;
	}
	gp_strcpy((INT8S*)addr, (INT8S *)R_CopyrightMSG_Buf);
	write(fd, addr, 66);
	close(fd);
	gp_free((void*) addr);
}
void save_VERSION_NUMBER_to_disk(void)
{
    INT8U  *p;
	INT16S fd;
	INT32U addr;
	INT32U product_num;
	INT32U data_num;
	product_num = PRODUCT_NUM;
	data_num = PRODUCT_DATA;
	fd = open("C:\\Version.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(22+5);
	if(!addr)
	{
		close(fd);
		return;
	}
	p = (INT8U*)addr;
	sprintf((char *)p, (const char *)"%08d,JH%04d,v0.00,", data_num, product_num);
	p[17] = ((PROGRAM_VERSION_NUM%1000)/100) + '0';
	p[19] = ((PROGRAM_VERSION_NUM%100)/10) + '0';
	p[20] = ((PROGRAM_VERSION_NUM%10)/1) + '0';
	write(fd, addr, 22);
	close(fd);
	gp_free((void*) addr);
}
void save_SENSOR_TYPE_to_disk(void)
{
	//INT8U  *p;
	INT16S fd;
	INT32U addr;
	fd = open("C:\\sensor.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(13+5);
	if(!addr)
	{
		close(fd);
		return;
	}

	gp_strcpy((INT8S*)addr, (INT8S *)sensor_type_name[USE_SENSOR_NAME]);

	write(fd, addr, strlen(sensor_type_name[USE_SENSOR_NAME]));
	close(fd);
	gp_free((void*) addr);
}
