/*
 *  shub-i2c.c - Linux kernel modules for Sensor Hub 
 *
 *  Copyright (C) 2014 LAPIS SEMICONDUCTOR CO., LTD.
 *
 *  This file is based on :
 *    ml610q792.c - Linux kernel modules for acceleration sensor
 *    http://android-dev.kyocera.co.jp/source/versionSelect_URBANO.html
 *    (C) 2012 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/moduleparam.h>
#include <linux/input-polldev.h>

#include <asm/uaccess.h> 
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
//#include <linux/wakelock.h>  /* SHMDS_HUB_0111_01 del */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
//#include <linux/delay.h>
#include <linux/io.h>
#include <linux/poll.h>
//#include <asm/gpio.h>            /* SHMDS_HUB_0111_01 del */
#include <linux/types.h>
#include <linux/sched.h>
//#include <linux/earlysuspend.h> /* SHMDS_HUB_0111_01 del */
#include <linux/miscdevice.h>
#include "ml630q790.h"

//#define SHUB_SW_I2C_LOG                               /* SHMDS_HUB_0323_02 add */

#define SHUB_I2C_WRITE_BUFF_SIZE        (512+128)       /* SHMDS_HUB_0323_02 add */
#define SHUB_I2C_READ_BUFF_SIZE         (FIFO_SIZE+1)   /* SHMDS_HUB_0323_02 add */

static int32_t i2c_probe( struct i2c_client *client, const struct i2c_device_id *id);
static int32_t i2c_remove( struct i2c_client *client );
/* SHMDS_HUB_0130_01 del S */
//static int32_t i2c_suspend( struct i2c_client *client, pm_message_t mesg );
//static int32_t i2c_resume( struct i2c_client *client );
/* SHMDS_HUB_0130_01 del E */
/* SHMDS_HUB_0130_01 mod S */
static int32_t pm_suspend( struct device *dev );
static int32_t pm_resume( struct device *dev );
/* SHMDS_HUB_0130_01 mod E */
static void    i2c_shutdown( struct i2c_client *client );

#ifdef CONFIG_OF
static struct of_device_id shub_match_table[] = {
	{.name = "sensorhub",},
	{ .compatible = "sharp,sensorhub", },
	{ },
};
#else
#define shub_match_table NULL;
#endif /* CONFIG_OF */

// SHMDS_HUB_0130_01 add 
static const struct dev_pm_ops shub_pm_ops = {
    .suspend     = pm_suspend,
    .resume      = pm_resume,
};
// SHMDS_HUB_0130_01 add 

static const struct i2c_device_id i2c_id[] = {
	{ SENOSR_HUB_DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, i2c_id);

static struct i2c_driver interface_driver = {
    .driver = {
        .name    = SENOSR_HUB_DRIVER_NAME,
        .owner   = THIS_MODULE,
        .pm      = &shub_pm_ops,               // SHMDS_HUB_0130_01 add
        .of_match_table = shub_match_table,
    },
    .probe       = i2c_probe,
    .remove      = i2c_remove,
//    .resume      = i2c_resume,
//    .suspend     = i2c_suspend,              // SHMDS_HUB_0130_01 del
    .shutdown    = i2c_shutdown,               // SHMDS_HUB_0130_01 del
    .id_table    = i2c_id,
};

struct i2c_client *this_client=NULL;

/* SHMDS_HUB_0109_02 add S */
static int shub_acc_axis_val = 0;
static int shub_gyro_axis_val = 0;
static int shub_mag_axis_val = 0;
/* SHMDS_HUB_0109_02 add E */

static int32_t i2c_remove( struct i2c_client *client )
{
	printk(KERN_INFO "i2c_remove \n");

    return 0;
}

/* SHMDS_HUB_0130_01 del S */
//static int32_t i2c_suspend( struct i2c_client *client, pm_message_t mesg )
//{
//	printk(KERN_INFO "i2c_suspend \n");
//
//    shub_suspend(client, mesg);
//    return 0;
//}
//
//static int32_t i2c_resume( struct i2c_client *client )
//{
//	printk(KERN_INFO "i2c_resume \n");
//
//    shub_resume(client);
//    return 0;
//}
/* SHMDS_HUB_0130_01 del E */
/* SHMDS_HUB_0130_01 add S */
static int32_t pm_suspend( struct device *dev )
{
//  pm_message_t mesg;
//  shub_suspend(dev, mesg);
    shub_suspend(dev);
    return 0;
}

static int32_t pm_resume( struct device *dev )
{
    shub_resume(dev);
    return 0;
}
/* SHMDS_HUB_0130_01 add E */

static void i2c_shutdown( struct i2c_client *client )
{
	printk(KERN_INFO "i2c_shutdown \n");
}

static int32_t i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
/* SHMDS_HUB_0109_02 add S */
    int rc;
    struct device_node *np = client->dev.of_node;
    u32 temp_val;
    rc = of_property_read_u32(np, "shub,shub_acc_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_acc_axis_val\n");
        shub_acc_axis_val = 0;
    }
    else {
        shub_acc_axis_val = temp_val;
    }

    rc = of_property_read_u32(np, "shub,shub_gyro_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_gyro_axis_val\n");
        shub_gyro_axis_val = 0;
    }
    else {
        shub_gyro_axis_val = temp_val;
    }

    rc = of_property_read_u32(np, "shub,shub_mag_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_mag_axis_val\n");
        shub_mag_axis_val = 0;
    }
    else {
        shub_mag_axis_val = temp_val;
    }
/* SHMDS_HUB_0109_02 add E */
	printk(KERN_INFO "i2c_probe \n");

    shub_set_gpio_no(client); /* SHMDS_HUB_0110_01 add */
	this_client = client;
    return shub_probe();
}

//int32_t init_i2c(void)
//{
//    int32_t ret;
//
//    ret = i2c_add_driver(&interface_driver);
//    if(ret != 0){
//		printk(KERN_INFO "can't regist i2c driver \n");
//    }
//
//    return ret;
//}
//EXPORT_SYMBOL(init_i2c);

/* SHMDS_HUB_3701_01 add S */
int shub_i2c_init(void)
{
    int ret;
    ret = i2c_add_driver(&interface_driver);
    if(ret != 0){
        printk("[shub] can't regist spi driver ret=%d\n", ret);
    }
    return ret;
}

void shub_i2c_exit(void)
{
    i2c_del_driver(&interface_driver);
}
/* SHMDS_HUB_3701_01 add E */
MODULE_ALIAS("i2c:" SENOSR_HUB_DRIVER_NAME);

/* SHMDS_HUB_0323_02 mod S */
#ifdef SHUB_SW_I2C_LOG
static void hostif_write_log(uint8_t adr, const uint8_t *data, uint16_t size)
{
    int len1,len2,ofs;
    char work1[256];
    char work2[8];
    
    printk(KERN_INFO "[shub-i2c]write adr=0x%02x, size=%d\n", adr, size);
    printk(KERN_INFO "[shub-i2c]write data=\n");
    
    for( len1=0 ; len1<(size/16) ; len1++ ){
        printk(KERN_INFO "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
               *(data+len1*16+0),
               *(data+len1*16+1),
               *(data+len1*16+2),
               *(data+len1*16+3),
               *(data+len1*16+4),
               *(data+len1*16+5),
               *(data+len1*16+6),
               *(data+len1*16+7),
               *(data+len1*16+8),
               *(data+len1*16+9),
               *(data+len1*16+10),
               *(data+len1*16+11),
               *(data+len1*16+12),
               *(data+len1*16+13),
               *(data+len1*16+14),
               *(data+len1*16+15)
               );
    }
    if( size%16 ){
        memset(work1, 0, sizeof(work1));
        ofs=0;
        for(len2=0;len2<(size%16);len2++){
            memset(work2, 0, sizeof(work2));
            sprintf(work2, "%02x ", *(data+len2));
            strcat(work1, work2);
        }
        printk(KERN_INFO "%s", work1);
    }
}
#endif

static int32_t hostif_write_exe(uint8_t adr, uint8_t *send_data, uint16_t size)
{
    struct i2c_msg msgs[] = {
        {
            .addr = this_client->addr,
            .flags =0,
            .len = size,
            .buf = send_data,
        },
    };
    if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
        pr_err("shub-i2c_write: transfer error\n");
        return -EIO;
    }else{
        return 0;
    }
}

int32_t hostif_write_proc(uint8_t adr, const uint8_t *data, uint16_t size)
{
    int32_t ret;
    uint8_t *buf;
    
    buf = kzalloc(SHUB_I2C_WRITE_BUFF_SIZE, GFP_KERNEL);
    if (buf == NULL){
        pr_err("shub-i2c_write: kzalloc error\n");
        return -ENOMEM;
    }
    
#ifdef SHUB_SW_I2C_LOG
    hostif_write_log(adr, data, size);
#endif
    
    buf[0] = adr;
    memcpy(&buf[1], data, size);
    
    ret = hostif_write_exe(adr, buf, size+1);
    
    kfree(buf);
    return ret;
}
EXPORT_SYMBOL(hostif_write_proc);

#ifdef SHUB_SW_I2C_LOG
static void hostif_read_log(uint8_t adr, uint8_t *data, uint16_t size)
{
    int len1,len2,ofs;
    char work1[512];
    char work2[8];
    
    printk(KERN_INFO "[shub-i2c]read adr=0x%02x, size=%d\n", adr, size);
    printk(KERN_INFO "[shub-i2c]read data=\n");
    
    for( len1=0 ; len1<(size/16) ; len1++ ){
        printk(KERN_INFO "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
               *(data+len1*16+0),
               *(data+len1*16+1),
               *(data+len1*16+2),
               *(data+len1*16+3),
               *(data+len1*16+4),
               *(data+len1*16+5),
               *(data+len1*16+6),
               *(data+len1*16+7),
               *(data+len1*16+8),
               *(data+len1*16+9),
               *(data+len1*16+10),
               *(data+len1*16+11),
               *(data+len1*16+12),
               *(data+len1*16+13),
               *(data+len1*16+14),
               *(data+len1*16+15)
               );
    }
    if( size%16 ){
        memset(work1, 0, sizeof(work1));
        ofs=0;
        for(len2=0;len2<(size%16);len2++){
            memset(work2, 0, sizeof(work2));
            sprintf(work2, "%02x ", *(data+len2));
            strcat(work1, work2);
        }
        printk(KERN_INFO "%s", work1);
    }
}
#endif

static int32_t hostif_read_exe(uint8_t adr, uint8_t *data, uint16_t size)
{
    uint8_t send_data[2]={0};
    struct i2c_msg msgs[] = {
        {
            .addr = this_client->addr,
            .flags = 0,
            .len = 1,
            .buf = send_data,
        },
        {
            .addr = this_client->addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = data,
        },
    };
    send_data[0] = adr;
    
    if (i2c_transfer(this_client->adapter, msgs, 2) < 0) {
        pr_err("shub-i2c_read: transfer error\n");
        return -EIO;
    } else {
        return 0;
    }
}

int32_t hostif_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret;
    uint8_t *buf;
    
/* SHMDS_HUB_0323_01 add S */
    if (size > FIFO_SIZE){
        pr_err("%s: Invalid argument, size=%d\n", __FUNCTION__, size);
        return -EINVAL;
    }
/* SHMDS_HUB_0323_01 add E */
    
    buf = kzalloc(SHUB_I2C_READ_BUFF_SIZE, GFP_KERNEL);
    if (buf == NULL){
        pr_err("shub-i2c_read: kzalloc error\n");
        return -ENOMEM;
    }
    
    ret = hostif_read_exe(adr, buf, size);
    if(ret == 0) {
        memcpy(data, buf, size);
#ifdef SHUB_SW_I2C_LOG
        hostif_read_log(adr, data, size);
#endif
    }
    
    kfree(buf);
    return ret;
}
/* SHMDS_HUB_0323_02 mod E */
EXPORT_SYMBOL(hostif_read_proc);

/* SHMDS_HUB_0109_02 add S */
int shub_get_acc_axis_val(void)
{
    printk("[shub]acc_axis_val=%d\n", shub_acc_axis_val);
    return shub_acc_axis_val;
}

int shub_get_gyro_axis_val(void)
{
    printk("[shub]gyro_axis_val=%d\n", shub_gyro_axis_val);
    return shub_gyro_axis_val;
}

int shub_get_mag_axis_val(void)
{
    printk("[shub]mag_axis_val=%d\n", shub_mag_axis_val);
    return shub_mag_axis_val;
}
/* SHMDS_HUB_0109_02 add E */

MODULE_DESCRIPTION("SensorHub I2C Driver");
MODULE_AUTHOR("LAPIS SEMICONDUCTOR");
MODULE_LICENSE("GPL v2");
