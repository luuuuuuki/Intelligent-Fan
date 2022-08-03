/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-20     Abbcc        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "sensor_asair_aht10.h"
#include "aht10.h"

#include <rthw.h>
#include <u8g2_port.h>
#define OLED_I2C_PIN_SCL                    26  // PB10
#define OLED_I2C_PIN_SDA                    27  // PB11
/* defined the LED2 pin: PE6 */
#define LED2_PIN    GET_PIN(E, 6)
#define AHT10_I2C_BUS  "i2c1"
#define MAC1_PIN    GET_PIN(D, 9)
#define MAC2_PIN    GET_PIN(D, 10)

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <onenet.h>

#define DBG_ENABLE
#define DBG_COLOR
#define DBG_SECTION_NAME    "onenet.sample"
#if ONENET_DEBUG
#define DBG_LEVEL           DBG_LOG
#else
#define DBG_LEVEL           DBG_INFO
#endif /* ONENET_DEBUG */

#include <rtdbg.h>

#ifdef FINSH_USING_MSH
#include <finsh.h>

static struct rt_event event;
struct rt_device_pwm *pwm_dev;
#define EVENT_FLAG3 (1 << 3)
#define EVENT_FLAG5 (1 << 5)
#define PWM_DEV_NAME        "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     1       /* PWM通道 */


int rt_hw_aht10_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name  = AHT10_I2C_BUS;
    cfg.intf.user_data = (void *)AHT10_I2C_ADDR;
    rt_hw_aht10_init("aht10", &cfg);
    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_aht10_port);


int main(void)
{
    rt_err_t result;

    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MAC1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MAC2_PIN, PIN_MODE_OUTPUT);
    result = rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init event failed.\n");
        return -1;
    }


    while (1)
    {
        rt_thread_mdelay(100);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(500);

    }
}



#define THREAD_PRIORITY      25
#define THREAD_TIMESLICE     5

/* 消息队列控制块 */
static struct rt_messagequeue mq;
/* 消息队列中用到的放置消息的内存池 */
static rt_uint8_t msg_pool[4096];

ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[2048];
static struct rt_thread thread1;

/* 线程1入口函数 */
static void thread1_entry(void *parameter)
{
    int level=0;
    char temp_level='0';
    rt_uint32_t e;
    char buf[3]={""};
    char device_on[]="ON";
    char device_off[]="OFF";
    char mode_auto[]="AUTO";
    char mode_manu[]="MANU";
    char speed_max[]="MAX";
    int state = 1;
    int temp=0;
    rt_uint8_t cnt = 0;
    u8g2_t u8g2;
    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 500000, 500000);
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);

    while (1)
    {
        if (rt_event_recv(&event, EVENT_FLAG3,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          0, &e) == RT_EOK)
        {
            state=!state;
        }


        /* 从消息队列中接收消息 */
        if (rt_mq_recv(&mq, &buf, 2, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_kprintf("thread1: recv msg from msg queue, the content:%c%c\n", buf[0],buf[1]);

             // Draw Graphics
            /* full buffer example, setup procedure ends in _f */
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
            u8g2_DrawStr(&u8g2, 0, 10, "temp            mode");
            u8g2_DrawStr(&u8g2, 0, 40, "switch         speed");
            u8g2_SendBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
            u8g2_DrawStr(&u8g2, 10, 25, buf);
            u8g2_SendBuffer(&u8g2);
            temp=atoi(buf);
            if(state)
            {
                onenet_mqtt_upload_string("mode", mode_auto);
                onenet_mqtt_upload_digit("mode_number", 0);
                if(temp>=28)
                {
                    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 500000, 0);
                    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
                    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
                    u8g2_DrawStr(&u8g2, 65, 25, "AUTO");
                    u8g2_DrawStr(&u8g2, 10, 55, "ON");
                    u8g2_DrawStr(&u8g2, 65, 55, " 3 ");
                    u8g2_SendBuffer(&u8g2);
                    onenet_mqtt_upload_string("device", device_on);
                    onenet_mqtt_upload_digit("speed", 3);

                }
                else
                {
                    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 500000, 500000);
                    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
                    u8g2_DrawStr(&u8g2, 65, 25, "AUTO");
                    u8g2_DrawStr(&u8g2, 10, 55, "OFF");
                    u8g2_DrawStr(&u8g2, 65, 55, " 0 ");
                    u8g2_SendBuffer(&u8g2);
                    onenet_mqtt_upload_string("device", device_off);
                    onenet_mqtt_upload_digit("speed", 0);
                }
            }
            else
            {
                if (rt_event_recv(&event, EVENT_FLAG5,
                                  RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                                  0, &e) == RT_EOK)
                {
                    level++;
                    level%=4;
                }
                temp_level=level+'0';

                onenet_mqtt_upload_string("mode", mode_manu);
                onenet_mqtt_upload_digit("mode_number", 1);
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 510000, 510000-level*170000);
                rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);

                if(!level)
                {
                    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
                    u8g2_DrawStr(&u8g2, 10, 55, "OFF");
                    u8g2_DrawStr(&u8g2, 65, 55, " 0 ");
                    u8g2_DrawStr(&u8g2, 65, 25, "MANU");
                    u8g2_SendBuffer(&u8g2);
                    onenet_mqtt_upload_string("device", device_off);
                }
                else
                {
                    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
                    u8g2_DrawStr(&u8g2, 10, 55, "ON");
                    if(level==1)
                    {
                        u8g2_DrawStr(&u8g2, 65, 55, " 1 ");
                    }
                    else if (level==2) {
                        u8g2_DrawStr(&u8g2, 65, 55, " 2 ");
                    }
                    else {
                        u8g2_DrawStr(&u8g2, 65, 55, " 3 ");
                    }
                    u8g2_DrawStr(&u8g2, 65, 25, "MANU");
                    u8g2_SendBuffer(&u8g2);
                    onenet_mqtt_upload_string("device", device_on);
                }

                onenet_mqtt_upload_digit("speed", level);

            }


        }
        /* 延时50ms */

        rt_thread_mdelay(50);
    }
    rt_kprintf("thread1: detach mq \n");
    rt_mq_detach(&mq);
}

ALIGN(RT_ALIGN_SIZE)
static char thread2_stack[1024];
static struct rt_thread thread2;

/* 线程2入口 */
static void thread2_entry(void *parameter)
{
    int temp=0;

    int result;
    char buf[2] = {0};
    /* set LED2 pin mode to output */
    aht10_device_t aht10=RT_NULL;
    aht10=aht10_init(AHT10_I2C_BUS);


    while (1)
    {
        temp=aht10_read_temperature(aht10);
        buf[0]=temp/10+'0';
        buf[1]=temp%10+'0';
        if (onenet_mqtt_upload_string("temperature", buf) < 0)
        {
            LOG_E("upload has an error, stop uploading");
            break;
        }
        else
        {
            LOG_D("buffer : {\"temperature\":%c%c}", buf[0],buf[1]);
        }
        /* 发送消息到消息队列中 */
        result = rt_mq_send(&mq, &buf, 2);
        rt_thread_mdelay(500);
        if (result != RT_EOK)
        {
            rt_kprintf("rt_mq_send ERR\n");
        }

        rt_kprintf("thread2: send message - %c%c\n", buf[0],buf[1]);
        /* 延时100ms */
        rt_thread_mdelay(100);
    }
}



/* 消息队列示例的初始化 */
int msgq_sample(void)
{
    rt_err_t result;

    /* 初始化消息队列 */
    result = rt_mq_init(&mq,
                        "mqt",
                        &msg_pool[0],               /* 内存池指向msg_pool */
                        2,                          /* 每个消息的大小是 2 字节 */
                        sizeof(msg_pool),           /* 内存池的大小是msg_pool的大小 */
                        RT_IPC_FLAG_FIFO);          /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    if (result != RT_EOK)
    {
        rt_kprintf("init message queue failed.\n");
        return -1;
    }

    rt_thread_init(&thread1,
                   "thread1",
                   thread1_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   24, 50);
    rt_thread_startup(&thread1);

    rt_thread_init(&thread2,
                   "thread2",
                   thread2_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   25, 50);
    rt_thread_startup(&thread2);

    return 0;
}
MSH_CMD_EXPORT(msgq_sample, msgq sample);



/* onenet mqtt command response callback function */
static void onenet_cmd_rsp_cb(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size)
{
    char res_buf[] = { "cmd is received!\n" };

    LOG_D("recv data is %.*s\n", recv_size, recv_data);


    if(recv_data[0]=='o')
    rt_event_send(&event, EVENT_FLAG3);
    if(recv_data[0]=='v')
    rt_event_send(&event, EVENT_FLAG5);


    /* user have to malloc memory for response data */
    *resp_data = (uint8_t *) ONENET_MALLOC(strlen(res_buf));

    strncpy((char *)*resp_data, res_buf, strlen(res_buf));

    *resp_size = strlen(res_buf);

    rt_thread_mdelay(100);
}

/* set the onenet mqtt command response callback function */
int onenet_set_cmd_rsp(int argc, char **argv)
{
    onenet_set_cmd_rsp_cb(onenet_cmd_rsp_cb);
    return 0;
}
INIT_APP_EXPORT(onenet_set_cmd_rsp);
#endif /* FINSH_USING_MSH */

