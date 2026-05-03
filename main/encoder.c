#include "encoder.h"

#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "math.h"

#define PI                  3.14159265f

static i2c_master_bus_handle_t AS5600_bus_handle = NULL;
static i2c_master_dev_handle_t AS5600_dev_handle = NULL;


void AS5600_init(gpio_num_t sda,gpio_num_t scl){
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .flags.enable_internal_pullup = GPIO_PULLUP_ENABLE,
    };
    i2c_new_master_bus(&bus_cfg,&AS5600_bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 400000,
    };
    i2c_master_bus_add_device(AS5600_bus_handle,&dev_cfg,&AS5600_dev_handle);


}

esp_err_t AS5600_read_word(uint8_t reg, uint16_t *data) {
    uint8_t buf[1] = {reg};
    uint8_t recv_buf[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(
        AS5600_dev_handle,
        buf, 1,
        recv_buf, 2,
        500
    );

    if(ret == ESP_OK) {
        uint16_t raw = (recv_buf[0] << 8) | recv_buf[1];
        *data = raw & 0x0FFF; 
    }
    return ret;
}
// esp_err_t AS5600_read_word(uint8_t reg,uint16_t* data)
// {
//     uint8_t buf[1] = {reg};
//     esp_err_t ret =  i2c_master_transmit_receive(AS5600_dev_handle,buf,1,(uint8_t*)data,2,500);
//     return ret;
// }

float getAngle(void) {
    uint16_t angle_data = 0;
    AS5600_read_word(0x0C,&angle_data);
    return angle_data * 0.08789f * PI / 180.0f;
}

float getAngle_total(void) {
    static float encoder_data = 0, angle_now=0, angle_last = 0;
    static int16_t rotation=0;
    angle_now = getAngle();
    if(abs(angle_now-angle_last)>0.8f * PI * 2) rotation += ((angle_now-angle_last)>0)?-1:1;
    angle_last = angle_now;
    return (float)rotation * PI * 2 + angle_now;
}

float get_velocity()
{
    static int64_t last_us = 0;
    static float last_angle = 0;
    int64_t now_us = esp_timer_get_time();
    float angle = getAngle_total();
    
    float Ts = (now_us - last_us) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 5e-3f; // 避免异常时间间隔导致的速度计算错误
    
    float vel = (angle - last_angle) / Ts;
    
    last_angle = angle;
    last_us = now_us;
    return vel;    
}

float LowPassFilter(float val)
{
    static int64_t last_us = 0;
    static float val_last = 0;
    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - last_us) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 5e-3f; 
    
    float alpha = 0.1f / (0.1f + Ts);
    float val_ret = alpha * val_last + (1.0f - alpha) * val;
    val_last = val_ret;
    last_us = now_us;
    return val_ret;
}

