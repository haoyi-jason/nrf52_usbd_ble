#ifndef _BMI160_CMD_H
#define _BMI160_CMD_H
#include <stdbool.h>
#include <stdint.h>


#define EV_BMI160_BASE     25
#define EV_BMI160_CMD_FOC  EVENT_MASK(EV_BMI160_BASE)
#define EV_BMI160_CMD_STOP EVENT_MASK(EV_BMI160_BASE+1)
#define EV_BMI160_CMD_SINGLE EVENT_MASK(EV_BMI160_BASE+2)

//extern struct bmi160_dev bmi160;
int8_t bmi160_read(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t bmi160_write(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n);
void bmi160_delay(uint32_t period);

int8_t bmi160_cmd_init(void);
void bmi160_cmd_acquire_one(struct bmi160_dev *dev, uint8_t *ptr);
void bmi160_cmd_testRead(struct bmi160_dev *dev);
int8_t bmi160_cmd_startfoc(struct bmi160_dev *dev);
void bmi160_cmd_config_int(struct bmi160_dev *dev, uint8_t st);
void bmi160_cmd_config_int_am(struct bmi160_dev *dev,uint8_t st);
int8_t bmi160_cmd_pwdn(struct bmi160_dev *dev);
int8_t bmi160_cmd_pwup(struct bmi160_dev *dev);

void bmi160_int_isr2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void bmi160_int_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif