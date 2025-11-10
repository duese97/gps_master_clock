#include "LCM1602.h"
#include <stdio.h>

#include "driver/i2c_master.h" // for i2c_master_bus_config_t
#include "rom/ets_sys.h"       // for microsecond delay
#include "bsp.h"

#include "custom_main.h"

#define LCM1602_MAX_ROWS 2
#define LCM1602_MAX_COLS 16

uint32_t max_wait_ticks = MAX_WAIT_TICKS; // amount of ticks to wait for blocking read/write
i2c_master_bus_config_t bus_conf =
{
	//.clk_source = I2C_CLK_SRC_DEFAULT,
	.i2c_port = I2C_PORT_NUM,
	.scl_io_num = I2C_SCL_IO,
	.sda_io_num = I2C_SDA_IO,
	.glitch_ignore_cnt = 7,
	.flags.enable_internal_pullup = true,
};

i2c_device_config_t dev_conf =
{
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address = LCM1602_ADDR,
	.scl_speed_hz = I2C_FREQ_HZ,
};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static void lcm1602_command(uint8_t value)
{
	uint8_t tx_data[] = {0x80, value};
	esp_err_t ret = i2c_master_transmit(dev_handle, tx_data, sizeof(tx_data), max_wait_ticks);
	if (ret != ESP_OK)
	{
		PRINT_LOG("Unable to send command: %d", ret);
	}
	vTaskDelay(1);
}

static void lcm1602_write(uint8_t value)
{
	uint8_t tx_data[] = {0x40, value};
	esp_err_t ret = i2c_master_transmit(dev_handle, tx_data, sizeof(tx_data), max_wait_ticks);
	if (ret != ESP_OK)
	{
		PRINT_LOG("Unable to write: %d", ret);
	}
	vTaskDelay(1);
}

void lcm1602_init(void)
{
    esp_err_t ret = i2c_new_master_bus(&bus_conf, &bus_handle);
	if (ret != ESP_OK)
	{
		PRINT_LOG("Unable add new master: %d", ret);
	}

	if (ret == ESP_OK)
	{
		ret = i2c_master_bus_add_device(bus_handle, &dev_conf, &dev_handle);
		if (ret != ESP_OK)
		{
			PRINT_LOG("Unable to add new device: %d", ret);
		}
	}

	if (ret == ESP_OK)
	{
		// probe just for good measure
		ret = i2c_master_probe(bus_handle, dev_conf.device_address, max_wait_ticks);
		if (ret != ESP_OK)
		{
			PRINT_LOG("Probe failed: %d", ret);
		}
	}

	if (ret == ESP_OK)
	{
		vTaskDelay(50);
	
		lcm1602_command(FUNCTIONSET | _2LINE);
		vTaskDelay(10);
	
	
		lcm1602_display();
		ets_delay_us(40);
		lcm1602_clear();
		vTaskDelay(2);
		lcm1602_command(ENTRYMODESET | ENTRYLEFT | ENTRYSHIFTDECREMENT);
	
		lcm1602_setCursor(0,0);
	}
}

void lcm1602_display() 
{
	lcm1602_command(DISPLAYCONTROL | DISPLAYON | CURSOROFF | BLINKOFF);
}

void lcm1602_noDisplay()
{
	lcm1602_command(DISPLAYCONTROL | DISPLAYOFF | CURSOROFF | BLINKOFF);
}

void lcm1602_clear()
{
	lcm1602_command(CLEARDISPLAY);
	ets_delay_us(2000);
	lcm1602_setCursor(0,0);
}

void lcm1602_setCursor(uint8_t col, uint8_t row)
{
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > LCM1602_MAX_ROWS) 
	{
		row = LCM1602_MAX_ROWS-1;    // we count rows starting w/0
	}
	lcm1602_command(SETDDRAMADDR | (col + row_offsets[row]));
}

void lcm1602_scrollDisplayRight(void) 
{
	lcm1602_command(CURSORSHIFT | DISPLAYMOVE | MOVERIGHT);
}

void lcm1602_scrollDisplayLeft(void) 
{
	lcm1602_command(CURSORSHIFT | DISPLAYMOVE | MOVELEFT);
}

void lcm1602_print(const char c[])
{
	for(int i =0; i < strlen(c); i++ ) 
	{
		char x = c[i];
		lcm1602_write((int)x);
	}	
}
