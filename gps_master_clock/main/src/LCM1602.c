// ---------------------------------------------------------------------------
// IMPORTANT!!!! DERIVED FROM ARDUINO LIB created by Francisco Malpartida on 20/08/11.
// Replaced all arduino specific stuff with esp idf and using it for 4 bit mode
// only! No more object oriented implementation.
// ---------------------------------------------------------------------------

#include "LCM1602.h"
#include <stdio.h>

#include "driver/i2c_master.h" // for i2c_master_bus_config_t
#include "rom/ets_sys.h"       // for microsecond delay
#include "bsp.h"

#include "custom_main.h"


static uint32_t max_wait_ticks = MAX_WAIT_TICKS; // amount of ticks to wait for blocking read/write
const i2c_master_bus_config_t bus_conf =
{
	.clk_source = I2C_CLK_SRC_DEFAULT,
	.i2c_port = I2C_PORT_NUM,
	.scl_io_num = I2C_SCL_IO,
	.sda_io_num = I2C_SDA_IO,
	.glitch_ignore_cnt = 7,
	.flags.enable_internal_pullup = true,
};

const i2c_device_config_t dev_conf =
{
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address = LCM1602_ADDR,
	.scl_speed_hz = I2C_FREQ_HZ,
};

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static uint8_t _displayfunction;
static uint8_t _displaycontrol; // LCD base control command LCD on/off, blink, cursor
// all commands are "ored" to its contents.
static uint8_t _displaymode; // Text entry mode to the LCD

static uint8_t _numlines; // Number of lines of the LCD, initialized with begin()
static uint8_t _cols;     // Number of columns in the LCD

// General LCD commands - generic methods used by the rest of the commands
// ---------------------------------------------------------------------------


static esp_err_t i2c_write(uint8_t data)
{
    return i2c_master_transmit(dev_handle, &data, 1, max_wait_ticks);
}

static esp_err_t writeNibble(uint8_t value, uint8_t mode)
{
    uint8_t data = PIN_BL;
    esp_err_t err;

	// Only interested in COMMAND or DATA
    if (mode == LCD_DATA) data |= PIN_RS;

    // Shift nibble on D4-D7
    data |= (value & 0x0F) << 4;

    // Enable = HIGH
    err = i2c_write(data | PIN_EN);
    if (err == ESP_OK)
    {
       ets_delay_us(1);
   
       // Enable = LOW
       err = i2c_write(data & ~PIN_EN);
       ets_delay_us(50);
    }
    return err;
}

static esp_err_t send(uint8_t value, uint8_t mode)
{
   esp_err_t err = ESP_OK;
   if (mode != FOUR_BITS)
   {
      err = writeNibble(value >> 4, mode);
   }

   if (err == ESP_OK)
   {
      err = writeNibble(value, mode);
   }

   return err;
}

static esp_err_t command(uint8_t value)
{
   return send(value, COMMAND);
}

esp_err_t LCD_I2C_begin(uint8_t cols, uint8_t lines)
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
		_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

		if (lines > 1)
		{
			_displayfunction |= LCD_2LINE;
		}
		_numlines = lines;
		_cols = cols;

		// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
		// according to datasheet, we need at least 40ms after power rises above 2.7V
		// before sending commands. Arduino can turn on way before 4.5V so we'll wait
		// 50
		// ---------------------------------------------------------------------------
		vTaskDelay(100); // 100ms delay

		// put the LCD into 4 bit mode

		// this is according to the hitachi HD44780 datasheet
		// figure 24, pg 46

		// we start in 8bit mode, try to set 4 bit mode
		// Special case of "Function Set"
		send(0x03, FOUR_BITS);
		ets_delay_us(4500); // wait min 4.1ms

		// second try
		send(0x03, FOUR_BITS);
		ets_delay_us(150); // wait min 100us

		// third go!
		send(0x03, FOUR_BITS);
		ets_delay_us(150); // wait min of 100us

		// finally, set to 4-bit interface
		send(0x02, FOUR_BITS);
		ets_delay_us(150); // wait min of 100us

		// finally, set # lines, font size, etc.
		command(LCD_FUNCTIONSET | _displayfunction);
		ets_delay_us(60); // wait more

		// turn the display on with no cursor or blinking default
		_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
		LCD_I2C_display();

		// clear the LCD
		LCD_I2C_clear();

		// Initialize to default text direction (for romance languages)
		_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
		// set the entry mode
		command(LCD_ENTRYMODESET | _displaymode);

		LCD_I2C_display();
	}

	return ret;
}

static size_t write(uint8_t value)
{
   esp_err_t err = send(value, LCD_DATA);
   return err == ESP_OK ? 1 : 0;
}

// Common LCD Commands
// ---------------------------------------------------------------------------
esp_err_t LCD_I2C_print(const char *str)
{
   esp_err_t err = ESP_OK;
   uint8_t len = strlen(str);
   if (len > _cols)
      len = _cols;

   for (uint8_t idx = 0; idx < len; idx++)
   {
      err = send(str[idx], LCD_DATA);
      if (err != ESP_OK)
      {
         break;
      }
   }

   return err;
}

void LCD_I2C_clear()
{
   command(LCD_CLEARDISPLAY);     // clear display, set cursor position to zero
   ets_delay_us(HOME_CLEAR_EXEC); // this command is time consuming
}

void LCD_I2C_home()
{
   command(LCD_RETURNHOME);       // set cursor position to zero
   ets_delay_us(HOME_CLEAR_EXEC); // This command is time consuming
}

void LCD_I2C_setCursor(uint8_t col, uint8_t row)
{
   const uint8_t row_offsetsDef[] = {0x00, 0x40, 0x14, 0x54};   // For regular LCDs
   const uint8_t row_offsetsLarge[] = {0x00, 0x40, 0x10, 0x50}; // For 16x4 LCDs

   if (row >= _numlines)
   {
      row = _numlines - 1; // rows start at 0
   }

   // 16x4 LCDs have special memory map layout
   // ----------------------------------------
   if (_cols == 16 && _numlines == 4)
   {
      command(LCD_SETDDRAMADDR | (col + row_offsetsLarge[row]));
   }
   else
   {
      command(LCD_SETDDRAMADDR | (col + row_offsetsDef[row]));
   }
}

// Turn the display on/off
void LCD_I2C_noDisplay()
{
   _displaycontrol &= ~LCD_DISPLAYON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD_I2C_display()
{
   _displaycontrol |= LCD_DISPLAYON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LCD_I2C_noCursor()
{
   _displaycontrol &= ~LCD_CURSORON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LCD_I2C_cursor()
{
   _displaycontrol |= LCD_CURSORON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns on/off the blinking cursor
void LCD_I2C_noBlink()
{
   _displaycontrol &= ~LCD_BLINKON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD_I2C_blink()
{
   _displaycontrol |= LCD_BLINKON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LCD_I2C_scrollDisplayLeft(void)
{
   command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCD_I2C_scrollDisplayRight(void)
{
   command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LCD_I2C_leftToRight(void)
{
   _displaymode |= LCD_ENTRYLEFT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LCD_I2C_rightToLeft(void)
{
   _displaymode &= ~LCD_ENTRYLEFT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// This method moves the cursor one space to the right
void LCD_I2C_moveCursorRight(void)
{
   command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
}

// This method moves the cursor one space to the left
void LCD_I2C_moveCursorLeft(void)
{
   command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
}

// This will 'right justify' text from the cursor
void LCD_I2C_autoscroll(void)
{
   _displaymode |= LCD_ENTRYSHIFTINCREMENT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LCD_I2C_noAutoscroll(void)
{
   _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// Write to CGRAM of new characters
void LCD_I2C_createChar(uint8_t location, uint8_t charmap[])
{
   location &= 0x7; // we only have 8 locations 0-7

   command(LCD_SETCGRAMADDR | (location << 3));
   ets_delay_us(30);

   for (uint8_t i = 0; i < 8; i++)
   {
      write(charmap[i]); // call the virtual write method
      ets_delay_us(40);
   }
}

