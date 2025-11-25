//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include "menu_handler.h"
#include "LCD.h"
#include "LCM1602.h"
#include <stdint.h>


//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
typedef enum
{
    MENU_SEL_EXIT,
    MENU_SEL_MASTER_ADVANCE,
    MENU_SEL_SLAVE_ADVANCE,
    MENU_SEL_PULSE_LEN,
    MENU_SEL_PULSE_PAUSE,
    MENU_SEL_FULL_RESET,

    MENU_SEL_NUM_ELEM

} main_menu_state_t;

typedef enum
{
    MENU_STATE_SEL_NONE, // default, show time
    MENU_STATE_SEL_TOP, // iterating over top menu
    MENU_STATE_SEL_SUB, // iterating over sub menu
    MENU_STATE_MODIFY_VALUE, // setting a value withing sub menu
} menu_state_t;

typedef union
{
    int16_t i16;
    uint16_t u16;
    uint32_t u32;
    int32_t i32;
    float f;

    int16_t* i16_ptr;
    uint16_t* u16_ptr;
    uint32_t* u32_ptr;
    int32_t* i32_ptr;
    float* f_ptr;
} val_union_t;

typedef struct
{
    const char* selector_str;
    
    // for altering values
    void (*modifier_fn)(val_union_t); 
    val_union_t fn_param; // parameter to be handed to modifier_fn

    void (*apply_fn)(void); // 

    void (*update_fn)(void); // in case submenu entry should be periodically refreshed
} submenu_elem_t;

typedef struct
{
    const char* submenu_initial_title_str; // what should be shown when first entering the menu
    const char* processing_str;
    const uint8_t num_submenu_elem;
    const submenu_elem_t* submenu_elems;

    val_union_t init_value;    // where value is to be initially loaded from
} submenu_t;

typedef struct
{
    const char* selector_str;
    const submenu_t* submenu_ptr;
} main_menu_t;

//---------------------------------------------------------------------------
// Local variables
//---------------------------------------------------------------------------

static main_menu_t const* main_ptr;
static const submenu_t* sub_ptr;
static menu_state_t menu_state;
static main_menu_state_t curr_main_menu_idx = MENU_SEL_EXIT;
static uint8_t curr_sub_menu_idx = 0;
static val_union_t local_storage; // scratch buffer for changing values


//---------------------------------------------------------------------------
// Master advance sub menu
//---------------------------------------------------------------------------

static void master_advance_fn(val_union_t val)
{
    // increment the minutes as desired
    local_storage.i32 += val.u32;
    local_storage.i32 %= MINUTES_PER_12H;

    // print it
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_printf("      %02u:%02u     ", local_storage.i32 / 60, local_storage.i32 % 60);
}

static void master_advance_apply_fn(void)
{
    ram_mirror.current_slave_minutes_12o_clock = local_storage.i32;
    store_ram_mirror();
}

static const submenu_elem_t master_advance_submenu_elems[] =
{
    {
        .selector_str = ">"BACK_ICO_STR"  +1m  +1h  OK"
    },
    {
        .selector_str = " "BACK_ICO_STR" >+1m  +1h  OK",
        .modifier_fn = master_advance_fn,
        .fn_param.u32 = 1,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +1m >+1h  OK",
        .modifier_fn = master_advance_fn,
        .fn_param.u32 = 60,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +1m  +1h >OK",
        .apply_fn = master_advance_apply_fn,
    },
};
static const submenu_t master_advance_submenu =
{
    .submenu_initial_title_str = "Master advance",
    .num_submenu_elem = ARRAY_LEN(master_advance_submenu_elems),
    .submenu_elems = master_advance_submenu_elems,
    .init_value = {.i32_ptr = &(ram_mirror.current_slave_minutes_12o_clock)},
};

//---------------------------------------------------------------------------
// Slave advance sub menu
//---------------------------------------------------------------------------

static void slave_advance_fn(val_union_t val)
{
    // Send the message with desired slave advance minutes to task
    task_msg_t msg = {
        .cmd = TASK_CMD_SLAVE_ADVANCE_MINUTES,
        .dst = TASK_TIMEKEEP,
        .slave_advance_minutes = val.u32
    };
    sendTaskMessage(&msg);
}
static const submenu_elem_t slave_advance_submenu_elems[] =
{
    {
        .selector_str = ">"BACK_ICO_STR"  +1m  +1h"
    },
    {
        .selector_str = " "BACK_ICO_STR" >+1m  +1h",
        .fn_param.u32 = 1,
        .modifier_fn = slave_advance_fn,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +1m >+1h",
        .fn_param.u32 = 60,
        .modifier_fn = slave_advance_fn,
    },
};
static const submenu_t slave_advance_submenu =
{
    .submenu_initial_title_str = "Slave advance",
    .num_submenu_elem = ARRAY_LEN(slave_advance_submenu_elems),
    .submenu_elems = slave_advance_submenu_elems,
};

//---------------------------------------------------------------------------
// Pulse length sub menu
//---------------------------------------------------------------------------

static void apply_pulse_len_fn(void)
{
    ram_mirror.pulse_len_ms = local_storage.i16;
    store_ram_mirror();
}

static void pulse_pause_len_change_fn(val_union_t val)
{
    local_storage.i16 += val.i16;
    if(local_storage.i16 < abs(val.i16)) // make sure not to allow negative values
    {
        local_storage.i16 = abs(val.i16);
    }
    // print it
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_printf("%5dms         ", local_storage.i16);
}
static const submenu_elem_t pulse_len_submenu_elems[] =
{
    {
        .selector_str = ">"BACK_ICO_STR"  +10  -10  OK"
    },
    {
        .selector_str = " "BACK_ICO_STR" >+10  -10  OK",
        .fn_param.i16 = 10,
        .modifier_fn = pulse_pause_len_change_fn,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +10 >-10  OK",
        .fn_param.i16 = -10,
        .modifier_fn = pulse_pause_len_change_fn,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +10  -10 >OK",
        .apply_fn = apply_pulse_len_fn,
    },
};
static const submenu_t pulse_len_submenu =
{
    .submenu_initial_title_str = "Pulse length",
    .num_submenu_elem = ARRAY_LEN(pulse_len_submenu_elems),
    .submenu_elems = pulse_len_submenu_elems,
    .init_value = { .u16_ptr = &(ram_mirror.pulse_len_ms) },
};

//---------------------------------------------------------------------------
// Pulse pause sub menu
//---------------------------------------------------------------------------

static void apply_pulse_pause_fn(void)
{
    ram_mirror.pulse_pause_ms = local_storage.i16;
    store_ram_mirror();
}

static const submenu_elem_t pulse_pause_submenu_elems[] =
{
    {
        .selector_str = ">"BACK_ICO_STR"  +10  -10  OK"
    },
    {
        .selector_str = " "BACK_ICO_STR" >+10  -10  OK",
        .fn_param.i16 = 10,
        .modifier_fn = pulse_pause_len_change_fn,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +10 >-10  OK",
        .fn_param.i16 = -10,
        .modifier_fn = pulse_pause_len_change_fn,
    },
    {
        .selector_str = " "BACK_ICO_STR"  +10  -10 >OK",
        .apply_fn = apply_pulse_pause_fn,
    },
};
static const submenu_t pulse_pause_submenu =
{
    .submenu_initial_title_str = "Pulse pause",
    .num_submenu_elem = ARRAY_LEN(pulse_pause_submenu_elems),
    .submenu_elems = pulse_pause_submenu_elems,
    .init_value = { .u16_ptr = &(ram_mirror.pulse_pause_ms) },
};

//---------------------------------------------------------------------------
// Factory reset sub menu
//---------------------------------------------------------------------------

static void factory_reset_submenu_set(void)
{
    ram_mirror = ram_mirror_default;
    store_ram_mirror();
}

static const submenu_elem_t factory_reset_submenu_elems[] =
{
    {
        .selector_str = ">"BACK_ICO_STR"  yes",
    },
    {
        .selector_str = " "BACK_ICO_STR" >yes",
        .apply_fn = factory_reset_submenu_set,
    },
};
static const submenu_t factory_reset_submenu =
{
    .submenu_initial_title_str = "FACTORY RESET?",
    .processing_str = "PRESS TO ABORT",
    .num_submenu_elem = ARRAY_LEN(factory_reset_submenu_elems),
    .submenu_elems = factory_reset_submenu_elems,
};


//---------------------------------------------------------------------------
// Main menu
//---------------------------------------------------------------------------

static const main_menu_t main_menu[] =
{
    [MENU_SEL_EXIT] =
    {
        .selector_str = ">Exit"
    },
    [MENU_SEL_MASTER_ADVANCE] = 
    {
        .selector_str = ">Master advance",
        .submenu_ptr = &master_advance_submenu,
    },
    [MENU_SEL_SLAVE_ADVANCE] =
    {
        .selector_str = ">Slave advance",
        .submenu_ptr = &slave_advance_submenu,
    },
    [MENU_SEL_PULSE_LEN] =
    {
        .selector_str = ">Pulse length",
        .submenu_ptr = &pulse_len_submenu,
    },
    [MENU_SEL_PULSE_PAUSE] =
    {
        .selector_str = ">Pulse pause",
        .submenu_ptr = &pulse_pause_submenu,
    },
    [MENU_SEL_FULL_RESET] =
    {
        .selector_str =  ">FULL RESET",
        .submenu_ptr = &factory_reset_submenu,
    },
};


//---------------------------------------------------------------------------
// Utilities and 'driver'
//---------------------------------------------------------------------------

static void print_topmenu(void)
{
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_print("******MENU******");
    LCD_I2C_setCursor(0, 1);
    LCD_I2C_printf("%-16s", main_ptr->selector_str);
}

static void change_selector(const char* selector_str, bool force_set)
{
    static bool show_selector;

    if (force_set)
    {
        show_selector = true;
    }
    else
    {
        show_selector = !show_selector; // prepare next toggle
    }
    
    char* found = strchr(selector_str, '>'); // look for current pos
    if (found == false) // check if it is even in the string
    {
        return;
    }

    // check where we currently are, calculate index
    LCD_I2C_setCursor(found - selector_str, 1);

    // only need to alter the 'cursor', rest of the shown string can remain untouched
    char* toggle_char = show_selector ? ">" : " ";
    LCD_I2C_print(toggle_char);
}

void menu_update(void)
{
    // updating only makes sense, when in submenu
    if (sub_ptr == NULL)
    { // sanity check
        return;
    }

    // get submenu element
    const submenu_elem_t* elem = sub_ptr->submenu_elems + curr_sub_menu_idx;
    
    // add a blinking effect to the 'cursor' as soon as modification is started
    if (menu_state == MENU_STATE_MODIFY_VALUE)
    {
        change_selector(elem->selector_str, false /*no force*/);
    }

    // check if update function is present
    if (elem->update_fn == NULL)
    {
        return;
    }
    elem->update_fn();
}

bool menu_statemachine(btn_state_t btn_state, bool* menu_changed)
{    
    switch(menu_state)
    {
        case MENU_STATE_SEL_NONE: // when nothing selected
        {
            if (btn_state == BTN_VERY_LONG_PRESS) // prevent going to menu if pressed briefly
            {
                menu_state = MENU_STATE_SEL_TOP;
                curr_main_menu_idx = MENU_SEL_EXIT;
                curr_sub_menu_idx = 0;

                main_ptr = main_menu;
                *menu_changed = true;
                print_topmenu();
            }
            break;
        }
        case MENU_STATE_SEL_TOP: // when moving over the top menu elements
        {
            if (btn_state == BTN_LONG_PRESS) // sub menu entered
            {
                if (curr_main_menu_idx == MENU_SEL_EXIT) // fist index is special: exit directly
                {
                    menu_state = MENU_STATE_SEL_NONE;
                    LCD_I2C_clear(); // cleanup in any case
                    *menu_changed = true;
                }
                else
                {
                    menu_state = MENU_STATE_SEL_SUB;
                    curr_sub_menu_idx = 0;
    
                    sub_ptr = main_ptr->submenu_ptr;
    
                    // Only needed once: Title of submenu
                    LCD_I2C_setCursor(0, 0);
                    LCD_I2C_printf("%-16s", sub_ptr->submenu_initial_title_str);
                    LCD_I2C_setCursor(0, 1);
                    LCD_I2C_printf("%-16s", sub_ptr->submenu_elems->selector_str);
                }
            }
            else if (btn_state == BTN_SHORT_PRESS) // iterate to next topmenu entry
            {
                if (curr_main_menu_idx + 1 >= MENU_SEL_NUM_ELEM)
                {
                    curr_main_menu_idx = MENU_SEL_EXIT;
                }
                else
                {
                    curr_main_menu_idx++;
                }

                main_ptr = main_menu + curr_main_menu_idx;
                
                // Next selected top level menu element
                LCD_I2C_setCursor(0, 1);
                LCD_I2C_printf("%-16s", main_ptr->selector_str);
            }

            break;
        }
        case MENU_STATE_SEL_SUB: // iterating within submenu
        {
            if (btn_state == BTN_LONG_PRESS)
            {
                const submenu_elem_t* elem = sub_ptr->submenu_elems + curr_sub_menu_idx;
                if (curr_sub_menu_idx == 0) // first element is always the exit
                {
                    // show top menu again
                    menu_state = MENU_STATE_SEL_TOP;
                    print_topmenu();
                }
                else if (elem->modifier_fn != NULL)
                {
                    menu_state = MENU_STATE_MODIFY_VALUE;

                    if (sub_ptr->init_value.u32_ptr != NULL) // check if we need to initialize the value
                    {
                        local_storage.u32 = *(sub_ptr->init_value.u32_ptr); // load word, care about type later
                        
                    }
                }
                else if (elem->apply_fn != 0)
                {
                    elem->apply_fn();
                    // show top menu again
                    menu_state = MENU_STATE_SEL_TOP;
                    print_topmenu();
                }
            }
            else if (btn_state == BTN_SHORT_PRESS)
            {
                if (curr_sub_menu_idx + 1 >= main_ptr->submenu_ptr->num_submenu_elem)
                {
                    curr_sub_menu_idx = 0;
                }
                else
                {
                    curr_sub_menu_idx++;
                }

                const submenu_elem_t* elem = sub_ptr->submenu_elems + curr_sub_menu_idx;

                LCD_I2C_setCursor(0, 1);
                LCD_I2C_printf("%-16s", elem->selector_str);
            }
            break;
        }
        case MENU_STATE_MODIFY_VALUE:
        {
            const submenu_elem_t* elem = sub_ptr->submenu_elems + curr_sub_menu_idx;

            if (btn_state == BTN_SHORT_PRESS)
            {
                elem->modifier_fn(elem->fn_param);
            }
            else if (btn_state == BTN_LONG_PRESS) // entering value done?
            {
                menu_state = MENU_STATE_SEL_SUB; // go back to top
                change_selector(elem->selector_str, true /*restore selector in case it is hidden right now*/);
            }

            break;
        }
        default:
            PRINT_LOG("Unknown or unhandled top menu state: %d", menu_state);
            break;
    }

    return menu_state != MENU_STATE_SEL_NONE;
}