#include "menu_handler.h"

#include "LCM1602.h"
#include <stdint.h>

typedef enum
{
    MENU_SEL_EXIT,
    MENU_SEL_MASTER_ADVANCE,
    MENU_SEL_SLAVE_ADVANCE,
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


typedef struct
{
    uint32_t u32;
    int32_t i32;
    float f;
} val_union_t;

typedef struct
{
    const char* selector_str;
    void (*modifier_fn)(val_union_t);
    void (*update_fn)(void);
    val_union_t fn_param;
} submenu_elem_t;

typedef struct
{
    const char* submenu_initial_title_str;
    const char* processing_str;
    const uint8_t num_submenu_elem;
    const submenu_elem_t* submenu_elems;
} submenu_t;

typedef struct
{
    const char* selector_str;
    submenu_t const* submenu_ptr;
} main_menu_t;


static main_menu_t const* main_ptr;
static submenu_t const* sub_ptr;
static menu_state_t menu_state;
static main_menu_state_t curr_main_menu_idx = MENU_SEL_EXIT;
static uint8_t curr_sub_menu_idx = 0;


static void master_advance_fn(val_union_t val)
{
    // increment the minutes as desired
    rm.current_minutes_12o_clock += val.u32;
    rm.current_minutes_12o_clock %= MINUTES_PER_12H;

    // get current configured advanced time
    uint8_t hours = rm.current_minutes_12o_clock / 60;
    uint8_t minutes = rm.current_minutes_12o_clock % 60;

    // print it
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_printf("      %02u:%02u     ", hours, minutes);
}

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
static void slave_advance_update(void)
{
    // get current configured advanced time
    uint8_t hours = rm.current_minutes_12o_clock / 60;
    uint8_t minutes = rm.current_minutes_12o_clock % 60;

    // print it
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_printf("      %02u:%02u     ", hours, minutes);
}

static void factory_reset_submenu_set(val_union_t val)
{

}

static const submenu_elem_t master_advance_submenu_elems[] =
{
    {
        .selector_str = ">exit  +1m  +1h "
    },
    {
        .selector_str = " exit >+1m  +1h ",
        .modifier_fn = master_advance_fn,
        .fn_param.u32 = 1,
    },
    {
        .selector_str = " exit  +1m >+1h ",
        .modifier_fn = master_advance_fn,
        .fn_param.u32 = 60,
    },
};
static const submenu_t master_advance_submenu =
{
    .submenu_initial_title_str = "Master advance  ",
    .num_submenu_elem = ARRAY_LEN(master_advance_submenu_elems),
    .submenu_elems = master_advance_submenu_elems,
};


static const submenu_elem_t slave_advance_submenu_elems[] =
{
    {
        .selector_str = ">exit  +1m  +1h "
    },
    {
        .selector_str = " exit >+1m  +1h ",
        .fn_param.u32 = 1,
        .modifier_fn = slave_advance_fn,
        .update_fn = slave_advance_update,
    },
    {
        .selector_str = " exit  +1m >+1h ",
        .fn_param.u32 = 60,
        .modifier_fn = slave_advance_fn,
        .update_fn = slave_advance_update,
    },
};
static const submenu_t slave_advance_submenu =
{
    .submenu_initial_title_str = "Slave advance   ",
    .num_submenu_elem = ARRAY_LEN(slave_advance_submenu_elems),
    .submenu_elems = slave_advance_submenu_elems,
};


static const submenu_elem_t factory_reset_submenu_elems[] =
{
    {
        .selector_str = ">exit  yes",
    },
    {
        .selector_str = " exit >yes",
        .modifier_fn = factory_reset_submenu_set,
    },
};
static const submenu_t factory_reset_submenu =
{
    .submenu_initial_title_str = "FACTORY RESET?",
    .processing_str = "PRESS TO ABORT",
    .num_submenu_elem = ARRAY_LEN(factory_reset_submenu_elems),
    .submenu_elems = factory_reset_submenu_elems,
};

static const main_menu_t main_menu[] =
{
    [MENU_SEL_EXIT] =
    {
        .selector_str = ">Exit           "
    },
    [MENU_SEL_MASTER_ADVANCE] = 
    {
        .selector_str = ">Master advance ",
        .submenu_ptr = &master_advance_submenu,
    },
    [MENU_SEL_SLAVE_ADVANCE] =
    {
        .selector_str = ">Slave advance  ",
        .submenu_ptr = &slave_advance_submenu,
    },
    [MENU_SEL_FULL_RESET] =
    {
        .selector_str =  ">FULL RESET    ",
        .submenu_ptr = &factory_reset_submenu,
    },
};

static void print_topmenu(void)
{
    LCD_I2C_setCursor(0, 0);
    LCD_I2C_print("******MENU******");
    LCD_I2C_setCursor(0, 1);
    LCD_I2C_print(main_ptr->selector_str);
}

void menu_update(void)
{
    if (menu_state != MENU_STATE_SEL_SUB || sub_ptr == NULL) // can only update when in submenu
    {
        return;
    }

    // get submenu element, check if update function is presen
    const submenu_elem_t* elem = sub_ptr->submenu_elems + curr_sub_menu_idx;
    if (elem->update_fn == NULL)
    {
        return;
    }
    elem->update_fn();
}

bool menu_statemachine(btn_state_t btn_state, bool* comm_changed)
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
                *comm_changed = true;
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
                    *comm_changed = true;
                }
                else
                {
                    menu_state = MENU_STATE_SEL_SUB;
                    curr_sub_menu_idx = 0;
    
                    sub_ptr = main_ptr->submenu_ptr;
    
                    // Only needed once: Title of submenu
                    LCD_I2C_setCursor(0, 0);
                    LCD_I2C_print(sub_ptr->submenu_initial_title_str);
                    LCD_I2C_setCursor(0, 1);
                    LCD_I2C_print(sub_ptr->submenu_elems->selector_str);
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
                LCD_I2C_print(main_ptr->selector_str);
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
                LCD_I2C_print(elem->selector_str);
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
            }
            break;
        }
        default:
            PRINT_LOG("Unknown or unhandled top menu state: %d", menu_state);
            break;
    }

    return menu_state != MENU_STATE_SEL_NONE;
}