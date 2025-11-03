/*
 * FancyCli.cpp
 *
 *  Created on: Nov 2, 2025
 *      Author: turtl
 */

#include "FancyCli.hpp"

// all the print buffers

static constexpr char software_info[FancyCli::STR_LEN_COMMON] = PROJECT_NAME " " PROJECT_VERSION " | " __DATE__ " " __TIME__ NEWLINE "\t" PROJECT_DESCRIPTION;
static constexpr char menu_header[] =   "--- options ----";
static constexpr char line_delim[]  =   "----------------";
static constexpr char input_start[] =   ":";
static constexpr char dir_prefix[]  =   "/";
char message[FancyCli::STR_LEN_COMMON];

enum CLISTATE : uint8_t {
    MENU_SELECTION = 0,
    USER_INPUT,
    _count,
};
CLISTATE clistate;
inline bool _charInput_contains(char const * arr, uint8_t len, char c){
    for(uint8_t i = 0; i < len; i++)
        if(arr[i] == c)
            return true;
    return false;
}

bool FancyCli::charInput(System::UART::UART * uart, char c){
    static constexpr char keys_selectUp[]   = {'1', '8'};
    static constexpr char keys_selectDown[] = {'2'};
    static constexpr char keys_select[]     = {'3', '6'};
    static constexpr char keys_unselect[]   = {'4'};
    static constexpr char keys_changeState[]= {'5','`','~'};

    bool ret = false;

    if(_charInput_contains(ARRANDN(keys_changeState), c)){
        clistate = static_cast<CLISTATE>((static_cast<uint8_t>(clistate) + 1) % static_cast<uint8_t>(CLISTATE::_count));
        return true;
    }

    switch(clistate){
        case CLISTATE::MENU_SELECTION:
            if(_charInput_contains(ARRANDN(keys_selectUp), c)){
                selections[selectionDepth]++;
//                if(selections[selectionDepth] > )
                {
//                    selections[selectionDepth] =
                }
            }

            ret = true;
            break;
        case CLISTATE::USER_INPUT:
            if(c == 8 || c == 127){ // character is backspace
                if(userInputLen != 0){
                    userInputLen--;
                    if(uart) DL_UART_transmitDataBlocking(uart->reg, c);
                }
            }

            if(c == 13){
                // trigger input
                ret = true;
                break;
            }

            if(c >= 32){ //character is printable char
                if(userInputLen < sizeof(userInput)){
                    userInput[userInputLen++] = c;
                    if(uart) DL_UART_transmitDataBlocking(uart->reg, c);
                }
            }

        default: break;
    }

    return ret;
}

FancyCli::MenuDir * FancyCli::getSelectedDir(){
    uint8_t sel = 0;

    if(selectionDepth >= (sizeof(selections)/sizeof(selections[0])))
        selectionDepth = (sizeof(selections)/sizeof(selections[0]))-1;

    MenuDir * dir = root;
    uint8_t i = 0;
    for(i = 0; i < selectionDepth; i++){
        if(!dir->dirs)
            break;

        if(selections[i] >= dir->dirCount) // a leaf is selected
            break;

        dir = &dir->dirs[selections[i]];
    }
    if(i > 0)
        selectionDepth = i-1;
    return dir;
}

FancyCli::MenuLeaf * FancyCli::getSelectedLeaf(){
    MenuDir * dir = getSelectedDir();
    if(!dir)
        return NULL;

    if(selections[selectionDepth] >= dir->dirCount){
        if(dir->leafCount == 0){
            selections[selectionDepth] = dir->dirCount;

            if(selections[selectionDepth])
                selections[selectionDepth]--;
            return NULL;
        }

        uint8_t sel_leaf = selections[selectionDepth] - dir->dirCount;
        if(sel_leaf > dir->leafCount){
            sel_leaf = dir->leafCount-1;
            selections[selectionDepth] = dir->dirCount + sel_leaf;
        }

        return &dir->leafs[sel_leaf];
    }

    return NULL;
}

void FancyCli::printFrame(System::UART::UART& uart, bool update){
    uart.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD));
    uart.nputs(ARRANDN(software_info));
    uart.nputs(ARRANDN(CLIRESET NEWLINE CLIWARN));
    uart.nputs(ARRANDN("mode: "));
    switch(clistate){
        case CLISTATE::USER_INPUT:      uart.nputs(ARRANDN("USER_INPUT")); break;
        case CLISTATE::MENU_SELECTION:  uart.nputs(ARRANDN("MENU_SELECTION")); break;
        default:                        uart.nputs(ARRANDN("un oh"));
            clistate = CLISTATE::MENU_SELECTION; break;
    }
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // print menu path
    MenuDir * dir;
    {
        for(uint8_t i = 0; i < selectionDepth; i++){
            dir = getSelectedDir();
            if(!dir)
                break;
            for(uint8_t j = 0; j < i; j++){
                uart.nputs(ARRANDN("  "));
            }
            uart.nputs(ARRANDN(CLIYES "^"));
            uart.putu32d(selections[i]);
            uart.nputs(ARRANDN(":" CLIRESET));
            uart.nputs(ARRANDN(dir->name));
            uart.nputs(ARRANDN(NEWLINE));
        }
    }

    uart.nputs(ARRANDN(CLIRESET CLIHIGHLIGHT));
    uart.nputs(ARRANDN(menu_header));
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // print node overton window
    char * si_description = NULL;
    NodeAccept * accept = NULL;
    if(dir)
    {
        uint8_t a, b;
        if(selections[selectionDepth] < DISPLAY_ITEM_COUNT/2)
            a = 0;
        else
            a = selections[selectionDepth];
        b = a + DISPLAY_ITEM_COUNT;
        if(b > (dir->dirCount + dir->leafCount))
            b = (dir->dirCount + dir->leafCount);

        uint8_t og_sel = selections[selectionDepth];
        uart.nputs(ARRANDN(NEWLINE "a: "));
        uart.putu32d(a);
        uart.nputs(ARRANDN(NEWLINE "b: "));
        uart.putu32d(b);
        uart.nputs(ARRANDN(NEWLINE));
        for(selections[selectionDepth] = a; selections[selectionDepth] < b; selections[selectionDepth]++){
            MenuLeaf * leaf = getSelectedLeaf();
            if(leaf){
                if(selections[selectionDepth] == og_sel){
                    accept = &leaf->accept;
                    si_description = leaf->description;
                    uart.nputs(ARRANDN(CLIYES "  > " CLIRESET));
                } else {
                    uart.nputs(ARRANDN("    "));
                }
                uart.nputs(ARRANDN(leaf->name));
                uart.nputs(ARRANDN(NEWLINE));
            } else {
                MenuDir * d = getSelectedDir();
                if(d){
                    if(selections[selectionDepth] == og_sel){
                        si_description = d->description;
                        uart.nputs(ARRANDN(CLIYES "  >"));
                        uart.nputs(ARRANDN(dir_prefix));
                        uart.nputs(ARRANDN(CLIRESET));
                    } else {
                        uart.nputs(ARRANDN("    "));
                    }
                    uart.nputs(ARRANDN(leaf->name));
                    uart.nputs(ARRANDN(NEWLINE));
                }
            }
        }
        selections[selectionDepth] = og_sel;
    }

    uart.nputs(ARRANDN(CLIHIGHLIGHT NEWLINE));
    uart.nputs(ARRANDN(line_delim));
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // print description
    if(si_description){
        uart.nputs(si_description, STR_LEN_COMMON);
    } else {
        uart.nputs(ARRANDN("<NO DESCRIPTION>"));
    }

    uart.nputs(ARRANDN(CLIHIGHLIGHT NEWLINE));
    uart.nputs(ARRANDN(line_delim));
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // print message
    if(update && accept){
        (*accept)(NULL, 0, ARRANDN(message));
        uart.nputs(ARRANDN(message));
    } else {
        uart.nputs(ARRANDN("<NO MESSAGE>"));
    }

    uart.nputs(ARRANDN(CLIRESET NEWLINE CLIHIGHLIGHT));
    uart.nputs(ARRANDN(line_delim));
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // mirror user input
    uart.nputs(ARRANDN(input_start));
    uart.nputs(userInput, userInputLen);
}
