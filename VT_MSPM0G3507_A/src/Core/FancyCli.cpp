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

    MenuDir * currentDir = getSelectedDir();
    uint8_t totalItems = currentDir ? (currentDir->dirCount + currentDir->leafCount) : 0;
    uint8_t currentSelection = selections[selectionDepth];

    if(_charInput_contains(ARRANDN(keys_changeState), c)){
        clistate = static_cast<CLISTATE>((static_cast<uint8_t>(clistate) + 1) % static_cast<uint8_t>(CLISTATE::_count));
        return true;
    }

    switch(clistate){
        case CLISTATE::MENU_SELECTION:
            // Select Up/Previous
            if(_charInput_contains(ARRANDN(keys_selectUp), c)){
                if(currentSelection > 0) {
                        selections[selectionDepth]--;
                    }
                    getSelectedLeaf(); // Recalculate and bound
                }

                // Select Down/Next
            if(_charInput_contains(ARRANDN(keys_selectDown), c)){
                if (currentSelection < totalItems - 1) { // Check against the maximum index
                        selections[selectionDepth]++;
                    }
                    getSelectedLeaf(); // Recalculate and bound
                }

            if (_charInput_contains(ARRANDN(keys_select), c)) {
                MenuLeaf* leaf = getSelectedLeaf();

                if (leaf) {
                    if (leaf->accept) {
                        message[0] = '\0';
                        leaf->accept(userInput, userInputLen, ARRANDN(message));
                    }
                } else {
                    selectionDepth++;
                    getSelectedDir();
                }

                ret = true;
            }

            if(_charInput_contains(ARRANDN(keys_unselect), c)){
                selectionDepth--;
                getSelectedDir();
            }

            ret = true;
            break;
        case CLISTATE::USER_INPUT:
            if(c == 8 || c == 127){ // character is backspace
                if(userInputLen != 0){
                    userInputLen--;
                    if(uart) DL_UART_transmitDataBlocking(uart->reg, c);
                }
                break;
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
        if(!dir->dirs || dir->dirCount == 0) // No more sub-directories to enter
            break;

        // Ensure selection index is within the valid directory range
        if(selections[i] >= dir->dirCount){
            selections[i] = dir->dirCount - 1; // Correct to the last valid dir
        }

        dir = &dir->dirs[selections[i]];
    }

    // After the loop, the current directory (dir) is now the current selectionDepth's parent.
    // If we broke out early because the path was invalid, we set selectionDepth back.
    if(i < selectionDepth){
        selectionDepth = i; // Reset depth to the last valid one we found.
    }

    // Now, apply the check for the current directory we are about to display
    if(dir->dirCount + dir->leafCount > 0){
        if(selections[selectionDepth] >= dir->dirCount + dir->leafCount){
            selections[selectionDepth] = dir->dirCount + dir->leafCount - 1;
        }
    } else {
        selections[selectionDepth] = 0; // If directory is empty, selection must be 0
    }

    return dir;
}

FancyCli::MenuLeaf * FancyCli::getSelectedLeaf(){
    MenuDir * dir = getSelectedDir();
    if(!dir)
        return NULL;

    // Check if the current selection index falls within the leaf range
    if(selections[selectionDepth] >= dir->dirCount){

        // If there are NO leaves, correct the selection back to the last directory index
        if(dir->leafCount == 0){
            // If there are also no directories, selection defaults to 0.
            selections[selectionDepth] = dir->dirCount ? dir->dirCount - 1 : 0;
            return NULL;
        }

        // Calculate the index of the leaf (relative to the start of the leaves)
        uint8_t sel_leaf = selections[selectionDepth] - dir->dirCount;

        // If the calculated leaf index is OUT OF BOUNDS (it's >= leafCount)
        if(sel_leaf >= dir->leafCount){
            // Correct the leaf index to the last valid leaf
            sel_leaf = dir->leafCount - 1;
            // Correct the overall selection index
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
    MenuDir * dir = root;
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
    uart.putu32d(selections[selectionDepth] + 1);
    uart.nputs(ARRANDN("/"));
    if(dir) uart.putu32d(dir->dirCount + dir->leafCount);
    else    uart.nputs(ARRANDN("?"));
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

    // print node overton window
    char * si_description = NULL;
    NodeAccept accept = NULL;
    if(dir)
    {
        // Corrected Window Logic
        uint8_t a, b;
        uint8_t total_count = dir->dirCount + dir->leafCount;
        uint8_t og_sel = selections[selectionDepth];

        // 1. Calculate the starting index 'a'
        // Keep the selection near the top of the display window, but start at 0 if possible
        if (og_sel < total_count) {
            if (og_sel < DISPLAY_ITEM_COUNT - 1) { // If selection is within the first N-1 items
                a = 0;
            } else {
                // Start the window N items before the current selection
                // This keeps the selected item near the top of the window, allowing scrolling
                a = og_sel - (DISPLAY_ITEM_COUNT - 1);
            }
        } else {
            // Should not happen if other bounds checks are working, but as a safeguard:
            a = 0;
        }

        // 2. Calculate the ending index 'b'
        b = a + DISPLAY_ITEM_COUNT;
        if(b > total_count) {
            b = total_count;
            // PUSH-BACK CORRECTION: If 'b' was capped, ensure 'a' didn't start too high.
            // This makes the list stick to the bottom when the end is reached.
            if (b >= DISPLAY_ITEM_COUNT) {
                a = b - DISPLAY_ITEM_COUNT;
            } else {
                a = 0;
            }
        }
        if(b > (dir->dirCount + dir->leafCount))
            b = (dir->dirCount + dir->leafCount);

        for(selections[selectionDepth] = a; selections[selectionDepth] < b; selections[selectionDepth]++){
            MenuLeaf * leaf = getSelectedLeaf();
            if(leaf){
                if(selections[selectionDepth] == og_sel){
                    accept = leaf->accept;
                    si_description = leaf->description;
                    uart.nputs(ARRANDN(CLIYES "  > " CLIRESET));
                } else {
                    uart.nputs(ARRANDN("    "));
                }
                uart.nputs(leaf->name, STR_LEN_COMMON);
                uart.nputs(ARRANDN(NEWLINE));
            } else {
                MenuDir * parent = getSelectedDir();
                if (parent) {
                    MenuDir * childDir = &parent->dirs[selections[selectionDepth]];
                    if (childDir) {
                        if (selections[selectionDepth] == og_sel) {
                            si_description = childDir->description;
                            uart.nputs(ARRANDN(CLIYES "  >"));
                            uart.nputs(ARRANDN(dir_prefix));
                            uart.nputs(ARRANDN(CLIRESET));
                        } else {
                            uart.nputs(ARRANDN("    "));
                        }
                        uart.nputs(childDir->name, STR_LEN_COMMON);
                        uart.nputs(ARRANDN(NEWLINE));
                    }
                }
            }
        }
        selections[selectionDepth] = og_sel;
    }

    uart.nputs(ARRANDN(CLIHIGHLIGHT));
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
    if (message[0] != '\0') {
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
