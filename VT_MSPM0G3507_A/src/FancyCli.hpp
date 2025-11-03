/*
 * FancyCli.hpp
 *
 *  Created on: Nov 2, 2025
 *      Author: turtl
 */

#ifndef SRC_FANCYCLI_HPP_
#define SRC_FANCYCLI_HPP_

#include "Core/system.hpp"

class FancyCli {
public:
    static constexpr uint8_t STR_LEN_COMMON = MAX_STR_LEN_COMMON;
    static constexpr uint8_t STR_LEN_SMALL  = MAX_STR_LEN_COMMON / 2;

    typedef bool(*NodeAccept)(char * userInput, uint8_t strlen, char * msg, uint8_t msglen);

    /* epic rendation of dog water cli
     *
     *<window start>
     * <software info>
     * <menu path>
     * --- options ---<selection #>
     *   <item>
     * > <item (selected)>
     *   <item>
     *   <item>
     *   <item>
     * ------------
     * <selected node description>
     * ------------
     * <message>
     * ------------
     * # <user input shown here>
     *<window end>
     */
    static constexpr uint8_t DISPLAY_ITEM_COUNT = 5;

    char userInput[MAX_STR_LEN_COMMON];
    uint8_t userInputLen = 0;

    struct MenuLeaf {
        char * name;
        char * description;

        NodeAccept accept;
    };

    struct MenuDir {
        char * name;
        char * description;

        MenuLeaf * leafs;
        uint8_t leafCount;
        MenuDir * dirs;
        uint8_t dirCount;
    };

    MenuDir * root;
    uint8_t selections[5] = {0}; // size is max depth
    uint8_t selectionDepth = 0; // selected selection. zero indexed

    void printFrame(System::UART::UART&, bool update_buffers);

    bool charInput(System::UART::UART *, char);

    // returns the dir closest to what ever is selected(not including).
    MenuDir * getSelectedDir();
    // returns the selected node if its a leaf, null otherwise.
    MenuLeaf * getSelectedLeaf();

    static_assert(sizeof(selections) > 0);
    static_assert(DISPLAY_ITEM_COUNT > 0);
};

#endif /* SRC_FANCYCLI_HPP_ */
