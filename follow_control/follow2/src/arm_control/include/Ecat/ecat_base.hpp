#pragma once

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"
// #include "ethercat_to_can/ecat_can_base/ecat_typedef.hpp"
// /home/razer/桌面/Ecat/x5_ecat_test/src/arm_control/include/Ecat/ecat_base.hpp
// #include "/home/razer/桌面/Ecat/x5_ecat_test/src/arm_control/include/Ecat/ecat_typedef.hpp"
// #include "../../include/Ecat/ecat_base.hpp"
#include "../../include/Ecat/ecat_typedef.hpp"


#define EC_VER1

namespace ecat
{
    class EcatBase
    {
    public:
        EcatBase(int _slave_num);
        ~EcatBase();

        void EcatStart(char *ifname);
        void EcatSyncMsg();
        void EcatStop();

        Ecat_Outputs_Pack packet_tx[2];
        Ecat_Inputs_Pack packet_rx[2];

    private:


        int slave_num = 1;
        int pdo_output_byte = 30;
        int pdo_input_byte = 34;

        char IOmap[4096];
        volatile int wkc;
        int expectedWKC;
        int EC_TIMEOUTMON = 500;
    };
}

// #include "ethercat_to_can/ecat_can_base/ecat_typedef.h"

