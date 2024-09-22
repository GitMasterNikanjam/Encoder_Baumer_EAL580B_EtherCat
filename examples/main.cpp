// For compile: 
// g++ -o main ./*.cpp   ../*.cpp  ../../simpleEthercat/simpleEthercat.cpp -lsoem -Wall -Wextra -std=c++17

// For run:
// sudo ./main

// ###############################################
// Header Includes:
#include <iostream>
#include "../../simpleEthercat/simpleEthercat.h"
#include "../EAL580B.h"
#include <chrono>

// ###############################################
// Global Variables

// Ethercat object for general ethercat managements.
SIMPLE_ETHERCAT ETHERCAT;

//  Port name for ethercat slave connections.
const char port_name[] = "enp2s0";

// Encoder object.
EAL580B encoder1;

// ################################################
// Declare functions

void loop(void);

// #################################################

int main(void)
{
    // Init Ethercat object. connect lan port to ethercat.
    if(ETHERCAT.init(port_name))
    {
        printf("Ethercat on %s succeeded.\n",port_name);
        if(ETHERCAT.configSlaves())
        {
            printf("Slaves Configured!\n");
            printf("Slave state are in PRE_OP state.\n");
            printf("%d slaves found and configured.\n",ETHERCAT.getSlaveCount());

            encoder1.setPresetValue(0);

            if(encoder1.autoSetup(1, 2))
            {
                printf("encoder autoSetup finished successfully.\n");
            }
            encoder1.setSpeedMeasuringUnit(0x00);

            ETHERCAT.configMap();
            printf("IOmap Configured.\n");

            ETHERCAT.configDc();
            printf("Distribution clock configured.\n");

            if(ETHERCAT.setSafeOperationalState())
            {
                printf("Slave state are in SafeOperation state.\n");
            }

            ETHERCAT.setOperationalState();
            ETHERCAT.listSlaves();
        }
        else
        {
            printf("Failed to config slaves\n");
            printf("Error: No slaves detected!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root maybe solve problem \n",port_name);
    }

    if(ETHERCAT.getState() == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves.\n");
        loop();
        printf("\nRequest init state for all slaves\n");
        ETHERCAT.setInitState();
    }
    else
    {
        /*
        If not all slaves reach the operational state within the specified timeout, 
        the code prints a message indicating which slaves failed to reach the operational state.
        */
        printf("Not all slaves reached operational state.\n");
        ETHERCAT.showStates();
    }

    printf("close ethercat socket\n");
    ETHERCAT.close();
    
    return 0;
}

void loop(void)
{   
    // Using time point and system_clock
    std::chrono::time_point<std::chrono::system_clock> start, end;

    while(1)
    {
        start = std::chrono::system_clock::now();

        bool flag_proccess = ETHERCAT.updateProccess();

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        
        if(flag_proccess)
        {
            uint16_t pos2Bytes;
            uint32_t pos;
            uint32_t posRaw;
            int32_t vel;

            // --------------------------------------------------------------
            // SDO mode test:
            pos2Bytes = encoder1.getPositionValue2BytesSDO();
            pos = encoder1.getPositionValueSDO();
            posRaw = encoder1.getPositionRawValueSDO();
            vel = encoder1.getSpeedValue4BytesSDO();
            // -------------------------------------------------------------
            // PDO mode test:
            // pos2Bytes = encoder1.getPositionValue2BytesPDO();
            // pos = encoder1.getPositionValuePDO();
            // posRaw = encoder1.getPositionRawValuePDO();
            // vel = encoder1.getSpeedValue4BytesPDO();
            // ------------------------------------------------------------

            printf("pos2Bytes: %12d", pos2Bytes); printf(", ");
            printf("posRaw: %12d", posRaw); printf(", ");
            printf("pos: %12d", pos); printf(", ");
            printf("vel: %12d", vel); printf("\n");
        }
        else
        {
            printf("Ethercat update proccess failed!\n");
        }        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}