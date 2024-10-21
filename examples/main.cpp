// For compile: 
// g++ -o main ./*.cpp   ../*.cpp  ../../SimpleEthercat/SimpleEthercat.cpp -lsoem -Wall -Wextra -std=c++17

// For run:
// sudo ./main

// ###############################################
// Header Includes:
#include <iostream>
#include "../../SimpleEthercat/SimpleEthercat.h"
#include "../EAL580B.h"
#include <chrono>

// ############################################################################
// Define macros:

#define ENCODER1_ETH_ID              2
#define ENCODER2_ETH_ID              3

#define PDO_MODE                     1
#define SDO_MODE                     0

#define ENCODER1_GEARRATIO           1.78571    // (130/20*25/91)

// ###############################################
// Global Variables and objects:

// Ethercat object for general ethercat managements.
SimpleEthercat ethercat;

//  Port name for ethercat slave connections.
const char port_name[] = "enp2s0";

// Encoder object.
EAL580B encoder1;
EAL580B encoder2;

uint32_t encoderOneRevolutionSteps;

// ################################################
// Declare functions

bool init(void);
void loop(void);

// #################################################

int main(void)
{
    if(init() == true)
    {
        printf("Operational state reached for all slaves.\n");
        loop();
        printf("\nRequest init state for all slaves\n");
        ethercat.setInitState();
    }
    else
    {
        /*
        If not all slaves reach the operational state within the specified timeout, 
        the code prints a message indicating which slaves failed to reach the operational state.
        */
        printf("Not all slaves reached operational state.\n");
        ethercat.showStates();
    }

    printf("close ethercat socket\n");
    ethercat.close();
    
    return 0;
}

void loop(void)
{   
    // Using time point and system_clock
    std::chrono::time_point<std::chrono::system_clock> start, end;

    while(1)
    {
        start = std::chrono::system_clock::now();

        bool flag_proccess = ethercat.updateProccess();

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        
        if(flag_proccess)
        {
            // SDO mode test:
            if(SDO_MODE == 1)
            {
                encoder1.updateStatesSDO();
            }

            // -------------------------------------------------------------
            // PDO mode test:
            if(PDO_MODE == 1)
            {
                encoder1.updateStatesPDO();
            }
            // ------------------------------------------------------------

            printf("pos2Bytes: %12d", encoder1.value.pos2Bytes); printf(", ");
            printf("posRaw: %12d", encoder1.value.posRaw); printf(", ");
            printf("pos: %f [deg]", (ENCODER1_GEARRATIO)*360*(float)encoder1.value.pos/(float)encoderOneRevolutionSteps); printf(", ");
            printf("vel: %f [deg/s]", (ENCODER1_GEARRATIO)*360*(float)encoder1.value.vel/(float)encoderOneRevolutionSteps); printf("\n");
        }
        else
        {
            printf("Ethercat update proccess failed!\n");
        }        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool init(void)
{
    // Init Ethercat object. connect lan port to ethercat.
    if(ethercat.init(port_name))
    {
        printf("Ethercat on %s succeeded.\n",port_name);
        if(ethercat.configSlaves())
        {
            printf("Slaves Configured!\n");
            printf("Slave state are in PRE_OP state.\n");
            printf("%d slaves found and configured.\n",ethercat.getSlaveCount());

            if(encoder1.autoSetup(ENCODER1_ETH_ID, 2) && encoder2.autoSetup(ENCODER2_ETH_ID, 2))
            {
                printf("Encoders setup finished successfully.\n");
            }
            else
            {
                printf("Encoders setup can not finished.\n");
                return false;
            }

            bool state;
            
            state = encoder1.setSpeedMeasuringUnit(0x00);
            state = state && encoder1.setRotationDirection(1);
            state = state && encoder1.setPresetValue(0);
            
            if(state == false)
            {
                printf("Encoder configuration not successed.\n");
                return false;
            }
            
            encoderOneRevolutionSteps = encoder1.getSingleTurnResolution();
            printf("Encoder single turn position value step count: %d\n", encoderOneRevolutionSteps);

            if(ethercat.configMap() == true)
            {
                printf("IOmap configured.\n");
            }
            else
            {
                printf("IOmap could not configured.\n");
                return false;
            }
            
            ethercat.configDc();
            printf("Distribution clock configured.\n");

            if(ethercat.setSafeOperationalState())
            {
                printf("Slave state are in SafeOperation state.\n");
            }
            else
            {
                std::cout << ethercat.errorMessage << std::endl;
            }

            if(ethercat.setOperationalState() == false)
            {
                return false;
            }
        }
        else
        {
            printf("Failed to config slaves\n");
            printf("Error: No slaves detected!\n");
            return false;
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root maybe solve problem \n",port_name);
        return false;
    }

    ethercat.listSlaves();

    return true;
}