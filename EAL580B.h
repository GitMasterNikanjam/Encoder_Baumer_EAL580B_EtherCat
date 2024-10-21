#ifndef _EAL580B_H
#define _EAL580B_H

// Header Includes:
#include <iostream>                 // standard I/O operations
#include <chrono>                   // For time managements
#include <thread>                   // For thread programming
#include "ethercat.h"               // EtherCAT functionality 

// #################################################################################

class EAL580B
{
    public:

        // struct ParameterStruct
        // {
            
        // }parameters;

        struct ValueStruct
        {
            uint16_t pos2Bytes;
            uint32_t pos;
            uint32_t posRaw;
            int32_t vel;
        }value;

        // Set slave num/ID in ethercat slaves detected.
        void setSlaveID(uint32_t ID_num);

        /**
         * Use Sync Manager for assign certain TXPDO.
         * @note slave must in PRE_OP.
         * @note Use this function before ethercat configMap().
         * @note There are 7 value for pdo_rank. 1 to 7. other value return false.
         * @return true if success.
         *  */  
        bool assignTxPDO_rank(int pdo_rank);

        /**
         * @brief Return current rank of TxPDO
         * Rank is the which PDO mapping selected. Range: 1, 2, 3, 4, 5, 6, 7
         * @note return 0 if not successed.
         *  */ 
        uint16_t getTxPDO_rank(void);

        /**
         * Save all parameters in EEPROM memory.
         * @return true if successed.
         *  */ 
        bool saveParamsAll(void);

        /**
         * Restore and load all default parameters.
         * If the device later is powered off and on again the default parameters are written
         * @return true if successed.
         *  */ 
        bool loadParamsAll(void);

        /**
         * Get PositionValue2Bytes in SDO mode.
         * @return always 0 if not successed.
         *  */ 
        uint16_t getPositionValue2BytesSDO(void);

        /**
         * Get PositionValue2Bytes in PDO mode.
         * @note Hint: Use this function just when PositionValue2Bytes exist in TxPDO mapping, otherwise it return incorrect value.
         *  */ 
        uint16_t getPositionValue2BytesPDO(void);

        /**
         * Get SpeedValue4Bytes in SDO mode.
         * For each scaling option the measured value is provided as a „signed integer“. 
         * Positive values indicate the direction of rotation with rising position values.
         * Which rotational direction is assigned “positive” depends on the CW/CCW parameter setting.
         * @return always 0 if not successed.
         *  */ 
        int32_t getSpeedValue4BytesSDO(void);

        /**
         * Get SpeedValue4Bytes in PDO mode.
         * For each scaling option the measured value is provided as a „signed integer“. 
         * Positive values indicate the direction of rotation with rising position values.
         * Which rotational direction is assigned “positive” depends on the CW/CCW parameter setting.
         * @note Hint: Use this function just when SpeedValue4Bytes exist in TxPDO mapping, otherwise it return incorrect value.
         *  */ 
        int32_t getSpeedValue4BytesPDO(void);

        /** 
         * Configure the speed calculation which affects the speed value of the encoder
         * @param config: value for configuration.
         * @note Config value Range:
         * @note 0x00: steps/1000 ms 
         * @note 0x01: steps/100 ms 
         * @note 0x02: steps/10 ms 
         * @note 0x03: revolutions per Minute (rpm)
         *
         * @return true if successed.
         */
        bool setSpeedMeasuringUnit(uint8_t config);

        /**
         * Get SensorTemperature in SDO mode.
         * @return always 0 if not successed.
         *  */ 
        int32_t getSensorTemperatureSDO(void);

        /**
         * Get SensorTemperature in PDO mode.
         * @note Hint: Use this function just when SensorTemperature exist in TxPDO mapping, otherwise it return incorrect value.
         *  */ 
        int32_t getSensorTemperaturePDO(void);

        /**
         * Get scaled PositionValue in SDO mode.
         * @return always 0 if not successed.
         *  */ 
        uint32_t getPositionValueSDO(void);

        /**
         * Get scaled PositionValue in PDO mode.
         * @note Hint: Use this function just when PositionValue exist in TxPDO mapping, otherwise it return incorrect value.
         *  */ 
        uint32_t getPositionValuePDO(void);

        /**
         * Get PositionRawValue in SDO mode.
         * @return always 0 if not successed.
         *  */ 
        uint32_t getPositionRawValueSDO(void);

        /**
         * Get PositionRawValue in PDO mode.
         * @note Hint: Use this function just when PositionRawValue exist in TxPDO mapping, otherwise it return incorrect value.
         *  */ 
        uint32_t getPositionRawValuePDO(void);

        /**
         * @brief Position data behavior relates to the rotation direction of the shaft of the encoder when looking at the flange.
         * @param dir is direction behavior. 0: CW, 1:CCW     
         * @note 
         * - If dir be 0 the position value increases if the shaft is rotated clockwise (looking at the shaft).  
         *    
         * - If dir be 1 the position value increases if the shaft is rotated counterclockwise (looking at the shaft). 
         *    
         * - If dir be more than 1 value, function return false.   
         * @return true if successed.
         *  */  
        bool setRotationDirection(uint8_t dir);

        /**
         * If enable is false scaling of the position value is disabled.
         * If enable is true scaling of the position value is enabled.
         * @return true if successed.
         *  */ 
        bool setScalingFunctionControl(bool enable);

        /**
         * Get the maximum singleturn resolution in steps.
         * @return 0 if not successed.
         *  */ 
        uint32_t getSingleTurnResolution(void);

        /**
         * Get total measuring range (TMR).
         * This is maximum value for PositionValue4byte and PositionValue2byte.
         *  */ 
        uint32_t getTotalMeasuringRange(void);

        /**
         * Set total measuring range (TMR).
         * This is maximum value for PositionValue4byte and PositionValue2byte.
         * @note Hint: before this function use setScalingFunctionControl(TRUE) to enable change for total measuring range. 
         *  */ 
        bool setTotalMeasuringRange(uint32_t range);

        /**
         * Set gear factor functionality to active or inactive.
         * if enable is true, gear factor functionality is active.
         * @note Any change of GearFactorFunctionality clears internal position offsets (if any). So the current position reference is lost.
         * @return true if successed.
         *  */ 
        bool setGearFactorFunctionality(bool enable);

        /**
         * Set gear factor functionality scale for PositionValue calculation.
         * This function is only taken into account when gear factor functionality is active.
         * 
         * @note - For the “numerator” the following restrictions apply:   
         * @note - EAL580 MT encoder ST13 MT16, optical: numerator <= 8192   
         * @note - EAL580 MT encoder ST18 MT13, optical: numerator <= 4096   
         * @note - EAM580 MT encoder ST14 MT16, magnetic: numerator <= 16384   
         *
         * @note ### Measuring units per revolution = total measuring range ∗ (denominator / numerator)  
         * 
         * @return true if successed.
         */
        bool setGearFactorScale(uint32_t numerator, uint32_t denominator);

        /**
         * Get the maximum number of revolutions.
         * @return 0 if not successed.
         *  */ 
        uint32_t getNumberOfDistinguishableRevolutions(void);

        /**
         * the preset offset of the encoder. The value of this object is calculated when object 0x6003 (preset)
         * is written or when a preset is triggered via the push button.
         * @return OffsetValue.
         *  */  
        int32_t getOffsetValue(void);

        /**
         * @brief Set the PresetValue object that contains the desired absolute preset value. Writing this object executes a preset.
         * The encoder internally calculates a preset offset value which is being stored in a non-volatile memory
         * (no store command via CoE object 0x1010 required).
         * @param value is desired value for position at current position.  
         * @return true if successed.
         * @warning Use this function after setting direction rotation of encoder. Otherwise it maybe not correct set.
         *  */  
        bool setPresetValue(uint32_t value);

        /**
         * Automatic init and setup of driver.
         * @param ID: slave num/ID in ethercat slaves detected.
         * @param config_num: Configuration type.
         * 
         * @note config_num:1 -> PDOmap = {PositionValue}   
         * @note config_num:2 -> PDOmap = {PositionValue, SpeedValue4Bytes}     
         * @note config_num:3 -> PDOmap = {PositionRawValue}  
         * @note config_num:4 -> PDOmap = {PositionValue2Bytes}  
         *  */ 
        bool autoSetup(uint32_t ID, uint8_t config_num);

        bool updateStatesPDO(void);

        bool updateStatesSDO(void);

    private:

        // ID for salve number that used in simple_ethercat library.
        int _slaveID;

        // Access the process data inputs.
        uint8 *_inputs;

        // rank range: 1, 2, 3, 4, 5, 6, 7
        uint8_t _TxPDO_rank;     

        // Offset value for TX mapping:
        uint8_t _TxMapOffset_SystemTime;
        uint8_t _TxMapOffset_PositionValue2Bytes;
        uint8_t _TxMapOffset_SpeedValue4Bytes;
        uint8_t _TxMapOffset_SensorTemperature;
        uint8_t _TxMapOffset_PositionValue;
        uint8_t _TxMapOffset_PositionRawValue;

        /*
        _TxMapFlag indexes :
        0: SystemTime
        1: PositionValue2Bytes
        2: SpeedValue4Bytes
        3: SensorTemperature
        4: PositionValue
        5: PositionRawValue
        */
        uint8_t _TxMapFlag[6] = {0, 0, 0, 0, 0, 0};

        /**
         * @brief Set TxPDO object vector.
         * Elements of mapping array can be:
         * @brief 1) MapValue_SystemTime
         * @brief 2) MapValue_PositionValue2Bytes
         * @brief 3) MapValue_SpeedValue4Bytes
         * @brief 4) MapValue_SensorTemperature
         * @brief 5) MapValue_PositionValue
         * @brief 6) MapValue_PositionRawValue   
         * 
         * @param num_enteries: number of object that want to set in PDO mapping.
         * @param mapping_entry: array of objects for set PDO mapping.
         * 
         * @note set assignTxPDO_rank(int pdo_rank) before use this function.
         * @note slave must in PRE_OP.
         * @note Use this function before ethercat configMap().
         * @return true if successed. 
         *  */ 
        bool _setTxPDO(uint8_t num_enteries, uint32_t* mapping_entry);

};


#endif