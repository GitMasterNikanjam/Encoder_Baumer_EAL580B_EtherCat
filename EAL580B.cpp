#include "EAL580B.h"
#include "EAL580B_objDict.h"        // Object dictionary for L7NH drivers

// #######################################################################

using namespace EAL580B_Namespace;

// #######################################################################

EAL580B::EAL580B()
{
    parameters.ETHERCAT_ID = -1;
    parameters.GEAR_RATIO = 0;
    parameters.PDOMAP_CONFIG_TYPE = 1;
    parameters.ROTATION_DIR = 0;
    parameters.SPD_UNIT = SPD_UNIT_STEP_1000MS;

    value.pos2BytesDeg = 0;
    value.pos2BytesStep = 0;
    value.posDeg = 0;
    value.posRawDeg = 0;
    value.posRawStep = 0;
    value.posStep = 0;
    value.velDegSec = 0;
    value.velStep = 0;

    _oneRevolutionMaxSteps = 1;
    _totalMeasuringMaxRange = 1;
    _velConStep2DegSec = 1;
    _virtualOffset = 0;
}

bool EAL580B::init(void)
{
    if(checkParameters() == false)
    {
        return false;
    }

    _oneRevolutionMaxSteps = getSingleTurnResolution();
    
    if(_oneRevolutionMaxSteps == 0)
    {
        return false;
    }

    _totalMeasuringMaxRange = getTotalMeasuringRange();

    if(_totalMeasuringMaxRange == 0)
    {
        return false;
    }

    if(!setRotationDirection(parameters.ROTATION_DIR))
    {
        return false;
    }

    _virtualOffset = _totalMeasuringMaxRange/2;

    switch(parameters.SPD_UNIT)
    {
        case SPD_UNIT_STEP_1000MS:
            _velConStep2DegSec = 360.0 / (float)_oneRevolutionMaxSteps;
        break;
        case SPD_UNIT_STEP_100MS:
            _velConStep2DegSec = 3600.0 / (float)_oneRevolutionMaxSteps;
        break;
        case SPD_UNIT_STEP_10MS:
            _velConStep2DegSec = 36000.0 / (float)_oneRevolutionMaxSteps;
        break;
        case SPD_UNIT_RPM:
            _velConStep2DegSec = 6.0;
        break;
        default:
            _velConStep2DegSec = 1.0;
    }

    if(!setSpeedMeasuringUnit(parameters.SPD_UNIT))
    {
        return false;
    }

    uint32_t mapping_value[10];

    switch(parameters.PDOMAP_CONFIG_TYPE)
    {
        case 1:    
            if(assignTxPDO_rank(1) == FALSE)
            {
                return false;
            }
                
            mapping_value[0] = MapValue_PositionValue;
            if(!_setTxPDO(1, mapping_value))
            {
                return false;
            }
        break;
        case 2:    
            if(assignTxPDO_rank(2) == FALSE)
            {
                return false;
            }
            
            mapping_value[0] = MapValue_PositionValue;
            mapping_value[1] = MapValue_SpeedValue4Bytes;
            if(!_setTxPDO(2, mapping_value))
            {
                return false;
            }
        break;
        case 3:  
            if(assignTxPDO_rank(4) == FALSE)
            {
                return false;
            }

            mapping_value[0] = MapValue_PositionRawValue;
            if(!_setTxPDO(1, mapping_value))
            {
                return false;
            }
        break;
        case 4:
            if(assignTxPDO_rank(7) == FALSE)
            {
                return false;
            }

            mapping_value[0] = MapValue_PositionValue2Bytes;
            if(!_setTxPDO(1, mapping_value))
            {
                return false;
            }
        break;
        default:
            errorMessage = "Error Encoder: init() was not successed.";
            return false;
    }

    return true;
}

bool EAL580B::checkParameters(void)
{
    bool state = (parameters.ETHERCAT_ID > 0) &&
                 (parameters.GEAR_RATIO >= 0) &&
                 (parameters.PDOMAP_CONFIG_TYPE >= 1) && (parameters.PDOMAP_CONFIG_TYPE <= 4) &&
                 (parameters.ROTATION_DIR <= 1) &&
                 (parameters.SPD_UNIT <= 3); 

    if(state == false)
    {
        errorMessage = "Error Encoder EAL580B: One or some parameters are not correct.";
        return false;
    }

    return true;
}

bool EAL580B::assignTxPDO_rank(int pdo_rank)
{
    // Read state of all slaves in ethercat.
    ec_readstate();

    if(ec_slave[parameters.ETHERCAT_ID].state != EC_STATE_PRE_OP)
    {
        errorMessage = "Error Encoder EAL580B: Assign TxPDO rank not successed beacuse salve not in pre operational state.";
        return FALSE;
    }
        
    int wkc;                // Working counter     
    uint16_t index;         // Certain index for assign in syncManager    
    uint8_t data;

    switch(pdo_rank)
    {
        case 1:
            index = Index_TPDOmapping_1;
        break;
        case 2:
            index = Index_TPDOmapping_2;
        break;
        case 3:
            index = Index_TPDOmapping_3;
        break;
        case 4:
            index = Index_TPDOmapping_4;
        break;
        case 5:
            index = Index_TPDOmapping_5;
        break;
        case 6:
            index = Index_TPDOmapping_6;
        break;
        case 7:
            index = Index_TPDOmapping_7;
        break;
        default:
            errorMessage = "Error Encoder: assignTxPDO_rank() was not successed.";
            return false;
    }

    data = 0;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_SyncManager3PDOAssignment, 0, FALSE, 1, &data, EC_TIMEOUTRXM);
    osal_usleep(10000);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: assignTxPDO_rank() was not successed.";
        return FALSE;
    }
        

    // Assign TxPDO index.
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_SyncManager3PDOAssignment, 1, FALSE, 2, &index, EC_TIMEOUTRXM);
    osal_usleep(10000);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: assignTxPDO_rank() was not successed.";
        return FALSE;
    }

    data = 1;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_SyncManager3PDOAssignment, 0, FALSE, 1, &data, EC_TIMEOUTRXM);
    osal_usleep(10000);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: assignTxPDO_rank() was not successed.";
        return FALSE;
    }
        
    _TxPDO_rank = pdo_rank;
    return TRUE;
}

uint16_t EAL580B::getTxPDO_rank(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_SyncManager3PDOAssignment, 1, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder EAL580B: SDO write not successed.";
        return FALSE;
    }

    return data;
}

bool EAL580B::_setTxPDO(uint8_t num_enteries, uint32_t* mapping_entry)
{
    _TxMapFlag[0] = 0;
    _TxMapFlag[1] = 0;
    _TxMapFlag[2] = 0;
    _TxMapFlag[3] = 0;
    _TxMapFlag[4] = 0;
    _TxMapFlag[5] = 0;

    // **Note: TxPDO of encoder is just read only access. So it must be at comment some bellow sections:

    // ------------------------------------------------------
    // int wkc;
    // uint16_t index;

    // switch(_TxPDO_rank)
    // {
    //     case 1:
    //         index = Index_TPDOmapping_1;
    //     break;
    //     case 2:
    //         index = Index_TPDOmapping_2;
    //     break;
    //     case 3:
    //         index = Index_TPDOmapping_3;
    //     break;
    //     case 4:
    //         index = Index_TPDOmapping_4;
    //     break;
    //     case 5:
    //         index = Index_TPDOmapping_5;
    //     break;
    //     case 6:
    //         index = Index_TPDOmapping_6;
    //     break;
    //     case 7:
    //         index = Index_TPDOmapping_7;
    //     break;
    //     default:
    //         return FALSE;
    // }

    // wkc = ec_SDOwrite(parameters.ETHERCAT_ID, index, 0, FALSE, 1, &num_enteries, EC_TIMEOUTRXM);

    // if(wkc <= 0)
    //     return FALSE;

    // ---------------------------------------------------------------

    uint8_t offset = 0;

    for(int subindex=1; subindex <= num_enteries; subindex++)
    {
        // ** Note: TxPDO just read only access!
        // --------------------------------------------------------------------
        // wkc = ec_SDOwrite(parameters.ETHERCAT_ID, index, subindex, FALSE, 4, &mapping_entry[subindex - 1], EC_TIMEOUTRXM);
        // osal_usleep(10000);   // delay 10ms

        // if(wkc <= 0)
        // {  
        //     return FALSE;
        // }
        // --------------------------------------------------------------------

        switch(mapping_entry[subindex - 1])
        {
            case MapValue_SystemTime:
                _TxMapOffset_SystemTime = offset;
                _TxMapFlag[0] = 1;
                offset += 4;
            break;
            case MapValue_PositionValue2Bytes:
                _TxMapOffset_PositionValue2Bytes = offset;
                _TxMapFlag[1] = 1;
                offset += 2;
            break;
            case MapValue_SpeedValue4Bytes:
                _TxMapOffset_SpeedValue4Bytes = offset;
                _TxMapFlag[2] = 1;
                offset += 4;
            break;
            case MapValue_SensorTemperature:
                _TxMapOffset_SensorTemperature = offset;
                _TxMapFlag[3] = 1;
                offset += 4;
            break;
            case MapValue_PositionValue:
                _TxMapOffset_PositionValue = offset;
                _TxMapFlag[4] = 1;
                offset += 4;
            break;
            case MapValue_PositionRawValue:
                _TxMapOffset_PositionRawValue = offset;
                _TxMapFlag[5] = 1;
                offset += 4;
            break;
            default:
                errorMessage = "Error Encoder: _setTxPDO() was not successed.";
                return FALSE;
        }
    
    }

    return true;
}

void EAL580B::_updateValuesConversion(void)
{
    value.pos2BytesDeg = 360.0 * (double)value.pos2BytesStep / (double)_oneRevolutionMaxSteps;
    value.posDeg = 360.0 * ((double)value.posStep - (double)_virtualOffset) / (double)_oneRevolutionMaxSteps;
    value.posRawDeg = 360.0 * (double)value.posRawStep / (double)_oneRevolutionMaxSteps;
    value.velDegSec = _velConStep2DegSec * (double)value.velStep;

    if(parameters.GEAR_RATIO > 0)
    {
        value.posDeg =  (double)parameters.GEAR_RATIO * value.posDeg;
        value.velDegSec = (double)parameters.GEAR_RATIO * value.velDegSec;
    }
}

bool EAL580B::saveParamsAll(void)
{
    int wkc;
    uint32_t data = SAVE;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_SaveParameters, 1, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);
    
    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool EAL580B::loadParamsAll(void)
{
    int wkc;
    uint32_t data = LOAD;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_RestoreParameters, 1, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

uint16_t EAL580B::getPositionValue2BytesSDO(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_PositionValue2Bytes, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;
        
    return data;
}

uint16_t EAL580B::getPositionValue2BytesPDO(void)
{
    if(_TxMapFlag[1] == 0)
    {
        return 0;
    }

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[parameters.ETHERCAT_ID].inputs;
    
    // Write the torque to the specified offset
    uint16_t data = *(uint16_t *)(inputs + _TxMapOffset_PositionValue2Bytes);

    return data;
}

int32_t EAL580B::getSpeedValue4BytesSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_SpeedValue4Bytes, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return data;
}

int32_t EAL580B::getSpeedValue4BytesPDO(void)
{
    if(_TxMapFlag[2] == 0)
    {
        return 0;
    }
    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[parameters.ETHERCAT_ID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + _TxMapOffset_SpeedValue4Bytes);

    return data;
}

bool EAL580B::setSpeedMeasuringUnit(uint8_t unit_num)
{
    int wkc;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_SpeedCalculationConfiguration, 2, FALSE, 1, &unit_num, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder EAL580B: setSpeedMeasuringUnit() was not successed.";
        return FALSE;
    }
        
    return TRUE; 
}

int32_t EAL580B::getSensorTemperatureSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_SensorTemperature, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return data;
}

int32_t EAL580B::getSensorTemperaturePDO(void)
{
    if(_TxMapFlag[3] == 0)
    {
        return 0;
    }

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[parameters.ETHERCAT_ID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + _TxMapOffset_SensorTemperature);

    return data;
}

uint32_t EAL580B::getPositionValueSDO(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_PositionValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

uint32_t EAL580B::getPositionValuePDO(void)
{
    if(_TxMapFlag[4] == 0)
    {
        return 0;
    }

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[parameters.ETHERCAT_ID].inputs;
    
    // Write the torque to the specified offset
    uint32_t data = *(uint32_t *)(inputs + _TxMapOffset_PositionValue);

    return data;
}

uint32_t EAL580B::getPositionRawValueSDO(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_PositionRawValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

uint32_t EAL580B::getPositionRawValuePDO(void)
{
    if(_TxMapFlag[5] == 0)
    {
        return 0;
    }

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[parameters.ETHERCAT_ID].inputs;
    
    // Write the torque to the specified offset
    uint32_t data = *(uint32_t *)(inputs + _TxMapOffset_PositionRawValue);

    return data;
}

uint32_t EAL580B::getSingleTurnResolution(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_SingleTurnResolution, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: getSingleTurnResolution() not successed.";
        return 0;
    }
        
    return data;
}

uint32_t EAL580B::getTotalMeasuringRange(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_TotalMeasuringRange, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: getTotalMeasuringRange() is not successed.";
        return 0;
    }
        
    return data;
}

bool EAL580B::setTotalMeasuringRange(uint32_t range)
{
    int wkc;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_TotalMeasuringRange, 0, FALSE, 4, &range, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool EAL580B::setGearFactorFunctionality(bool enable)
{
    int wkc;
    uint16_t data;

    if(enable)
    {
        data = 1;
    }
    else
    {
        data = 0;
    }

    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_GearFactorConfiguration, 1, FALSE, 2, &data, EC_TIMEOUTRXM);
    osal_usleep(10000); // delay 10ms

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: setGearFactorFunctionality() was not successed.";
        return false;
    }
        
    return true;
}

bool EAL580B::setGearFactorScale(uint32_t numerator, uint32_t denominator)
{
    int wkc;

    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_GearFactorConfiguration, 2, FALSE, 4, &numerator, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: setGearFactorScale() was not successed.";
        return false;
    }
        

    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_GearFactorConfiguration, 3, FALSE, 4, &denominator, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder: setGearFactorScale() was not successed.";
        return false;
    }

    return TRUE;
}

uint32_t EAL580B::getNumberOfDistinguishableRevolutions(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_NumberOfDistinguishableRevolutions, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

int32_t EAL580B::getOffsetValue(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_OffsetValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);
    
    if(wkc <= 0)
        return FALSE;
    
    return data;
}

bool EAL580B::setRotationDirection(uint8_t dir)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_OperatingParameters, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder EAL580B: setRotationDirection() was not successed.";
        return FALSE;
    }
        
    if(dir == 1)
        data |= (1 << 0);
    else if(dir == 0)
        data &= ~(1 << 0);
    else 
    {
        errorMessage = "Error Encoder EAL580B: setRotationDirection() was not successed.";
        return FALSE;
    }

    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_OperatingParameters, 0, FALSE, 2, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder EAL580B: setRotationDirection() was not successed.";
        return FALSE;
    }

    return TRUE;
}

bool EAL580B::setScalingFunctionControl(bool enable)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(parameters.ETHERCAT_ID, Index_OperatingParameters, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;
        
    if(enable)
        data |= (1 << 2);
    else
        data &= ~(1 << 2);

    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_OperatingParameters, 0, FALSE, 2, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool EAL580B::setPresetValueStep(uint32_t value)
{
    int wkc;
    wkc = ec_SDOwrite(parameters.ETHERCAT_ID, Index_PresetValue, 0, FALSE, 4, &value, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        errorMessage = "Error Encoder EAL580B: setPresetValueStep() was not successed."; 
        return FALSE;
    }

    return TRUE;
}

bool EAL580B::setPresetValueDeg(float value)
{
    double valueStep;

    valueStep = (double)_oneRevolutionMaxSteps * (double)value / 360.0;

    if(parameters.GEAR_RATIO > 0)
    {
        valueStep /= parameters.GEAR_RATIO;
    }

    valueStep += _virtualOffset;

    if(!setPresetValueStep((uint32_t)valueStep))
    {
        return false;
    }

    return true;
}

void EAL580B::updateValuesPDO(void)
{
    value.pos2BytesStep = getPositionValue2BytesPDO();
    value.posStep = getPositionValuePDO();
    value.posRawStep = getPositionRawValuePDO();
    value.velStep = getSpeedValue4BytesPDO();
    _updateValuesConversion();
}

void EAL580B::updateValuesSDO(void)
{
    value.pos2BytesStep = getPositionValue2BytesSDO();
    value.posStep = getPositionValueSDO();
    value.posRawStep = getPositionRawValueSDO();
    value.velStep = getSpeedValue4BytesSDO();

    _updateValuesConversion();
}












