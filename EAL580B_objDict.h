
// Basic commands order
#define SAVE        0x65766173  // 0:'S', 1:'A', 2:'V', 3:'E'
#define LOAD        0x64616f6c  // 0:'L', 1:'O', 2:'A', 3:'D'
#define RSET        0x74657372  // 0:'R', 1:'S', 2:'E', 3:'T'
#define READ        0x64616572  // 0:'R', 1:'E', 2:'A', 3:'D'  

#define MapValue_SystemTime                         0x20000020
#define MapValue_PositionValue2Bytes                0x20030010
#define MapValue_SpeedValue4Bytes                   0x20040020
#define MapValue_SensorTemperature                  0x21200020
#define MapValue_PositionValue                      0x60040020
#define MapValue_PositionRawValue                   0x600C0020

// ######################################################################
// Standard CoE objects (index range 0x1000 to 0x1FFF)

// Error Register
// This object contains an error status of the encoder. As described below only bit 0 is supported.
// Bit 0: generic error
#define Index_ErrorRegister                         0x1001

// Device Name
// This object contains the name of the encoder as a string.
#define Index_DeviceName                            0x1008

// Save Parameters
// This object is used to request the encoder to store parameters in non-volatile memory.
#define Index_SaveParameters                        0x1010

// Restore Parameters
// This object is used to restore hard-coded default values of the encoder.
#define Index_RestoreParameters                     0x1011

// TPDO mapping
//This object describes the content of transmit PDO
#define Index_TPDOmapping_1                         0x1A00
#define Index_TPDOmapping_2                         0x1A01
#define Index_TPDOmapping_3                         0x1A02
#define Index_TPDOmapping_4                         0x1A03
#define Index_TPDOmapping_5                         0x1A04
#define Index_TPDOmapping_6                         0x1A05
#define Index_TPDOmapping_7                         0x1A06

// Sync Manager 3 PDO Assignment
// This object is used to configure the layout of the cyclic EtherCAT process data which is sent from 
// slave to master. Object 0x1C13 contains the PDO assignment which is currently active.
#define Index_SyncManager3PDOAssignment             0x1C13



// ######################################################################
// Vendor-specific CoE objects (index range 0x2000 to 0x5FFF)

// System Time
// This object contains a timestamp of when the position data was sampled internally.
#define Index_SystemTime                            0x2000

//Gear Factor Configuration
// This object is used to configure the gear factor functionality. 
// The gear factor configuration affects the (scaled) position value.
#define Index_GearFactorConfiguration               0x2001

// Speed Calculation Configuration
// This object is used to configure the speed calculation which affects the speed value of the encoder.
#define Index_SpeedCalculationConfiguration         0x2002

// Position value 2 bytes
// This object contains the scaled position value if scaling is enabled. The object contains the raw position value 
// if scaling is disabled. To enable or disable position scaling bit 2 of object 0x6000 has to be set or reset.
#define Index_PositionValue2Bytes                   0x2003
    
// Speed value 4 bytes
// This object contains the speed value and can be mapped into a PDO.
// Notice:
// If the encoder is used in operation mode "Synchronous with SM3 Event" object 0x2201 has to be set 
// correctly in order to read out a valid speed value in object 0x2004.
#define Index_SpeedValue4Bytes                      0x2004

// Sensor Temperature
// This object contains the signed sensor temperature in degrees Celsius.
#define Index_SensorTemperature                     0x2120

// #######################################################################
// Profile-specific CoE objects (index range 0x6000 to 0xFFFF)

// Operating parameters
// This object is used to configure the code sequence and scaling.
#define Index_OperatingParameters                   0x6000

// Measuring units per revolution
// This object contains the desired singleturn resolution within the range from 1 to maximum encoder resolution.
#define Index_MeasuringUnitsPerRevolution           0x6001

// Total measuring range in measuring units         
#define Index_TotalMeasuringRange                   0x6002

// Preset value
// This object contains the desired absolute preset value. Writing this object executes a preset. 
// The encoder position will immediately be set to the absolute position value given in object 0x6003. 
// The encoder internally calculates a preset offset value which is being stored in a non-volatile memory 
// (no store command via CoE object 0x1010 required).
#define Index_PresetValue                           0x6003

// Position value
// This object contains the scaled position value if scaling is enabled. The object contains the raw position 
// value if scaling is disabled. To enable or disable position scaling bit 2 of object 0x6000 has to be set or reset.
#define Index_PositionValue                         0x6004

// Position raw value
// This object contains the raw position value (without the influence of scaling).
#define Index_PositionRawValue                      0x600C

// Singleturn resolution
// This object contains the maximum singleturn resolution in steps.
#define Index_SingleTurnResolution                  0x6501

// Number of distinguishable revolutions
// This object contains the maximum number of revolutions.
#define Index_NumberOfDistinguishableRevolutions    0x6502

// Offset value
// This object contains the preset offset of the encoder. The value of this object is calculated when 
// object 0x6003 (preset) is written or when a preset is triggered via the push button.
#define Index_OffsetValue                           0x6509