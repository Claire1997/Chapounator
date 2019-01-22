#ifndef LIBCM5_AXS1_H
#define LIBCM5_AXS1_H



/*
 * AXS1 Constants
 */

#define AXS1_MODEL_NUMBER 0x000d
#define AXS1_NB_MELODIES 27



/*
 * AXS1 Control Table indexes
 */

#define AXS1_CTAB_ID_ModelNumberLow               0x00
#define AXS1_CTAB_ID_ModelNumberHi                0x01
#define AXS1_CTAB_ID_FirmwareVersion              0x02
#define AXS1_CTAB_ID_Id                           0x03
#define AXS1_CTAB_ID_BaudRate                     0x04
#define AXS1_CTAB_ID_ReturnDelayTime              0x05
#define AXS1_CTAB_ID_HighestLimitTemperature      0x0b
#define AXS1_CTAB_ID_LowestLimitVoltage           0x0c
#define AXS1_CTAB_ID_HighestLimitVoltage          0x0d
#define AXS1_CTAB_ID_StatusReturnLevel            0x10
#define AXS1_CTAB_ID_ObstacleDetectedCompareValue 0x14
#define AXS1_CTAB_ID_LightDetectedCompareValue    0x15
#define AXS1_CTAB_ID_LeftIRSensorData             0x1a
#define AXS1_CTAB_ID_CenterIRSensorData           0x1b
#define AXS1_CTAB_ID_RightIRSensorData            0x1c
#define AXS1_CTAB_ID_LeftLuminosity               0x1d
#define AXS1_CTAB_ID_CenterLuminosity             0x1e
#define AXS1_CTAB_ID_RightLuminosity              0x1f
#define AXS1_CTAB_ID_ObstacleDetectionFlag        0x20
#define AXS1_CTAB_ID_LuminosityDetectionFlag      0x21
#define AXS1_CTAB_ID_SoundData                    0x23
#define AXS1_CTAB_ID_SoundDataMaxHold             0x24
#define AXS1_CTAB_ID_SoundDetectedCount           0x25
#define AXS1_CTAB_ID_SoundDetectedTimeLo          0x26
#define AXS1_CTAB_ID_SoundDetectedTimeHi          0x27
#define AXS1_CTAB_ID_BuzzerIndex                  0x28
#define AXS1_CTAB_ID_BuzzerTime                   0x29
#define AXS1_CTAB_ID_PresentVoltage               0x2a
#define AXS1_CTAB_ID_PresentTemperature           0x2b
#define AXS1_CTAB_ID_RegisteredInstruction        0x2c
#define AXS1_CTAB_ID_IRRemoconArrived             0x2e
#define AXS1_CTAB_ID_Lock                         0x2f
#define AXS1_CTAB_ID_IRRemoconRXData0             0x30
#define AXS1_CTAB_ID_IRRemoconRXData1             0x31
#define AXS1_CTAB_ID_IRRemoconTXData0             0x32
#define AXS1_CTAB_ID_IRRemoconTXData1             0x33
#define AXS1_CTAB_ID_ObstacleDetectedCompare      0x34
#define AXS1_CTAB_ID_LightDetectedCompare         0x35



#endif /* LIBCM5_AXS1_H */
