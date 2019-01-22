#ifndef LIBCM5_AX12_H
#define LIBCM5_AX12_H


/*
 * AX12 Constants
 */

#define AX12_MODEL_NUMBER       0x000c
#define AX12_GOAL_POSITION_MIN  0x0000
#define AX12_GOAL_POSITION_MAX  0x03ff
#define AX12_SPEED_UNCONTROLED  0x0000
#define AX12_SPEED_MIN          0x0001
#define AX12_SPEED_MAX          0x03ff
#define AX12_CCW_TURN_DIRECTION 0x0000
#define AX12_CW_TURN_DIRECTION  0x0400
#define AX12_MAX_SPEED_IN_RPM   114



/*
 * AX12 Control Table indexes
 */

#define AX12_CTAB_ID_ModelNumberLow          0x00
#define AX12_CTAB_ID_ModelNumberHi           0x01
#define AX12_CTAB_ID_FirmwareVersion         0x02
#define AX12_CTAB_ID_Id                      0x03
#define AX12_CTAB_ID_BaudRate                0x04
#define AX12_CTAB_ID_ReturnDelayTime         0x05
#define AX12_CTAB_ID_CWAngleLimitLo          0x06
#define AX12_CTAB_ID_CWAngleLimitHi          0x07
#define AX12_CTAB_ID_CCWAngleLimitLo         0x08
#define AX12_CTAB_ID_CCWAngleLimitHi         0x09
#define AX12_CTAB_ID_HighestLimitTemperature 0x0b
#define AX12_CTAB_ID_LowestLimitVoltage      0x0c
#define AX12_CTAB_ID_HighestLimitVoltage     0x0d
#define AX12_CTAB_ID_MaxTorqueLo             0x0e
#define AX12_CTAB_ID_MaxTorqueHi             0x0f
#define AX12_CTAB_ID_StatusReturnLevel       0x10
#define AX12_CTAB_ID_AlarmLed                0x11
#define AX12_CTAB_ID_AlarmShutdown           0x12
#define AX12_CTAB_ID_DownCalibrationLo       0x14
#define AX12_CTAB_ID_DownCalibrationHi       0x15
#define AX12_CTAB_ID_UpCalibrationLo         0x16
#define AX12_CTAB_ID_UpCalibrationHi         0x17
#define AX12_CTAB_ID_TorqueEnable            0x18
#define AX12_CTAB_ID_Led                     0x19 
#define AX12_CTAB_ID_CWComplianceMargin      0x1a
#define AX12_CTAB_ID_CCWComplianceMargin     0x1b
#define AX12_CTAB_ID_CWComplianceSlope       0x1c
#define AX12_CTAB_ID_CCWComplianceSlope      0x1d
#define AX12_CTAB_ID_GoalPositionLo          0x1e
#define AX12_CTAB_ID_GoalPositionHi          0x1f
#define AX12_CTAB_ID_MovingSpeedLo           0x20
#define AX12_CTAB_ID_MovingSpeedHi           0x21
#define AX12_CTAB_ID_TorqueLimitLo           0x22
#define AX12_CTAB_ID_TorqueLimitHi           0x23
#define AX12_CTAB_ID_PresentPosLo            0x24
#define AX12_CTAB_ID_PresentPosHi            0x25
#define AX12_CTAB_ID_PresentSpeedLo          0x26
#define AX12_CTAB_ID_PresentSpeedHi          0x27
#define AX12_CTAB_ID_PresentLoadLo           0x28
#define AX12_CTAB_ID_PresentLoadHi           0x29
#define AX12_CTAB_ID_PresentVoltage          0x2a
#define AX12_CTAB_ID_PresentTemperature      0x2b
#define AX12_CTAB_ID_RegisteredInstruction   0x2c
#define AX12_CTAB_ID_Moving                  0x2e
#define AX12_CTAB_ID_Lock                    0x2f
#define AX12_CTAB_ID_PunchLo                 0x30
#define AX12_CTAB_ID_PunchHi                 0x31



#endif /* LIBCM5_AX12_H */
