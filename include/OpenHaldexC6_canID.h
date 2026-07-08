#pragma once

// all the CAN addresses in here.  Not all used, but worth keeping note of for future projects
#define MOTOR1_ID 0x280
#define MOTOR2_ID 0x288
#define MOTOR3_ID 0x380
#define MOTOR5_ID 0x480
#define MOTOR6_ID 0x488
#define MOTOR7_ID 0x588
#define MOTOR8_ID 0x48A
#define MOTORBREMS_ID 0x284
#define MOTOR_FLEX_ID 0x580
#define BRAKES1_ID 0x1A0   // DLC 8 x
#define BRAKES2_ID 0x5A0   // DLC 8 x
#define BRAKES3_ID 0x4A0   // DLC 8 x
#define BRAKES4_ID 0x2A0   // DLC 3 x
#define BRAKES5_ID 0x4A8   // DLC 8 x
#define BRAKES6_ID 0x1A8   // DLC 3 x
#define BRAKES8_ID 0x1AC   // DLC 8 x
#define BRAKES9_ID 0x0AE   // DLC 8
#define BRAKES10_ID 0x3A0  // DLC 8 x
#define BRAKES11_ID 0x5B7  // DLC 8 x

#define GRA_ID 0x38A
#define HALDEX_ID 0x2C0

#define ZAS_ID 0x573
#define GATEWAY_ID 0x720
#define mGetriebe_1 0x440
#define mGetriebe_2 0x540
#define mGetriebe_4 0x548
#define mGetriebe_5 0x542
#define mGetriebe_6 0x44C
#define mGetriebe_7 0x544
#define mGetriebe_8 0x450
#define mGetriebe_9 0x454

#define mLW_1 0x0C2         // DLC 7
#define mLenkhilfe_1 0x3D0  // DLC 6
#define mLenkhilfe_2 0x3D2  // DLC 6
#define mLenkhilfe_3 0x0D0  // DLC 6

#define mGate_Komf_1 0x390           // DLC 8 100ms
#define mGate_Komf_2 0x392           // DLC 8/(4) 100ms
#define mGate_Komf_3 0x393           // DLC 8/ 200ms
#define mBSG_Last 0x570              // DLC 5 100ms
#define mDiagnose_1 0x7D0            // DLC 8 1000ms
#define mSoll_Verbauliste_neu 0x5DC  // DLC 8 100ms
#define mSysteminfo_1 0x5D0          // DLC 8(6) 100ms
// PQ ACAN engine/motor periodic broadcasts (KMatrix V5.20.6F)
#define mMotor_12 0x58F              // PQ ACAN, DLC 8, cycle 500ms, timeout 5000ms (SCR/E85/CO2/coolant temp)
#define NMH_Gateway 0x720            // DLC 7 200ms
#define mKombi_1 0x320
#define mKombi_2 0x420
#define mKombi_3 0x520

// Custom CAN IDs
#define OPENHALDEX_BROADCAST_ID 0x6B0         // broadcast OpenHaldex via. CAN over this address - used for stating mode/performance etc
#define OPENHALDEX_EXTERNAL_CONTROL_ID 0x6A0  // recieve OpenHaldex via. CAN over this address - used for changing modes etc

#define diagnostics_1_ID 0x764
#define diagnostics_2_ID 0x200
#define diagnostics_3_ID 0x710
#define diagnostics_4_ID 0x71D
#define diagnostics_5_ID 0x70F

#define LWI_01 0x086
#define ESP_14 0x08A
#define MOTOR_11 0x0A7
#define MOTOR_12 0x0A8
#define GETRIEBE_11 0x0AD
#define GETRIEBE_17 0x0b1
#define MOTOR_04 0x107

#define ESP_19 0x0b2
#define ESP_21 0x0fd

#define ESP_02 0x101
#define EPB_01 0x104 
#define ESP_05 0x106

#define ESP_10 0x116
#define MOTOR_20 0x121
#define ESP_18 0x135

#define ESP_29 0x18c
#define KOMBI_01 0x30b

#define CHARISMA_01 0x385
#define ESP_07 0x392

#define MOTOR_14 0x3be
#define GETRIEBE_14 0x3c8
#define GATEWAY_72 0x3db
#define BLINKMODI_02 0x366 // MQB hazard / turn-signal status broadcast

#define Parkhilfe_04 0x54b
#define SYSTEMINFO_01 0x585
#define ESP_23 0x5be

#define MOTOR_07 0x640
#define MOTOR_CODE_01 0x641
#define unknown2 0x64a

#define ESP_20 0x65d
#define DIAGNOSE_01 0x6b2
#define KOMBI_02 0x6b7

#define HALDEX_ID_GEN5 0x118

// Gen41 (Vauxhall / GMW8762 PPEI FDCM) Haldex-originated feedback frames.
// These are TX'd BY the Haldex; the ESP must RX-only and never retransmit them.
//   Bus1 (Haldex CAN):
#define HALDEX_GEN41_SEC_AXLE_STATUS_ID    0x1CC // DLC=3, Secondary Axle Status (clutch torque feedback + state)
#define HALDEX_GEN41_REAR_AXLE_STATUS_ID   0x1D1 // DLC=4, Rear Axle Status (status flags + diagnostic metrics)
//   Bus0 (vehicle CAN):
#define HALDEX_GEN41_SEC_AXLE_GENINFO_ID   0x1CF // DLC=1, Secondary Axle General Information (heartbeat, D0=0x03)
#define HALDEX_GEN41_DRIVETRAIN_STATE_ID   0x331 // DLC=2, Drive Train State (heartbeat, [F8 FF])

// ─── Gen42 (Ford Focus RS / CGEA1.2 PT-CAN) ──────────────────────────────────
// Sources: ford_cgea1_2_ptcan_2011.dbc, focus_rs_understood_signals.dbc,
//          focus_rs_can_reformatted_reference.md, VehicleCAN.dbc
// All Bus1 frames simulated by the ESP (replacing the vehicle powertrain CAN).
// 0x260 is the TCCM (Haldex AWD module) response, RX-only on Bus0.
// Senders: PCM=Powertrain, ABS=ABS/ESC, TCM=Transmission, BCM=Body,
//          IPC=Instrument Cluster, GWM=Gateway, TCCM=AWD Control Module.

// ── 10 ms ────────────────────────────────────────────────────────────────────
#define FORD_GEN42_WHEEL_SPEEDS_ID        0x4B0 // DLC=8, ABS. FL/FR/RL/RR wheel speeds. Encoding: (raw − 10000) / 100 km/h; raw=0x2710 at standstill. [WHEEL_SPEEDS / ABS_BrkBst_Data]
#define FORD_GEN42_TORQUE_FLAGS_ID        0x200 // DLC=8, PCM. PrplWhlTot_Tq_Actl D0:D1 (4 Nm/LSB, offset −131072), Tq_LimMn D2:D3, Tq_Rq D4:D5, BrkOnOffSwtch D6 b6:b7. [TorqueDataEngFlags]
#define FORD_GEN42_ENG_SPD_THROTTLE_ID    0x201 // DLC=8, PCM. EngAout_N_Actl D0:D1 (2 rpm/LSB Motorola b7|13), Veh_V_ActlEng D2:D3 (0.01 kph/LSB), ApedPos_Pc D4:D5 (0.1 %/LSB). [EngVehicleSpThrottle_CG1]
#define FORD_GEN42_POWERTRAIN_DATA6_ID    0x205 // DLC=8, PCM. Fuel/torque-model data; bytes 4-5 provisional fuel-related hypothesis. [PowertrainData_6 / PROVISIONAL_0x205]
#define FORD_GEN42_TRANS_GEAR_DATA2_ID    0x231 // DLC=7, TCM. MtrGen1Aout_Tq_Rq b53|14 (0.1 Nm/LSB, −800 Nm offset), TrnMil_D_Rq, EngExhBrkTq_Pc_Rq b23|7 (%). [TransGearData_2]
#define FORD_GEN42_BODY_INFO4_ID          0x240 // DLC=2 (log variant). BCM. Static 0x00 0x40. DBC defines 8-byte superset with EngOff_T_Actl (seconds), AmbTempImpr (0.25 °C/LSB, −128 °C). [Body_Information_4_CG1]

// ── 20 ms ────────────────────────────────────────────────────────────────────
#define FORD_GEN42_ACCEL_BRAKE_STATUS_ID  0x080 // DLC=7, PCM. AccelPedal_Pc b6|10 (0.1 %/LSB, Motorola), BrakePedal_D b4|2. D7=8-bit rolling counter shared with 0x20F D8 and 0x211 D8. [ACCEL_BRAKE_STATUS]
#define FORD_GEN42_ENGINE_DATA_ID         0x090 // DLC=8, PCM. EngAout_N_Actl b35|13 (2 rpm/LSB, Motorola). D1 high nibble = 4-bit rolling counter 0x0-0xF. [ENGINE_DATA]
#define FORD_GEN42_BRAKE_DATA_ID          0x20F // DLC=8, ABS. Provisional brake pressure at D2:D3 (0x0FB0 = full brake). D6 + D7 complement pair summing to 0xC8. D8=shared 8-bit counter.
#define FORD_GEN42_DESIRED_TORQ_BRK_ID    0x211 // DLC=8, ABS. PrplWhlTot_Tq_RqMx D0:D1 (4 Nm/LSB, −131072), RearDiffLck_Tq_RqMx D2:D3, AbsActv_B_Actl b3, VehLongOvrGnd_A_Est D5:D6 (0.035 m/s², −17.9). D8=shared counter. [DesiredTorqBrk_CG1]
#define FORD_GEN42_DRIVE_MODE_ID          0x190 // DLC=8, GWM/PCM. D3=drive-mode (0x20=idle/off, 0x60=power/demand). D6=4-bit nibble counter 0x00-0x0F. D7:D8=provisional steering angle (0x8000=0°). [PROVISIONAL_0x190]

// ── 100 ms ───────────────────────────────────────────────────────────────────
#define FORD_GEN42_WARN_STATUS_ID         0x212 // DLC=8, IPC/ABS. Provisional warning lights: D3/D5 DSC+TCS candidates, D4 b5=brake warn, D4 b3=ABS warn. [PROVISIONAL_0x212]
#define FORD_GEN42_COUNTER_275_ID         0x275 // DLC=3. D1 steps +0x20 per 100 ms (0x80, 0xA0, ... 0x60, 0x80 wrap). Not matched in available DBCs.

// ── 200 ms ───────────────────────────────────────────────────────────────────
#define FORD_GEN42_UNKNOWN_270_ID         0x270 // DLC=8. 200 ms heartbeat. Not matched in available DBCs.
#define FORD_GEN42_UNKNOWN_280_ID         0x280 // DLC=8. All-zero 200 ms heartbeat. Not matched in available DBCs.
#define FORD_GEN42_FOUR_BY_FOUR_SW_ID     0x460 // DLC=8, GWM. 4WD selector switch state. [FourByFourSwitchData_FD1]

// ── 1000 ms ──────────────────────────────────────────────────────────────────
#define FORD_GEN42_PTRAIN_DATA1_ID        0x420 // DLC=8, PCM. CluPdlPos_Pc_Meas D0:D1 (0.1 %/LSB), TrnTotLss_Tq_Est D3 (0.5 Nm/LSB), FuelAlcohol_Pc D7 (0.39 %/LSB). D1 toggles 0x62/0x63. Cluster drive-mode hint (D6:D7): Normal=0x10C4, Sport=0x11CC, Drift=0x12C4. [PowertrainData_1_CG1 / PROVISIONAL_0x420]
#define FORD_GEN42_UNKNOWN_428_ID         0x428 // DLC=4. Possible StrgWheel_PolicePkg (CGEA2011); not confirmed for Focus RS. Static payload FF 7D 6A 31.
#define FORD_GEN42_CLUSTER_INFO_ID        0x430 // DLC=2 (log variant). IPC. Cluster_Information fragment. DBC 8-byte superset includes OdometerMasterValue D2:D4 (km), DrvSlipCtlMde_D_Rq. [Cluster_Information]
#define FORD_GEN42_CLUSTER_INFO3_ID       0x433 // DLC=8, IPC. FuelLvl_Pc_Dsply D0:D1 (0.109 %/LSB, −5.22 offset), hill-descent switch b38, camera config bits D7. [Cluster_Information_3_CG1]
#define FORD_GEN42_UNKNOWN_4F0_ID         0x4F0 // DLC=6. Static slow heartbeat; not matched in available DBCs.
#define FORD_GEN42_UNKNOWN_4F1_ID         0x4F1 // DLC=8. Static slow heartbeat; not matched in available DBCs.
#define FORD_GEN42_VIN_ASCII_ID           0x4F3 // DLC=8. Static ASCII payload "D7U6 9016"; possibly VIN/build-variant fragment. Not matched in available DBCs.

// ── Haldex (TCCM) response — RX-only on Bus0 ─────────────────────────────────
#define FORD_GEN42_AWD_STATUS_ID          0x260 // DLC=8, TCCM. AwdLck_Tq_Actl D0:D1 b15|12 (1 Nm/LSB), AwdRnge_D_Actl D0 b2:b0, AwdStat_D_RqDsply D7 b4:b0, TrnAout_Tq_RqMx D5:D6 b47|13 (1 Nm/LSB, −1250). [Information4x4_CG1]