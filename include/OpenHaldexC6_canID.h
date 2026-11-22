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
#define BRAKES4_ID 0x2A0   // DLC 3 in vw_pq.dbc, but the Gen4 0BR needs at least 7 (OEM code: spec_gen4_0br §1) - we always send 8 [REA audit_gen4_0br #7: RX len 7; audit_gen5_0ay #4: 0AY stores 3]
#define BRAKES5_ID 0x4A8   // DLC 8 x
#define BRAKES6_ID 0x1A8   // DLC 3 x
#define BRAKES8_ID 0x1AC   // DLC 8 x
#define BRAKES9_ID 0x0AE   // DLC 8 [KM - (not in vw_pq.dbc)] [REA audit_gen1_02d #2 / audit_gen2_7h0 F15: not received by 02D / 7H0 / 1K0]
#define BRAKES10_ID 0x3A0  // DLC 8 x
#define BRAKES11_ID 0x5B7  // DLC 8 x

#define GRA_ID 0x38A
#define HALDEX_ID 0x2C0 // [KM Allrad_1, DLC 5 in vw_pq.dbc] [REA audit_gen1_02d #9: 02D sends DLC 2; audit_gen2_7h0 #4: 7H0 DLC 8, 1K0 DLC 4; audit_gen4_0br #6 + audit_gen5_0ay #7: 0BR / 0AY DLC 8]

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

#define mLW_1 0x0C2         // [KM Lenkwinkel_1, DLC 8 in vw_pq.dbc]; the Gen4 0BR needs at least 6 (OEM code: spec_gen4_0br §1, audit #7), the 0AY stores 6 (audit_gen5_0ay #4) - we always send 8
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
// (NMH_Gateway removed V11.00.8: same ID as GATEWAY_ID 0x720 - DLC 7, 200 ms)
#define mKombi_1 0x320
#define mKombi_2 0x420
#define mKombi_3 0x520

// Custom CAN IDs
#define OPENHALDEX_BROADCAST_ID 0x6B0         // broadcast OpenHaldex via. CAN over this address - used for stating mode/performance etc
#define OPENHALDEX_EXTERNAL_CONTROL_ID 0x6A0  // recieve OpenHaldex via. CAN over this address - used for changing modes etc

// OpenHaldex supplier-specific UDS DIDs answered on the Haldex physical address
// (request ISO_ALLRAD_REQ 0x70F, response ISO_ALLRAD_RESP 0x779) so an OBD tester (e.g.
// Rokketek gauge) can read lock/slip and set the drive mode even though the
// car's gateway will not forward the passive 0x6B0 broadcast that far.
#define OPENHALDEX_LOCK_DID 0xFDA0 // RDBI: [cmd%][engagement raw][mode]
#define OPENHALDEX_SLIP_DID 0xFDA1 // RDBI: 4 signed slip bytes [FL, FR, RL, RR]
#define OPENHALDEX_MODE_DID 0xFDA2 // WDBI: [mode] sets the drive mode

#define diagnostics_1_ID 0x764
#define diagnostics_2_ID 0x200
#define diagnostics_3_ID 0x710
#define diagnostics_4_ID 0x71D
// (diagnostics_5_ID 0x70F = ISO_ALLRAD_REQ below - one name since V11.00.8)

// Gen5 0CQ / VAQ tags below: [KM MQB_FCAN_KMatrix_V7.20F name] [REA spec_gen5_0cq §1 RX PDU / audit R6].
#define LWI_01 0x086      // [KM LWI_01] [REA 0CQ RX PDU 0x11, min DLC 4]
#define ESP_14 0x08A      // [KM ESP_14] [REA 0CQ RX PDU 0x10 - THE lock request (b6/b7); VAQ reads b3-b5, 6-byte buffer]
#define MOTOR_11 0x0A7    // [KM Motor_11] [REA 0CQ RX PDU 0x0F]
#define MOTOR_12 0x0A8    // [KM Motor_12] [REA 0CQ RX PDU 0x0E]
#define GETRIEBE_11 0x0AD // [KM Getriebe_11] [REA 0CQ RX PDU 0x0D]
#define GETRIEBE_17 0x0b1
#define MOTOR_04 0x107

#define ESP_19 0x0b2 // [KM ESP_19] [REA 0CQ RX PDU 0x0C]
#define ESP_21 0x0fd // [KM ESP_21] [REA 0CQ RX PDU 0x0B]

#define ESP_02 0x101 // [KM ESP_02] [REA 0CQ RX PDU 0x0A, min DLC 7]
#define EPB_01 0x104 // [KM EPB_01] [REA: not in the 0CQ / VAQ RX table]
#define ESP_05 0x106 // [KM ESP_05] [REA 0CQ RX PDU 0x09]

#define ESP_10 0x116   // [KM ESP_10 (wheel pulses, not yaw)] [REA: not in the 0CQ / VAQ RX table]
#define MOTOR_20 0x121 // [KM Motor_20] [REA 0CQ RX PDU 0x08, min DLC 3]
#define KLEMMEN_STATUS_01 0x3C0 // MQB ignition status (ZAS_Kl_15)
#define ESP_18 0x135 // [KM ESP_18] [REA: not in the 0CQ / VAQ RX table]

#define ESP_29 0x18c   // [KM ESP_29] [REA: not in the 0CQ / VAQ RX table]
#define KOMBI_01 0x30b // [KM Kombi_01] [REA 0CQ RX PDU 0x07, min DLC 3]

#define CHARISMA_01 0x385 // MQB drive mode select: the gateway's requested mode for each system (engine, gearbox, Haldex ...), 1 s [REA: VAQ RX only (audit_gen5_0cq §4), not 0CQ]
#define CHARISMA_07 0x3E8 // MQB drive mode select: byte 1 bits 0-3 = the drive profile the driver picked (CHA_Current_Mode), 1 s
#define ESP_07 0x392 // [KM ESP_07] [REA 0CQ RX PDU 0x06 (b7.6 only)]

#define MOTOR_14 0x3be    // [KM Motor_14] [REA 0CQ RX PDU 0x05, min DLC 5]
#define GETRIEBE_14 0x3c8 // [KM Getriebe_14] [REA: not in the 0CQ / VAQ RX table]
#define GATEWAY_72 0x3db  // [KM Gateway_72] [REA: not in the 0CQ / VAQ RX table]
#define BLINKMODI_02 0x366 // MQB hazard / turn-signal status broadcast

#define Parkhilfe_04 0x54b  // [KM Parkhilfe_04] [REA: not in the 0CQ / VAQ RX table]
#define SYSTEMINFO_01 0x585 // [KM Systeminfo_01] [REA 0CQ RX PDU 0x04, min DLC 2]
#define ESP_23 0x5be        // [KM ESP_23] [REA: not in the 0CQ / VAQ RX table]

#define MOTOR_07 0x640      // [KM Motor_07] [REA: VAQ RX only (audit_gen5_0cq §4), not 0CQ]
#define MOTOR_CODE_01 0x641 // [KM Motor_Code_01] [REA 0CQ RX PDU 0x03]
#define unknown2 0x64a

#define ESP_20 0x65d      // [KM ESP_20] [REA 0CQ RX PDU 0x02]
#define DIAGNOSE_01 0x6b2 // [KM Diagnose_01] [REA 0CQ RX PDU 0x01]
#define KOMBI_02 0x6b7    // [KM Kombi_02] [REA 0CQ RX PDU 0x00, min DLC 5]

#define HALDEX_ID_GEN5 0x118 // [KM Allrad_03, DLC 4, TX Allrad_Haldex] [REA audit_gen5_0cq A11/R3: TX HTH11 DLC 4, buffer 0xCB56, builder FUN_c449ea; 0CQ only - the VAQ never sends it]
#define HALDEX_GEN5_STATUS_ID 0x17F0000F // 29-bit ID, sent by the 0CQ (DLC 8): byte 0 bit 0 = fault reaction active, byte 1 = 0x0F (OEM code: spec_gen5_0cq §6) [KM KN_Allrad: b0.0 Allrad_KompSchutz, b1 Allrad_SNI, b7.7 ALR_KD_Fehler] [REA audit S18: b0.0 = 1 iff 0xCC10 in {2,3,4}; VAQ sends 0x17F0001E (KN_Quersperre) instead]
#define HALDEX_GEN5_STATUS_TIMEOUT_MS 3000 // forget the 0x17F0000F fault bit if the frame stops (seen about twice a second in our logs)

// Gen5 VAQ (MQB front-axle transverse lock, K-matrix node "VAQ" / "Quersperre").
// Source: MQB_FCAN_KMatrix (Documents gitignore/can-databases/vw-mqb). The VAQ
// node receives 22 of the 25 periodic frames the Haldex does (ESP_10 is the one
// periodic frame it does NOT list), reads ESP_14 bytes 4/5 (BR_Vorg_Quer_Min/Max)
// where the Haldex reads bytes 6/7 (BR_Vorg_Allrad_*), and answers diagnostics on
// its own ISO-TP pair. Its feedback frame is the Quersperre_03 counterpart of the
// Haldex's Allrad_03 (0x118). Which of these the physical unit really uses is
// confirmed on the bench with the serial lab's RXIDS / DTC commands.
// REA (audit_gen5_0cq §4, VAQ 0CQ907554A/E code): the real VAQ RX table is the
// 0CQ's 18 operational IDs + 0x640 Motor_07 + 0x385 Charisma_01 (not the 22
// K-matrix frames above - ESP_18/ESP_23/Parkhilfe_04/Gateway_72/Getriebe_14/
// ESP_29/EPB_01 are not received); diag 0x71E / ext 0x17FC041E; TX 0x788,
// 0x137 (DLC 4, buffer 0xCB9C), ext 0x17F0001E, 0x17FE041E, 0x7FF.
#define QUERSPERRE_03 0x137       // TX by Quersperre/VAQ, DLC 4, 100 ms: QUER_Sta_Quersperre b1[4..6], QUER_Gleichlauf b1[7], QUER_Ist_Proz b2 (0.4 %/bit), Charisma b3 [KM Quersperre_03] [REA audit §4: builder FUN_c1fb42; b2 scale in the missing cal block - 0.4 %/bit is KM/bench only]
#define QUERSPERRE_04 0x3FF       // TX by Quersperre/VAQ, DLC 8, 500 ms: cluster display bits (lock engage/disengage/closed) b1[4..6] [REA audit §4: 0x3FF is NOT in the VAQ TX table]
#define ISO_QUERSPERRE_REQ  0x71E // UDS physical request to the VAQ / Quersperre ECU (K-matrix ISO_Quersperre_Req) [REA audit §4: VAQ diag RX 0x71E]
#define ISO_QUERSPERRE_RESP 0x788 // UDS response from the VAQ / Quersperre ECU  (K-matrix ISO_Quersperre_Resp) [REA audit §4: VAQ TX 0x788]
#define ISO_ALLRAD_REQ      0x70F // UDS physical request to the Haldex (Allrad) ECU (also the 5th passthrough diag ID) [REA spec_gen5_0cq §1: 0CQ RX PDU 0x16]
#define ISO_ALLRAD_RESP     0x779 // UDS response from the Haldex (Allrad) ECU (also what our own DID answers go out as) [REA spec §1: 0CQ TX]


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

// ─── Generation 45: Ford Kuga Mk1 AWD module (V11.00.9) ──────────────────────
// Ford C1 platform (= Focus Mk2), 500 kbit/s, 11-bit, Motorola byte order. Receive set = the unit's
// own CAN tables (firmware 8V41-4C045-AC, documents/haldex-binaries/leaf_trace/trackF_kuga_8v41.md),
// bytes / periods = a real 2007 Focus Mk2 idling (tools/ford_kuga_kwp/c1_traces/parked.trc).
// The unit reads nothing else (0x190 / 0x200 / 0x20F / 0x212 / 0x240 / ... are never received).
//   [REA audit_ford_kuga #2/#3/#76/#95: 8V41 also receives 0x046 (temperature model only, optional) and 0x6F6
//    (CCP calibration command channel, answers on 0x6F2-0x6F9 - IS read, trackF/H said never); 9V4N adds
//    0x190 (b2 bit 4 read by its normal strategy) and 0x428 (never read). None of these is presence-checked.]
// Needed (bench drop test): missing ABS frames -> U0121, missing PCM frames -> U0100.
// KM: Ford C1 has no KMatrix; per-byte DBC / REA tags are in StandaloneCAN.cpp above KUGA_080_DATA.
#define FORD_KUGA_ACCEL_ID      0x080 // 20 ms, DLC 7, ABS group. b0-1 dynamics signal (16 bit); b6 = 8-bit counter
                                      //   [KM RS ACCEL_BRAKE_STATUS (overlap only)] [REA #25: b0-1 F1030; copies 6 bytes - b6 never read]
#define FORD_KUGA_DYNAMICS_ID   0x090 // 10 ms, DLC 8, ABS group. b0[3:0]:b1 and b4[3:0]:b5 dynamics signals; b0 bits 4-7 = counter
                                      //   [KM -] [REA #25: F102C / F105C; counter not read]
#define FORD_KUGA_ENGINE_ID     0x201 // 10 ms, DLC 8, PCM. b0-1 rpm (1 rpm/bit, base demand > ~401), b4-5 road speed 0.01 km/h (not read),
                                      //   b6 pedal 0.5 % (> 5 enables the pump demand)
                                      //   [KM - (CGEA EngVehicleSpThrottle_CG1 layout differs)] [REA #18/#19/#41: rpm is 15-bit b0[6:0]:b1]
#define FORD_KUGA_TORQUE_ID     0x205 // 10 ms, DLC 8, PCM. w0 b0[4:0]:b1 / w3 b6[4:0]:b7 engine torque (1 Nm/bit, +512) = the torque
                                      //   demand input; b2 bit 6 = brake switch (forces a reduced mode); w1 / w2 not read
                                      //   [KM - (VehicleCAN PowertrainData_6 differs)] [REA #20 / audit C, FI-9: b2 bit 6 = mode 2 +
                                      //    preload off - "brake switch" is INF (Focus RS log), it reduces, never opens]
#define FORD_KUGA_ABS_ID        0x211 // 10 ms, DLC 8, ABS. b6 bits 1-2 ABS / ESP request: 0 normal, 1 open the clutch, 2 ramp down,
                                      //   3 torque ceiling = b2[3:0]:b3 x 8 = OUR LOCK REQUEST (V11.00.11); b7 = 8-bit counter. Missing -> control OFF
                                      //   [KM - (CGEA DesiredTorqBrk_CG1 layout differs)] [REA #21/#22: copies 7 - b7 never read;
                                      //    #24: b5 bit 0 no reader in code, but released the clutch on the bench]
#define FORD_KUGA_GEAR_ID       0x231 // 10 ms, DLC 7, PCM. Presence only [KM CGEA TransGearData_2 - C1 unconfirmed] [REA: b0[7:4] presence]
#define FORD_KUGA_HANDBRAKE_ID  0x433 // 100 ms, DLC 8, cluster. b3 bit 0 = handbrake (forces a reduced mode); bytes 0-2 not read
                                      //   [KM -] [REA #26/#35: copies 4; b3.0 -> mode 5 latch (8V41) / mode 4 alive-gated (9V4N);
                                      //    9V4N also reads b3 bit 1 (thermal-limit ramp, trackH)]
#define FORD_KUGA_WHEELS_ID     0x4B0 // 10 ms, DLC 8, ABS. FL FR RL RR 16-bit, (raw - 10000) / 100 km/h
                                      //   [KM RS WHEEL_SPEEDS.Wheel_Speed_1..4 0.01 km/h -100] [REA #17: abs value - direction not read]
#define FORD_KUGA_CONFIG_4F0_ID 0x4F0 // 1000 ms, DLC 6, PCM. Presence only [KM -] [REA: 8V41 presence; 9V4N b1/b2 pick a cal set - ours = set 0]
#define FORD_KUGA_CONFIG_4F1_ID 0x4F1 // 500 ms, DLC 8, PCM. Presence only [KM -] [REA: 8V41 presence; 9V4N b0[3:0] picks a cal set - ours = set 0]
#define FORD_KUGA_OVERRIDE_ID   0x6F1 // the unit's test / EOL channel (V11.00.9 / 10 lock request; since V11.00.11 only a one-shot release):
                                      //   b0 0x01 + b1-2 setpoint override (skips the pump demand -> pump on its standby floor);
                                      //   b0 bits 2-5 = 10 / 11 switch the 0x6F2-0x6F9 service stream on / off. LATCHED: send 00s to release
                                      //   [KM -] [REA #28/#78: 8V41 only - b0 bits 0+1 = F106F 2 = TORQUE override b3-4; 9V4N #66/#93:
                                      //    different protocol (cmd = b0[5:2]), alive-checked, auto-release after ~100 cycles]
#define FORD_KUGA_HALDEX_ID     0x260 // unit -> us, 20 ms. b1 bits 6-7 status (00 active, 40 ABS release, 80 no demand / override / fault,
                                      //   C0 monitor fault); b1 bit 2 = ABS ceiling active; b2[3:0]:b3 = clutch torque from measured
                                      //   pressure / 8; b4-b7 hard-coded 0F FF 0F FF (never data)
                                      //   [KM - (CGEA Information4x4_CG1.AwdLck_Tq_Actl = b1:b2[7:4] - differs)]
                                      //   [REA #9-#13/#39/#61: 80 also oil > 106 C; b1 bit 2 8V41 only; b2:b3 8V41 = +2 once latched,
                                      //    full scale moves with oil temp, 0x0FF = sensor fault; 9V4N = request echo, b4-b7 never written]
#define FORD_KUGA_DIAG_REQ      0x761 // KWP2000: 18 00 FF 00 read faults, 14 FF 00 clear; sessions 10 81 only for normal running
#define FORD_KUGA_DIAG_RESP     0x769

// ─── Generation 46: Land Rover Gen4 Haldex - Freelander 2 + Evoque (V11.00.13) ────────
// One generation for both controllers (FL2 BH52-4C045-AC, Evoque BJ32-4C045-AC): we send the UNION of both
// receive sets and decode whichever feedback arrives. All 11-bit, DLC 8, Motorola, no checksum / counter on any
// RX signal [REA audit_landrover #5/#41]. No Land Rover KMatrix here and no Ford DBC matches these layouts, so
// every signal is [KM -]. UNTESTED (no car log): IDs + bit fields from the binary trace (audit_landrover.md
// over trackG_landrover.md); periods are not known (the COM table's per-signal number 6/8/12/100 is
// unverified - audit #17), so ours are [GUESS - bench].
// Shared IDs (read by both units, different bits - see the builder):
#define LR_TERRAIN_ID     0x083 // FL2 b5 bits 0-2 / EVQ b5 bits 5-7 = terrain / drive mode + 1 -> F1547 [REA #32; INF meaning]
#define LR_143_ID         0x143 // FL2 b2[1:0]:b3 (raw-512) 'E'-variant torque path [REA #26]; EVQ b3 bit 7 = inhibit (F158E),
                                //   b3[1:0]:b4 = FL2's 0x315 role [REA audit E5]
#define LR_150_ID         0x150 // FL2 b0 bit 0 (F158F), b1 bit 2 (F1588) [REA #30]; EVQ wheel speeds 3/4 b3[6:0]:b4, b6[6:0]:b7,
                                //   b3 bit 7 (F158F), quality b5 bits 0-1 / 3-4 [REA audit E5/E9]
#define LR_REQUEST_20A_ID   0x20A // THE LOCK REQUEST (both). FL2: b6 bits 5-7 ABS request field, b2 bits 4-6 qualifier, b6[4:0]:b7 torque;
                                //   EVQ: b4[4:0]:b5 torque, b2 bits 4-5 quality, b2[3:0]:b3 dynamics [REA #29, E6]
#define LR_REQUEST_2B5_ID   0x2B5 // FL2 wheel speeds b6[6:0]:b7, b4[6:0]:b5 + quality b2 bits 6-7 / b0 bits 2-3 [REA #19/#20];
                                //   EVQ: b1 bits 2-4 = ABS request field (lock request), wheel speeds b3[6:0]:b4, b5[6:0]:b6, quality b7 [REA E6]
#define LR_REQUEST_250_ID   0x250 // FL2 wheel speeds b6[6:0]:b7, b4[6:0]:b5 + quality b2 bits 5-6 / b0 bits 0-1 [REA #19/#20];
                                //   EVQ: b0 bits 0-2 = field-4 qualifier (lock request), b4[1:0]:b5 dynamics, quality b6 bits 6-7 [REA E6]
#define LR_3EE_ID         0x3EE // FL2 b1 bit 7 (F15A3 -> mode 4), b1 bits 3-5 mode request [REA #37]; EVQ b5:b6 16-bit (FL2 0x39B role)
// Freelander 2 only:
#define LR_FL2_TORQUE_ID  0x125 // b4[3:0]:b5 torque (offset 2047), b2[1:0]:b3 pedal 0.1 %, quality b4 bits 6-7 / b6 bits 5-6 [REA #23/#25/#26]
#define LR_FL2_ENGINE_ID  0x12A // b6[4:0]:b7 rpm (1 rpm/bit, DID F40C; gate > 401), b4[3:0]:b5 torque, b0 bit 2 (F1554), quality b6 bits 5-6 [REA #21-#24]
#define LR_FL2_DYN_ID     0x155 // b2[1:0]:b3 F1596, b6[3:0]:b7 F15A6, quality b6 bits 4-7 [REA #27]
#define LR_FL2_POWER_ID   0x160 // b1[1:0] = 3 + b5[6:2] = power mode; b1 bit 4 -> F1547 = -1; b0 bit 6 = 0x405 temperature table on [REA #31-#33]
#define LR_FL2_315_ID     0x315 // b3[1:0]:b4 x 25 -> F157C (speed-like hysteresis) [REA #34]
#define LR_FL2_INHIBIT_ID 0x378 // b7 bit 3 -> F158E: blocks the base demand while set + 70 cycles [REA #35]
#define LR_FL2_39B_ID     0x39B // b6:b7 -> F1578 = (raw-1) x 256 / 10, F1532 = raw != 0 [REA #36]
#define LR_FL2_HALDEX_ID  0x310 // unit -> us (FL2): b6[4:0]:b7 = commanded torque F2612 >> 4, b6 bits 5-7 status [REA #43-#46]
// Evoque only:
#define LR_EVQ_ENGINE_ID  0x100 // b4[4:0]:b5 rpm, b0 bit 4 (F1554), quality b2 bits 6-7 [REA E5]
#define LR_EVQ_POWER_ID   0x101 // b1[1:0] = 3 + b5[6:2] = power mode [REA E5]
#define LR_EVQ_TORQUE_ID  0x115 // = FL2 0x125 layout (torque b4[3:0]:b5, pedal b2[1:0]:b3, quality b4 / b6) [REA E5]
#define LR_EVQ_116_ID     0x116 // b2[1:0]:b3 (FL2 0x143 role), b6[3:0]:b7 (FL2 0x12A torque role) [REA E5; role INF]
#define LR_EVQ_34F_ID     0x34F // b7 bits 1-3 = mode request echoed in 0x301 b5 [REA E5]
#define LR_EVQ_369_ID     0x369 // b5[3:0]: 6 / 10 = ramp request that BEATS the lock request, 0xC -> F15C4, 3 -> F160D; b0 bit 6 [REA E6/E7]
#define LR_EVQ_496_ID     0x496 // b2 bit 6 only [trackG Evoque; REA: partly read]
#define LR_EVQ_HALDEX_ID  0x301 // unit -> us (Evoque): same layout as FL2 0x310 [REA audit E3/E4]
// CAN2 of the unit (both): vehicle configuration + dynamics
#define LR_094_ID         0x094 // FL2 b6[6:0]:b7 F155E (sign b0 bit 6); EVQ b6:b7 (sign b5 bit 5), b3:b4 [REA #28, E5] - in the normal config D
#define LR_CCF_400_ID     0x400 // CCF mux (b0 = page): pages 1 / 2 / 9 / 0x13 -> FUN_02f53c.. [REA #42; content INF]
#define LR_CCF_405_ID     0x405 // CCF mux: page 5 b6:b7 = rolling circumference -> radius F1582; page 2 b2 = temperature + 40 [REA #38/#39]
// Diagnostics (UDS on ISO-TP, default session, no security - dtc_check_ford_lr.md): request on the unit's
// CAN2 controller, reply from its CAN1 controller (one physical bus not proven - audit #13)
#define LR_DIAG_REQ       0x795 // 19 02 <mask> read, 14 FF FF FF clear
#define LR_DIAG_RESP      0x79D

// ─── Volvo / SAAB (V10.00.0)─────────────────────────────────────────────────
// Starting point only: IDs, rates and lock bytes. The Volvo network heartbeat
// matches a real Volvo PCAN trace.
// Byte meanings are NOT confirmed on our own cars yet - adjust with the serial lab
// (OVR) / SavvyCAN, then replace with decoded values once logs are in.
// Volvo uses 29-bit (extended) IDs on P2; send them with frame.extd = 1.

// Volvo P2 Gen2 (pre-facelift) - 250 kbit/s, 29-bit. Haldex replies on 0x0120000A.
// V11.00.13 comment pass: NO binary backing for anything in gen 21 - none of these IDs is in the S60R,
// DEM.bin or XC166 dumps (audit_volvo_c166 #86) and no KMatrix covers the MY2003 250k set (the Volvo DBCs
// held are 11-bit P3 / V40 sets). Every tag here is [KM -][REA none]; meanings are from logs only
// ([LOG] = documents/VOLVO_GEN2_VS_GEN4_LOGS_2026-11-11.md, 2003 V70 D5 FWD + 2003 XC70 CEM list).
#define VOLVO_P2G2_224024_ID  0x00224024 // 10 ms, static [KM -][REA none]
#define VOLVO_P2G2_LOCK_ID    0x0042401E // 10 ms, byte 2 = 0x20 (0x21 at full lock), byte 3 = "lock request" [KM -][REA none]; [LOG]: b2[4:0]:b3 = ENGINE RPM 1 rpm/bit (970 at idle), so we really send 0-244 rpm (500 at >= 95 %)
#define VOLVO_P2G2_ACTIVE_ID  0x0062401E // 10 ms, bytes 6-7 = 60 00 idle / 67 D0 lock active [KM -][REA none]; [LOG]: layout does NOT match gen 3's 0x0062401E
#define VOLVO_P2G2_C24000_ID  0x00C24000 // 10 ms, static [KM -][REA none]; [LOG]: four 14-bit wheel speeds + 2-bit qualifiers (same layout as 0x10004002)
#define VOLVO_P2G2_1E2402C_ID 0x01E2402C // 20 ms, static [KM -][REA none]; [LOG]: immobiliser response CEM -> ECM
#define VOLVO_P2G2_NETWORK_ID 0x0242C07C // 100 ms, network heartbeat (bytes 6-7 FE F8 <-> 01 07) [KM -][REA none]; [LOG]: CEM pages like 0x10A2407C (incl. 0x14)
#define VOLVO_P2G2_3200002_ID 0x03200002 // 100 ms, static [KM -][REA none]
#define VOLVO_P2G2_3400002_ID 0x03400002 // 100 ms, static [KM -][REA none]
#define VOLVO_P2G2_HALDEX_ID  0x0120000A // from the Haldex: byte 6 = status (scaling unknown) [KM -][REA none; ID from the 2003 XC70 CEM receive list]

// Volvo P2 Gen3 (facelift) - 500 kbit/s, 29-bit. Haldex replies on 0x1020000A.
// V11.00.0: all of these IDs are confirmed in the S60R DEM's own message table
// (OEM code: spec_volvo_p2_s60r.md section 1). "500k S60 log" below = the public
// log volvo-CAN-exploration/0000286.TXT (500 kbit/s, model year not stated, it
// has these facelift IDs). The 2002/2003 P2 logs are pre-facelift and use other IDs.
// V11.00.13 tags: [KM -] on every gen 3 ID (no KMatrix: the Volvo DBCs held are 11-bit P3 / V40 sets).
// [REA sNN] = S60R DEM signal-table entry 0xNN (audit_volvo_c166, scripts a03_lanes.txt): "ctl" = used for
// control, "mon" = only in the signal-error (monitoring) table @0x18000, "nr" = defined, referenced nowhere.
#define VOLVO_P2G3_ACTIVE_ID  0x10004002 // 10 ms, bytes 4-7 = C0 00 C0 00 idle / C1 42 C1 42 active [KM -][REA s21/s20/s1D/s1C ctl: FOUR 14-bit wheel speeds b0-1/b2-3/b4-5/b6-7 (not 10-bit, #77); s05 b2[7:6] mode ctl; s1F/s1E b4/b6[7:6] qualifiers mon; b0[7:6] no signal]
#define VOLVO_P2G3_42406C_ID  0x0042406C // 10 ms, byte 0 bits 6-7 = 2-bit rolling counter (500k S60 log) [KM -][REA #47: the DEM DROPS a frame whose byte 0 repeats - counter required]
#define VOLVO_P2G3_LOCK_ID    0x0062401E // 10 ms, byte 1 = our lock request byte, bytes 0/4-7 = active flags [KM -][REA s09 ctl: b0[1:0]:b1 = 10-bit ENGINE TORQUE, C030 = 4 x raw - 2048 - NOT a lock request (#30)]
#define VOLVO_P2G3_MODE_ID    0x0080401E // 10 ms, bytes 2-3 = 80 00 idle / 87 D0 active, byte 0 toggles 1E/06 [KM -][REA s07 ctl: b2[4:0]:b3 = 13-bit ENGINE SPEED 1 rpm/bit, ready gate ~401 rpm (#34)]
#define VOLVO_P2G3_224024_ID  0x00224024 // 10 ms, byte 0 bit 2 toggles every send (500k S60 log, 14 ms there) [KM -][REA s00 ctl: b6 bit 2 only; b0 toggle not read]
#define VOLVO_P2G3_E24026_ID  0x00E24026 // 10 ms, static (14 ms in the 500k S60 log) [KM -][REA s02 ctl: b4 bit 5 only (blocks the brake vote C02A when set)]
#define VOLVO_P2G3_NETWORK_ID 0x10A2407C // 100 ms, network heartbeat (bytes 6-7 FE F5 <-> 01 0A) [KM -][REA s0C 8-byte block: page 1 b1 bit 4 + b2-b5 read (#48), b6-7 heartbeat read on no path found; page 5 b6-7 = axle-speed divisors; pages 0x10 b1 / 0x14 b7 = calibration set (#46)]
#define VOLVO_P2G3_HALDEX_ID  0x1020000A // from the Haldex: byte 6 = measured clutch value, byte 7 = status bits [KM -][REA #49-#59: b6 = C01E >> 6; b7 s2C/s2A/s2B; b5 bits 0-2 = update bits; b0-b4 never written]
// V11.00.0: four more frames the DEM listens for (OEM code: MO table) that we
// never sent. Content = steady values from the 500k S60 log, meanings unknown.
#define VOLVO_P2G3_C0402A_ID  0x00C0402A // 10 ms in the log, steady 1E E4 00 00 27 FF D3 FF [KM -][REA s19 b3 mon only; s1A b0 bits 5-6 nr (#67)]
#define VOLVO_P2G3_10E24000_ID 0x10E24000 // 56 ms in the log (we send every 50 ms), all zero [KM -][REA s24/s22/s0F (+s23/s10) mon only - no control use (#69)]
#define VOLVO_P2G3_11220028_ID 0x11220028 // 60 ms in the log (we send every 50 ms), 23 60 0F 3C 01 00 35 2E [KM -][REA s0B b3[1:0]:b4 nr - not read (#70)]
#define VOLVO_P2G3_11420006_ID 0x11420006 // 120 ms in the log (we send every 100 ms), 00 00 00 0F 00 00 C0 00 [KM -][REA s11 b6[1:0]:b7 / s12 b6 bits 2-3 nr - not read (#70)]
// 0x01A2402A is also in the DEM's table but is NOT on the bus in the 500k S60
// log, so its content is unknown and it is not sent (V11 TODO (bench)).
#define VOLVO_P2G3_1A2402A_ID 0x01A2402A // not sent - content unknown [KM -][REA s15 b6[6:0]:b7 15-bit, s16 b6 bit 7, s17 b2 bits 1-2 (mon); the only masked MO (#15)]

// Volvo Gen4 (P3) - 500 kbit/s, 29-bit. V11.00.5 from the 2026-11-07 bench session
// on a real unit (31219215 AA / software 31219186 AE,
// documents/VOLVO_GEN4_BENCH_2026-11-07.md). It uses the P2 facelift DEM's message
// set (the VOLVO_P2G3_* frames above), NOT the 11-bit V60 DBC frames (0x12A /
// 0x20A / 0x220 / 0x405, reply 0x243) that gen 43 sent before - those got no
// answer at all. No public DBC covers this 500k set (the 2002 S60 / 2003 V70 DBCs
// are the 250k pre-facelift set), so each frame below is decoded from three
// sources, marked:
//   [OEM]   which bits the S60R DEM code actually reads (leaf_trace/spec_volvo_p2_s60r.md,
//           volvo_bitrate.md "RX byte lanes"; re-checked in audit_volvo_c166.md - "sNN" = its
//           signal-table entry). That is the P2 facelift unit, NOT the P3: there is NO P3 dump,
//           so for gen 43 only the field LAYOUTS and the D2 IDs have binary backing. Every
//           threshold / scale / meaning below that is not tagged [OEM] is [no OEM code].
//   [KM -]  for every gen 43 frame: no KMatrix covers this 29-bit set (opendbc
//           volvo_v60_2015_pt / volvo_v40_2017_pt are 11-bit, no ID matches)
//   [LOG]   how the field behaves in the 500k S60 drive log (vehicle-logs-public/
//           volvo-CAN-exploration/0000286.TXT: idle, brake on/off at stops, a
//           pull-away to ~40 km/h, one turn, one wheelspin with traction control)
//   [BENCH] what this unit did when we changed it (2026-11-07)
// LOCK = the field changes the clutch. Frames marked "not needed" made no fault and
// no lock change when switched off on the bench (standalone does not send them).
//
// 0x10004002 (10-14 ms, from the ABS / DSTC module) - LOCK
//   b0[5:0]:b1  REAR RIGHT    14-bit, 1/64 km/h (0.0156)
//   b2[5:0]:b3  REAR LEFT     "
//   b4[5:0]:b5  FRONT RIGHT   "
//   b6[5:0]:b7  FRONT LEFT    "
//     Corners + scale: the AWD module's live data (D2 A6 00 06) returns these four
//     values unchanged [BENCH], and vtl/volvo-ddd (2005 XC70) reads that reply as
//     FR / FL / RR / RL x 0.0156. Cross-checked in the S60 log against 0x11600002 /
//     0x11800002 (same speeds in 0.01 km/h, within 0.5 %), and against 0x10800006's
//     four pulse counters (same order). [WEB][BENCH][LOG]
//     [OEM s21/s20/s1D/s1C: four 14-bit fields; each pair averaged, own divisor from CEM page 5;
//     which pair is front is NOT in the code - front = b4-7 is [LOG][BENCH] (#36)]
//   b0[7:6]     RR qualifier, car sends 2 - small effect only [BENCH] [OEM: no signal on these bits]
//   b2[7:6]     RL qualifier = ABS / STABILITY-CONTROL BRAKING: 1 or 2 -> the unit opens
//               the clutch at once, moving or stood still; 0 (car, normal) and 3 = no
//               reaction [BENCH]. The DEM code reads it as a "mode" input [OEM s05: 1/2 ->
//               C026 = 1 -> strategy mode 4, feedback b7[7:6] = 1 (#38)].
//               Which of 1 / 2 is ABS and which is DSTC isn't known (no ABS event in any log).
//   b4[7:6], b6[7:6]  front qualifiers, car sends 3 = valid. Anything else: the front
//               speeds are ignored and fault 0x0012 is stored [BENCH] [OEM s1F/s1E: in the
//               signal-error table with parameter 3 only - monitored, no control use (#37)]
//   [BENCH] lock request: all four 0 = stood still -> full pull-away pre-lock (0xFF). Equal
//   speeds lower the pre-load (600 = 9 km/h -> FF, 1200 = 19 -> 91, 2000 = 31 -> 57,
//   4000 = 62 km/h -> 36). Fronts faster than rears by 150..600 at 4000 -> pressure
//   0x3A..0xEF; over ~650 (10 km/h of slip) the unit opens the clutch.
#define VOLVO_G4_WHEELS_ID    0x10004002
// 0x0080401E (10 ms, from the engine ECU) - LOCK
//   b0          1E / 06 alternating (alive toggle) [LOG] [OEM: no signal]
//   b1          small signed value around 0, rises with throttle (torque change?) [LOG] [OEM: no signal]
//   b2[4:0]:b3  ENGINE RPM, 13-bit, 1 rpm/bit (log idle 650-870, r = 1.00 with the
//               tacho trace) [OEM s07 -> C024 = rad/s x 64, #33/#34][LOG]
//   b2 bit 7    a 1 Hz pulse [LOG] [OEM: no signal]
//   b5          slow-moving value (2B -> 24 over the log, a temperature?) [LOG] [OEM: no signal]
//   b6 bit 3    BRAKE SWITCH (set exactly while the log's brake pressure is > 0) [LOG]
//               [OEM s03 ctl: brake vote C02A = 1 while set and 0x00E24026 b4 bit 5 clear (#41)]
//   b6 bits 4-5 / 6-7  [OEM s08 / s04: signal-error table only (mon)]
//   [BENCH] rpm under ~300 = pump off (0 %); 300 and up = engine running, lock allowed
//   [no OEM code for the P3 gate; the S60R's "ready" ramp needs > ~401 rpm, drops < ~200 (#56)]
#define VOLVO_G4_ENGINE_ID    0x0080401E
// 0x0062401E (10 ms, from the engine ECU) - LOCK
//   b0[1:0]:b1  ENGINE TORQUE, 10-bit, ~512 = 0 Nm (log 502-629, r = 0.88 with rpm) [OEM s09:
//               C030 = 4 x raw - 2048, the torque feed-forward (#27)][LOG]
//   b4[1:0]:b5  second engine torque (requested / indicated?), 10-bit, same scale [OEM s06: C016 = 4 x raw - 2048][LOG]
//   b4 bit 4    BRAKE SWITCH (ECM copy, same windows as 0x0080401E b6 bit 3) [LOG] [OEM: no signal - the S60R does not read it]
//   b6[1:0]:b7  ACCELERATOR PEDAL, 10-bit, 0 = foot off, 223 seen on a hard pull [OEM s01:
//               C038 = (2 x raw + 5) / 10, compared with 10 / 40 -> 0.2 %/bit if that unit is % (#64)][LOG]
//   b6 bits 2-3 [OEM s0A: signal-error table only (mon)]
//   b2          bit 5 toggles once a second, low bits 3 / 4 [LOG]; b3 moves, unknown [OEM: no signal on b2 / b3]
//   [BENCH] positive torque / pedal pull the lock DOWN and stored fault 0x0012; with
//   the frame missing lock drops ~40 %. Held at the measured values (see StandaloneCAN).
#define VOLVO_G4_TORQUE_ID    0x0062401E
// 0x10A2407C (100 ms, from the CEM) - NEEDED (page 0x14)
//   byte 0 = page. Page 1 = heartbeat (b6-7 swap to their exact opposite each send).
//   The car also sends pages 02, 05 (b6-7 = a ratio the DEM reads [OEM]), 0x10-0x23.
//   [OEM s0C, #44/#46/#48 - partly: page 5 b6-7 = the two axle-speed divisors; page 1 b1 bit 4
//   + b2-b5 and page 2 read by the network thread, page 1 b6-7 read on no path found; page 0x10 b1
//   and page 0x14 b7 pick one of 6 calibration sets - the log car's 0x59 = set 0, the default]
//   [BENCH][no OEM code for the fault] page 0x14 (configuration) byte 7 = 0x59 must arrive, or fault 0x0015 is
//   stored ~32 s after power-up and the unit goes into limp. All-zero is refused.
//   Once received, the unit remembers it until it is powered off.
#define VOLVO_G4_NETWORK_ID   0x10A2407C
// 0x00224024 (14 ms, from the ABS / DSTC module) - not needed
//   b0 bit 2    alive toggle [LOG]
//   b2          BRAKE PRESSURE (~70 held at a stop, 0 off the pedal, 20-53 slowing) [LOG]
//   b3          0-27, only while moving, unknown [LOG]
//   b4 bit 7    TRACTION CONTROL ACTIVE (set in the log's wheelspin) [LOG];
//   b4 bit 3    set once the car first moves [LOG]; b6 bit 2 read by the DEM [OEM s00 ctl -> C027,
//               mode logic (#66)], 0 in the log. Every other bit: no signal [OEM]
// 0x0042406C (10 ms, from the gearbox TCM) - not needed
//   b0 bits 6-7 2-bit rolling counter; b1 changes every frame (counter / check);
//   b2 bits 6-7 slower counter; b3 bit 7 = 0.1 s pulse at each gear-lock request change [LOG]
//   [OEM #47: the S60R RX DROPS a frame whose byte 0 repeats -> keep the counter. Signals: b2 bit 2
//   s1B ctl (C018 latch), b4 bit 7 s13 ctl (C00C), b3 bit 7 s14 / b4 bits 5-6 s0E / b6 bits 5-7 s0D mon]
// 0x00C0402A (10 ms, from the gearbox TCM) - not needed
//   b4 / b6     GEAR LOCK REQUEST POSITION: 27/D3 = drive, 07/C7 = the other position used at
//               stops, 3F/1F/CB/CF while moving the lock request [LOG]; b3 = [OEM s19: monitored for
//               errors only, no control use]; b0[6:5] = [OEM s1A: defined, referenced nowhere] (#67), 0
// 0x00E24026 (14 ms) - not needed
//   b2 bit 7    alive toggle; b5 = DISTANCE COUNTER (counts up only while moving) [LOG]; b4 bit 5
//               [OEM s02 ctl: set = blocks the brake vote C02A (#41)], 0
// 0x10E24000 (56 ms) - not needed
//   Three analog fields b2[3:0]:b3, b4[3:0]:b5 (12-bit), b6[1:0]:b7 (10-bit) + b6 bits 2-5
//   [OEM s24/s22/s0F/s23/s10: signal-error table only - monitored, NOT read for control (#69)]. All zero
//   for the whole log, turn included - most likely the DSTC sensor cluster (STEERING
//   ANGLE / yaw / lateral g), absent on that car. Unconfirmed.
// 0x11220028 (60 ms) - not needed
//   b0 / b1 toggle, b6-7 drift slowly [LOG]; b3[1:0]:b4 10-bit [OEM s0B: defined, referenced
//   nowhere - NOT read (#70, was mistagged "read by the DEM")], 1 in the log
// 0x11420006 (120 ms) - not needed
//   b6 bit 5 sets together with the traction-control flag [LOG]; b6[1:0]:b7 10-bit + b6 bits 2-3
//   [OEM s11 / s12: defined, referenced nowhere - NOT read (#70, was mistagged "read by the DEM")]
// Traction control in the log's one wheelspin: 0x00E24026 b6 bit 2 sets first (39.75 s -
// the DSTC asking the engine to cut torque?), then 0x00224024 b4 bit 7 (40.00 s, pulsing
// with the intervention) and 0x11420006 b6 bit 5 (40.01 s, the dash SPIN lamp?) [LOG].
// None of them moves the clutch [BENCH].
// Not found: the DSTC / spin-control OFF button. The AWD module neither reacts to nor
// stores it (no live-data change for any input but the brake switch, bench), so it has to
// come from a car log with the button pressed. Published decodes (MaxxECU P2 250k,
// SwedeSpeed "DSTC deactivation" thread) were not readable.
//
// Car frames the AWD module does NOT receive (S60 log - for OEM-mode chassis data):
//   0x11600002 (100 ms): b2-3 STEERING ANGLE - centre ~0x62A0, + = right, holds while a
//               turn holds, moves while parked; ~0.4 deg per count (ESTIMATE from one
//               junction turn's wheel-speed geometry). b0 high nibble counter; b1 rises
//               with the turn (yaw / lateral g?). b4-5 FL, b6-7 FR wheel speed, 0.01 km/h
//   0x11800002 (100 ms): b4-5 RR, b6-7 RL wheel speed, 0.01 km/h; b2 status bits
//   0x10800006 (50 ms):  b0-1 yaw rate or lateral g (0x0080 straight / stopped, falls in a
//               right turn, grows ~ speed squared); b4-7 = RR / RL / FR / FL wheel-pulse
//               counters (8-bit, wrapping); b3 bit 4 toggles
//   0x01400006 (10 ms):  b6-7 longitudinal acceleration? (~0x0200 at rest, differs per stop
//               like a slope, rises on every pull-away)
//   0x00A20016 (10 ms):  b3 = copy of the accelerator pedal
//
// From the Haldex, 0x1020000A every 20 ms: b6 = measured clutch pressure (0 open,
// 0xFF full), b7 = status: bit 0 = fault stored / limp, bits 2-3 = 3 when its
// input monitors are happy, bits 6-7 = 2 when no clutch demand. [BENCH][no OEM code for
// these meanings on the P3; the S60R layout is b6 = s29, b7[7:6] s2C / [3:2] s2A / [1:0] s2B,
// b5 bits 0-2 = update bits, b0-b4 never written (#49-#59)]
#define VOLVO_G4_HALDEX_ID    0x1020000A
//   Diagnostics (Volvo "D2 over CAN"): request 0x000FFFFE [0xC8+n, 0x1A, service ...],
//   reply 0x01204001. Module 0x1A. AE 11 = read faults, AF 11 = clear,
//   A6 00 <id> 01 = live data, B9 F0 = part numbers. [BENCH][OEM #71/#15: S60R MO03 = exact-match
//   RX of 0x000FFFFE (sig 0x18), MO01 TX 0x01204001, module byte 0x1A checked. NOT the test channel:
//   frames F7/EF 1A <cmd> <arg> on the same ID are tester commands that can force the clutch-pressure
//   setpoint (#45) - never send those; our 0xC8+n headers never match them]
#define VOLVO_G4_DIAG_REQUEST_ID  0x000FFFFE
#define VOLVO_G4_DIAG_RESPONSE_ID 0x01204001
#define VOLVO_G4_DIAG_MODULE      0x1A

// Volvo Gen5 "SPC" Haldex 31380867 (generation 53, V11.00.0) - 11-bit IDs, 500 kbit/s
// assumed (OEM code: spec_ppc_volvo_spc_bmw.md, Volvo part). A different unit and
// ID set from our P3 generation 43 (29-bit, reply 0x1020000A).
// Lock request: 0x041 bytes 6-7, signed 16-bit, high byte first, with byte 5 bit 2 = update bit (V11.00.13).
// It is a torque CEILING on the unit's own demand (0 = none), proportional below ~3222. No checksum / counter.
#define VOLVO_SPC_REQUEST_ID    0x041 // clutch torque ceiling: bytes 6-7 (OEM code COM signal 12, update bit b5.2) [KM -: no 0x041 in volvo_v60_2015_pt / v40_2017_pt][REA audit_ppc_spc_bmw #35/#40]
// The other frames the unit receives (CanIf table). Meanings not decoded yet:
// sent as all-zero starting values (switch with VOLVO_SPC_SEND_PLACEHOLDERS).
// [KM -] for all of them except 0x235 (below). [REA audit_ppc_spc_bmw placeholders: 39 of the unit's 40 plain
// RX signals have COM update bits - all-zero frames leave them clear, so NOTHING in these frames is taken and
// their deadline monitors are never reloaded; per-signal update bits are listed in scripts/audit_ppc_spc_bmw/out/v_com_all.txt]
#define VOLVO_SPC_04A_ID      0x04A // receive frame, content unknown
#define VOLVO_SPC_050_ID      0x050 // receive frame, content unknown
#define VOLVO_SPC_056_ID      0x056 // receive frame, content unknown
#define VOLVO_SPC_066_ID      0x066 // receive frame, content unknown
#define VOLVO_SPC_071_ID      0x071 // receive frame, content unknown
#define VOLVO_SPC_076_ID      0x076 // receive frame, content unknown
#define VOLVO_SPC_081_ID      0x081 // receive frame, content unknown
#define VOLVO_SPC_086_ID      0x086 // receive frame, content unknown
#define VOLVO_SPC_131_ID      0x131 // receive frame, content unknown
#define VOLVO_SPC_155_ID      0x155 // receive frame, content unknown
#define VOLVO_SPC_158_ID      0x158 // receive frame, content unknown
#define VOLVO_SPC_161_ID      0x161 // receive frame, content unknown
#define VOLVO_SPC_165_ID      0x165 // receive frame, content unknown
#define VOLVO_SPC_171_ID      0x171 // receive frame, content unknown
#define VOLVO_SPC_200_ID      0x200 // receive frame, content unknown
#define VOLVO_SPC_211_ID      0x211 // receive frame, content unknown
#define VOLVO_SPC_231_ID      0x231 // receive frame, content unknown
#define VOLVO_SPC_235_ID      0x235 // receive frame, content unknown [KM V60 2015 wheelspeed0: WhlSpdRF b4-5 / WhlSpdLF b6-7, 0.01 km/h, -327.68 offset - ID match only, not proven the same signal on this unit's car]
#define VOLVO_SPC_281_ID      0x281 // receive frame, content unknown
#define VOLVO_SPC_285_ID      0x285 // receive frame, content unknown
#define VOLVO_SPC_57E_ID      0x57E // receive frame, content unknown
#define VOLVO_SPC_57F_ID      0x57F // receive frame, content unknown
#define VOLVO_SPC_587_ID      0x587 // receive frame, content unknown
#define VOLVO_SPC_DIAG_REQ    0x738 // V11.00.13 UDS physical request (0x7FF = functional) [REA dtc_check_volvo: CanIf RX PDU 1 -> CanTp]
#define VOLVO_SPC_DIAG_RESP   0x638 // V11.00.13 UDS response [REA dtc_check_volvo: CanIf TX PDU 2]
#define VOLVO_SPC_NM_ID       0x501 // network management keep-awake (the unit listens to 0x500-0x53F; its own is 0x51E) [KM -][REA #4/#8/#48: 0x500-0x53F accepted (CanIf range + masked MB); CanNm byte layout not read]
#define VOLVO_SPC_HALDEX_ID   0x126 // from the Haldex: bytes 0-1 = the unit's final torque SETPOINT (audit #27), byte 3 bits 3-4 = status [KM -: V60 has 0x127 CCButtons, no 0x126][REA #19-#21/#27]
// Scaling. The unit turns 0x041 into clutch torque through calibration tables we
// have not read (OEM code: CAL_dc54 / CAL_c5fc), so the real full-lock count is unknown.
#define VOLVO_SPC_SEND_PLACEHOLDERS 1       // ** CAN CHANGE THIS ** 1 = also send the 23 unknown receive frames as all-zero starting values, 0 = send only 0x041 + 0x501
// Not used by us: diagnostics 0x7FF / 0x738 in, 0x638 out; XCP 0x5A8 / 0x597.

// Volvo P2 Haldex 31431022 (XC166) - NOT a generation yet. Its OEM code only gave
// up the diagnostic channel (spec_volvo_p2_xc166.md): 11-bit, 500 kbit/s,
// requests in on 0x795 (physical) / 0x7DF (OBD broadcast), replies out on 0x79D.
// Its lock / feedback frames need a bench capture first. Listed for reference only.

// Volvo P1 (S40 / V50 AWD, generation 31, V11.00.1) - 500 kbit/s, 29-bit IDs, all
// frames 8 bytes, high byte first. From Adam's 2005 V50 T5 AWD logs
// (documents/vehicle-logs/gen3/P1_V50_T5_AWD_DECODE.md). None of the P2 IDs are
// used on P1. The AWD module sends ONE frame (0x1960000A); everything else below
// is the car's high-speed bus, which standalone sends in full. Send times are from
// the full C30 PCAN trace in the same folder.
// V11.00.13 tags: every P1 ID and byte is [KM -][REA none] - no KMatrix covers the P1 29-bit set
// (the Volvo DBCs held are 11-bit P3 / V40) and no P1 AWD-module dump exists (audit_volvo_c166 §5).
// All meanings below are [LOG] (V50 / C30 traces) only.
#define VOLVO_P1_ENGINE_TORQUE_ID 0x0300410E // 10 ms: two engine torques (12-bit, 2048 = 0), brake switch b4 bit 4 (+ bit 6), pedal b7
#define VOLVO_P1_ENGINE_RPM_ID    0x0090411E // 20 ms: RPM = byte 2 bits 0-4 + byte 3, 1 rpm per count
#define VOLVO_P1_WHEELS_ID        0x01C04026 // 14 ms: four wheel speeds RL, RR, FL, FR - 16-bit, 0x8000 = stopped
#define VOLVO_P1_SPEED_ID         0x02104136 // 14 ms: vehicle speed bytes 6-7 (same scale as the wheels)
#define VOLVO_P1_BRAKE_ID         0x02000026 // 14 ms: brake pressure b5 + b4 bits 6-7, DSTC off b6 bit 4
#define VOLVO_P1_HANDBRAKE_ID     0x0110483C // 10 ms: handbrake b2 bit 6, b2 bit 7 set every 5th send
#define VOLVO_P1_STEERING_ID      0x02804026 // 14 ms: steering angle b0 bits 0-3 + b1 (2048 = straight), yaw / lateral b2 bits 0-3 + b3
#define VOLVO_P1_0100082C_ID      0x0100082C // 10 ms: b4 bit 4 set every 10th send, rest steady
#define VOLVO_P1_STEER_SENSOR_ID  0x0340412E // 10 ms: steering angle sensor - b0 counter + direction, b1-2 fine rate, b3 check byte, b4-5 coarse rate, b6-7 angle
#define VOLVO_P1_02C00020_ID      0x02C00020 // 14 ms: steady
#define VOLVO_P1_00800006_ID      0x00800006 // 20 ms: b0 bits 6-7 = 2-bit counter
#define VOLVO_P1_03800006_ID      0x03800006 // 20 ms: b0 bit 6 set every 5th send
#define VOLVO_P1_19000026_ID      0x19000026 // 15 ms: b6 bit 3 flips every send
#define VOLVO_P1_PULSES_ID        0x19A00002 // 28 ms: b0 page 02/12/6E/12, b1-b4 = four wheel pulse counters, b5 = distance
#define VOLVO_P1_19E00006_ID      0x19E00006 // 50 ms: steady
#define VOLVO_P1_1B200002_ID      0x1B200002 // 85 ms: steady
#define VOLVO_P1_1B600002_ID      0x1B600002 // 170 ms: b0 bit 5 every 6th send, b1 6-step pattern
#define VOLVO_P1_1B700030_ID      0x1B700030 // 210 ms: b0 bit 4 set every 3rd send
#define VOLVO_P1_1A200020_ID      0x1A200020 // 315 ms: b0 bits 6-7 = 2-bit counter
#define VOLVO_P1_1AE0092C_ID      0x1AE0092C // 420 ms: b0 bits 6-7 = 2-bit counter
#define VOLVO_P1_1BA0493C_ID      0x1BA0493C // 250 ms: 20 pages (b0 = 1-20), each with its own steady data
#define VOLVO_P1_NETWORK_ID       0x1BE0493C // 250 ms: pages by b0 - 01 = heartbeat + clock, 02 = status, 05, 10-12 = VIN (not sent)
#define VOLVO_P1_HALDEX_ID        0x1960000A // 20 ms, from the AWD module: b6 bits 0-1 + b7 = engagement (10-bit), b5 = status
// Standalone: how the lock % is shown to the module is chosen on the Settings page
// (Volvo P1 Standalone card: Lock Source, Road Speed, Gear, Torque at 100 %, Front
// Wheel Slip at 100 % - saved settings, see volvoP1* in defs.h). In the logs the module
// follows engine torque almost 1:1 (about 1 count of engagement per count of
// torque, 100-400 ms behind) and goes to full on big front wheelspin. It never
// locked while stopped, so the road speed is never 0.
// Engine rpm = Road Speed x the Gear setting's ratio, so rpm and road speed always
// match a real gear (a made-up pair looks like a slipping clutch). Ratios are rpm
// per km/h, measured from the V50 logs under load (25 Oct 2026 histogram of every
// frame above 8 km/h): three clean peaks. 4th and up were never logged (87 km/h max).
#define VOLVO_P1_RPM_PER_KMH_GEAR1 117 // ** CAN CHANGE THIS ** 1st gear, rpm per km/h (logs: 116-118)
#define VOLVO_P1_RPM_PER_KMH_GEAR2 66  // ** CAN CHANGE THIS ** 2nd gear, rpm per km/h (logs: 65-66; 30 km/h = ~2000 rpm, as in Drive.csv at 56.9 s)
#define VOLVO_P1_RPM_PER_KMH_GEAR3 44  // ** CAN CHANGE THIS ** 3rd gear, rpm per km/h (logs: 43-44)
#define VOLVO_P1_RPM_MAX 6500          // ** CAN CHANGE THIS ** rpm never sent higher than this (the pull log topped out at ~5,600)
#define VOLVO_P1_STEER_CHECK_BASE 0x02 // 0x0340412E byte 3 = this - (byte 0 nibbles swapped + byte 1 bit 7 + bytes 4-7), with byte 0 bit 6 clear (0x04 when set). Fits all 26,820 logged frames
#define VOLVO_P1_LEARN_SETTLE_MS 600  // ** CAN CHANGE THIS ** Learn: wait this long at each step (engagement trails torque by 100-400 ms in the logs; other generations use 400)

// SAAB 9-3 XWD (GM family) - 500 kbit/s, 11-bit. Haldex replies on 0x210.
// GM-style "alive rolling count" counters, as our Gen41 uses.
#define SAAB_93_110_ID        0x110 // 10 ms, bytes 1-2 = 00 00 off / 1F 40 lock active
#define SAAB_93_120_ID        0x120 // 10 ms, byte 1 = lock request, byte 2 = 00 off / 83 active
#define SAAB_93_140_ID        0x140 // 10 ms, byte 0 bits 0-1 = 2-bit rolling counter
#define SAAB_93_160_ID        0x160 // 10 ms, bytes 2/3 = counter (bits 6-7) / protection (0 - counter)
#define SAAB_93_180_ID        0x180 // 10 ms, static
#define SAAB_93_288_ID        0x288 // 10 ms, static
#define SAAB_93_280_ID        0x280 // 20 ms, static
#define SAAB_93_300_ID        0x300 // 20 ms, static
#define SAAB_93_301_ID        0x301 // 100 ms, static
#define SAAB_93_320_ID        0x320 // 100 ms, static
#define SAAB_93_3ED_ID        0x3ED // 100 ms, static
#define SAAB_93_410_ID        0x410 // 100 ms, static
#define SAAB_93_398_ID        0x398 // 1000 ms, static
#define SAAB_93_510_ID        0x510 // 1000 ms, static
#define SAAB_93_HALDEX_ID     0x210 // from the Haldex: ((b0 & 7) << 8 | b1) >> 3 = status, b2 - 40 = temperature degC