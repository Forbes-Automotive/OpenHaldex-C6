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
#define BRAKES4_ID 0x2A0   // DLC 3 in vw_pq.dbc, but the Gen4 0BR needs at least 7 (OEM code: spec_gen4_0br §1) - we always send 8
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

#define mLW_1 0x0C2         // DLC 7 here; the Gen4 0BR needs at least 6 (OEM code: spec_gen4_0br §1) - we always send 8
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

// OpenHaldex supplier-specific UDS DIDs answered on the Haldex physical address
// (request on diagnostics_5_ID 0x70F, response 0x779) so an OBD tester (e.g.
// Rokketek gauge) can read lock/slip and set the drive mode even though the
// car's gateway will not forward the passive 0x6B0 broadcast that far.
#define OPENHALDEX_UDS_RESPONSE_ID 0x779
#define OPENHALDEX_LOCK_DID 0xFDA0 // RDBI: [cmd%][engagement raw][mode]
#define OPENHALDEX_SLIP_DID 0xFDA1 // RDBI: 4 signed slip bytes [FL, FR, RL, RR]
#define OPENHALDEX_MODE_DID 0xFDA2 // WDBI: [mode] sets the drive mode

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
#define KLEMMEN_STATUS_01 0x3C0 // MQB ignition status (ZAS_Kl_15)
#define ESP_18 0x135

#define ESP_29 0x18c
#define KOMBI_01 0x30b

#define CHARISMA_01 0x385 // MQB drive mode select: the gateway's requested mode for each system (engine, gearbox, Haldex ...), 1 s
#define CHARISMA_07 0x3E8 // MQB drive mode select: byte 1 bits 0-3 = the drive profile the driver picked (CHA_Current_Mode), 1 s
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
#define HALDEX_GEN5_STATUS_ID 0x17F0000F // 29-bit ID, sent by the 0CQ (DLC 8): byte 0 bit 0 = fault reaction active, byte 1 = 0x0F (OEM code: spec_gen5_0cq §6)
#define HALDEX_GEN5_STATUS_TIMEOUT_MS 3000 // forget the 0x17F0000F fault bit if the frame stops (seen about twice a second in our logs)

// Gen5 VAQ (MQB front-axle transverse lock, K-matrix node "VAQ" / "Quersperre").
// Source: MQB_FCAN_KMatrix (Documents gitignore/can-databases/vw-mqb). The VAQ
// node receives 22 of the 25 periodic frames the Haldex does (ESP_10 is the one
// periodic frame it does NOT list), reads ESP_14 bytes 4/5 (BR_Vorg_Quer_Min/Max)
// where the Haldex reads bytes 6/7 (BR_Vorg_Allrad_*), and answers diagnostics on
// its own ISO-TP pair. Its feedback frame is the Quersperre_03 counterpart of the
// Haldex's Allrad_03 (0x118). Which of these the physical unit really uses is
// confirmed on the bench with the serial lab's RXIDS / DTC commands.
#define QUERSPERRE_03 0x137       // TX by Quersperre/VAQ, DLC 4, 100 ms: QUER_Sta_Quersperre b1[4..6], QUER_Gleichlauf b1[7], QUER_Ist_Proz b2 (0.4 %/bit), Charisma b3
#define QUERSPERRE_04 0x3FF       // TX by Quersperre/VAQ, DLC 8, 500 ms: cluster display bits (lock engage/disengage/closed) b1[4..6]
#define ISO_QUERSPERRE_REQ  0x71E // UDS physical request to the VAQ / Quersperre ECU (K-matrix ISO_Quersperre_Req)
#define ISO_QUERSPERRE_RESP 0x788 // UDS response from the VAQ / Quersperre ECU  (K-matrix ISO_Quersperre_Resp)
#define ISO_ALLRAD_REQ      0x70F // UDS physical request to the Haldex (Allrad) ECU - same as diagnostics_5_ID
#define ISO_ALLRAD_RESP     0x779 // UDS response from the Haldex (Allrad) ECU  - same as OPENHALDEX_UDS_RESPONSE_ID


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

// ─── Volvo / SAAB (V10.00.0) ─────────────────────────────────────────────────
// Starting point only: IDs, rates and lock bytes. The Volvo network heartbeat
// matches a real Volvo PCAN trace.
// Byte meanings are NOT confirmed on our own cars yet - adjust with the serial lab
// (OVR) / SavvyCAN, then replace with decoded values once logs are in.
// Volvo uses 29-bit (extended) IDs on P2; send them with frame.extd = 1.

// Volvo P2 Gen2 (pre-facelift) - 250 kbit/s, 29-bit. Haldex replies on 0x0120000A.
#define VOLVO_P2G2_224024_ID  0x00224024 // 10 ms, static
#define VOLVO_P2G2_LOCK_ID    0x0042401E // 10 ms, byte 2 = 0x20 (0x21 at full lock), byte 3 = lock request
#define VOLVO_P2G2_ACTIVE_ID  0x0062401E // 10 ms, bytes 6-7 = 60 00 idle / 67 D0 lock active
#define VOLVO_P2G2_C24000_ID  0x00C24000 // 10 ms, static
#define VOLVO_P2G2_1E2402C_ID 0x01E2402C // 20 ms, static
#define VOLVO_P2G2_NETWORK_ID 0x0242C07C // 100 ms, network heartbeat (bytes 6-7 FE F8 <-> 01 07)
#define VOLVO_P2G2_3200002_ID 0x03200002 // 100 ms, static
#define VOLVO_P2G2_3400002_ID 0x03400002 // 100 ms, static
#define VOLVO_P2G2_HALDEX_ID  0x0120000A // from the Haldex: byte 6 = status (scaling unknown)

// Volvo P2 Gen3 (facelift) - 500 kbit/s, 29-bit. Haldex replies on 0x1020000A.
// V11.00.0: all of these IDs are confirmed in the S60R DEM's own message table
// (OEM code: spec_volvo_p2_s60r.md section 1). "500k S60 log" below = the public
// log volvo-CAN-exploration/0000286.TXT (500 kbit/s, model year not stated, it
// has these facelift IDs). The 2002/2003 P2 logs are pre-facelift and use other IDs.
#define VOLVO_P2G3_ACTIVE_ID  0x10004002 // 10 ms, bytes 4-7 = C0 00 C0 00 idle / C1 42 C1 42 active (500k S60 log: four 10-bit wheel speeds, 14 ms)
#define VOLVO_P2G3_42406C_ID  0x0042406C // 10 ms, byte 0 bits 6-7 = 2-bit rolling counter (500k S60 log)
#define VOLVO_P2G3_LOCK_ID    0x0062401E // 10 ms, byte 1 = lock request, bytes 0/4-7 = active flags
#define VOLVO_P2G3_MODE_ID    0x0080401E // 10 ms, bytes 2-3 = 80 00 idle / 87 D0 active (500k S60 log: looks like engine rpm, 0x7D0 = 2000), byte 0 toggles 1E/06
#define VOLVO_P2G3_224024_ID  0x00224024 // 10 ms, byte 0 bit 2 toggles every send (500k S60 log, 14 ms there)
#define VOLVO_P2G3_E24026_ID  0x00E24026 // 10 ms, static (14 ms in the 500k S60 log)
#define VOLVO_P2G3_NETWORK_ID 0x10A2407C // 100 ms, network heartbeat (bytes 6-7 FE F5 <-> 01 0A)
#define VOLVO_P2G3_HALDEX_ID  0x1020000A // from the Haldex: byte 6 = measured clutch value, byte 7 = status bits (OEM code, leaf_trace/volvo_bitrate.md)
#define VOLVO_P2G3_ENGAGEMENT_FULL_SCALE 150 // byte 6 value read as 100 % engagement - OEM code says ~150 at sensor full scale ** CAN CHANGE THIS ** after a bench check
// V11.00.0: four more frames the DEM listens for (OEM code: MO table) that we
// never sent. Content = steady values from the 500k S60 log, meanings unknown.
#define VOLVO_P2G3_C0402A_ID  0x00C0402A // 10 ms in the log, steady 1E E4 00 00 27 FF D3 FF
#define VOLVO_P2G3_10E24000_ID 0x10E24000 // 56 ms in the log (we send every 50 ms), all zero
#define VOLVO_P2G3_11220028_ID 0x11220028 // 60 ms in the log (we send every 50 ms), 23 60 0F 3C 01 00 35 2E
#define VOLVO_P2G3_11420006_ID 0x11420006 // 120 ms in the log (we send every 100 ms), 00 00 00 0F 00 00 C0 00
// 0x01A2402A is also in the DEM's table but is NOT on the bus in the 500k S60
// log, so its content is unknown and it is not sent (V11 TODO (bench)).
#define VOLVO_P2G3_1A2402A_ID 0x01A2402A // not sent - content unknown

// Volvo Gen4 (P3 / Freelander 2 family) - 500 kbit/s, 11-bit. Haldex replies on 0x243.
#define VOLVO_G4_MODE_ID      0x12A // 10 ms, EngineData: byte 4 bit 6 = brake, bytes 6-7 = engine RPM (13-bit)
#define VOLVO_G4_20A_ID       0x20A // 20 ms, Brake_Info, static
#define VOLVO_G4_220_ID       0x220 // 20 ms, rear wheel speeds (0x8000 = 0 km/h), static
#define VOLVO_G4_NETWORK_ID   0x405 // 100 ms, network heartbeat (bytes 6-7 F4 45 <-> 0B BA)
#define VOLVO_G4_HALDEX_ID    0x243 // from the Haldex: byte 7 = status (scaling unknown)

// Volvo Gen5 "SPC" Haldex 31380867 (generation 53, V11.00.0) - 11-bit IDs, 500 kbit/s
// assumed (OEM code: spec_ppc_volvo_spc_bmw.md, Volvo part). A different unit and
// ID set from our P3 generation 43 - none of 43's IDs (0x12A/0x20A/0x220/0x405,
// reply 0x243) are in this unit's receive table.
// Lock lever: 0x041 bytes 6-7, signed 16-bit, high byte first. 0 = clutch open,
// bigger = more lock, saturates at full lock. No checksum or counter is checked.
#define VOLVO_SPC_LEVER_ID    0x041 // clutch demand: bytes 6-7 (OEM code COM signal 12)
// The other frames the unit receives (CanIf table). Meanings not decoded yet:
// sent as all-zero starting values (switch with VOLVO_SPC_SEND_PLACEHOLDERS).
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
#define VOLVO_SPC_235_ID      0x235 // receive frame, content unknown
#define VOLVO_SPC_281_ID      0x281 // receive frame, content unknown
#define VOLVO_SPC_285_ID      0x285 // receive frame, content unknown
#define VOLVO_SPC_57E_ID      0x57E // receive frame, content unknown
#define VOLVO_SPC_57F_ID      0x57F // receive frame, content unknown
#define VOLVO_SPC_587_ID      0x587 // receive frame, content unknown
#define VOLVO_SPC_NM_ID       0x501 // network management keep-awake (the unit listens to 0x500-0x53F; its own is 0x51E)
#define VOLVO_SPC_HALDEX_ID   0x126 // from the Haldex: bytes 0-1 = delivered clutch torque, byte 3 bits 3-4 = status
// Scaling. The unit turns 0x041 into clutch torque through calibration tables we
// have not read (OEM code: CAL_dc54 / CAL_c5fc), so the real full-lock count is unknown.
#define VOLVO_SPC_LEVER_FULL_SCALE 0x7FFF   // ** CAN CHANGE THIS ** 0x041 bytes 6-7 sent for 100 % (largest positive value, certain to saturate). V11 TODO (bench): lower to the count where lock stops rising
#define VOLVO_SPC_FEEDBACK_FULL_SCALE 6710  // 0x126 bytes 0-1 count shown as 100 % (= the biggest value the OEM code can put there: 0xFFF x 1024 / 625). V11 TODO (bench): set to the count at real full lock
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
#define VOLVO_P1_ENGAGEMENT_FULL_SCALE 540   // ** CAN CHANGE THIS ** engagement count shown as 100 % (the ceiling in the full pull and the wheelspin logs)
// Standalone: how the lock % is shown to the module is chosen on the Settings page
// (Volvo P1 Standalone card: Lock Source, Road Speed, Torque at 100 %, Front Wheel
// Slip at 100 % - saved settings, see volvoP1* in defs.h). In the logs the module
// follows engine torque almost 1:1 (about 1 count of engagement per count of
// torque, 100-400 ms behind) and goes to full on big front wheelspin. It never
// locked while stopped, so the road speed is never 0.
#define VOLVO_P1_PEDAL_FULL 231       // ** CAN CHANGE THIS ** pedal byte at 100 % lock (full throttle in the pull log; 0 at 0 %)
#define VOLVO_P1_RPM_AT_ZERO 2000     // ** CAN CHANGE THIS ** rpm at 0 % lock (the drive log at 30 km/h)
#define VOLVO_P1_RPM_FULL 4500        // ** CAN CHANGE THIS ** rpm at 100 % lock
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