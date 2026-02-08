#ifndef OPENHALDEXC6_CANID_H
#define OPENHALDEXC6_CANID_H

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

#endif // OPENHALDEXC6_CANID_H
