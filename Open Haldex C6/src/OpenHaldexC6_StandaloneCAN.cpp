#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>

// Gen1 Standalone Frames
void Gen1_frames10() {}

void Gen1_frames20() {
  twai_message_t frame;
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[2] = 0x21;
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false);
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false);

  switch (state.mode) {
    case MODE_FWD:
      appliedTorque = get_lock_target_adjusted_value(0xFE, true);
      break;
    case MODE_5050:
      appliedTorque = get_lock_target_adjusted_value(0x16, false);
      break;
    case MODE_6040:
      appliedTorque = get_lock_target_adjusted_value(0x22, false);
      break;
    case MODE_7525:
      appliedTorque = get_lock_target_adjusted_value(0x50, false);
      break;
    default:
      appliedTorque = 0;
      break;
  }

  frame.data[6] = appliedTorque;
  frame.data[7] = 0x00;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR3_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x50;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0xFE;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;
  frame.data[1] = get_lock_target_adjusted_value(0x00, false);
  frame.data[2] = 0x00;
  frame.data[3] = 0x0A;
  frame.data[4] = 0xFE;
  frame.data[5] = 0xFE;
  frame.data[6] = 0x00;
  frame.data[7] = BRAKES1_counter;
  if (++BRAKES1_counter > 0xF) BRAKES1_counter = 0;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[1] = 0x0A;
  frame.data[2] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[3] = 0x0A;
  frame.data[4] = 0x00;
  frame.data[5] = 0x0A;
  frame.data[6] = 0x00;
  frame.data[7] = 0x0A;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen1_frames25() {}
void Gen1_frames100() {}
void Gen1_frames200() {}
void Gen1_frames1000() {}

// Gen2 Standalone Frames
void Gen2_frames10() {
  twai_message_t frame;
  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x41;
  frame.data[2] = 0x00;
  frame.data[3] = 0xFE;
  frame.data[4] = 0xFE;
  frame.data[5] = 0xFE;
  frame.data[6] = 0x00;
  frame.data[7] = BRAKES1_counter;
  if (++BRAKES1_counter > 0xF) BRAKES1_counter = 0;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x7F;
  frame.data[1] = 0xAE;
  frame.data[2] = 0x3D;
  frame.data[3] = BRAKES2_counter;
  frame.data[4] = get_lock_target_adjusted_value(0x7F, false);
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[6] = 0x5E;
  frame.data[7] = 0x2B;
  BRAKES2_counter = BRAKES2_counter + 10;
  if (BRAKES2_counter > 0xF7) BRAKES2_counter = 7;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[1] = 0x0A;
  frame.data[2] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[3] = 0x0A;
  frame.data[4] = 0x00;
  frame.data[5] = 0x0A;
  frame.data[6] = 0x00;
  frame.data[7] = 0x0A;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = mLW_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x20;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x80;
  frame.data[5] = mLW_1_counter;
  frame.data[6] = 0x00;
  mLW_1_crc = 255 - (frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3] + frame.data[5]);
  frame.data[7] = mLW_1_crc;
  mLW_1_counter = mLW_1_counter + 16;
  if (mLW_1_counter >= 0xF0) mLW_1_counter = 0;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen2_frames20() {
  twai_message_t frame;
  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x08;
  frame.data[1] = 0xFA;
  frame.data[2] = 0x20;
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false);
  frame.data[4] = 0xFA;
  frame.data[5] = 0xFA;
  frame.data[6] = get_lock_target_adjusted_value(0x20, false);
  frame.data[7] = 0xFA;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x30;
  frame.data[2] = 0x00;
  frame.data[3] = 0x0A;
  frame.data[4] = 0x0A;
  frame.data[5] = 0x10;
  frame.data[6] = 0xFE;
  frame.data[7] = 0xFE;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen2_frames25() {
  twai_message_t frame;
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x02;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x36;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen2_frames100() {}
void Gen2_frames200() {}
void Gen2_frames1000() {}

// Gen4 Standalone Frames
void Gen4_frames10() {
  twai_message_t frame;
  frame.identifier = mLW_1;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = lws_2[mLW_1_counter][0];
  frame.data[1] = lws_2[mLW_1_counter][1];
  frame.data[2] = lws_2[mLW_1_counter][2];
  frame.data[3] = lws_2[mLW_1_counter][3];
  frame.data[4] = lws_2[mLW_1_counter][4];
  frame.data[5] = lws_2[mLW_1_counter][5];
  frame.data[6] = lws_2[mLW_1_counter][6];
  frame.data[7] = lws_2[mLW_1_counter][7];

  mLW_1_counter++;
  if (mLW_1_counter > 15) mLW_1_counter = 0;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES1_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x20;
  frame.data[1] = 0x40;
  frame.data[2] = 0xF0;
  frame.data[3] = 0x07;
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[6] = 0x00;
  frame.data[7] = BRAKES1_counter;
  if (++BRAKES1_counter > 0x1F) BRAKES1_counter = 10;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES3_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xB6, false);
  frame.data[1] = 0x07;
  frame.data[2] = get_lock_target_adjusted_value(0xCC, false);
  frame.data[3] = 0x07;
  frame.data[4] = get_lock_target_adjusted_value(0xD2, false);
  frame.data[5] = 0x07;
  frame.data[6] = get_lock_target_adjusted_value(0xD2, false);
  frame.data[7] = 0x07;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES4_ID;
  frame.data_length_code = 8;
  frame.data[0] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x64;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = BRAKES4_counter;
  BRAKES4_crc = 0;
  for (uint8_t i = 0; i < 7; i++) {
    BRAKES4_crc ^= frame.data[i];
  }
  frame.data[7] = BRAKES4_crc;
  BRAKES4_counter = BRAKES4_counter + 16;
  if (BRAKES4_counter > 0xF0) BRAKES4_counter = 0x00;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR1_ID;
  frame.data_length_code = 8;
  frame.data[1] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[2] = get_lock_target_adjusted_value(0x20, false);
  frame.data[3] = get_lock_target_adjusted_value(0x4E, false);
  frame.data[4] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[5] = get_lock_target_adjusted_value(0xFE, false);
  frame.data[6] = get_lock_target_adjusted_value(0x16, false);
  frame.data[7] = get_lock_target_adjusted_value(0xFE, false);
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen4_frames20() {
  twai_message_t frame;
  frame.identifier = BRAKES2_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x80;
  frame.data[1] = 0x7A;
  frame.data[2] = 0x05;
  frame.data[3] = BRAKES2_counter;
  frame.data[4] = get_lock_target_adjusted_value(0x7F, false);
  frame.data[5] = 0xCA;
  frame.data[6] = 0x1B;
  frame.data[7] = 0xAB;
  twai_transmit_v2(twai_bus_1, &frame, 0);
  BRAKES2_counter = BRAKES2_counter + 16;
  if (BRAKES2_counter > 0xF0) BRAKES2_counter = 0;
}

void Gen4_frames25() {
  twai_message_t frame;
  frame.identifier = mKombi_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x24;
  frame.data[1] = 0x00;
  frame.data[2] = 0x1D;
  frame.data[3] = 0xB9;
  frame.data[4] = 0x07;
  frame.data[5] = 0x42;
  frame.data[6] = 0x09;
  frame.data[7] = 0x81;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = mKombi_3;
  frame.data_length_code = 8;
  frame.data[0] = 0x60;
  frame.data[1] = 0x43;
  frame.data[2] = 0x01;
  frame.data[3] = 0x10;
  frame.data[4] = 0x66;
  frame.data[5] = 0xF1;
  frame.data[6] = 0x03;
  frame.data[7] = 0x02;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen4_frames100() {
  twai_message_t frame;
  frame.identifier = mGate_Komf_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x03;
  frame.data[1] = 0x11;
  frame.data[2] = 0x58;
  frame.data[3] = 0x00;
  frame.data[4] = 0x40;
  frame.data[5] = 0x00;
  frame.data[6] = 0x01;
  frame.data[7] = 0x08;
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES11_ID;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0XC0;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen4_frames200() {
  twai_message_t frame;
  frame.identifier = mKombi_2;
  frame.data_length_code = 8;
  frame.data[0] = 0x4C;
  frame.data[1] = 0x86;
  frame.data[2] = 0x85;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x30;
  frame.data[6] = 0xFF;
  frame.data[7] = 0x04;
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen4_frames1000() {
  twai_message_t frame;
  frame.identifier = mDiagnose_1;
  frame.data_length_code = 8;
  frame.data[0] = 0x26;
  frame.data[1] = 0xF2;
  frame.data[2] = 0x03;
  frame.data[3] = 0x12;
  frame.data[4] = 0x70;
  frame.data[5] = 0x19;
  frame.data[6] = 0x25;
  frame.data[7] = mDiagnose_1_counter;
  twai_transmit_v2(twai_bus_1, &frame, 0);
  mDiagnose_1_counter++;
  if (mDiagnose_1_counter > 0x1F) mDiagnose_1_counter = 0;
}
