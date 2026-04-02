#include <OpenHaldexC6_StandaloneCAN.h>
#include <OpenHaldexC6_Calculations.h>

// Periodic frame tasks
void frames10(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames10();
        break;
      case 2:
        Gen2_frames10();
        break;
      case 4:
        Gen4_frames10();
        break;
      case 5:
        Gen5_frames10();
        break;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void frames20(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames20();
        break;
      case 2:
        Gen2_frames20();
        break;
      case 4:
        Gen4_frames20();
        break;
      case 5:
        Gen5_frames20();
        break;
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void frames25(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(25 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames25();
        break;
      case 2:
        Gen2_frames25();
        break;
      case 4:
        Gen4_frames25();
        break;
      case 5:
        Gen5_frames25();
        break;
      }
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void frames100(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      lock_target = get_lock_target_adjustment();
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames100();
        break;
      case 2:
        Gen2_frames100();
        break;
      case 4:
        Gen4_frames100();
        break;
      case 5:
        Gen5_frames100();
        break;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void frames200(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames200();
        break;
      case 2:
        Gen2_frames200();
        break;
      case 4:
        Gen4_frames200();
        break;
      case 5:
        Gen5_frames200();
        break;
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void frames1000(void *arg)
{
  while (1)
  {
    // Analyzer mode runs as a passive bridge; skip standalone frame generation.
    if (analyzerMode)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    if (isStandalone)
    {
      switch (haldexGeneration)
      {
      case 1:
        Gen1_frames1000();
        break;
      case 2:
        Gen2_frames1000();
        break;
      case 4:
        Gen4_frames1000();
        break;
      case 5:
        Gen5_frames1000();
        break;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Gen1 Standalone Frames
void Gen1_frames10() {
  // not used in Gen1
}

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

void Gen1_frames25() {
  // not used in Gen1
}
void Gen1_frames100() {
  // not used in Gen1
}
void Gen1_frames200() {
  // not used in Gen1
}
void Gen1_frames1000() {
  // not used in Gen1
}

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

  frame.identifier = BRAKES4_ID;    // 0x2A0 includes coupling moment? Affects vehicle mode(!) do not have!
  frame.data_length_code = 8;       // DLC 8 (/4)
  frame.data[0] = 0x00;             // affects estimated torque AND vehicle mode(!)
  frame.data[1] = 0x00;             //
  frame.data[2] = 0x00;             //
  frame.data[3] = 0x00;             // 32605
  frame.data[4] = 0x00;             //
  frame.data[5] = 0x00;             //
  frame.data[6] = BRAKES4_counter;  // checksum
  frame.data[7] = BRAKES4_counter;  // checksum
  BRAKES4_counter = BRAKES4_counter + 10;
  if (BRAKES4_counter > 0xF0) {
    BRAKES4_counter = 0;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES5_ID;     // 0x2A0 includes coupling moment? Affects vehicle mode(!) do not have!
  frame.data_length_code = 8;        // DLC 8 (/4)
  frame.data[0] = 0xFE;              // affects estimated torque AND vehicle mode(!)
  frame.data[1] = 0x7F;              //
  frame.data[2] = 0x03;              //
  frame.data[3] = 0x00;              // 32605
  frame.data[4] = 0x00;              //
  frame.data[5] = 0x00;              //
  frame.data[6] = BRAKES5_counter;   // checksum
  frame.data[7] = BRAKES5_counter2;  // checksum
  BRAKES5_counter = BRAKES5_counter + 10;
  if (BRAKES5_counter > 0xF0) {
    BRAKES5_counter = 0;
  }
  BRAKES5_counter2 = BRAKES5_counter2 + 10;
  if (BRAKES5_counter2 > 0xF3) {
    BRAKES5_counter2 = 3;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES9_ID;     // 0x0AE do not have!
  frame.data_length_code = 8;        // DLC 8
  frame.data[0] = BRAKES9_counter;   // checksum
  frame.data[1] = BRAKES9_counter2;  // checksum
  frame.data[2] = 0x00;              // no effect
  frame.data[3] = 0x00;              // no effect
  frame.data[4] = 0x00;              // no effect
  frame.data[5] = 0x00;              // no effect
  frame.data[6] = 0x02;              // 0x01
  frame.data[7] = 0x00;              // no effect
  twai_transmit_v2(twai_bus_1, &frame, 0);
  BRAKES9_counter = BRAKES9_counter + 10;
  if (BRAKES9_counter > 0xF1) {
    BRAKES9_counter = 0x11;
  }
  BRAKES9_counter2 = BRAKES9_counter2 + 10;
  if (BRAKES9_counter2 > 0xF0) {
    BRAKES9_counter2 = 0x00;
  }

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

   frame.identifier = MOTOR5_ID;    // 0x1A0
  frame.data_length_code = 8;      // DLC 8
  frame.data[0] = 0xFE;            // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed
  frame.data[1] = 0x00;            // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43?
  frame.data[2] = 0x00;            // was 0x00 no effect
  frame.data[3] = 0x00;            // was 0xFE no effect
  frame.data[4] = 0x00;            // was 0xFE miasrl no effect
  frame.data[5] = 0x00;            // was 0xFE miasrs no effect
  frame.data[6] = 0x00;            // was 0x00
  frame.data[7] = MOTOR5_counter;  // checksum
  if (++BRAKES1_counter > 255) {   // 0xF
    BRAKES1_counter = 0;           // 0
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = BRAKES10_ID;    // 0x1A0
  frame.data_length_code = 8;        // DLC 8
  frame.data[0] = 0xA6;              // ASR 0x04 sets bit 4.  0x08 removes set.  Coupling open/closed
  frame.data[1] = BRAKES10_counter;  // can use to disable (>130 dec).  Was 0x00; 0x41?  0x43?
  frame.data[2] = 0x75;              // was 0x00 no effect
  frame.data[3] = 0xD4;              // was 0xFE no effect
  frame.data[4] = 0x51;              // was 0xFE miasrl no effect
  frame.data[5] = 0x47;              // was 0xFE miasrs no effect
  frame.data[6] = 0x1D;              // was 0x00
  frame.data[7] = 0x0F;              // checksum
  BRAKES10_counter = BRAKES10_counter + 1;
  if (BRAKES10_counter > 0xF) {
    BRAKES10_counter = 0;
  }
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

void Gen5_frames10()
{
  twai_message_t frame;
  frame.identifier = ESP_18; // 0x135.  Fixed response, no changes
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00; // supposed to have CRC? doesn't affect
  frame.data[1] = 0xC0; // always 0xC0, never changes
  frame.data[2] = 0x00; // doesn't affect
  frame.data[3] = 0x00; // doesn't affect
  frame.data[4] = 0x00; // doesn't affect
  frame.data[5] = 0x00; // doesn't affect
  frame.data[6] = 0x00; // doesn't affect
  frame.data[7] = 0x00; // doesn't affect
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = ESP_19; // ESP_19 0x0b2
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8; // mad, but wheel speed MUST change - static makes it reduce over time and eventually stop.  Changing it to 0x0B makes it stop immediately.  Counter from 0x20 to 0x2F keeps it going.
  // moved to a slow incrememter (1000ms)
  frame.data[0] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // HL - wheel speed
  frame.data[1] = get_lock_target_adjusted_value(ESP_19_counter, false);         // HL - wheel speed
  frame.data[2] = get_lock_target_adjusted_value(ESP_19_counter2, false);        // HR - wheel speed
  frame.data[3] = get_lock_target_adjusted_value(ESP_19_counter, false);         // HR - wheel speed
  frame.data[4] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xCA, false); // VL - wheel speed 0xDB
  frame.data[5] = get_lock_target_adjusted_value(ESP_19_counter, false);         // VL - wheel speed -- affects if =0x0B
  frame.data[6] = get_lock_target_adjusted_value(ESP_19_counter2 + 0xCA, false); // VR - wheel speed 0xDB
  frame.data[7] = get_lock_target_adjusted_value(ESP_19_counter, false);         // VR - wheel speed -- affects if =0x0B
  twai_transmit_v2(twai_bus_1, &frame, 0);
  ESP_19_counter++;
  ESP_19_counter2++;
  if (ESP_19_counter > 0x1A) // 0x1e
  {
    ESP_19_counter = 0x01; // 0x10
  }
  if (ESP_19_counter2 > 0x0E) // 0x0a
  {
    ESP_19_counter2 = 0x00; // 0x00
  }

  frame.identifier = GETRIEBE_11; // 0x0AD
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;                // checksum placeholder none affect
  frame.data[1] = GETRIEBE_11_counter; // rolling - 0x00>0x0F
  frame.data[2] = 0x00;                // Was 0xFE Torque intervention at the engine. Requests a short-term reduction or increase in torque from the ECU. This signal is only valid in combination with GE_MMom_Status (for MQB) or GE_MMom_Status_02 (for MLBevo).
  frame.data[3] = 0xFE;                // Pre-control torque (anticipatory torque request) (GE_MMom_Vorhalt_02)
  frame.data[4] = 0x00;                // Actual gear/range selected (5=P, 6=R, 7=N, 8=D, 9=S, 10=E, 13/14=T)
  frame.data[5] = 0x00;                // Shift sequence state (0=idle, 1=shift in progress, etc.) - does not affect (>0x00)
  frame.data[6] = 0x00;                // Power transmission status / clutch lock-up state - does not affect (>0x00)
  frame.data[7] = 0x00;                // Target gear of current shift

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0AD); // for 0x0AD

  GETRIEBE_11_counter++;
  if (GETRIEBE_11_counter > 0x0F)
  {
    GETRIEBE_11_counter = 0;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR_12; // Motor_12 0x0A8
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;             // checksum placeholder
  frame.data[1] = MOTOR_12_counter; // rolling - 0x70>0x7F
  frame.data[2] = 0x00;             // doesn't affect Negative available torque (maximum engine braking) (MO_Mom_neg_verfuegbar) - does not affect
  frame.data[3] = 0x00;             // doesn't affect sometimes 0xC0, sometimes 0x00 Static torque limit
  frame.data[4] = 0x00;             // doesn't affect sometimes 0x3A, somtimes 0x39 Dynamic torque limit
  frame.data[5] = 0x64;             // doesn't affect Vehicle speed signal quality bit (0x64=good, 0x00=bad).  True?
  frame.data[6] = 0x0F;             // Engine speed signal quality bit. Was 0xD4 - does affect.  Bool
  frame.data[7] = get_lock_target_adjusted_value(MOTOR_12_counter, false); // Engine speed / RPM was 0xAE affects.  >30 slows down.  Only adds 5%. Was 0x10 - does affect

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A8); // for 0x0A8

  MOTOR_12_counter++;
  if (MOTOR_12_counter > 0x7F)
  {
    MOTOR_12_counter = 0x70;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR_11; // MOTOR_11 0x0A7
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;             // checksum placeholder
  frame.data[1] = MOTOR_11_counter; // rolling - 0x40>0x4F
  frame.data[2] = 0xFA;             // Raw target torque (unfiltered driver demand) (MO_Mom_Soll_Roh)
  frame.data[3] = 0xFA;             // Actual total torque output (Actual total torque output)
  frame.data[4] = 0x00;             // doesn't affect Total inertia torque component (MO_Mom_Traegheit_Summe)
  frame.data[5] = 0xFA;             // Filtered target torque (MO_Mom_Soll_gefiltert)
  frame.data[6] = get_lock_target_adjusted_value(0xFA, false);             // (MO_Mom_Soll_01?).  Massive effect.  Was 0x78
  frame.data[7] = get_lock_target_adjusted_value(0xFA, false);             // massive effect.  Was 0x3E

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0A7); // for 0x0A7

  MOTOR_11_counter++;
  if (MOTOR_11_counter > 0x4F)
  {
    MOTOR_11_counter = 0x40;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0xd2,0x3d,0xcd,0x28,0x4c,0x14,0x22,0x4b,0x24,0xac,0xfa,0x55,0x66,0x80,0x0d,0x6c
  */

  frame.identifier = ESP_14; // ESP_14 0x08A
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // checksum placeholder
  frame.data[1] = ESP_14_counter; // rolling - 0x10>0x1F
  frame.data[2] = 0x00;           // doesn't affect
  frame.data[3] = 0x00;           // doesn't affect sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x00;           // doesn't affect BR_Vorg_Quer_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
  frame.data[5] = 0x00;           // BR_Vorg_Quer_Max Maximum predefined limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
  frame.data[6] = 0x00;           // doesn't affect BR_Vorg_Allrad_Min Minimum specified limit value of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm
  frame.data[7] = get_lock_target_adjusted_value(0xFE, false);           // BR_Vorg_Allrad_Max Maximum specified limit of the clutch's operating range by the ESP MQB Haldex: 100% torque corresponds to 2000 Nm.
  // massive effects (4>7)

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_08A); // for 0x08A

  ESP_14_counter++;
  if (ESP_14_counter > 0x1F)
  {
    ESP_14_counter = 0x10;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);

  /*
  0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4,0xd4
  */

  frame.identifier = LWI_01; // LWI_01 0x086
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x00;           // checksum placeholder
  frame.data[1] = LWI_01_counter; // rolling - 0x10>0x1F
  frame.data[2] = 0x01;           // LWI_SensorStatus
  frame.data[3] = 0x00;           // LWI_Qbit_sub_daten
  frame.data[4] = 0x00;           // LWI_Qbit_Lendradwiken
  frame.data[5] = 0x00;           // LWI_lendradwinken
  frame.data[6] = 0x00;           // LWI_lendradw_geschw
  frame.data[7] = 0x00;           // LWI_lendradw_geschw Unit Degress of Arc per Second

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_086); // for 0x086

  LWI_01_counter++;
  if (LWI_01_counter > 0x1F)
  {
    LWI_01_counter = 0x10;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86,0x86
  */
}

void Gen5_frames20()
{
  twai_message_t frame;
  frame.identifier = MOTOR_20;      // MOTOR_20 0x121
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x00;             // checksum
  frame.data[1] = MOTOR_20_counter; // rolling - 0x00>0x0F && MO_Accelerator_Raw_Value_01!
  frame.data[2] = 0x40;             // no affect MO_Accelerator_Raw_Value_01 sss
  frame.data[3] = 0x40;             // no affect sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x19;             // no affect sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x59;             // no affect
  frame.data[6] = 0x7E;             // no affect
  frame.data[7] = 0xFE;             // no affect

  frame.data[0] = calcChecksum(frame.data, ID_SEQ_121); // for 0x121

  MOTOR_20_counter++;
  if (MOTOR_20_counter > 0x0F)
  {
    MOTOR_20_counter = 0x00;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0xe9,0x65,0xae,0x6b,0x7b,0x35,0xe5,0x5f,0x4e,0xc7,0x86,0xa2,0xbb,0xdd,0xeb,0xb4
  */

  frame.identifier = ESP_10;                // ESP_10 0x116
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = ESP_10_counter;           // rolling - 0x00>0x0F
  frame.data[2] = 0x01;                     // no affect all these affect, find which one
  frame.data[3] = 0x04;                     // no effect sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x00;                     // no effect sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x40;                     // no effect
  frame.data[6] = 0x00;                     // no effect
  frame.data[7] = get_lock_target_adjusted_value(ESP_10_counter, false);           // this affects(!) - a good 40%.  Was 0xFF
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_116); // for 0x116

  ESP_10_counter++;
  if (ESP_10_counter > 0x0F)
  {
    ESP_10_counter = 0x00;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac,0xac
  */

  frame.identifier = ESP_05;                // ESP_05 0x106
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = ESP_05_counter;           // rolling - 0x80>0x8F
  frame.data[2] = 0x64;                     // no effect
  frame.data[3] = 0xC0;                     // this affects(!) sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x00;                     // no effect sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x00;                     // no effect
  frame.data[6] = 0xFD;                     // no effect
  frame.data[7] = 0x00;                     // this affects(!) - on/off.  Was 0x10.  0x00 doesn't hurt
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_106); // for 0x106

  ESP_05_counter++;
  if (ESP_05_counter > 0x8F)
  {
    ESP_05_counter = 0x80;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07
  */

  frame.identifier = EPB_01;      // EPB_01 0x104
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // checksum
  frame.data[1] = EPB_01_counter; // rolling - 0x30>0x3F - none affect
  frame.data[2] = 0xA6;
  frame.data[3] = 0x00; // sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0xE6; // sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x31;
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_104); // for 0x104

  EPB_01_counter++;
  if (EPB_01_counter > 0x3F)
  {
    EPB_01_counter = 0x30;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05
  */

  frame.identifier = ESP_02;                // ESP_02 0x101
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = ESP_02_counter;           // rolling - 0x00>0x1F
  frame.data[2] = 0x7E;                     // doesn't effect one of these affects, find which one - doesn't affect
  frame.data[3] = 0x0F;                     // doesn't effect sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x82;                     // doesn't effect sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x0C;                     // doesn't effect rolling?
  frame.data[6] = 0x40;                     // doesn't efffect
  frame.data[7] = 0x00;                     // doesn't effect
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_101); // for 0x101

  ESP_02_counter++;
  if (ESP_02_counter > 0x1F)
  {
    ESP_02_counter = 0x00;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa
  */

  frame.identifier = ESP_21;      // ESP_21 0x0fd
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // checksum
  frame.data[1] = ESP_21_counter; // rolling - 0x00>0x1F
  frame.data[2] = 0x1F;           // in diagnosis? none affect
  frame.data[3] = 0x80;           // sometimes 0xC0, sometimes 0x00
  frame.data[4] = 0x00;           // sometimes 0x3A, somtimes 0x39
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_0fd); // for 0x0fd

  ESP_21_counter++;
  if (ESP_21_counter > 0x1F)
  {
    ESP_21_counter = 0x00;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);

  /*
  0xb4,0xef,0xf8,0x49,0x1e,0xe5,0xc2,0xc0,0x97,0x19,0x3c,0xc9,0xf1,0x98,0xd6,0x61
  */
}

void Gen5_frames25()
{
  twai_message_t frame;
  frame.identifier = KOMBI_01; // kombi 1 0x30b
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x10;        // angle of turn (block 011) low byte
  frame.data[1] = 0x20;        // checksum (0x20>0x2F)
  frame.data[2] = 0x02;        //
  frame.data[3] = 0x00;        //
  frame.data[4] = 0x0C;        //
  frame.data[5] = 0x00;        //
  frame.data[6] = 0x00;        //
  frame.data[7] = 0x24;        //
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen5_frames100()
{
  twai_message_t frame;
  frame.identifier = ESP_23;                // ESP_23 0x5be - this is fixed in Savvy but CHKS in Kmatrix?
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum placeholder no effect
  frame.data[1] = ESP_23_counter;           // ESP_23_counter;           // no effect B high byte
  frame.data[2] = 0xBF;                     // no effect C
  frame.data[3] = 0x7F;                     // no effect D
  frame.data[4] = 0x00;                     // rate of change (block 010)
  frame.data[5] = 0x00;                     // rate of change (block 010)
  frame.data[6] = 0x7C;                     // rate of change (block 010)
  frame.data[7] = 0x78;                     // rate of change (block 010)
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_5be); // for 0x5be

  ESP_23_counter++;
  if (ESP_23_counter > 0x1F)
  {
    ESP_23_counter = 0x00;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
    0xc9,0x21,0x6f,0x63,0xd2,0x42,0x6a,0x77,0x4a,0x3d,0xb0,0x62,0x9f,0x38,0xcd,0x5c
    */

  frame.identifier = Parkhilfe_04; // Parkhilfe_04 0x54B
  frame.data_length_code = 8;      // DLC 8
  frame.data[0] = 0x00;            // angle of turn (block 011) low byte
  frame.data[1] = 0x00;            // no effect B high byte
  frame.data[2] = 0x00;            // no effect C
  frame.data[3] = 0x00;            // no effect D
  frame.data[4] = 0x00;            // rate of change (block 010)
  frame.data[5] = 0x00;            // rate of change (block 010)
  frame.data[6] = 0x00;            // rate of change (block 010)
  frame.data[7] = 0x24;            // rate of change (block 010)
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = GATEWAY_72; // gateway 72 0x3db
  frame.data_length_code = 8;    // DLC 8
  frame.data[0] = 0x50;          //
  frame.data[1] = 0x80;          //
  frame.data[2] = 0x00;          //
  frame.data[3] = 0x00;          //
  frame.data[4] = 0x05;          //
  frame.data[5] = 0x10;          //
  frame.data[6] = 0x01;          //
  frame.data[7] = 0x78;          //
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = GETRIEBE_14; // getriebe 14 0x3c8
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x00;           // Maximum possible acceleration (limited by gear/clutch)
  frame.data[1] = 0x00;           // Charisma drive programme selected (affects shift mapping)
  frame.data[2] = 0x54;           // Charisma system status
  frame.data[3] = 0x24;           // Drag/friction loss torque in transmission
  frame.data[4] = 0x00;           // Launch control active
  frame.data[5] = 0x60;           //
  frame.data[6] = 0x01;           //
  frame.data[7] = 0x51;           //
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR_14;              // motor 14 0x3be
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = MOTOR_14_counter;         // 0x10 to 0x1F
  frame.data[2] = 0xE6;                     // doesn't effect Start/stop system state (0=inactive, 1=stopping, 2=stopped, 3=restarting)
  frame.data[3] = 0x01;                     // this affects(!) on/off Restart event flag
  frame.data[4] = 0xC8;                     // doesn't effect Engine stop event flag
  frame.data[5] = 0x80;                     // doesn't effect
  frame.data[6] = 0x00;                     // doesn't effect
  frame.data[7] = 0x80;                     // doesn't effect
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_3be); // for 0x3be

  MOTOR_14_counter++;
  if (MOTOR_14_counter > 0x1F)
  {
    MOTOR_14_counter = 0x10;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0x1f,0x28,0xc6,0x85,0xe6,0xf8,0xb0,0x19,0x5b,0x64,0x35,0x21,0xe4,0xf7,0x9c,0x24
  */

  frame.identifier = ESP_07;                // ESP_07 0x392
  frame.data_length_code = 8;               // DLC 8
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = ESP_07_counter;           // 0x20>0x2F
  frame.data[2] = 0x00;                     // one of these affects, find which one
  frame.data[3] = 0x00;                     // no effect
  frame.data[4] = 0x00;                     // no effect
  frame.data[5] = 0x00;                     // no efefct
  frame.data[6] = 0x00;                     // no effect
  frame.data[7] = 0x00;                     // no effect
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_392); // for 0x392

  ESP_07_counter++;
  if (ESP_07_counter > 0x1F)
  {
    ESP_07_counter = 0x00;
  }
  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
   0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91,0x91
   */

  frame.identifier = ESP_29;  // esp_29 0x18c
  frame.data_length_code = 8; // DLC 8
  frame.data[0] = 0x00;       //
  frame.data[1] = 0x20;       // checksum (0x20>0x2F)?  Not in Savvy
  frame.data[2] = 0x59;       //
  frame.data[3] = 0x00;       //
  frame.data[4] = 0x00;       //
  frame.data[5] = 0x00;       //
  frame.data[6] = 0x00;       //
  frame.data[7] = 0x00;       //
  twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen5_frames200()
{
  twai_message_t frame;
  frame.identifier = mKombi_2; // electronic power steering 0x0C2
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4C;        // angle of turn (block 011) low byte
  frame.data[1] = 0x86;        // no effect B high byte
  frame.data[2] = 0x85;        // no effect C
  frame.data[3] = 0x00;        // no effect D
  frame.data[4] = 0x00;        // rate of change (block 010)
  frame.data[5] = 0x30;        // rate of change (block 010)
  frame.data[6] = 0xFF;        // rate of change (block 010)
  frame.data[7] = 0x04;        // rate of change (block 010)
  // twai_transmit_v2(twai_bus_1, &frame, 0);
}

void Gen5_frames1000()
{
  twai_message_t frame;
  frame.identifier = MOTOR_07; // motor 07, 1000ms
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0xA0;        // no effect from any
  frame.data[1] = 0x5A;        //
  frame.data[2] = 0x56;        //
  frame.data[3] = 0xA3;        //
  frame.data[4] = 0x80;        //
  frame.data[5] = 0xA0;        //
  frame.data[6] = 0x59;        //
  frame.data[7] = 0x01;        //
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = CHARISMA_01; // charisma_01 0x385
  frame.data_length_code = 8;     // DLC 8 no effect from any
  frame.data[0] = 0x00;           // CHA_Target_Driving_Program_AGA & CHA_Target_Driving_Prior_ESP
  frame.data[1] = 0x00;           // CHA_Target_Driving_Pri_Freewheel & void
  frame.data[2] = 0x22;           // CHA_Target_Driving_Program_MO & CHA_Target_Driving_Program_GE
  frame.data[3] = 0x02;           // CHA_Target_Driving_PR_ALR (inc. AWD) & CHA_Target_Driving_Program_MO_BZS
  frame.data[4] = 0x02;           // CHA_Target_Driving_Project_DR & CHA_Target_Driving_Prior_VAQ
  frame.data[5] = 0x20;           // CHA_Target_Driving_PR_AFS & CHA_Target_Driving_Program_RGS
  frame.data[6] = 0x02;           // CHA_Target_Driving_Price_EPS & CHA_Target_Driving_Principal_ACC
  frame.data[7] = 0x02;           // CHA_Target_Driving_Prior_SAK & CHA_Target_Driving_Program_MO_StSt
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = SYSTEMINFO_01; // systeminfo_01 0x585
  frame.data_length_code = 8;       // DLC 8
  frame.data[0] = 0x84;             //
  frame.data[1] = 0x3C;             //
  frame.data[2] = 0x00;             //
  frame.data[3] = 0x7F;             //
  frame.data[4] = 0x14;             //
  frame.data[5] = 0x00;             //
  frame.data[6] = 0x00;             //
  frame.data[7] = 0x00;             //
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = MOTOR_CODE_01;         // motor_code_01 0x641
  frame.data_length_code = 8;               // DLC 8 no affect from any
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = MOTOR_CODE_01_counter;    // rolling (10>1F)
  frame.data[2] = 0x2B;                     //
  frame.data[3] = 0x53;                     //
  frame.data[4] = 0x14;                     //
  frame.data[5] = 0x14;                     //
  frame.data[6] = 0xD7;                     //
  frame.data[7] = 0x24;                     //
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_641); // for 0x641

  MOTOR_CODE_01_counter++;
  if (MOTOR_CODE_01_counter > 0x1F)
  {
    MOTOR_CODE_01_counter = 0x10;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47
  */

  frame.identifier = ESP_20;                // esp_20 0x65d
  frame.data_length_code = 8;               // DLC 8 no affect from any
  frame.data[0] = 0x00;                     // checksum
  frame.data[1] = ESP_20_counter;           // rolling (30>3F)
  frame.data[2] = 0x2B;                     // no effect C
  frame.data[3] = 0x10;                     // no effect D
  frame.data[4] = 0x00;                     //
  frame.data[5] = 0x00;                     //
  frame.data[6] = 0xE2;                     //
  frame.data[7] = 0x79;                     // BR_Tire circumference
  frame.data[0] = calcChecksum(frame.data, ID_SEQ_65d); // for 0x65d

  ESP_20_counter++;
  if (ESP_20_counter > 0x3F)
  {
    ESP_20_counter = 0x30;
  }

  twai_transmit_v2(twai_bus_1, &frame, 0);
  /*
  0xac,0xb3,0xab,0xeb,0x7a,0xe1,0x3b,0xf7,0x73,0xba,0x7c,0x9e,0x06,0x5f,0x02,0xd9
  */

  frame.identifier = DIAGNOSE_01; // diagnose_01 0x6b2
  frame.data_length_code = 8;     // DLC 8
  frame.data[0] = 0x30;           //
  frame.data[1] = 0x4D;           //
  frame.data[2] = 0x58;           //
  frame.data[3] = 0xA2;           //
  frame.data[4] = 0x89;           //
  frame.data[5] = 0x85;           //
  frame.data[6] = 0x3F;           // 0x3F OR 0xBF? (3F, then BF, then 3F, then BF...)
  frame.data[7] = 0x30;           // 2D, then 2D, then 2E, then 2E, then 2F, then 2F... roll over? When?
  twai_transmit_v2(twai_bus_1, &frame, 0);

  frame.identifier = KOMBI_02; // kombi2 0x6b7
  frame.data_length_code = 8;  // DLC 8
  frame.data[0] = 0x4D;        // no effect from any
  frame.data[1] = 0x58;        //
  frame.data[2] = 0xF2;        //
  frame.data[3] = 0xEE;        //
  frame.data[4] = 0x04;        //
  frame.data[5] = 0x2B;        //
  frame.data[6] = 0x00;        //
  frame.data[7] = 0x78;        //
  twai_transmit_v2(twai_bus_1, &frame, 0);
}