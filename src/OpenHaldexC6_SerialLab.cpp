#include <OpenHaldexC6_defs.h>
#include <OpenHaldexC6_Calculations.h>
#include <OpenHaldexC6_Analyzer.h>
#include <OpenHaldexC6_can.h> // canTransmit (serial-lab UDS transactions on Bus 1)

// =============================================================================
// Serial Lab - USB diagnostic brute-forcer
// =============================================================================
// Py-based control + telemetry over the USB CDC port so a host script can
// change one value at a time and watch what the Haldex actually does, in real
// time without reflashing per experiment. Built to answer questions the code
// alone can't: which byte/value moves engagement, why lock fades from 100%,
// what a given torque ceiling really delivers.
//
// Telemetry (one line per interval, CSV after the tag):
//   BPK,<t_ms>,<ceiling_nm>,<floor_pct>,<lock_target>,<cf>,<torque_nm>,
//       <ist_nm>,<solf_nm>,<engagement>,<raw_engagement>,<fixhunt>,<qbit>,
//       <mode>,<speed>,<standalone>,<hdx_can>,<chs_can>
//
// Commands (ASCII, newline-terminated, case-insensitive):
//   PING                -> OK,PING
//   GET                 -> one STAT line (same fields as BPK, tagged STAT)
//   LOG <ms>            stream telemetry every <ms> (0 = off, min 20)
//   CEIL <10..500>      bpkCeilingNm
//   FLOOR <0..100>      esp14MinFloorPct
//   FIXHUNT <0|1>       Motor_11 BPK packing on/off
//   QBIT <0|1>          Motor_11 MO_QBit_Motormomente (0x20 vs 0xA0)
//   MODE <0..5>         state.mode (Stock/FWD/50:50/60:40/75:25/Expert)
//   CF <0..100>         drive the raw lock request directly (learn-style),
//                       bypassing mode/lock_target - the lever for sweeps
//   CF OFF              release the direct request, back to normal control
//   GEN <n>             haldexGeneration live (1,2,4,41,50,51,52) - not persisted
//   SA <0|1>            isStandalone live (frame tasks resumed/suspended) - not persisted
//   RXIDS / RXCLR       census of every CAN ID seen on Bus 1 (what the module sends)
//   DIAGID <req> <rsp>  pin the UDS request/response pair (hex); DIAGID AUTO = by generation
//   UDSRAW <hex bytes>  one UDS request to the module, reply as UDSR,<hex> (multi-frame ok)
//   DTC                 ReadDTCInformation (0x19 02) -> DTCR lines + OK,DTC,<n>
//   DTCCLR              ClearDiagnosticInformation (0x14 FF FF FF)
//   SESS <1|3>          DiagnosticSessionControl;  TP = TesterPresent
//   WSLR <raw>          front left-vs-right wheel-speed split (VL - VR), VAQ lever
//   TXADD <id> <ms> <bytes..> [MAGIC <hex>]   extra periodic frame (bench); TXDEL / TXCLR / TXS
//   HELP                -> list of commands
// Telemetry also carries an FB line per sample: the raw feedback frame the
// per-generation decoder last accepted (0x118 / 0x137 / 0x2C0 ...), its age,
// and the decoded engagement/state - what the module said, next to what we sent.
// Every command answers with a single OK,... or ERR,... line so a host can
// wait for a deterministic reply rather than guessing at timing.
//
// NOTE: SavvyCAN's serial GVRET mode (analyzerSerial) also owns the serial
// port and speaks binary. This task stands down entirely while that is on, so
// the two can't fight over Serial.read().
// =============================================================================

static uint32_t labLogIntervalMs = 0; // 0 = telemetry off
static uint32_t labLastEmitMs = 0;
static char labLine[96];
static uint8_t labLineLen = 0;

static void labPrintStat(const char *tag)
{
  // Snapshot once so every field in a line belongs to the same instant.
  const uint32_t t = millis();
  // Haldex-reported status bits packed into one field, so a dropout can be
  // attributed rather than guessed at: b0 temp protection, b1 coupling open,
  // b2 speed limit, b3 limp, b4 AWD warning, b5/b6 clutch reports.
  const uint8_t flags =
      (uint8_t)((received_temp_protection ? 0x01 : 0) |
                (received_coupling_open ? 0x02 : 0) |
                (received_speed_limit ? 0x04 : 0) |
                (received_limp_mode ? 0x08 : 0) |
                (received_awd_warning ? 0x10 : 0) |
                (received_report_clutch1 ? 0x20 : 0) |
                (received_report_clutch2 ? 0x40 : 0));

  Serial.printf("%s,%lu,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
                ",%u,%u,%u,%u,%u,%u,%u,%d,%d"
                ",%02X%02X%02X%02X%02X%02X%02X%02X\n",
                tag,
                (unsigned long)t,
                (unsigned)bpkCeilingNm,
                (unsigned)esp14MinFloorPct,
                (int)lock_target,
                (unsigned)haldexLearnCF,
                (unsigned)bpkLastTorqueNm,
                (unsigned)bpkLastIstNm,
                (unsigned)bpkLastSolfNm,
                (unsigned)received_haldex_engagement,
                (unsigned)received_haldex_engagement_raw,
                (unsigned)(fixHunting ? 1 : 0),
                (unsigned)((bpkStatusFl & 0x80) ? 1 : 0),
                (unsigned)state.mode,
                (unsigned)received_vehicle_speed,
                (unsigned)(isStandalone ? 1 : 0),
                (unsigned)(hasCANHaldex ? 1 : 0),
                (unsigned)(hasCANChassis ? 1 : 0),
                // extended fields
                (unsigned)flags,
                (unsigned)received_long_lock_state,
                (unsigned)bpkFloorNm,
                (unsigned)bpkSlewIst,
                (unsigned)bpkSlewSolf,
                (unsigned)bpkStatusFl,
                (unsigned)(wsFreeze ? 1 : 0),
                (int)wsBaseRaw,
                (int)wsFrontDeltaRaw,
                bpkLastFrame[0], bpkLastFrame[1], bpkLastFrame[2], bpkLastFrame[3],
                bpkLastFrame[4], bpkLastFrame[5], bpkLastFrame[6], bpkLastFrame[7]);

  // UDS live-diagnostics tail on its own line, tagged so the host can pair it
  // with the sample above. Only meaningful while liveDiagEnabled and Gen5;
  // values hold their last polled reading otherwise.
  // Trailing fields: session in use (0 idle / 1 default / 3 extended) and the
  // canTransmit() drop counters, so a lab run can correlate faults with the
  // diagnostic channel state and bridge health.
  Serial.printf("UDS,%lu,%u,%.3f,%.2f,%.2f,%.2f,%.3f,%u,%u,%lu,%lu\n",
                (unsigned long)t,
                (unsigned)udsClutchPWM,
                (double)udsClutchCurrent,
                (double)udsClutchTemp,
                (double)udsCoolingFinTemp,
                (double)udsModuleTemp,
                (double)udsClutchVoltage,
                (unsigned)(liveDiagEnabled ? 1 : 0),
                (unsigned)udsSessionMode,
                (unsigned long)canTxDropBus0,
                (unsigned long)canTxDropBus1);

  // Module feedback: last accepted engagement frame, raw, plus what was decoded
  // from it. Age lets a host tell "module went quiet" from "module reports 0".
  //   FB,<t>,<id hex>,<dlc>,<8 bytes hex>,<age_ms>,<eng%>,<eng_raw>,<quer_state>,<quer_sync>,<long_lock_state>,<hdx_state>
  const uint32_t fbMs = hdxFbMs;
  Serial.printf("FB,%lu,%03lX,%u,%02X%02X%02X%02X%02X%02X%02X%02X,%lu,%u,%u,%u,%u,%u,%u\n",
                (unsigned long)t,
                (unsigned long)hdxFbId,
                (unsigned)hdxFbDlc,
                hdxFbData[0], hdxFbData[1], hdxFbData[2], hdxFbData[3],
                hdxFbData[4], hdxFbData[5], hdxFbData[6], hdxFbData[7],
                (unsigned long)(fbMs ? (t - fbMs) : 0xFFFFFFFFUL),
                (unsigned)received_haldex_engagement,
                (unsigned)received_haldex_engagement_raw,
                (unsigned)received_quer_state,
                (unsigned)received_quer_sync,
                (unsigned)received_long_lock_state,
                (unsigned)received_haldex_state);
}

// ---------------------------------------------------------------------------
// Serial-lab UDS transactions (Bus 1, module under test)
// ---------------------------------------------------------------------------
// One ISO-TP request/response against udsHaldexReqId/udsHaldexRespId. Replies
// arrive through the /api/uds/read copy-tap in parseCAN_hdx (udsWebRxQueue),
// which also keeps them off Bus 0 - the same plumbing the web endpoint uses,
// so the two share its one-at-a-time rule (udsWebRespId doubles as the busy
// flag). Handles SF and FF/CF replies, sends the flow control itself, and
// waits through 0x78 responsePending. Refuses to run while the live-diag
// poller owns the channel: two testers interleaving on one ECU would give
// answers attributed to the wrong question.
static uint8_t labUdsBuf[256];

static int labUdsTransact(const uint8_t *req, size_t reqLen, uint8_t *rsp, size_t cap,
                          size_t &rspLen, uint32_t timeoutMs)
{
  rspLen = 0;
  if (reqLen == 0 || reqLen > 7)
    return -5;
  if (udsWebRespId != 0)
    return -1; // web read in flight
  if (udsPollActive)
    return -2; // live-diag poller owns the channel (UDS 0 first)
  if (udsWebRxQueue == nullptr)
    udsWebRxQueue = xQueueCreate(8, sizeof(twai_message_t));
  if (udsWebRxQueue == nullptr)
    return -3;
  xQueueReset(udsWebRxQueue);
  udsWebBus = 1;
  udsWebRespId = udsHaldexRespId;

  twai_message_t m{};
  m.identifier = udsHaldexReqId;
  m.data_length_code = 8;
  m.data[0] = (uint8_t)reqLen; // SF PCI
  memcpy(&m.data[1], req, reqLen);
  for (size_t i = reqLen + 1; i < 8; i++)
    m.data[i] = 0xAA;
  if (!canTransmit(twai_bus_1, &m))
  {
    udsWebRespId = 0;
    return -4;
  }

  int rc = -6; // timeout unless something lands
  uint32_t deadline = millis() + timeoutMs;
  twai_message_t f;
  uint16_t total = 0;
  size_t got = 0;
  bool multi = false;
  while ((int32_t)(deadline - millis()) > 0)
  {
    const uint32_t rem = deadline - millis();
    if (xQueueReceive(udsWebRxQueue, &f, pdMS_TO_TICKS(rem < 50 ? rem : 50)) != pdTRUE)
      continue;
    const uint8_t pci = f.data[0] & 0xF0;
    if (!multi)
    {
      if (pci == 0x00)
      {
        size_t n = f.data[0] & 0x0F;
        if (n >= 3 && f.data[1] == 0x7F && f.data[3] == 0x78)
        {
          deadline = millis() + timeoutMs; // responsePending - the real answer follows
          continue;
        }
        if (n > cap)
          n = cap;
        memcpy(rsp, &f.data[1], n);
        rspLen = n;
        rc = 0;
        break;
      }
      if (pci == 0x10)
      {
        total = (uint16_t)(((f.data[0] & 0x0F) << 8) | f.data[1]);
        size_t n = total < 6 ? total : 6;
        if (n > cap)
          n = cap;
        memcpy(rsp, &f.data[2], n);
        got = n;
        multi = true;
        twai_message_t fc{};
        fc.identifier = udsHaldexReqId;
        fc.data_length_code = 8;
        fc.data[0] = 0x30; // CTS
        fc.data[1] = 0x00; // BS: all
        fc.data[2] = 0x00; // STmin 0
        for (uint8_t i = 3; i < 8; i++)
          fc.data[i] = 0xAA;
        canTransmit(twai_bus_1, &fc);
        if (got >= total || got >= cap)
        {
          rspLen = got;
          rc = 0;
          break;
        }
        continue;
      }
      continue; // stray FC / CF - not ours
    }
    if (pci != 0x20)
      continue;
    size_t n = (size_t)total - got;
    if (n > 7)
      n = 7;
    if (got + n > cap)
      n = cap - got;
    memcpy(rsp + got, &f.data[1], n);
    got += n;
    if (got >= total || got >= cap)
    {
      rspLen = got;
      rc = 0;
      break;
    }
  }
  udsWebRespId = 0;
  return rc;
}

static const char *labUdsErr(int rc)
{
  switch (rc)
  {
  case -1: return "busy (web UDS read in flight)";
  case -2: return "live-diag poller owns the channel - send UDS 0 first";
  case -3: return "no memory for rx queue";
  case -4: return "CAN transmit failed on Bus 1";
  case -5: return "request must be 1..7 bytes";
  case -6: return "timeout (no reply on response ID)";
  default: return "unknown";
  }
}

static void labPrintHex(const uint8_t *b, size_t n)
{
  for (size_t i = 0; i < n; i++)
    Serial.printf("%02X", b[i]);
}

// Parse "22 F1 87" or "22F187" into bytes. Returns count, 0 on a bad token.
static size_t labParseHexBytes(const char *s, uint8_t *out, size_t cap)
{
  char buf[64];
  size_t k = 0;
  for (; *s && k < sizeof(buf) - 1; s++)
    if (*s != ' ' && *s != ',')
      buf[k++] = *s;
  buf[k] = '\0';
  if (k == 0 || (k & 1))
    return 0;
  size_t n = 0;
  for (size_t i = 0; i < k && n < cap; i += 2)
  {
    char h[3] = {buf[i], buf[i + 1], '\0'};
    char *end = nullptr;
    long v = strtol(h, &end, 16);
    if (end != h + 2)
      return 0;
    out[n++] = (uint8_t)v;
  }
  return n;
}

// ReadDTCInformation / reportDTCByStatusMask. Walks a few status masks because
// implementations differ in which ones they accept (0xFF everything, 0x09 /
// 0x08 confirmed, 0xAE the VW tester default). One DTCR line per record:
//   DTCR,<3-byte DTC hex>,<status hex>,<SAE code>-<failure type>
static void labDtcRead()
{
  static const uint8_t masks[] = {0xFF, 0xAE, 0x09, 0x08, 0x01};
  for (uint8_t mi = 0; mi < sizeof(masks); mi++)
  {
    const uint8_t req[3] = {0x19, 0x02, masks[mi]};
    size_t n = 0;
    const int rc = labUdsTransact(req, sizeof(req), labUdsBuf, sizeof(labUdsBuf), n, 1500);
    if (rc < 0)
    {
      Serial.printf("ERR,DTC,%s\n", labUdsErr(rc));
      return;
    }
    if (n >= 3 && labUdsBuf[0] == 0x7F)
    {
      const uint8_t nrc = labUdsBuf[2];
      // 0x31 requestOutOfRange / 0x12 subFunctionNotSupported: try the next mask.
      if ((nrc == 0x31 || nrc == 0x12) && mi + 1 < sizeof(masks))
        continue;
      Serial.printf("ERR,DTC,NRC 0x%02X (mask 0x%02X)\n", nrc, masks[mi]);
      return;
    }
    if (n < 3 || labUdsBuf[0] != 0x59 || labUdsBuf[1] != 0x02)
    {
      Serial.print("ERR,DTC,unexpected reply ");
      labPrintHex(labUdsBuf, n);
      Serial.println();
      return;
    }
    const uint8_t avail = labUdsBuf[2];
    uint16_t count = 0;
    for (size_t i = 3; i + 4 <= n; i += 4)
    {
      const uint8_t b0 = labUdsBuf[i], b1 = labUdsBuf[i + 1], b2 = labUdsBuf[i + 2], st = labUdsBuf[i + 3];
      static const char letters[4] = {'P', 'C', 'B', 'U'};
      Serial.printf("DTCR,%02X%02X%02X,%02X,%c%u%X%02X-%02X\n", b0, b1, b2, st,
                    letters[(b0 >> 6) & 0x03], (unsigned)((b0 >> 4) & 0x03), (unsigned)(b0 & 0x0F), b1, b2);
      count++;
    }
    Serial.printf("OK,DTC,%u,0x%02X,0x%02X\n", (unsigned)count, masks[mi], avail);
    return;
  }
}

// One request, reply printed raw. Negative responses come back as UDSR too
// (7F <sid> <nrc>) so a host can see exactly what the module said.
static void labUdsRawCmd(const uint8_t *req, size_t reqLen, uint32_t timeoutMs)
{
  size_t n = 0;
  const int rc = labUdsTransact(req, reqLen, labUdsBuf, sizeof(labUdsBuf), n, timeoutMs);
  if (rc < 0)
  {
    Serial.printf("ERR,UDS,%s\n", labUdsErr(rc));
    return;
  }
  Serial.print("UDSR,");
  labPrintHex(labUdsBuf, n);
  Serial.println();
}

static void labSetStandalone(bool on)
{
  isStandalone = on;
  // Mirror /api/settings: the frame tasks only run while standalone.
  TaskHandle_t hs[] = {handle_frames1000, handle_frames200, handle_frames100, handle_frames25,
                       handle_frames20, handle_frames10, handle_frames13, handle_frames50,
                       handle_frames250, handle_gen41_dual_bus_rates};
  for (TaskHandle_t h : hs)
  {
    if (h == nullptr)
      continue;
    if (on)
      vTaskResume(h);
    else
      vTaskSuspend(h);
  }
}

// Direct lock request. Piggybacks the learn plumbing (haldexLearnActive makes
// every lock-derived byte scale by haldexLearnCF/100), which is exactly what
// the learn sweeps already use - so a CF here produces the same CAN output a
// sweep would at that step, without running a whole sweep.
static void labSetCF(int cf)
{
  if (cf < 0)
  {
    haldexLearnActive = false;
    haldexLearnCF = 0;
    haldexLearnStep = 0;
    return;
  }
  haldexLearnCancel = false;
  haldexLearnActive = true;
  haldexLearnCF = (uint8_t)cf;
  haldexLearnStep = (uint8_t)cf;
}

static bool labArgInt(const char *s, int &out)
{
  while (*s == ' ')
    s++;
  if (*s == '\0')
    return false;
  char *end = nullptr;
  const long v = strtol(s, &end, 10);
  if (end == s)
    return false;
  out = (int)v;
  return true;
}

// ---------------------------------------------------------------------------
// Extra periodic frames (bench only). Transmit an ID the standalone set does
// not generate - e.g. Klemmen_Status_01 0x3C0 - to test whether the module
// under test wants it, without a firmware change per candidate.
//   TXADD <id hex> <period ms> <b0..b7 hex> [MAGIC <hex>]
//     With MAGIC: b1 low nibble rolls 0..15 on every send and b0 is the VW
//     E2E CRC-8 (B1..B7 then the constant DataID), like ESP_14 / LWI_01.
//   TXDEL <id hex>   TXCLR   TXS
// ---------------------------------------------------------------------------
#define LAB_EXTRA_MAX 4
struct LabExtraFrame
{
  uint32_t id;
  uint16_t periodMs;
  uint8_t dlc;
  uint8_t data[8];
  int16_t magic; // -1 = raw bytes as given, else CRC + rolling counter
  uint8_t ctr;
  uint32_t lastMs;
};
static LabExtraFrame labExtra[LAB_EXTRA_MAX] = {};

static void labExtraSend(uint32_t now)
{
  for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++)
  {
    LabExtraFrame &e = labExtra[i];
    if (e.id == 0 || now - e.lastMs < e.periodMs)
      continue;
    e.lastMs = now;
    twai_message_t m = {};
    m.identifier = e.id;
    m.extd = e.id > 0x7FF;
    m.data_length_code = e.dlc;
    for (uint8_t b = 0; b < 8; b++)
      m.data[b] = e.data[b];
    if (e.magic >= 0 && e.dlc >= 2)
    {
      m.data[1] = (uint8_t)((e.data[1] & 0xF0) | (e.ctr & 0x0F));
      e.ctr = (uint8_t)((e.ctr + 1) & 0x0F);
      // CRC covers B1..B(dlc-1) then the DataID - short frames (DLC 4, e.g.
      // Klemmen_Status_01) are not zero-padded to 8.
      uint8_t in[8];
      uint8_t n = 0;
      for (uint8_t b = 1; b < e.dlc; b++)
        in[n++] = m.data[b];
      in[n++] = (uint8_t)e.magic;
      m.data[0] = crc8_autosar(in, n);
    }
    canTransmit(twai_bus_1, &m);
  }
}

static void labExtraCmd(const char *verb, char *arg)
{
  if (strcmp(verb, "TXCLR") == 0)
  {
    for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++) labExtra[i].id = 0;
    Serial.println("OK,TXCLR");
    return;
  }
  if (strcmp(verb, "TXS") == 0)
  {
    for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++)
      if (labExtra[i].id)
      {
        Serial.printf("TXS,%03lX,%u,%u,", (unsigned long)labExtra[i].id, (unsigned)labExtra[i].periodMs,
                      (unsigned)labExtra[i].dlc);
        for (uint8_t b = 0; b < 8; b++) Serial.printf("%02X", labExtra[i].data[b]);
        if (labExtra[i].magic >= 0) Serial.printf(",MAGIC %02X", (unsigned)labExtra[i].magic);
        Serial.println();
      }
    Serial.println("OK,TXS");
    return;
  }
  char *tok = strtok(arg, " ");
  if (!tok) { Serial.printf("ERR,%s needs <id hex> ...\n", verb); return; }
  const uint32_t id = (uint32_t)strtoul(tok, nullptr, 16);
  if (id == 0 || id > 0x1FFFFFFF) { Serial.printf("ERR,%s bad id\n", verb); return; }
  if (strcmp(verb, "TXDEL") == 0)
  {
    for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++)
      if (labExtra[i].id == id) labExtra[i].id = 0;
    Serial.printf("OK,TXDEL,%03lX\n", (unsigned long)id);
    return;
  }
  // TXADD
  tok = strtok(nullptr, " ");
  const int period = tok ? atoi(tok) : 0;
  if (period < 10 || period > 10000) { Serial.println("ERR,TXADD period 10..10000 ms"); return; }
  LabExtraFrame e = {};
  e.id = id;
  e.periodMs = (uint16_t)period;
  e.magic = -1;
  while ((tok = strtok(nullptr, " ")) != nullptr)
  {
    if (strcmp(tok, "MAGIC") == 0)
    {
      tok = strtok(nullptr, " ");
      if (!tok) { Serial.println("ERR,TXADD MAGIC needs a hex byte"); return; }
      e.magic = (int16_t)(strtoul(tok, nullptr, 16) & 0xFF);
      continue;
    }
    if (e.dlc >= 8) { Serial.println("ERR,TXADD more than 8 data bytes"); return; }
    e.data[e.dlc++] = (uint8_t)strtoul(tok, nullptr, 16);
  }
  if (e.dlc == 0) { Serial.println("ERR,TXADD needs data bytes"); return; }
  int slot = -1;
  for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++)
    if (labExtra[i].id == id) { slot = i; break; }
  if (slot < 0)
    for (uint8_t i = 0; i < LAB_EXTRA_MAX; i++)
      if (labExtra[i].id == 0) { slot = i; break; }
  if (slot < 0) { Serial.println("ERR,TXADD no free slots (TXCLR first)"); return; }
  labExtra[slot] = e;
  Serial.printf("OK,TXADD,%03lX,%u,%u%s\n", (unsigned long)id, (unsigned)e.periodMs, (unsigned)e.dlc,
                e.magic >= 0 ? ",CRC" : "");
}

static void labHandleLine(char *line)
{
  // Split verb / argument.
  char *arg = line;
  while (*arg && *arg != ' ')
    arg++;
  if (*arg == ' ')
    *arg++ = '\0';
  for (char *p = line; *p; p++)
    *p = (char)toupper((unsigned char)*p);

  int v = 0;

  if (strcmp(line, "PING") == 0)
  {
    Serial.println("OK,PING");
  }
  else if (strcmp(line, "GET") == 0)
  {
    labPrintStat("STAT");
  }
  else if (strcmp(line, "LOG") == 0)
  {
    if (!labArgInt(arg, v))
    {
      Serial.println("ERR,LOG needs ms");
      return;
    }
    labLogIntervalMs = (v <= 0) ? 0 : (uint32_t)max(v, 20);
    Serial.printf("OK,LOG,%lu\n", (unsigned long)labLogIntervalMs);
  }
  else if (strcmp(line, "CEIL") == 0)
  {
    if (!labArgInt(arg, v))
    {
      Serial.println("ERR,CEIL needs Nm");
      return;
    }
    bpkCeilingNm = (uint16_t)constrain(v, 10, 500);
    Serial.printf("OK,CEIL,%u\n", (unsigned)bpkCeilingNm);
  }
  else if (strcmp(line, "FLOOR") == 0)
  {
    if (!labArgInt(arg, v))
    {
      Serial.println("ERR,FLOOR needs %");
      return;
    }
    esp14MinFloorPct = (uint8_t)constrain(v, 0, 100);
    Serial.printf("OK,FLOOR,%u\n", (unsigned)esp14MinFloorPct);
  }
  else if (strcmp(line, "FIXHUNT") == 0)
  {
    if (!labArgInt(arg, v))
    {
      Serial.println("ERR,FIXHUNT needs 0|1");
      return;
    }
    fixHunting = (v != 0);
    Serial.printf("OK,FIXHUNT,%u\n", (unsigned)(fixHunting ? 1 : 0));
  }
  else if (strcmp(line, "QBIT") == 0)
  {
    if (!labArgInt(arg, v))
    {
      Serial.println("ERR,QBIT needs 0|1");
      return;
    }
    // MO_QBit_Motormomente is bit 63 = 0x80 of the byte-7 status field.
    bpkStatusFl = (uint8_t)(v ? (bpkStatusFl | 0x80) : (bpkStatusFl & ~0x80));
    Serial.printf("OK,QBIT,%u,STATUS,0x%02X\n", (unsigned)(v ? 1 : 0), (unsigned)bpkStatusFl);
  }
  else if (strcmp(line, "BPKFLOOR") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,BPKFLOOR needs Nm"); return; }
    bpkFloorNm = (uint16_t)constrain(v, 0, 500);
    Serial.printf("OK,BPKFLOOR,%u\n", (unsigned)bpkFloorNm);
  }
  else if (strcmp(line, "SLEWIST") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,SLEWIST needs Nm/cycle (0=instant)"); return; }
    bpkSlewIst = (uint16_t)constrain(v, 0, 1000);
    Serial.printf("OK,SLEWIST,%u\n", (unsigned)bpkSlewIst);
  }
  else if (strcmp(line, "SLEWSOLF") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,SLEWSOLF needs Nm/cycle (0=instant)"); return; }
    bpkSlewSolf = (uint16_t)constrain(v, 0, 1000);
    Serial.printf("OK,SLEWSOLF,%u\n", (unsigned)bpkSlewSolf);
  }
  else if (strcmp(line, "TRAEG") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,TRAEG needs raw 0..1023 (509=0Nm)"); return; }
    bpkTraegRaw = (uint16_t)constrain(v, 0, 1023);
    Serial.printf("OK,TRAEG,%u\n", (unsigned)bpkTraegRaw);
  }
  else if (strcmp(line, "SCHUB") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,SCHUB needs raw 0..511 (509=0Nm)"); return; }
    bpkSchubRaw = (uint16_t)constrain(v, 0, 511);
    Serial.printf("OK,SCHUB,%u\n", (unsigned)bpkSchubRaw);
  }
  else if (strcmp(line, "STATUS") == 0)
  {
    // Accepts decimal or 0x-prefixed hex for the whole byte-7 status field.
    while (*arg == ' ') arg++;
    long sv = strtol(arg, nullptr, 0);
    if (arg[0] == '\0') { Serial.println("ERR,STATUS needs a byte e.g. 0xA0"); return; }
    bpkStatusFl = (uint8_t)(sv & 0xFF);
    Serial.printf("OK,STATUS,0x%02X\n", (unsigned)bpkStatusFl);
  }
  else if (strcmp(line, "FORCEIST") == 0)
  {
    while (*arg == ' ') arg++;
    if (strncasecmp(arg, "AUTO", 4) == 0) { bpkForceIstNm = -1; Serial.println("OK,FORCEIST,AUTO"); return; }
    if (!labArgInt(arg, v)) { Serial.println("ERR,FORCEIST needs Nm or AUTO"); return; }
    bpkForceIstNm = constrain(v, 0, 500);
    Serial.printf("OK,FORCEIST,%d\n", (int)bpkForceIstNm);
  }
  else if (strcmp(line, "FORCESOLF") == 0)
  {
    while (*arg == ' ') arg++;
    if (strncasecmp(arg, "AUTO", 4) == 0) { bpkForceSolfNm = -1; Serial.println("OK,FORCESOLF,AUTO"); return; }
    if (!labArgInt(arg, v)) { Serial.println("ERR,FORCESOLF needs Nm or AUTO"); return; }
    bpkForceSolfNm = constrain(v, 0, 500);
    Serial.printf("OK,FORCESOLF,%d\n", (int)bpkForceSolfNm);
  }
  else if (strcmp(line, "WSFREEZE") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,WSFREEZE needs 0|1"); return; }
    wsFreeze = (v != 0);
    Serial.printf("OK,WSFREEZE,%u\n", (unsigned)(wsFreeze ? 1 : 0));
  }
  else if (strcmp(line, "WSBASE") == 0)
  {
    // Raw ESP_19 counts per corner; 0 restores the legacy free-running counter.
    // 0.0075 km/h per count, so 1333 counts ~= 10 km/h.
    if (!labArgInt(arg, v)) { Serial.println("ERR,WSBASE needs raw counts (0=legacy)"); return; }
    wsBaseRaw = (uint16_t)constrain(v, 0, 65535);
    Serial.printf("OK,WSBASE,%u\n", (unsigned)wsBaseRaw);
  }
  else if (strcmp(line, "WSDITHER") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,WSDITHER needs raw counts"); return; }
    wsDitherRaw = (uint16_t)constrain(v, 0, 10000);
    Serial.printf("OK,WSDITHER,%u\n", (unsigned)wsDitherRaw);
  }
  else if (strcmp(line, "WSFRONT") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,WSFRONT needs raw counts (can be negative)"); return; }
    wsFrontDeltaRaw = constrain(v, -20000, 20000);
    Serial.printf("OK,WSFRONT,%d\n", (int)wsFrontDeltaRaw);
  }
  else if (strcmp(line, "WSLR") == 0)
  {
    // Front left-vs-right split (VL - VR) in raw counts, 0.0075 km/h each.
    // Works in both legacy and WSBASE mode. The VAQ-specific slip lever.
    if (!labArgInt(arg, v)) { Serial.println("ERR,WSLR needs raw counts (VL-VR, can be negative)"); return; }
    wsLeftRightDeltaRaw = constrain(v, -40000, 40000);
    Serial.printf("OK,WSLR,%d\n", (int)wsLeftRightDeltaRaw);
  }
  else if (strcmp(line, "MODE") == 0)
  {
    if (!labArgInt(arg, v) || v < 0 || v > 5)
    {
      Serial.println("ERR,MODE needs 0..5");
      return;
    }
    state.mode = (openhaldex_mode_t)v;
    Serial.printf("OK,MODE,%u\n", (unsigned)state.mode);
  }
  else if (strcmp(line, "CF") == 0)
  {
    while (*arg == ' ')
      arg++;
    if (strncasecmp(arg, "OFF", 3) == 0)
    {
      labSetCF(-1);
      Serial.println("OK,CF,OFF");
      return;
    }
    if (!labArgInt(arg, v) || v < 0 || v > 100)
    {
      Serial.println("ERR,CF needs 0..100 or OFF");
      return;
    }
    labSetCF(v);
    Serial.printf("OK,CF,%u\n", (unsigned)haldexLearnCF);
  }
  else if (strcmp(line, "DANGER") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,DANGER needs 0|1"); return; }
    dangerZoneEnabled = (v != 0);
    Serial.printf("OK,DANGER,%u\n", (unsigned)(dangerZoneEnabled ? 1 : 0));
  }
  else if (strcmp(line, "UDS") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.println("ERR,UDS needs 0|1"); return; }
    liveDiagEnabled = (v != 0);
    Serial.printf("OK,UDS,%u\n", (unsigned)(liveDiagEnabled ? 1 : 0));
  }
  else if (strcmp(line, "BLOCKS") == 0)
  {
    // List the frame-edit blocks for the live generation, so individual CAN
    // frames can be isolated - the lever for attributing behaviour to a frame.
    const int gi = frameEditGenIdx(haldexGeneration);
    if (gi < 0) { Serial.println("ERR,BLOCKS,no gated blocks for this generation"); return; }
    const uint64_t m = activeFrameEditMask()[gi];
    for (uint16_t i = 0; i < frameEditBlockCount; i++)
    {
      if (frameEditBlocks[i].genIdx != (uint8_t)gi) continue;
      const uint8_t b = frameEditBlocks[i].bit;
      Serial.printf("BLK,%u,%u,%s\n", (unsigned)b,
                    (unsigned)((m >> b) & 0x1ULL), frameEditBlocks[i].name);
    }
    Serial.printf("OK,BLOCKS,0x%08lX\n", (unsigned long)(m & 0xFFFFFFFFULL));
  }
  else if (strcmp(line, "BLOCK") == 0)
  {
    // BLOCK <bit> <0|1> - enable/disable editing of one CAN frame live.
    while (*arg == ' ') arg++;
    char *sp2 = arg;
    while (*sp2 && *sp2 != ' ') sp2++;
    if (*sp2 != ' ') { Serial.println("ERR,BLOCK needs <bit> <0|1>"); return; }
    *sp2++ = '\0';
    int bit = atoi(arg), on = atoi(sp2);
    const int gi = frameEditGenIdx(haldexGeneration);
    if (gi < 0 || bit < 0 || bit > 63) { Serial.println("ERR,BLOCK bad bit"); return; }
    uint64_t *mask = activeFrameEditMask();
    if (on) mask[gi] |= (1ULL << bit);
    else    mask[gi] &= ~(1ULL << bit);
    Serial.printf("OK,BLOCK,%d,%d,0x%08lX\n", bit, on ? 1 : 0,
                  (unsigned long)(mask[gi] & 0xFFFFFFFFULL));
  }
  else if (strcmp(line, "OVR") == 0)
  {
    // OVR <hexCanId> <byteIdx> <value|AUTO> - force one byte of one frame.
    while (*arg == ' ') arg++;
    char *a2 = arg; while (*a2 && *a2 != ' ') a2++;
    if (*a2 != ' ') { Serial.println("ERR,OVR needs <hexid> <byte> <val|AUTO>"); return; }
    *a2++ = '\0';
    char *a3 = a2; while (*a3 && *a3 != ' ') a3++;
    if (*a3 != ' ') { Serial.println("ERR,OVR needs <hexid> <byte> <val|AUTO>"); return; }
    *a3++ = '\0';
    const uint16_t id = (uint16_t)strtol(arg, nullptr, 16);
    const uint8_t bi = (uint8_t)atoi(a2);
    if (bi > 7) { Serial.println("ERR,OVR byte must be 0..7"); return; }
    const bool clear = (strncasecmp(a3, "AUTO", 4) == 0);
    // Replace an existing slot for this id/byte, else take a free one.
    int slot = -1;
    for (uint8_t i = 0; i < LAB_OVR_MAX; i++)
      if (labOverrides[i].canId == id && labOverrides[i].byteIdx == bi) { slot = i; break; }
    if (clear)
    {
      if (slot >= 0) labOverrides[slot].canId = 0;
      Serial.printf("OK,OVR,%03X,%u,AUTO\n", id, bi);
      return;
    }
    if (slot < 0)
      for (uint8_t i = 0; i < LAB_OVR_MAX; i++)
        if (labOverrides[i].canId == 0) { slot = i; break; }
    if (slot < 0) { Serial.println("ERR,OVR no free slots (OVRCLR first)"); return; }
    labOverrides[slot].canId = id;
    labOverrides[slot].byteIdx = bi;
    labOverrides[slot].value = (uint8_t)strtol(a3, nullptr, 0);
    Serial.printf("OK,OVR,%03X,%u,0x%02X\n", id, bi, labOverrides[slot].value);
  }
  else if (strcmp(line, "OVRCLR") == 0)
  {
    for (uint8_t i = 0; i < LAB_OVR_MAX; i++) labOverrides[i].canId = 0;
    Serial.println("OK,OVRCLR");
  }
  else if (strcmp(line, "OVRS") == 0)
  {
    for (uint8_t i = 0; i < LAB_OVR_MAX; i++)
      if (labOverrides[i].canId)
        Serial.printf("OVRS,%03X,%u,0x%02X\n", labOverrides[i].canId,
                      labOverrides[i].byteIdx, labOverrides[i].value);
    Serial.println("OK,OVRS");
  }
  else if (strcmp(line, "TXADD") == 0 || strcmp(line, "TXDEL") == 0 || strcmp(line, "TXCLR") == 0 ||
           strcmp(line, "TXS") == 0)
  {
    labExtraCmd(line, arg);
  }
  else if (strcmp(line, "GEN") == 0)
  {
    // Live generation switch (not persisted). Lets one bench session A/B the
    // 0CQ and VAQ frame sets, and moves the UDS pair with it.
    if (!labArgInt(arg, v))
    {
      // No argument: report. A host script needs to know what it is driving.
      Serial.printf("OK,GEN,%u,DIAGID,%03lX,%03lX\n", (unsigned)haldexGeneration,
                    (unsigned long)udsHaldexReqId, (unsigned long)udsHaldexRespId);
      return;
    }
    if (!(v == 1 || v == 2 || v == 4 || v == 41 || v == 42 || v == 50 || v == 51 || v == 52))
    { Serial.println("ERR,GEN unsupported generation"); return; }
    haldexGeneration = (uint8_t)v;
    udsApplyDefaultIds();
    Serial.printf("OK,GEN,%u,DIAGID,%03lX,%03lX\n", (unsigned)haldexGeneration,
                  (unsigned long)udsHaldexReqId, (unsigned long)udsHaldexRespId);
  }
  else if (strcmp(line, "SA") == 0)
  {
    if (!labArgInt(arg, v)) { Serial.printf("OK,SA,%u\n", (unsigned)(isStandalone ? 1 : 0)); return; }
    labSetStandalone(v != 0);
    Serial.printf("OK,SA,%u\n", (unsigned)(isStandalone ? 1 : 0));
  }
  else if (strcmp(line, "RXIDS") == 0)
  {
    // Every CAN ID seen on Bus 1 since boot / RXCLR. In standalone that is
    // exactly what the module transmits.
    //   RXID,<id hex>,<ext>,<count>,<period ms>,<dlc>,<8 bytes hex>,<age ms>
    const uint32_t now = millis();
    unsigned n = 0;
    for (uint8_t i = 0; i < HDX_RX_STATS_MAX; i++)
    {
      const HdxRxStat &s = hdxRxStats[i];
      if (s.id == 0) continue;
      Serial.printf("RXID,%0*lX,%u,%lu,%u,%u,%02X%02X%02X%02X%02X%02X%02X%02X,%lu\n",
                    s.extd ? 8 : 3, (unsigned long)s.id, (unsigned)s.extd, (unsigned long)s.count,
                    (unsigned)s.periodMs, (unsigned)s.dlc,
                    s.data[0], s.data[1], s.data[2], s.data[3], s.data[4], s.data[5], s.data[6], s.data[7],
                    (unsigned long)(now - s.lastMs));
      n++;
    }
    Serial.printf("OK,RXIDS,%u,%lu\n", n, (unsigned long)hdxRxDroppedIds);
  }
  else if (strcmp(line, "RXCLR") == 0)
  {
    for (uint8_t i = 0; i < HDX_RX_STATS_MAX; i++) hdxRxStats[i].id = 0;
    hdxRxDroppedIds = 0;
    Serial.println("OK,RXCLR");
  }
  else if (strcmp(line, "DIAGID") == 0)
  {
    // DIAGID <reqhex> <rsphex> pins the pair; DIAGID AUTO hands it back to the
    // per-generation default (0x70F/0x779 Haldex, 0x71E/0x788 VAQ).
    while (*arg == ' ') arg++;
    if (*arg == '\0')
    {
      Serial.printf("OK,DIAGID,%s,%03lX,%03lX\n", udsIdsManual ? "MANUAL" : "AUTO",
                    (unsigned long)udsHaldexReqId, (unsigned long)udsHaldexRespId);
      return;
    }
    if (strncasecmp(arg, "AUTO", 4) == 0)
    {
      udsIdsManual = false;
      udsApplyDefaultIds();
      Serial.printf("OK,DIAGID,AUTO,%03lX,%03lX\n", (unsigned long)udsHaldexReqId, (unsigned long)udsHaldexRespId);
      return;
    }
    char *a2 = arg; while (*a2 && *a2 != ' ') a2++;
    if (*a2 != ' ') { Serial.println("ERR,DIAGID needs <reqhex> <rsphex> or AUTO"); return; }
    *a2++ = '\0';
    const uint32_t rq = (uint32_t)strtoul(arg, nullptr, 16);
    const uint32_t rs = (uint32_t)strtoul(a2, nullptr, 16);
    if (rq == 0 || rs == 0 || rq > 0x7FF || rs > 0x7FF) { Serial.println("ERR,DIAGID ids must be 001..7FF"); return; }
    udsHaldexReqId = rq;
    udsHaldexRespId = rs;
    udsIdsManual = true;
    Serial.printf("OK,DIAGID,%03lX,%03lX\n", (unsigned long)udsHaldexReqId, (unsigned long)udsHaldexRespId);
  }
  else if (strcmp(line, "UDSRAW") == 0)
  {
    uint8_t req[7];
    const size_t n = labParseHexBytes(arg, req, sizeof(req));
    if (n == 0) { Serial.println("ERR,UDSRAW needs 1..7 hex bytes e.g. 22 F1 87"); return; }
    labUdsRawCmd(req, n, 1500);
  }
  else if (strcmp(line, "DTC") == 0)
  {
    labDtcRead();
  }
  else if (strcmp(line, "DTCCLR") == 0)
  {
    const uint8_t req[4] = {0x14, 0xFF, 0xFF, 0xFF};
    size_t n = 0;
    const int rc = labUdsTransact(req, sizeof(req), labUdsBuf, sizeof(labUdsBuf), n, 2000);
    if (rc < 0) { Serial.printf("ERR,DTCCLR,%s\n", labUdsErr(rc)); return; }
    if (n >= 1 && labUdsBuf[0] == 0x54) { Serial.println("OK,DTCCLR"); return; }
    Serial.print("ERR,DTCCLR,reply ");
    labPrintHex(labUdsBuf, n);
    Serial.println();
  }
  else if (strcmp(line, "SESS") == 0)
  {
    if (!labArgInt(arg, v) || v < 1 || v > 0x7F) { Serial.println("ERR,SESS needs 1|2|3|4..."); return; }
    const uint8_t req[2] = {0x10, (uint8_t)v};
    labUdsRawCmd(req, sizeof(req), 1500);
  }
  else if (strcmp(line, "TP") == 0)
  {
    const uint8_t req[2] = {0x3E, 0x00};
    labUdsRawCmd(req, sizeof(req), 800);
  }
  else if (strcmp(line, "HELP") == 0)
  {
    Serial.println("OK,HELP,PING GET LOG CEIL FLOOR FIXHUNT QBIT MODE CF"
                   " BPKFLOOR SLEWIST SLEWSOLF TRAEG SCHUB STATUS FORCEIST FORCESOLF"
                   " WSFREEZE WSBASE WSDITHER WSFRONT WSLR UDS BLOCKS BLOCK OVR OVRS OVRCLR DANGER"
                   " GEN SA RXIDS RXCLR DIAGID UDSRAW DTC DTCCLR SESS TP TXADD TXDEL TXCLR TXS HELP");
  }
  else
  {
    Serial.printf("ERR,unknown,%s\n", line);
  }
}

static void serialLabTask(void *arg)
{
  while (1)
  {
    // SavvyCAN serial mode owns the port (binary GVRET) - stay out of its way.
    if (analyzerSerial)
    {
      labLogIntervalMs = 0;
      labLineLen = 0;
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }

    while (Serial.available())
    {
      const char c = (char)Serial.read();
      if (c == '\r')
        continue;
      if (c == '\n')
      {
        labLine[labLineLen] = '\0';
        if (labLineLen > 0)
          labHandleLine(labLine);
        labLineLen = 0;
        continue;
      }
      if (labLineLen < sizeof(labLine) - 1)
        labLine[labLineLen++] = c;
      else
        labLineLen = 0; // overlong garbage - resync on the next newline
    }

    if (isStandalone)
      labExtraSend(millis());

    if (labLogIntervalMs > 0)
    {
      const uint32_t now = millis();
      if (now - labLastEmitMs >= labLogIntervalMs)
      {
        labLastEmitMs = now;
        labPrintStat("BPK");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setupSerialLab()
{
  // main.cpp only calls Serial.begin() inside a #if on the debug flags, and in
  // a normal build every one of those is 0 - so Serial (native USB CDC) is
  // never actually started. The port still enumerates because the USB
  // Serial/JTAG peripheral is hardware, and ESP-IDF's own log output still
  // gets through on its own path, which makes it look alive: but nothing
  // drains host writes (they time out) and nothing the app prints arrives.
  // Start it here so the harness works in a normal build. Beginning twice is
  // harmless if a debug build already did it, and the short TX timeout stops
  // a detached host from blocking this task on a full buffer.
  Serial.begin(500000);
  Serial.setTxTimeoutMs(10);
  xTaskCreate(serialLabTask, "serialLab", 4096, NULL, 2, NULL);
  Serial.println("OK,LAB,READY"); // positive sign-of-life after a reset
}
