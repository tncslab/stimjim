//    stimjimAWG — SdLog implementation: the log file, the `LOG` command backend
//    and the `SD` file-access group. GPL-3.0-or-later; see Config.h header.

#include "SdLog.h"
#include "Config.h"
#include "FastIO.h"
#include "Protocol.h"
#include "TrainStore.h"
#include "Clock.h"
#include <string.h>

#if SJ_USE_SD
#include <SD.h>       // Teensy wrapper over the bundled SdFat (MIT), BUILTIN_SDCARD
#endif

namespace SdLog {

#if !SJ_USE_SD

// A build without a card socket (Teensy 4.0) still answers every command --
// reporting the reason is more useful than a link error or a missing verb.
static void noCard(const char* cmd) {
  Serial.printf("ERR %s: no SD support in this build (%s)\n", cmd, SJ_HW_NAME);
}
void begin()                            { Serial.println("# SD: not supported by this build"); }
void poll()                             { }
void writeRow(const char*)              { }
void flushNow()                         { }
bool isOpen()                           { return false; }
bool cardPresent()                      { return false; }
const char* name()                      { return ""; }
uint32_t bytes()                        { return 0; }
void noteTrain(uint8_t)                 { }
void noteEvent(const char*, const char*) { }
bool autoOpen()                         { return false; }
void status()                           { noCard("LOG"); }
void openLog(const char*)               { noCard("LOG"); }
void closeLog()                         { noCard("LOG"); }
void info(bool)                         { noCard("SDINFO"); }
void list(const char*)                  { noCard("SDLIST"); }
void get(const char*, uint64_t, uint64_t) { noCard("SDGET"); }
void del(const char*)                   { noCard("SDDEL"); }

#else

#define SJ_LOGNAME_MAX 64
#define SJ_FLUSH_ROWS  64      // rows between forced flushes
#define SJ_FLUSH_MS    1000    // ... or milliseconds, whichever comes first
#define SJ_ANCHOR_MS   60000   // longest gap between two wall-clock anchor lines

static bool     mounted = false;
static File     logFile;
static char     logName[SJ_LOGNAME_MAX] = "";
static uint32_t logBytes = 0;
static uint32_t rowsSinceFlush = 0;
static uint32_t lastFlushMs = 0;
static uint32_t lastAnchorMs = 0;

// One wall-clock anchor line. The CSV columns deliberately do not carry a wall
// clock: host tools depend on their shape, and rendering one per row would add
// cost to the path phase 15 set out to make cheaper. Instead every row's `us`
// maps to a wall clock through the nearest anchor, and the drift between two
// anchors is visible in the file rather than hidden inside it.
static void writeAnchor() {
  if (!logFile) return;
  char anchor[96];
  Clock::anchorLine(anchor, sizeof anchor);
  logBytes += (uint32_t)logFile.printf("# clock: %s\n", anchor);
  lastAnchorMs = millis();
}

// FAT directory timestamps for files this firmware creates. SdFat asks for a
// packed date and time plus hundredths since the last even second. A SRC_BUILD
// epoch is refused outright: a wrong file date is worse than none, because a
// host sorting by date would silently believe it.
static void fatDateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  uint32_t s;
  uint16_t ms;
  if (Clock::source() == Clock::SRC_BUILD || !Clock::read(s, ms)) {
    *date = 0;
    *time = 0;
    *ms10 = 0;
    return;
  }
  Clock::Civil c;
  Clock::civilFromUnix(s, ms, c);
  *date = FS_DATE(c.year, c.mon, c.day);
  *time = FS_TIME(c.hour, c.min, c.sec);
  // FS_TIME has 2-second granularity, so the odd second and the fraction go
  // here: hundredths since the last even second, valid range 0-199.
  *ms10 = (uint8_t)((c.sec & 1u) * 100u + c.ms / 10u);
}

static bool mount() {
  if (mounted) return true;
  mounted = SD.begin(BUILTIN_SDCARD);      // native SDIO — not the DAC/ADC bus
  if (mounted) FsDateTime::setCallback(fatDateTime);
  return mounted;
}

bool isOpen()          { return (bool)logFile; }
bool cardPresent()     { return mounted; }
const char* name()     { return logName; }
uint32_t bytes()       { return logBytes; }

void begin() {
  if (mount()) {
    char kib[24];
    Protocol::u64str(SD.totalSize() >> 10, kib);
    Serial.printf("# SD: card mounted, %s KiB — LOG1 opens a log file\n", kib);
  } else {
    Serial.println("# SD: no card mounted (SDINFO reports it, LOG1 retries)");
  }
}

void poll() {
  if (!logFile || rowsSinceFlush == 0) return;
  uint32_t now = millis();
  // Only while rows are actually being written: a file left open on an idle
  // board collects no anchors, because there is nothing between them to anchor.
  if ((uint32_t)(now - lastAnchorMs) >= SJ_ANCHOR_MS) writeAnchor();
  if (rowsSinceFlush >= SJ_FLUSH_ROWS || (uint32_t)(now - lastFlushMs) >= SJ_FLUSH_MS) {
    logFile.flush();
    rowsSinceFlush = 0;
    lastFlushMs    = now;
  }
}

void writeRow(const char* row) {
  if (!logFile) return;
  logBytes += (uint32_t)logFile.println(row);
  rowsSinceFlush++;
}

void flushNow() {
  if (!logFile) return;
  logFile.flush();
  rowsSinceFlush = 0;
  lastFlushMs    = millis();
}

// The file header records the configuration as it stood when the file was
// opened; this block records what each train actually plays, which is the part
// that changes between trains.
void noteTrain(uint8_t slot) {
  if (!logFile) return;
  writeAnchor();                 // every train's rows start from a fresh anchor
  const TrainDef& t = TrainStore::slotConst(slot);
  char line[SJ_SERIALIZE_MAX];
  TrainStore::serializeTrain(slot, t, line, sizeof line);
  logBytes += (uint32_t)logFile.printf("# train: %s\n", line);
  if (t.type != SINE && !TrainStore::isDefaultEnv(t.env)) {
    TrainStore::serializeEnv(slot, t.env, line, sizeof line);
    logBytes += (uint32_t)logFile.printf("# train: %s\n", line);
  }
  TrainStore::serializeMeas(slot, t.meas, line, sizeof line);
  logBytes += (uint32_t)logFile.printf("# train: %s\n", line);
}

// A session event, stamped with the same microsecond timebase every row
// carries, so it maps to a wall clock through the nearest anchor exactly the
// way a row does. `us=` is the key an anchor line already uses for it.
void noteEvent(const char* tag, const char* text) {
  if (!logFile) return;
  char us[24];
  Protocol::u64str(SJ_CYC_TO_US(FastIO::cycles64()), us);
  logBytes += (uint32_t)logFile.printf("# %s: us=%s %s\n", tag, us, text);
  // Counted as a row so the flush policy in poll() eventually commits it; the
  // counter is a flush trigger and not a statistic, so mixing the two is fine.
  rowsSinceFlush++;
}

// ---------------------------------------------------------------- `LOG`

void status() {
  Serial.printf("LOG,%u,%s,%lu\n", logFile ? 1 : 0, logName, (unsigned long)logBytes);
}

// A log has to identify the waveforms that produced it even when the train was
// started by a trigger edge and no host was listening, so the header carries
// the identity block and the whole session configuration -- the same lines
// `DUMP` prints, hence paste-back-able out of the file.
static void writeHeader() {
  logFile.printf("# %s log — fw=%s, proto=%d\n",
                 SJ_FW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  Protocol::printIdentity(logFile);
  Commands::writeDump(logFile);
  logFile.println("# columns: timestamp_us,slot,pulse,point,V0_mV,I0_uA,V1_mV,I1_uA");
  writeAnchor();
}

bool autoOpen() {
  // cardPresent(), not mount(): a board with an empty socket must not pay an
  // SDIO probe timeout on every slot commit, and nothing here is worth a line
  // of output. LOG1 and SDINFO remain the way to pick up a card inserted after
  // boot, both of which remount.
  if (logFile || !mounted) return false;
  openLog(nullptr);
  return true;
}

void openLog(const char* name) {
  if (!mount()) { Serial.println("ERR LOG: no SD card"); return; }
  if (logFile) { logFile.flush(); logFile.close(); }

  char nm[SJ_LOGNAME_MAX];
  if (name && *name) {
    if (strlen(name) >= sizeof nm) { Serial.println("ERR LOG: file name too long"); return; }
    strcpy(nm, name);
  } else {
    // lowest free LOGnnnn.CSV. The index, not the clock, is what identifies a
    // run: the RTC epoch is a label whose source may be a compile time
    // (Clock.h), so a date-based name would promise more than it can keep. The
    // date reaches the file two other ways — the FAT timestamp fatDateTime
    // supplies and the in-file anchor lines — and the index must not be reused
    // either way.
    uint32_t i = 0;
    for (; i < 10000; i++) {
      snprintf(nm, sizeof nm, "LOG%04lu.CSV", (unsigned long)i);
      if (!SD.exists(nm)) break;
    }
    if (i == 10000) { Serial.println("ERR LOG: LOG0000-LOG9999.CSV all exist"); return; }
  }

  logFile = SD.open(nm, FILE_WRITE);
  if (!logFile) { Serial.printf("ERR LOG: cannot create %s\n", nm); return; }
  strcpy(logName, nm);
  writeHeader();
  logFile.flush();
  logBytes       = (uint32_t)logFile.size();
  rowsSinceFlush = 0;
  lastFlushMs    = millis();
  status();
}

void closeLog() {
  if (!logFile) { status(); return; }
  logFile.flush();
  logBytes = (uint32_t)logFile.size();
  logFile.close();
  status();                                // reports the closed state and the size
  logName[0] = '\0';
  logBytes   = 0;
}

// ------------------------------------------------------------- `SD` group

void info(bool withUsed) {
  if (!mount()) {
    Serial.println("SD,0,0,-1,0,,0");
    Serial.println("# SD: no card — insert one and retry (SDINFO remounts)");
    return;
  }
  // usedSize() walks the whole free-cluster chain: seconds on a large card,
  // during which loop() drains no MDATA and redraws nothing. So the field is
  // "-1 = not measured" unless SDINFO,1 asks for it. The field itself stays in
  // place either way, so the record shape does not depend on the argument.
  char tot[24], used[24] = "-1";
  Protocol::u64str(SD.totalSize() >> 10, tot);
  if (withUsed) Protocol::u64str(SD.usedSize() >> 10, used);
  Serial.printf("SD,1,%s,%s,%u,%s,%lu\n",
                tot, used, logFile ? 1 : 0, logName, (unsigned long)logBytes);
  if (!withUsed)
    Serial.println("# used space is not measured — SDINFO,1 scans for it "
                   "(seconds on a large card, so not while a train runs)");
}

void list(const char* dir) {
  if (!mount()) { Serial.println("ERR SDLIST: no SD card"); return; }
  File d = SD.open((dir && *dir) ? dir : "/");
  if (!d || !d.isDirectory()) {
    Serial.printf("ERR SDLIST: %s is not a directory\n", (dir && *dir) ? dir : "/");
    if (d) d.close();
    return;
  }
  for (;;) {
    File e = d.openNextFile();
    if (!e) break;
    Serial.printf("SDLIST,%s%s,%lu\n", e.name(), e.isDirectory() ? "/" : "",
                  (unsigned long)e.size());
    e.close();
    FastIO::cycles64();
  }
  d.close();
  Serial.println("OK");
}

// CRC-32/ISO-HDLC, bit-serial: no table, and the ~10 cycles per byte are
// irrelevant next to the USB transfer they verify.
static uint32_t crc32Update(uint32_t crc, const uint8_t* p, size_t n) {
  while (n--) {
    crc ^= *p++;
    for (uint8_t k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return crc;
}

void get(const char* name, uint64_t offset, uint64_t len) {
  if (!mount()) { Serial.println("ERR SDGET: no SD card"); return; }

  // Reading back the log that is still open is the normal case: flush it and
  // serve it through the same handle (FILE_WRITE is O_RDWR here), restoring
  // the write position afterwards. A second handle would see a stale size.
  const bool isLog = logFile && !strcasecmp(name, logName);
  File  tmp;
  File* f;
  uint64_t savedPos = 0;
  if (isLog) {
    flushNow();
    savedPos = logFile.position();
    f = &logFile;
  } else {
    tmp = SD.open(name, FILE_READ);
    if (!tmp) { Serial.printf("ERR SDGET: cannot open %s\n", name); return; }
    if (tmp.isDirectory()) {
      tmp.close();
      Serial.printf("ERR SDGET: %s is a directory\n", name);
      return;
    }
    f = &tmp;
  }

  const uint64_t total = f->size();
  if (offset > total) offset = total;
  const uint64_t avail = total - offset;
  if (len == 0 || len > avail) len = avail;

  char so[24], sl[24], st[24];
  Protocol::u64str(offset, so);
  Protocol::u64str(len,    sl);
  Protocol::u64str(total,  st);
  // Header first, then exactly `len` bytes, then one newline the host discards:
  // the byte count is what keeps arbitrary file content from being mistaken for
  // protocol, without an escaping scheme.
  Serial.printf("SDGET,%s,%s,%s,%s\n", name, so, sl, st);

  f->seek(offset);
  uint8_t  buf[512];
  uint32_t crc  = 0xFFFFFFFFu;
  uint64_t left = len;
  while (left) {
    size_t want = (left > sizeof buf) ? sizeof buf : (size_t)left;
    size_t got  = f->read(buf, want);
    if (got == 0) break;
    Serial.write(buf, got);
    crc   = crc32Update(crc, buf, got);
    left -= got;
    FastIO::cycles64();   // a host that stops reading must not stall the timebase
  }
  Serial.println();
  Serial.printf("# crc32=%08lx\n", (unsigned long)(crc ^ 0xFFFFFFFFu));
  if (left) {
    char sleft[24];
    Protocol::u64str(left, sleft);
    Serial.printf("# SDGET: short read — %s bytes were not sent\n", sleft);
  }

  if (isLog) f->seek(savedPos);
  else       tmp.close();
  Serial.println("OK");
}

void del(const char* name) {
  if (!mount()) { Serial.println("ERR SDDEL: no SD card"); return; }
  if (logFile && !strcasecmp(name, logName)) {
    Serial.println("ERR SDDEL: that file is the open log — close it first (LOG0)");
    return;
  }
  if (!SD.exists(name)) { Serial.printf("ERR SDDEL: %s does not exist\n", name); return; }
  if (!SD.remove(name)) { Serial.printf("ERR SDDEL: cannot remove %s\n", name); return; }
  Serial.printf("# SDDEL: removed %s\n", name);
  Serial.println("OK");
}

#endif // SJ_USE_SD

} // namespace SdLog
