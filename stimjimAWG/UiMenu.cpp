//    stimjimAWG — UiMenu implementation, Phase 1 scope (splash + input proof).
//    GPL-3.0-or-later; see Config.h header.

#include "UiMenu.h"
#include "UiInput.h"
#include "Config.h"

#if SJ_USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 display(SJ_OLED_WIDTH, SJ_OLED_HEIGHT, &Wire, -1);
static bool     displayOk  = false;
static bool     dirty      = false;
static uint32_t lastRender = 0;
static UiInput::Event lastEvent = UiInput::EV_NONE;
#endif

namespace UiMenu {

void begin() {
#if SJ_USE_DISPLAY
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, SJ_OLED_ADDR);
  if (!displayOk) {
    Serial.println("WARN display: SSD1306 init failed");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("%s %s\n", SJ_FW_NAME, SJ_FW_VERSION);
  display.println("Phase 1 scaffold");
  display.println("BENCH/IDN via serial");
  display.display();
#endif
}

void tick() {
#if SJ_USE_DISPLAY
  UiInput::Event e;
  while ((e = UiInput::pop()) != UiInput::EV_NONE) {
    lastEvent = e;
    dirty = true;
  }
  if (!displayOk || !dirty) return;
  uint32_t now = millis();
  if (now - lastRender < 50) return;   // render throttle
  lastRender = now;
  dirty = false;

  static const char* const names[] = {"-", "OK", "PREV", "NEXT", "BACK"};
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("%s %s\n", SJ_FW_NAME, SJ_FW_VERSION);
  display.println("menu FSM: Phase 8");
  display.printf("last button: %s", names[lastEvent]);
  display.display();
#else
  while (UiInput::pop() != UiInput::EV_NONE) ;   // keep the ring drained
#endif
}

} // namespace UiMenu
