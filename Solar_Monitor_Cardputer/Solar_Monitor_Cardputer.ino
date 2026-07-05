/********************************************************************************
 * Solar Inverter Monitor - M5Stack Cardputer (ESP32-S3, 240x135 color TFT)
 ********************************************************************************
 * Polls two inverter WiFi dongles over HTTP, parses the CSV status string each
 * one returns, and renders it across several pages on the built-in screen.
 * Page navigation uses the keyboard instead of external buttons - no wiring
 * needed at all, since the Cardputer has its own display + keys built in.
 *
 * KEYS:
 *   ;   - Page Up
 *   .   - Page Down
 *   Enter - Home (jump straight to page 1, the big-text power summary)
 *
 * PAGES (same layout as the DFR0676 build, corrected field mapping - see below):
 *   1 (Home) - big-text summary: Total Power, Inv1 Power, Inv2 Power
 *   2        - grid-connected power, voltage, current (per inverter)
 *   3        - total generated & today generated, kWh (per inverter)
 *   4        - total running time & today running time, hours (per inverter)
 *   5        - PV1 voltage & current (per inverter)
 *   6        - PV2 voltage & current (per inverter)
 *   7        - PV3 voltage & current - only shown if either inverter reports
 *              a real (non-sentinel) PV3 reading. Your capture confirms this
 *              inverter protocol has exactly 3 real PV/MPPT inputs (not 9 as
 *              first guessed) - see the correction notes below.
 *   last     - Cardputer Info: connected WiFi SSID, IP address, and battery
 *              charge (% and voltage). Always the final page, whether or not
 *              the PV3 page above it is present.
 *
 * POLLING:
 *   Every 60s while at least one inverter responds. Since the dongles are
 *   powered by the inverters themselves, both go dark once the panels stop
 *   producing at dusk. When BOTH fail to respond, polling backs off to every
 *   15 minutes until one answers again. The screen always keeps showing the
 *   last good reading, with an OFFLINE tag and how long ago it was last seen.
 *
 * LIBRARIES REQUIRED (Arduino Library Manager):
 *   - M5Cardputer  (github.com/m5stack/M5Cardputer)
 *   - M5GFX, M5Unified (pulled in automatically as dependencies)
 *   (WiFi.h / HTTPClient.h are built into the ESP32 Arduino core)
 *
 * BOARD SETUP: Install the M5Stack board package (Arduino IDE > Preferences >
 * Additional Board URLs, see https://docs.m5stack.com/en/arduino/arduino_ide),
 * then Tools -> Board -> M5Stack Arduino -> M5Cardputer.
 *
 * DISPLAY POWER SAVING:
 * The screen turns on at power-up, turns back on immediately on any key
 * press, and turns itself off after 2 minutes with no key presses. This
 * applies regardless of USB vs battery power. Polling/WiFi keep running in
 * the background the whole time - only the backlight is power-managed.
 * -----------------------------------------------------------------------------
 */

#include "M5Cardputer.h"
#include <M5_KMeter.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ---------------------------- USER CONFIG ----------------------------
const char* WIFI_SSID = "RouterRebootTest";
const char* WIFI_PASSWORD = "Today123!";

const char* INVERTER_IP[2] = { "192.168.6.10", "192.168.6.11" };
const char* STATUS_PATH = "/status/status.php";

const unsigned long POLL_INTERVAL_FAST_MS = 60UL * 1000UL;         // normal (daytime) cadence
const unsigned long POLL_INTERVAL_SLOW_MS = 15UL * 60UL * 1000UL;  // both inverters offline (nighttime)
const unsigned long RECONNECT_INTERVAL_MS = 10UL * 1000UL;
const uint16_t HTTP_TIMEOUT_MS = 6000;
// -----------------------------------------------------------------------

#define MAX_PV_CHANNELS 3  // this protocol has exactly 3 real PV/MPPT inputs
#define PV_SENTINEL 65535  // "not present / N/A" marker

struct InverterData 
{
  bool valid = false;
  bool online = false;
  bool extendedValid = false;

  float totalKWh = 0, totalHours = 0, todayKWh = 0, todayHours = 0;

  float pvV[MAX_PV_CHANNELS] = { 0, 0, 0 };
  float pvI[MAX_PV_CHANNELS] = { 0, 0, 0 };
  bool pvValid[MAX_PV_CHANNELS] = { false, false, false };

  int gridW = 0;
  float gridHz = 0, line1V = 0, line1A = 0;

  float busV = 0, tempC = 0, co2Kg = 0;
  int runState = -1;

  unsigned long lastGoodMillis = 0;
};

InverterData inv[2];
unsigned long lastPollMillis = 0;
unsigned long lastReconnectMillis = 0;
unsigned long currentPollInterval = POLL_INTERVAL_FAST_MS;

// ------------------------------ paging ------------------------------
#define NUM_FIXED_PAGES 6              // 0:PowerSummary 1:Home 2:Totals 3:RunTime 4:PV1 5:PV2
int extraPvChannels[MAX_PV_CHANNELS];  // channel numbers (3..) currently valid
int numExtraPvPages = 0;
int totalPages = NUM_FIXED_PAGES;
int currentPage = 0;

bool prevPageUp = false, prevPageDown = false, prevHome = false;

M5Canvas canvas(&M5Cardputer.Display);
const int SCREEN_W = 240;
const int SCREEN_H = 135;

// ------------------------------ power management ------------------------------
const unsigned long DISPLAY_TIMEOUT_MS = 2UL * 60UL * 1000UL;  // blank the screen after this long idle
const uint8_t DISPLAY_BRIGHTNESS = 100;

bool displayOn = true;
unsigned long lastActivityMillis = 0;

// Turns the backlight on for any key press, and off after DISPLAY_TIMEOUT_MS
// with no key presses - regardless of USB vs battery power.
// Returns true if the display just turned on this call (caller should redraw).
bool updateDisplayPower(bool anyKeyDown)
{
  unsigned long now = millis();
  bool justWoke = false;

  if (anyKeyDown)
  {
    lastActivityMillis = now;
    if (!displayOn)
    {
      displayOn = true;
      justWoke = true;
    }
  }
  else if (displayOn && now - lastActivityMillis >= DISPLAY_TIMEOUT_MS)
  {
    displayOn = false;
  }

  M5Cardputer.Display.setBrightness(displayOn ? DISPLAY_BRIGHTNESS : 0);
  return justWoke;
}

// ---------------------------------------------------------------------
String agoString(unsigned long ms)
{
  if (ms == 0) return "never";
  unsigned long secs = (millis() - ms) / 1000;
  if (secs < 60) return String(secs) + "s";
  return String(secs / 60) + "m";
}

void showSplash(const char* msg) 
{
  canvas.fillSprite(BLACK);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(4, 10);
  canvas.print(msg);
  canvas.pushSprite(0, 0);
}

void connectWiFi() 
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  char msg[48];
  snprintf(msg, sizeof(msg), "Connecting to %s ...", WIFI_SSID);
  showSplash(msg);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) 
  {
    delay(250);
  }
}

// Fetches + parses one inverter's status string.
// On success, updates inv[idx] and returns true.
// On any failure, leaves inv[idx] untouched and returns false.
bool fetchInverter(int idx) 
{
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = String("http://") + INVERTER_IP[idx] + STATUS_PATH;
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(url)) return false;

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST("t=l");  // matches the dongle's own web UI request
  if (code != HTTP_CODE_OK) 
  {
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();
  body.trim();

  const int MAX_FIELDS = 40;
  long f[MAX_FIELDS];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= (int)body.length() && count < MAX_FIELDS; i++) 
  {
    if (i == (int)body.length() || body[i] == ',') 
    {
      f[count++] = body.substring(start, i).toInt();
      start = i + 1;
    }
  }
  if (count < 27) return false;  // malformed / truncated response

  InverterData d;
  d.valid = true;
  d.online = true;
  d.totalKWh = f[1] * 0.01f;   // field 2
  d.totalHours = f[2] * 0.1f;  // field 3
  d.todayKWh = f[3] * 0.01f;   // field 4
  d.todayHours = f[4] * 0.1f;  // field 5

  int pvVIdx[MAX_PV_CHANNELS] = { 5, 7, 9 };   // fields 6, 8, 10
  int pvIIdx[MAX_PV_CHANNELS] = { 6, 8, 10 };  // fields 7, 9, 11
  for (int p = 0; p < MAX_PV_CHANNELS; p++) 
  {
    long rv = f[pvVIdx[p]];
    long ri = f[pvIIdx[p]];
    bool ok = (rv != PV_SENTINEL && ri != PV_SENTINEL);
    d.pvValid[p] = ok;
    d.pvV[p] = ok ? rv * 0.1f : 0;
    d.pvI[p] = ok ? ri * 0.01f : 0;
  }

  d.gridW = (int)f[23];      // field 24
  d.gridHz = f[24] * 0.01f;  // field 25
  d.line1V = f[25] * 0.1f;   // field 26
  d.line1A = f[26] * 0.01f;  // field 27

  if (count >= 35) 
  {
    d.extendedValid = true;
    d.busV = f[31] * 0.1f;    // field 32
    d.tempC = f[32] * 0.1f;   // field 33
    d.co2Kg = f[33] * 0.1f;   // field 34
    d.runState = (int)f[34];  // field 35 (2 = "Normal")
  }

  d.lastGoodMillis = millis();
  inv[idx] = d;
  return true;
}

void pollInverters() 
{
  for (int i = 0; i < 2; i++) 
  {
    if (!fetchInverter(i)) 
    {
      inv[i].online = false;  // keep last-known values, just flag them stale
    }
  }
  bool anyOnline = inv[0].online || inv[1].online;
  currentPollInterval = anyOnline ? POLL_INTERVAL_FAST_MS : POLL_INTERVAL_SLOW_MS;
}

// Recomputes which PV3+ pages currently have valid data on either inverter.
void computeActivePages() 
{
  numExtraPvPages = 0;
  for (int ch = 3; ch <= MAX_PV_CHANNELS; ch++) 
  {
    int idx = ch - 1;
    if (inv[0].pvValid[idx] || inv[1].pvValid[idx]) 
    {
      extraPvChannels[numExtraPvPages++] = ch;
    }
  }
  totalPages = NUM_FIXED_PAGES + numExtraPvPages + 1;  // +1 for the trailing Cardputer Info page
  if (currentPage >= totalPages) currentPage = 0;
}

// ------------------------------ rendering ------------------------------
const int TITLE_Y = 8;
const int DIV1_Y = 16;
const int SEC1_TOP = 18;
const int DIV2_Y = 76;
const int SEC2_TOP = 78;

void drawPageChrome(const char* title) 
{
  canvas.setTextColor(YELLOW);
  canvas.setCursor(2, TITLE_Y);
  canvas.print(title);

  char pageIndicator[12];
  snprintf(pageIndicator, sizeof(pageIndicator), "%d/%d", currentPage + 1, totalPages);
  int w = canvas.textWidth(pageIndicator);
  canvas.setCursor(SCREEN_W - w - 2, TITLE_Y);
  canvas.print(pageIndicator);

  canvas.setTextColor(DARKGREY);
  canvas.drawFastHLine(0, DIV1_Y, SCREEN_W, DARKGREY);
  canvas.drawFastHLine(0, DIV2_Y, SCREEN_W, DARKGREY);
}

// Prints a power reading as "1234W", above 999W as "1.23kW", and above
// 999kW as "1.23MW". Pass spaceBeforeUnit=true to match pages that put a
// space before the unit (e.g. "3561 W"), false for pages that don't
// (e.g. "3561W").
void printPower(int watts, bool spaceBeforeUnit)
{
  const char* space = spaceBeforeUnit ? " " : "";
  if (watts > 999000)
  {
    canvas.print(watts / 1000000.0f, 2);
    canvas.print(space);
    canvas.print("MW");
  }
  else if (watts > 999)
  {
    canvas.print(watts / 1000.0f, 2);
    canvas.print(space);
    canvas.print("kW");
  }
  else
  {
    canvas.print(watts);
    canvas.print(space);
    canvas.print("W");
  }
}

// Prints an energy reading (given in kWh) as "X.XkWh" using kWhDecimals
// decimal places, or above 999kWh as "X.XXMWh".
void printEnergy(float kWh, int kWhDecimals)
{
  if (kWh > 999)
  {
    canvas.print(kWh / 1000.0f, 2);
    canvas.print(" MWh");
  }
  else
  {
    canvas.print(kWh, kWhDecimals);
    canvas.print(" kWh");
  }
}

// Prints a running-time reading (given in hours) broken down as
// "<years>y <weeks>w <days>d <hours>h", using 24h days, 7-day weeks, and
// 52-week years (e.g. 17590h -> "2y 0w 4d 22h"). Fractional hours are
// truncated - not meaningful at this scale.
void printDuration(float totalHoursF)
{
  const long HOURS_PER_DAY = 24;
  const long HOURS_PER_WEEK = HOURS_PER_DAY * 7;
  const long HOURS_PER_YEAR = HOURS_PER_WEEK * 52;

  long totalHours = (long)totalHoursF;

  long years = totalHours / HOURS_PER_YEAR;
  long rem = totalHours % HOURS_PER_YEAR;
  long weeks = rem / HOURS_PER_WEEK;
  rem = rem % HOURS_PER_WEEK;
  long days = rem / HOURS_PER_DAY;
  long hours = rem % HOURS_PER_DAY;

  canvas.print(years);
  canvas.print("y ");
  canvas.print(weeks);
  canvas.print("w ");
  canvas.print(days);
  canvas.print("d ");
  canvas.print(hours);
  canvas.print("h");
}

void drawInverterHeader(int yTop, int idx)
{
  InverterData& d = inv[idx];
  canvas.setCursor(2, yTop + 8);
  canvas.setTextColor(WHITE);
  canvas.print("Inv");
  canvas.print(idx + 1);
  canvas.print(": ");
  if (!d.valid) 
  {
    canvas.setTextColor(DARKGREY);
    canvas.print("no data yet");
  } 
  else if (!d.online) 
  {
    canvas.setTextColor(RED);
    canvas.print("OFFLINE (");
    canvas.print(agoString(d.lastGoodMillis));
    canvas.print(")");
  } 
  else 
  {
    canvas.setTextColor(GREEN);
    canvas.print("OK");
  }
  canvas.setTextColor(WHITE);
}

// Big-text at-a-glance page: Total Power, Inv1 Power, Inv2 Power.
// Uses a bigger font than the other pages, which is the point of this page.
// NOTE: renderDisplay() resets text size to 1 before calling any page
// function, so bumping it up here doesn't leak into the other pages.
void drawPowerSummaryPage() 
{
  canvas.setTextColor(DARKGREY);
  char pageIndicator[12];
  snprintf(pageIndicator, sizeof(pageIndicator), "%d/%d", currentPage + 1, totalPages);
  int piw = canvas.textWidth(pageIndicator);
  canvas.setCursor(SCREEN_W - piw - 2, 2);
  canvas.print(pageIndicator);

  canvas.setTextSize(2);

  int totalW = 0;
  bool anyValid = false;
  if (inv[0].valid) 
  {
    totalW += inv[0].gridW;
    anyValid = true;
  }
  if (inv[1].valid) 
  {
    totalW += inv[1].gridW;
    anyValid = true;
  }

  canvas.setTextColor(WHITE);
  canvas.setCursor(2, 24);
  canvas.print("Total Power: ");
  if (anyValid)
  {
    printPower(totalW, false);
  }
  else
  {
    canvas.print("--");
  }

  canvas.setCursor(2, 58);
  canvas.setTextColor(!inv[0].valid ? DARKGREY : (inv[0].online ? GREEN : RED));
  canvas.print("Inv1 Power:  ");
  if (inv[0].valid)
  {
    printPower(inv[0].gridW, false);
  }
  else
  {
    canvas.print("--");
  }

  canvas.setCursor(2, 92);
  canvas.setTextColor(!inv[1].valid ? DARKGREY : (inv[1].online ? GREEN : RED));
  canvas.print("Inv2 Power:  ");
  if (inv[1].valid)
  {
    printPower(inv[1].gridW, false);
  }
  else
  {
    canvas.print("--");
  }
}

void drawHomePage() 
{
  drawPageChrome("Grid Power / Voltage / Current");
  for (int i = 0; i < 2; i++) 
  {
    int yTop = (i == 0) ? SEC1_TOP : SEC2_TOP;
    drawInverterHeader(yTop, i);
    InverterData& d = inv[i];
    if (!d.valid) continue;
    canvas.setCursor(2, yTop + 22);
    canvas.print("Power: ");
    printPower(d.gridW, true);
    canvas.setCursor(2, yTop + 36);
    canvas.print("Voltage: ");
    canvas.print(d.line1V, 1);
    canvas.print(" V   Current: ");
    canvas.print(d.line1A, 1);
    canvas.print(" A");
  }
}

void drawTotalsPage() 
{
  drawPageChrome("Generated Energy (kWh)");
  for (int i = 0; i < 2; i++) 
  {
    int yTop = (i == 0) ? SEC1_TOP : SEC2_TOP;
    drawInverterHeader(yTop, i);
    InverterData& d = inv[i];
    if (!d.valid) continue;
    canvas.setCursor(2, yTop + 22);
    canvas.print("Total: ");
    printEnergy(d.totalKWh, 1);
    canvas.setCursor(2, yTop + 36);
    canvas.print("Today: ");
    printEnergy(d.todayKWh, 2);
  }
}

void drawRunTimePage()
{
  drawPageChrome("Running Time");
  for (int i = 0; i < 2; i++)
  {
    int yTop = (i == 0) ? SEC1_TOP : SEC2_TOP;
    drawInverterHeader(yTop, i);
    InverterData& d = inv[i];
    if (!d.valid) continue;
    canvas.setCursor(2, yTop + 22);
    canvas.print("Total: ");
    printDuration(d.totalHours);
    canvas.setCursor(2, yTop + 36);
    canvas.print("Today: ");
    canvas.print(d.todayHours, 1);
    canvas.print(" h");
  }
}

void drawPvPage(int channel) 
{
  char title[24];
  snprintf(title, sizeof(title), "PV%d Voltage & Current", channel);
  drawPageChrome(title);
  int idx = channel - 1;
  for (int i = 0; i < 2; i++) 
  {
    int yTop = (i == 0) ? SEC1_TOP : SEC2_TOP;
    drawInverterHeader(yTop, i);
    InverterData& d = inv[i];
    if (!d.valid) continue;
    canvas.setCursor(2, yTop + 22);
    if (d.pvValid[idx]) 
    {
      canvas.print("Volt: ");
      canvas.print(d.pvV[idx], 1);
      canvas.print(" V");
      canvas.setCursor(2, yTop + 36);
      canvas.print("Curr: ");
      canvas.print(d.pvI[idx], 2);
      canvas.print(" A");
    }
    else 
    {
      canvas.print("(not present)");
    }
  }
}

// Cardputer's own stats - always the last page, after any PV3 page.
void drawDeviceInfoPage()
{
  canvas.setTextColor(YELLOW);
  canvas.setCursor(2, TITLE_Y);
  canvas.print("Cardputer Info");

  char pageIndicator[12];
  snprintf(pageIndicator, sizeof(pageIndicator), "%d/%d", currentPage + 1, totalPages);
  int w = canvas.textWidth(pageIndicator);
  canvas.setTextColor(DARKGREY);
  canvas.setCursor(SCREEN_W - w - 2, TITLE_Y);
  canvas.print(pageIndicator);

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  canvas.setTextColor(WHITE);
  canvas.setCursor(2, 30);
  canvas.print("WiFi: ");
  canvas.print(wifiConnected ? WiFi.SSID() : "not connected");

  canvas.setCursor(2, 46);
  canvas.print("IP: ");
  canvas.print(wifiConnected ? WiFi.localIP().toString() : "--");

  int batteryLevel = M5Cardputer.Power.getBatteryLevel();    // 0-100 %
  int batteryMv = M5Cardputer.Power.getBatteryVoltage();     // mV
  canvas.setCursor(2, 62);
  canvas.print("Battery: ");
  canvas.print(batteryLevel);
  canvas.print("%  (");
  canvas.print(batteryMv / 1000.0f, 2);
  canvas.print("V)");
}

void renderDisplay()
{
  canvas.fillSprite(BLACK);
  canvas.setTextSize(1);

  switch (currentPage) 
  {
    case 0:
      drawPowerSummaryPage();
      break;
    case 1:
      drawHomePage();
      break;
    case 2:
      drawTotalsPage();
      break;
    case 3:
      drawRunTimePage();
      break;
    case 4:
      drawPvPage(1);
      break;
    case 5:
      drawPvPage(2);
      break;
    default:
    {
      int extraIdx = currentPage - NUM_FIXED_PAGES;
      if (extraIdx < numExtraPvPages)
      {
        drawPvPage(extraPvChannels[extraIdx]);
      }
      else
      {
        drawDeviceInfoPage();
      }
      break;
    }
  }

  canvas.pushSprite(0, 0);
}

// Returns true if a key press changed the current page.
bool handleKeyboard() 
{
  M5Cardputer.update();

  bool pageUp = M5Cardputer.Keyboard.isKeyPressed(';');
  bool pageDown = M5Cardputer.Keyboard.isKeyPressed('.');
  bool home = M5Cardputer.Keyboard.isKeyPressed(KEY_ENTER);

  bool changed = false;
  if (home && !prevHome)
  {
    currentPage = 0;
    changed = true;
  }
  else if (pageUp && !prevPageUp) 
  {
    currentPage = (currentPage + 1) % totalPages;
    changed = true;
  }
  else if (pageDown && !prevPageDown) 
  {
    currentPage = (currentPage - 1 + totalPages) % totalPages;
    changed = true;
  }

  prevPageUp = pageUp;
  prevPageDown = pageDown;
  prevHome = home;
  return changed;
}

void setup() 
{
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);  // true = enable keyboard
  M5Cardputer.Display.setRotation(1);

  canvas.setColorDepth(8);
  canvas.createSprite(SCREEN_W, SCREEN_H);
  canvas.setTextSize(1);

  connectWiFi();

  lastPollMillis = 0;         // force an immediate poll on first loop
  lastActivityMillis = millis();  // start the 2-min idle timer from power-on
}

void loop()
{
  bool pageChanged = handleKeyboard();
  bool anyKeyDown = M5Cardputer.Keyboard.isPressed() > 0;

  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED && now - lastReconnectMillis >= RECONNECT_INTERVAL_MS)
  {
    WiFi.reconnect();
    lastReconnectMillis = now;
  }

  bool justWoke = updateDisplayPower(anyKeyDown);

  bool polled = false;
  if (lastPollMillis == 0 || now - lastPollMillis >= currentPollInterval)
  {
    pollInverters();
    computeActivePages();
    lastPollMillis = now;
    polled = true;
  }

  if (displayOn && (pageChanged || polled || justWoke))
  {
    renderDisplay();
  }

  delay(30);  // keep the keyboard responsive
}
