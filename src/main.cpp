
// ================================================================
// ROBOT BLE - DÒ LINE BÁM PID (GỘP 1 FILE: main.cpp)
// Phần cứng: ESP32 + 2x TB6612 + 8 mắt QTR qua mux CD74HC4067
// Lệnh BLE: CAL → calibrate | AUTO → chạy | MANUAL → dừng
//           PID:kp,kd,base → chỉnh PID | GETPID → đọc PID
//           TURN:out,in → chỉnh tốc độ cua 90 độ
// ================================================================
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <EEPROM.h>

// ================================================================
// 1. ĐỘNG CƠ TB6612
// ================================================================
// IC TRÁI (AIN1 nối BIN1, AIN2 nối BIN2, PWMA nối PWMB)
#define L_PWM 25
#define L_IN1 26
#define L_IN2 27
// IC PHẢI
#define R_PWM 14
#define R_IN1 12 // GPIO12 là chân strapping: không để kéo lên cao lúc khởi động
#define R_IN2 13
// Chân STBY chung (nếu nối cứng lên 3.3V thì để -1)
#define TB6612_STBY -1

#define PWM_FREQ 2000
#define PWM_RES 8
#define L_PWM_CH 0
#define R_PWM_CH 1

// Đặt 1 nếu bánh tương ứng quay ngược chiều mong muốn
#define REVERSE_LEFT 0
#define REVERSE_RIGHT 0

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_RES)
#define PWM_WRITE(pin, ch, d) ledcWrite(pin, d)
#else
#define PWM_ATTACH(pin, ch)                                                    \
    do {                                                                       \
        ledcSetup(ch, PWM_FREQ, PWM_RES);                                      \
        ledcAttachPin(pin, ch);                                                \
    } while (0)
#define PWM_WRITE(pin, ch, d) ledcWrite(ch, d)
#endif

void tb6612_coast() {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    PWM_WRITE(L_PWM, L_PWM_CH, 0);
    PWM_WRITE(R_PWM, R_PWM_CH, 0);
}

void tb6612_short_brake() {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, HIGH);
    PWM_WRITE(L_PWM, L_PWM_CH, 255);
    PWM_WRITE(R_PWM, R_PWM_CH, 255);
}

void tb6612_init() {
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    PWM_ATTACH(L_PWM, L_PWM_CH);
    PWM_ATTACH(R_PWM, R_PWM_CH);
#if TB6612_STBY >= 0
    pinMode(TB6612_STBY, OUTPUT);
    digitalWrite(TB6612_STBY, HIGH);
#endif
    tb6612_coast();
}

static void driveOne(int in1, int in2, int pin, int ch, int spd, bool rev) {
    spd = constrain(spd, -255, 255);
    if (rev)
        spd = -spd;
    if (spd > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (spd < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    PWM_WRITE(pin, ch, abs(spd));
}

// speed > 0: tiến, < 0: lùi, thang -255..255
void speed_run(int left, int right) {
    driveOne(L_IN1, L_IN2, L_PWM, L_PWM_CH, left, REVERSE_LEFT);
    driveOne(R_IN1, R_IN2, R_PWM, R_PWM_CH, right, REVERSE_RIGHT);
}

// ================================================================
// 2. CẢM BIẾN QTR 8 MẮT QUA MUX (class MyQTR8C)
// Sơ đồ: S0=IO18, S1=IO19, S2=IO23, S3=GND, EN=GND, SIG=IO32, L1..L8 = C0..C7
// ================================================================
#define QTR_USE_GRAY 1
#if QTR_USE_GRAY
#define QTR_ORDER {0, 1, 3, 2, 6, 7, 5, 4}
#else
#define QTR_ORDER {0, 1, 2, 3, 4, 5, 6, 7}
#endif

#define QTR_SETTLE_US 80  // chờ sau khi đổi kênh (tối ưu tốc độ cao: 80us)
#define QTR_DUMMY 1       // số lần đọc bỏ (1 lần để xả tụ ADC)
#define QTR_SAMPLES 2     // số lần đọc lấy trung bình (2 mẫu lọc nhiễu nhanh)
#define QTR_MIN_RANGE 200 // max-min tối thiểu để coi kênh đã calib tốt

#define QTR_EEPROM_SIZE 64
#define QTR_EEPROM_ADDR 0
#define QTR_EEPROM_MAGIC 0x51A8

class MyQTR8C {
  public:
    MyQTR8C(const uint8_t *selPins, uint8_t sigPin, bool invert = false)
        : _sig(sigPin), _invert(invert) {
        for (int i = 0; i < 3; i++)
            _sel[i] = selPins[i];
        resetCalibration();
    }

    void begin() {
        for (int i = 0; i < 3; i++) {
            pinMode(_sel[i], OUTPUT);
            digitalWrite(_sel[i], LOW);
        }
        pinMode(_sig, INPUT);
        analogReadResolution(12);
        analogSetPinAttenuation(_sig, ADC_11db);
        _cur = 0;
        EEPROM.begin(QTR_EEPROM_SIZE);
    }

    void resetCalibration() {
        for (int i = 0; i < 8; i++) {
            _min[i] = 4095;
            _max[i] = 0;
        }
    }

    void calibrate() {
        for (int k = 0; k < 8; k++) {
            uint8_t i = GRAY[k];
            uint16_t v = readRaw(i);
            if (v < _min[i])
                _min[i] = v;
            if (v > _max[i])
                _max[i] = v;
        }
    }

    bool channelOk(uint8_t ch) const {
        return _max[ch] > _min[ch] && (_max[ch] - _min[ch]) >= QTR_MIN_RANGE;
    }

    void saveCalibration() {
        int addr = QTR_EEPROM_ADDR;
        EEPROM.put(addr, (uint16_t)QTR_EEPROM_MAGIC);
        addr += sizeof(uint16_t);
        for (int i = 0; i < 8; i++) {
            EEPROM.put(addr, _min[i]);
            addr += sizeof(uint16_t);
        }
        for (int i = 0; i < 8; i++) {
            EEPROM.put(addr, _max[i]);
            addr += sizeof(uint16_t);
        }
        EEPROM.commit();
    }

    bool loadCalibration() {
        int addr = QTR_EEPROM_ADDR;
        uint16_t magic = 0;
        EEPROM.get(addr, magic);
        addr += sizeof(uint16_t);
        if (magic != QTR_EEPROM_MAGIC)
            return false;
        uint16_t mn[8], mx[8];
        for (int i = 0; i < 8; i++) {
            EEPROM.get(addr, mn[i]);
            addr += sizeof(uint16_t);
        }
        for (int i = 0; i < 8; i++) {
            EEPROM.get(addr, mx[i]);
            addr += sizeof(uint16_t);
        }
        for (int i = 0; i < 8; i++) {
            _min[i] = mn[i];
            _max[i] = mx[i];
        }
        return true;
    }

    // values[] = 0..1000 (1000 = thấy line), trả vị trí 0..7000
    uint16_t readLine(uint16_t *values) {
        for (int k = 0; k < 8; k++) {
            uint8_t i = GRAY[k];
            values[i] = normalize(i, readRaw(i));
        }
        long sum = 0, weighted = 0;
        bool seen = false;
        for (int i = 0; i < 8; i++) {
            uint16_t v = values[i];
            if (v > 200)
                seen = true;
            if (v > 50) {
                weighted += (long)v * i * 1000;
                sum += v;
            }
        }
        if (!seen || sum == 0) {
            if (_last < 2000)
                _last = 0;
            else if (_last > 5000)
                _last = 7000;
        } else {
            _last = weighted / sum;
        }
        return _last;
    }

    uint16_t raw(uint8_t ch) { return readRaw(ch); }

  private:
    uint8_t _sel[3], _sig;
    bool _invert;
    uint16_t _min[8], _max[8];
    uint16_t _last = 3500;
    uint8_t _cur = 0;
    const uint8_t GRAY[8] = QTR_ORDER;

    void selectChannel(uint8_t ch) {
        uint8_t diff = ch ^ _cur;
        for (int b = 0; b < 3; b++)
            if (diff & (1 << b))
                digitalWrite(_sel[b], (ch >> b) & 1);
        _cur = ch;
    }

    uint16_t readRaw(uint8_t ch) {
        selectChannel(ch);
        delayMicroseconds(QTR_SETTLE_US);
        for (int d = 0; d < QTR_DUMMY; d++)
            analogRead(_sig);
        uint32_t acc = 0;
        for (int k = 0; k < QTR_SAMPLES; k++)
            acc += analogRead(_sig);
        return acc / QTR_SAMPLES;
    }

    uint16_t normalize(uint8_t i, uint16_t raw) {
        if (!channelOk(i))
            return 0;
        uint16_t range = _max[i] - _min[i];
        long v = ((long)(raw - min(raw, _min[i])) * 1000) / range;
        v = constrain(v, 0, 1000);
        return _invert ? 1000 - v : v;
    }
};

// ================================================================
// 3. BLE UUIDs
// ================================================================
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ================================================================
// 4. CẢM BIẾN: CẤU HÌNH + ĐỌC
// ================================================================
const uint8_t S_PINS[] = {18, 19, 23}; // S0, S1, S2
const uint8_t SIG_PIN = 32;

// Đặt 1 nếu cảm biến đọc NGƯỢC trái/phải
#define REVERSE_SENSOR 0
// true = đảo màu (đen thành trắng, trắng thành đen)
#define INVERT_COLOR true

MyQTR8C qtr(S_PINS, SIG_PIN, INVERT_COLOR);
uint16_t sensorValues[8];
uint8_t currentPattern = 0;
unsigned int lastLinePos = 3500;

#define SETPOINT 3500
#define SENSOR_THRESHOLD 500 // ngưỡng "thấy line" (0..1000)
#define DEBUG_RAW 0          // 1 = bật lệnh TEST gửi giá trị 8 mắt

int docSensor() {
    unsigned int pos = qtr.readLine(sensorValues);
#if REVERSE_SENSOR
    lastLinePos = 7000 - pos;
#else
    lastLinePos = pos;
#endif
    currentPattern = 0;
    int black = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] > SENSOR_THRESHOLD) {
#if REVERSE_SENSOR
            currentPattern |= (1 << i);
#else
            currentPattern |= (1 << (7 - i));
#endif
            black++;
        }
    }
    return black;
}

// ================================================================
// 5. PID & THAM SỐ
// ================================================================
float KP = 0.04f, KD = 0.3f;
int BASE = 150;
// Cua góc vuông: bánh ngoài = BASE + TURN_OUT, bánh trong = BASE - TURN_IN
int TURN_OUT = 70, TURN_IN = 90;
const int BOOST = 20, BOOST_THRESHOLD = 150;

// Đoạn nét đứt (00000000) và vạch kết thúc (11111111)
#define GAP_STRAIGHT_MS 100
#define GAP_MAX_COUNT 5
#define GAP_ERR_MAX 1500
#define END_BLACK_COUNT 1
#define END_CONFIRM 3
int cnt = 0;
int cntDen = 0;
bool inGap = false, inFullBlack = false;
int fullBlackFrames = 0;
unsigned long gapStart = 0;

// ================================================================
// 6. TRẠNG THÁI & TIMER
// ================================================================
bool runLine = false;
#if DEBUG_RAW
bool testMode = false;
unsigned long lastRawTime = 0;
#endif
int speed_dk = 120;
int lastError = 0;

bool calibrated = false;
bool calibrating = false;
unsigned long calEnd = 0;
#define CAL_TIME 5000

const char *currentModeName = "IDLE";
const char *currentCaseName = "-";
unsigned long lastStatusTime = 0;
#define STATUS_INTERVAL 150

// Hardware Timer bấm giờ vòng đua
volatile uint64_t raceStartTime = 0;
volatile bool raceRunning = false;

// ================================================================
// 7. NÚT BẤM (GPIO 16) & CÒI BUZZER (GPIO 17)
// ================================================================
#define PIN_BUTTON 16
#define PIN_BUZZER 17

enum BuzzerPattern {
    BUZZ_NONE,
    BUZZ_SHORT,   // 1 bíp ngắn (100ms) - Bắt đầu Calib / Dừng xe
    BUZZ_DOUBLE,  // 2 bíp (100ms ON, 80ms OFF, 120ms ON) - Hoàn thành Calib / Về đích
    BUZZ_LONG     // 1 bíp dài (350ms) - Bắt đầu chạy AUTO
};

struct BuzzerStep {
    uint8_t state;
    uint16_t duration;
};

BuzzerStep buzzQueue[6];
volatile uint8_t buzzQueueLen = 0;
volatile uint8_t buzzQueueIdx = 0;
volatile unsigned long buzzStepTimer = 0;

void buzzerPlay(BuzzerPattern pat) {
    buzzQueueIdx = 0;
    if (pat == BUZZ_SHORT) {
        buzzQueue[0] = {1, 100};
        buzzQueue[1] = {0, 0};
        buzzQueueLen = 2;
    } else if (pat == BUZZ_DOUBLE) {
        buzzQueue[0] = {1, 100};
        buzzQueue[1] = {0, 80};
        buzzQueue[2] = {1, 120};
        buzzQueue[3] = {0, 0};
        buzzQueueLen = 4;
    } else if (pat == BUZZ_LONG) {
        buzzQueue[0] = {1, 350};
        buzzQueue[1] = {0, 0};
        buzzQueueLen = 2;
    } else {
        buzzQueueLen = 0;
        digitalWrite(PIN_BUZZER, LOW);
        return;
    }
    digitalWrite(PIN_BUZZER, buzzQueue[0].state);
    buzzStepTimer = millis();
}

void buzzerUpdate() {
    if (buzzQueueIdx < buzzQueueLen) {
        if (millis() - buzzStepTimer >= buzzQueue[buzzQueueIdx].duration) {
            buzzQueueIdx = buzzQueueIdx + 1;
            if (buzzQueueIdx < buzzQueueLen && buzzQueue[buzzQueueIdx].duration > 0) {
                digitalWrite(PIN_BUZZER, buzzQueue[buzzQueueIdx].state);
                buzzStepTimer = millis();
            } else {
                digitalWrite(PIN_BUZZER, LOW);
            }
        }
    }
}

// Khai báo nguyên mẫu các hàm hành động
void startCalibration();
void startAuto();
void stopManual();

bool lastButtonRaw = HIGH;
unsigned long buttonChangeTime = 0;
bool buttonPressedDebounced = false;
unsigned long buttonPressStartTime = 0;
bool longPressTriggered = false;
bool sessionCalibrated = false; // Lần đầu bật nguồn bấm nút sẽ ưu tiên Calibrate

void updateButton() {
    bool raw = digitalRead(PIN_BUTTON);
    if (raw != lastButtonRaw) {
        lastButtonRaw = raw;
        buttonChangeTime = millis();
    }

    if (millis() - buttonChangeTime >= 40) { // 40ms debounce chống dội phím
        if (raw == LOW && !buttonPressedDebounced) {
            buttonPressedDebounced = true;
            buttonPressStartTime = millis();
            longPressTriggered = false;
        } else if (raw == HIGH && buttonPressedDebounced) {
            buttonPressedDebounced = false;
            unsigned long pressDuration = millis() - buttonPressStartTime;
            if (!longPressTriggered && pressDuration >= 40 && pressDuration < 1200) {
                // Nhấn nhả bình thường (Click)
                if (runLine) {
                    // Xe đang chạy -> Dừng khẩn cấp
                    stopManual();
                } else if (calibrating) {
                    // Xe đang calib -> Bỏ qua
                } else {
                    // Xe đang dừng
                    if (!sessionCalibrated) {
                        // Lần 1: Bắt đầu Calibrate
                        startCalibration();
                    } else {
                        // Lần 2 (hoặc sau khi đã calib): Chạy AUTO
                        startAuto();
                    }
                }
            }
        } else if (raw == LOW && buttonPressedDebounced && !longPressTriggered) {
            // Nhấn giữ > 1.5s để ép Calibrate lại từ đầu bất cứ lúc nào
            if (millis() - buttonPressStartTime >= 1500) {
                longPressTriggered = true;
                if (!runLine && !calibrating) {
                    startCalibration();
                }
            }
        }
    }
}

// ================================================================
// 8. ĐIỀU KHIỂN ĐỘNG CƠ
// ================================================================
void move(int left, int right) {
    speed_run(constrain(left, -255, 255), constrain(right, -255, 255));
}

void stopMotor() { tb6612_coast(); }

void movePID(int base, int adjust) {
    int L = base + adjust, R = base - adjust;
    if (adjust > BOOST_THRESHOLD)
        L += BOOST;
    if (adjust < -BOOST_THRESHOLD)
        R += BOOST;
    move(L, R);
}

// ================================================================
// 8. BLE SEND
// ================================================================
void bleSend(const char *msg) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue((uint8_t *)msg, strlen(msg));
        pTxCharacteristic->notify();
        delay(5);
    }
    Serial.println(msg);
}

void sendStatus() {
    if (!deviceConnected)
        return;
    if (millis() - lastStatusTime < STATUS_INTERVAL)
        return;
    lastStatusTime = millis();

    char patStr[9];
    for (int i = 7; i >= 0; i--)
        patStr[7 - i] = (currentPattern & (1 << i)) ? '1' : '0';
    patStr[8] = '\0';

    float curLapTime = 0.0f;
    if (raceRunning) {
        curLapTime = (float)(esp_timer_get_time() - raceStartTime) / 1000000.0f;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"%s\",\"case\":\"%s\","
             "\"pos\":%u,\"pat\":\"%s\","
             "\"spd\":%d,\"err\":%d,\"cal\":%d,\"time\":%.2f}",
             currentModeName, currentCaseName, lastLinePos,
             patStr, speed_dk, lastError,
             calibrating ? 2 : (calibrated ? 1 : 0), curLapTime);
    pTxCharacteristic->setValue((uint8_t *)buf, strlen(buf));
    pTxCharacteristic->notify();
}

// Kết thúc line: phanh ghìm rồi thả trôi + chốt giờ hoàn thành
void endLine() {
    uint64_t finishTime = esp_timer_get_time();
    tb6612_short_brake();
    vTaskDelay(pdMS_TO_TICKS(150));
    tb6612_coast();
    runLine = false;
    currentModeName = "IDLE";
    currentCaseName = "END_LINE";

    if (raceRunning) {
        raceRunning = false;
        float elapsedSeconds = (float)(finishTime - raceStartTime) / 1000000.0f;
        char lapMsg[64];
        snprintf(lapMsg, sizeof(lapMsg), "LOG:RACE_FINISH:%.3f", elapsedSeconds);
        bleSend(lapMsg);
        buzzerPlay(BUZZ_DOUBLE);
    } else {
        bleSend("LOG:END_LINE");
    }
}

// ================================================================
// 9. DÒ LINE BÁM PID
// ================================================================
void lineFollowPID() {
    int blackCount = docSensor();

    if (currentPattern == 0b11111111) {
        fullBlackFrames++;
        if (fullBlackFrames >= END_CONFIRM && !inFullBlack) {
            inFullBlack = true;
            cntDen++;
            if (cntDen >= END_BLACK_COUNT) {
                endLine();
                return;
            }
        }
    } else {
        fullBlackFrames = 0;
        inFullBlack = false;
    }
    if (blackCount != 0)
        inGap = false;

    // Cua 90 độ
    if ((currentPattern & 0b00000001) && blackCount < 5) {
        currentCaseName = "TURN_RIGHT";
        move(BASE + TURN_OUT, BASE - TURN_IN);
        return;
    }
    if ((currentPattern & 0b10000000) && blackCount < 5) {
        currentCaseName = "TURN_LEFT";
        move(BASE - TURN_IN, BASE + TURN_OUT);
        return;
    }

    // Mất line
    if (blackCount == 0) {
        if (!inGap && cnt < GAP_MAX_COUNT && abs(lastError) <= GAP_ERR_MAX) {
            inGap = true;
            gapStart = millis();
            cnt++;
        }
        if (inGap && millis() - gapStart < GAP_STRAIGHT_MS) {
            currentCaseName = "GAP_STRAIGHT";
            move(BASE, BASE);
            return;
        }
        currentCaseName = "LOST_LINE";
        if (lastError > 0)
            move(BASE + BOOST, -BASE);
        else
            move(-BASE, BASE + BOOST);
        return;
    }

    // PID
    currentCaseName = "PID_FOLLOW";
    int error = (int)lastLinePos - SETPOINT;
    int adjust = (int)(error * KP) + (int)(KD * (error - lastError));
    lastError = error;
    movePID(BASE, adjust);
}

// ================================================================
// CÁC HÀM ĐIỀU KHIỂN CHẾ ĐỘ
// ================================================================
void startCalibration() {
    runLine = false;
    raceRunning = false;
    stopMotor();
    calibrated = false;
    calibrating = true;
    calEnd = millis() + CAL_TIME;
    qtr.resetCalibration();
    currentPattern = 0;
    currentModeName = "CALIB";
    currentCaseName = "CALIBRATING";
    sessionCalibrated = true;
    bleSend("LOG:CAL_START");
    buzzerPlay(BUZZ_SHORT);
}

void startAuto() {
    if (!calibrated) {
        bleSend("LOG:NEED_CAL");
        buzzerPlay(BUZZ_DOUBLE);
        return;
    }
    runLine = true;
    lastError = 0;
    lastLinePos = SETPOINT;
    cnt = 0;
    cntDen = 0;
    inGap = false;
    fullBlackFrames = 0;
    inFullBlack = false;
    currentModeName = "AUTO";
    currentCaseName = "PID_FOLLOW";
    raceStartTime = esp_timer_get_time();
    raceRunning = true;
    bleSend("LOG:AUTO_START");
    buzzerPlay(BUZZ_LONG);
}

void stopManual() {
    runLine = false;
    raceRunning = false;
    stopMotor();
    currentModeName = "MANUAL";
    currentCaseName = "IDLE";
    bleSend("LOG:MODE_MANUAL");
    buzzerPlay(BUZZ_SHORT);
}

// ================================================================
// 10. LỆNH BLE
// ================================================================
void handleBLECommand(String cmd) {
    cmd.trim();

    if (cmd.startsWith("SPD:")) {
        speed_dk = constrain(cmd.substring(4).toInt(), 30, 255);
        char buf[30];
        snprintf(buf, sizeof(buf), "LOG:SPD=%d", speed_dk);
        bleSend(buf);
        return;
    }

#if DEBUG_RAW
    if (cmd == "TEST") {
        testMode = !testMode;
        runLine = false;
        stopMotor();
        bleSend(testMode ? "LOG:TEST_ON" : "LOG:TEST_OFF");
        return;
    }
#endif

    if (cmd == "CAL") {
        startCalibration();
        return;
    }

    if (cmd.startsWith("PID:")) {
        float kp, kd;
        int base;
        if (sscanf(cmd.c_str() + 4, "%f,%f,%d", &kp, &kd, &base) == 3) {
            KP = kp;
            KD = kd;
            BASE = constrain(base, 0, 255);
            bleSend("LOG:PID_OK");
        } else
            bleSend("LOG:PID_ERR");
        return;
    }
    if (cmd.startsWith("TURN:")) {
        int o, i;
        if (sscanf(cmd.c_str() + 5, "%d,%d", &o, &i) == 2) {
            TURN_OUT = constrain(o, 0, 255);
            TURN_IN = constrain(i, -255, 400);
            bleSend("LOG:TURN_OK");
        } else
            bleSend("LOG:TURN_ERR");
        return;
    }
    if (cmd == "GETPID") {
        char buf[80];
        snprintf(buf, sizeof(buf), "PID:%.4f,%.4f,%d,%d,%d", KP, KD, BASE,
                 TURN_OUT, TURN_IN);
        bleSend(buf);
        return;
    }

    if (cmd == "AUTO") {
        startAuto();
        return;
    }
    if (cmd == "MANUAL") {
        stopManual();
        return;
    }

    if (!runLine) {
        if (cmd == "FWD")
            move(speed_dk, speed_dk);
        else if (cmd == "BWD")
            move(-speed_dk, -speed_dk);
        else if (cmd == "LFT")
            move(-speed_dk, speed_dk);
        else if (cmd == "RGT")
            move(speed_dk, -speed_dk);
        else if (cmd == "STP")
            stopMotor();
    }
}

// ================================================================
// 11. BLE CALLBACKS
// ================================================================
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String v = pChar->getValue().c_str();
        if (v.length() > 0)
            handleBLECommand(v);
    }
};
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *p) override {
        deviceConnected = true;
        bleSend("LOG:BLE_CONNECTED");
    }
    void onDisconnect(BLEServer *p) override {
        deviceConnected = false;
        stopMotor();
        runLine = false;
        raceRunning = false;
        BLEDevice::startAdvertising();
    }
};

// ================================================================
// 12. FREERTOS TASKS (TÁCH 2 CORE)
// ================================================================

// TASK 1 (Core 1): Chuyên xử lý Động cơ, Đọc Cảm biến & Thuật toán PID bám line
void taskMotorPID(void *pvParameters) {
    for (;;) {
        if (calibrating) {
            qtr.calibrate();
            if (millis() >= calEnd) {
                calibrating = false;
                calibrated = true;
                currentModeName = "IDLE";
                currentCaseName = "-";
                char bad[48] = "LOG:CAL_WARN mat:";
                bool anyBad = false;
                for (int c = 0; c < 8; c++) {
                    if (!qtr.channelOk(c)) {
                        char t[4];
                        snprintf(t, sizeof(t), " %d", c + 1);
                        strncat(bad, t, sizeof(bad) - strlen(bad) - 1);
                        anyBad = true;
                    }
                }
                if (anyBad)
                    bleSend(bad);
                qtr.saveCalibration();
                sessionCalibrated = true;
                bleSend("LOG:CAL_DONE");
                buzzerPlay(BUZZ_DOUBLE);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (runLine) {
            lineFollowPID();
            vTaskDelay(pdMS_TO_TICKS(1)); // Nhường 1ms cho watchdog
        } else {
            if (calibrated)
                docSensor();
            else
                currentPattern = 0;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// TASK 2 (Core 0): Chuyên giao tiếp Bluetooth BLE, Quét nút bấm & Còi Buzzer
void taskBLETelemetry(void *pvParameters) {
    for (;;) {
        updateButton();
        buzzerUpdate();

        if (!deviceConnected && oldDeviceConnected) {
            vTaskDelay(pdMS_TO_TICKS(300));
            BLEDevice::startAdvertising();
            oldDeviceConnected = false;
        }
        if (deviceConnected && !oldDeviceConnected)
            oldDeviceConnected = true;

#if DEBUG_RAW
        if (testMode && !calibrating && !runLine) {
            docSensor();
            if (deviceConnected && millis() - lastRawTime > 200) {
                lastRawTime = millis();
                char raw[64];
                snprintf(raw, sizeof(raw), "RAW:%u,%u,%u,%u,%u,%u,%u,%u",
                         sensorValues[0], sensorValues[1], sensorValues[2],
                         sensorValues[3], sensorValues[4], sensorValues[5],
                         sensorValues[6], sensorValues[7]);
                bleSend(raw);
            }
        }
#endif

        sendStatus();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================================================================
// 13. SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    tb6612_init();

    // Khởi tạo Chân Nút bấm (GPIO 16) và Còi Buzzer (GPIO 17)
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    buzzerPlay(BUZZ_SHORT); // Bíp 1 tiếng ngắn khi khởi động xong

    BLEDevice::init("ROBOT_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pSvc = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRx = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE |
                                    BLECharacteristic::PROPERTY_WRITE_NR);
    pRx->setCallbacks(new MyCallbacks());
    pSvc->start();
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising... ROBOT_BLE");

    qtr.begin();
    if (qtr.loadCalibration()) {
        calibrated = true;
        Serial.println("Da nap calib tu EEPROM");
    }
    Serial.println("READY! Gui CAL de calibrate, sau do AUTO de chay");
    bleSend("LOG:ROBOT_READY");

    // Khởi tạo FreeRTOS Task trên 2 Core độc lập:
    // Core 1: Ưu tiên cao nhất cho PID & Động cơ (tránh giật lag)
    xTaskCreatePinnedToCore(
        taskMotorPID,
        "TaskMotorPID",
        4096,
        NULL,
        3,
        NULL,
        1
    );

    // Core 0: Xử lý truyền nhận BLE & Telemetry
    xTaskCreatePinnedToCore(
        taskBLETelemetry,
        "TaskBLETelemetry",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    Serial.println(">> FreeRTOS Dual-Core Tasks Started! <<");
}

// ================================================================
// 14. LOOP (Đã chuyển toàn bộ xử lý vào 2 Task FreeRTOS)
// ================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
