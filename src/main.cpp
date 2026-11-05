#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// SSD1306 128x64 over I2C. ESP32-C3 Super Mini default pins: SDA = 8, SCL = 9
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 9, /* data=*/ 8);

#define BUTTON_PIN 3

// =====================================================================
//  Layout / tuning
// =====================================================================
static const unsigned long FRAME_MS = 20;      // target frame time
static const unsigned long HAPPY_MS = 2000;    // how long "happy" lasts after a press

static const int   EYE_CX_L = 42;              // base centre of the left eye
static const int   EYE_CX_R = 86;              // base centre of the right eye
static const int   EYE_CY   = 32;
static const float BASE_W   = 22;
static const float BASE_H   = 28;
static const float BASE_R   = 7;
static const float LOOK_X   = 11;              // max gaze shift in pixels
static const float LOOK_Y   = 8;

// =====================================================================
//  Eye shape. Every field is smoothed towards its target each frame,
//  so nothing ever "snaps" between expressions.
// =====================================================================
struct EyeShape {
    float w, h;      // size
    float x, y;      // offset from the eye centre
    float r;         // corner radius
    float cut;       // "smile" cut from the bottom (0 = none) -> arch-shaped happy eye
    float tilt;      // top lid slope in px: >0 inner corner lowered (focused), <0 outer corner lowered (sleepy)
};

static const int HEART_POS[4][2] = {
    {10, 32},   // Левый бок
    {118, 19},  // Правый бок
    {35, 53},   // Снизу слева
    {93, 60}    // Снизу справа
};

enum Pose { P_NORMAL, P_SQUINT, P_CURIOUS, P_SLEEPY, P_WIDE, P_FOCUSED, P_HAPPY_PRE, P_HAPPY };
enum Mode { M_IDLE, M_HAPPY };

static EyeShape pose[2];            // target shape of each eye for the current pose
static EyeShape cur[2];             // what is actually displayed
static Pose     curPose   = P_NORMAL;
static float    tauShape  = 110;    // ms, smaller = faster transition
static int      curiousSide = 0;    // which eye is the big one in the "curious" pose

// gaze (-1..1)
static float gazeX = 0, gazeY = 0, gazeTX = 0, gazeTY = 0;
static float tauGaze = 120;
static bool  lockGaze = false;      // true while an action controls the gaze itself
static unsigned long nextLookAt = 0;

// blink
static bool  blinkActive = false;
static bool  blinkPending2 = false; // a second blink follows (double blink)
static float blinkAmount = 0;       // 0 = open, 1 = closed
static unsigned long blinkStart = 0, nextBlinkAt = 0;
static float bClose = 70, bHold = 25, bOpen = 130;

// modes / idle actions
static Mode mode = M_IDLE;
static unsigned long happyStart = 0, happyUntil = 0;
static bool prevPressed = false;

static bool scanActive = false;
static unsigned long scanStart = 0;
static unsigned long nextActionAt = 0, actionEndAt = 0;

static unsigned long lastFrame = 0;

// =====================================================================
//  Helpers
// =====================================================================
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline void approach(float &c, float t, float a) { c += (t - c) * a; }

static void approachShape(EyeShape &c, const EyeShape &t, float a) {
    approach(c.w, t.w, a);   approach(c.h, t.h, a);
    approach(c.x, t.x, a);   approach(c.y, t.y, a);
    approach(c.r, t.r, a);   approach(c.cut, t.cut, a);
    approach(c.tilt, t.tilt, a);
}

static void setPose(Pose p) {
    curPose = p;
    const EyeShape n = { BASE_W, BASE_H, 0, 0, BASE_R, 0, 0 };
    pose[0] = n;
    pose[1] = n;
    tauShape = 110;

    switch (p) {
        case P_NORMAL:
            break;
        case P_SQUINT:
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 24, 15, 0, 2, 5, 0, 0 };
            tauShape = 90;
            break;
        case P_CURIOUS:
            pose[curiousSide]     = EyeShape{ 24, 36, 0, -2, 8, 0, 0 };   // big eye
            pose[1 - curiousSide] = EyeShape{ 22, 18, 0,  2, 6, 0, 3 };   // small, "raised brow" eye
            tauShape = 100;
            break;
        case P_SLEEPY:
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 24, 12, 0, 4, 4, 0, -2 };
            tauShape = 170;
            break;
        case P_WIDE:
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 26, 36, 0, 0, 10, 0, 0 };
            tauShape = 45;
            break;
        case P_FOCUSED:
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 24, 21, 0, 1, 4, 0, 4 };
            tauShape = 100;
            break;
        case P_HAPPY_PRE:   // short anticipation: eyes pop open a little before the smile
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 24, 31, 0, -2, 9, 0, 0 };
            tauShape = 45;
            break;
        case P_HAPPY:       // "^ ^" eyes: rounded box with a big ellipse cut out of the bottom
            for (int i = 0; i < 2; i++) pose[i] = EyeShape{ 26, 24, 0, -3, 10, 17, 0 };
            tauShape = 80;
            break;
    }
}

static void setGaze(float x, float y, float tau) {
    gazeTX = x; gazeTY = y; tauGaze = tau;
}

static void scheduleBlink(unsigned long now) {
    nextBlinkAt = now + random(2200, 5500);
}

static void startBlink(unsigned long now) {
    blinkActive = true;
    blinkStart = now;
    if (curPose == P_SLEEPY) { bClose = 170; bHold = 110; bOpen = 260; }   // lazy blink
    else                     { bClose = 70;  bHold = 25;  bOpen = 130; }
}

// =====================================================================
//  Happy
// =====================================================================
static void startHappy(unsigned long now) {
    mode = M_HAPPY;
    happyStart = now;
    happyUntil = now + HAPPY_MS;
    scanActive = false;
    lockGaze = true;
    blinkActive = false;
    blinkPending2 = false;
    setGaze(0, 0, 80);
    setPose(P_HAPPY_PRE);
}

static void endHappy(unsigned long now) {
    mode = M_IDLE;
    lockGaze = false;
    setPose(P_NORMAL);
    nextBlinkAt = now + 150;                    // blink right after the smile fades
    nextLookAt = now + 1500;
    nextActionAt = now + random(3500, 7000);
    actionEndAt = 0;
}

static void updateHappy(unsigned long now, bool pressed) {
    if (pressed && now + 1200 > happyUntil) happyUntil = now + 1200;   // holding the button keeps it going
    if (now >= happyUntil) { endHappy(now); return; }
    if (curPose == P_HAPPY_PRE && now - happyStart >= 150) setPose(P_HAPPY);
}

// =====================================================================
//  Idle: random mini-actions between the normal state
// =====================================================================
static void startIdleAction(unsigned long now) {
    int r = random(100);
    lockGaze = true;
    scanActive = false;

    if (r < 16) {                               // look left - right - centre
        scanActive = true;
        scanStart = now;
        actionEndAt = now + 1000;
        setPose(P_NORMAL);
    } else if (r < 32) {                        // squint
        lockGaze = false;
        actionEndAt = now + random(1500, 2500);
        setPose(P_SQUINT);
    } else if (r < 52) {                        // curious: one eye bigger, look up at it
        curiousSide = random(2);
        actionEndAt = now + random(1600, 2600);
        setPose(P_CURIOUS);
        setGaze(curiousSide ? 0.6f : -0.6f, -0.5f, 90);
    } else if (r < 66) {                        // sleepy
        actionEndAt = now + random(2200, 3200);
        setPose(P_SLEEPY);
        setGaze(0, 0.5f, 250);
    } else if (r < 80) {                        // surprised
        actionEndAt = now + random(900, 1400);
        setPose(P_WIDE);
        setGaze(0, 0, 45);
    } else {                                    // focused
        actionEndAt = now + random(1500, 2500);
        setPose(P_FOCUSED);
        setGaze(0, 0, 100);
    }
}

static void endIdleAction(unsigned long now) {
    scanActive = false;
    lockGaze = false;
    actionEndAt = 0;
    setPose(P_NORMAL);
    nextLookAt = now + 250;
    nextActionAt = now + random(3000, 7000);
}

static void updateIdle(unsigned long now) {
    if (actionEndAt == 0) {                     // nothing running
        if (now >= nextActionAt) startIdleAction(now);
        return;
    }
    if (scanActive) {
        unsigned long e = now - scanStart;
        if (e < 300)      setGaze(-1.0f, 0, 60);
        else if (e < 650) setGaze( 1.0f, 0, 60);
        else              setGaze( 0.0f, 0, 90);
    }
    if (now >= actionEndAt) endIdleAction(now);
}

// =====================================================================
//  Random look around (saccades)
// =====================================================================
static void updateLook(unsigned long now) {
    if (lockGaze || mode == M_HAPPY) return;
    if (now < nextLookAt) return;

    float gx, gy;
    if (random(100) < 30) {                     // often come back to the middle
        gx = random(-15, 16) / 100.0f;
        gy = random(-15, 16) / 100.0f;
    } else {
        gx = random(-100, 101) / 100.0f;
        gy = random(-70, 71) / 100.0f;
    }
    // sometimes a quick dart, sometimes a lazy drift
    setGaze(gx, gy, random(100) < 35 ? 45.0f : 120.0f);
    nextLookAt = now + random(900, 3800);
}

// =====================================================================
//  Blink: fast eased close, tiny hold, slower eased open (+ occasional double blink)
// =====================================================================
static void updateBlink(unsigned long now) {
    if (mode == M_HAPPY) { blinkActive = false; blinkAmount = 0; return; }

    if (!blinkActive) {
        blinkAmount = 0;
        if (now >= nextBlinkAt) {
            if (!blinkPending2) blinkPending2 = (random(100) < 18);   // maybe blink twice
            else                blinkPending2 = false;
            startBlink(now);
        }
        return;
    }

    float e = (float)(now - blinkStart);
    if (e < bClose) {
        float p = e / bClose;
        blinkAmount = p * p;                    // accelerates shut
    } else if (e < bClose + bHold) {
        blinkAmount = 1.0f;
    } else if (e < bClose + bHold + bOpen) {
        float p = (e - bClose - bHold) / bOpen;
        blinkAmount = (1.0f - p) * (1.0f - p);  // opens fast, then eases out
    } else {
        blinkActive = false;
        blinkAmount = 0;
        if (blinkPending2) nextBlinkAt = now + 90;     // second blink of a double blink
        else               scheduleBlink(now);
    }
}

// =====================================================================
//  Drawing
// =====================================================================

// Координаты для 4 маленьких сердечек: {x, y}
static void drawMiniHeart(int xm, int ym, int offsetX, int offsetY) {
    xm += offsetX;
    ym += offsetY;
    u8g2.setDrawColor(1);
    u8g2.drawBox(xm - 2, ym - 2, 2, 1); 
    u8g2.drawBox(xm + 1, ym - 2, 2, 1); 
    u8g2.drawBox(xm - 3, ym - 1, 7, 1); 
    u8g2.drawBox(xm - 3, ym,     7, 1); 
    u8g2.drawBox(xm - 2, ym + 1, 5, 1); 
    u8g2.drawBox(xm - 1, ym + 2, 3, 1); 
    u8g2.drawPixel(xm,   ym + 3);       
}



static void drawEye(float cx, float cy, const EyeShape &s, float blink, bool isRight) {
    // blink: squash to a thin line and widen a little
    float h = s.h + (3.0f - s.h) * blink;
    float w = s.w + 4.0f * blink;

    int wi = (int)lroundf(w); if (wi < 3) wi = 3;
    int hi = (int)lroundf(h); if (hi < 2) hi = 2;

    int x0 = (int)lroundf(cx + s.x - wi / 2.0f);
    int y0 = (int)lroundf(cy + s.y - hi / 2.0f);
    x0 = clampi(x0, 0, 128 - wi);               // u8g2 uses unsigned coords: never go negative
    y0 = clampi(y0, 0, 64 - hi);

    int ri = (int)lroundf(s.r);
    int rmax = ((wi < hi ? wi : hi) - 1) / 2;
    ri = clampi(ri, 0, rmax);

    u8g2.setDrawColor(1);
    if (ri > 0) u8g2.drawRBox(x0, y0, wi, hi, ri);
    else        u8g2.drawBox(x0, y0, wi, hi);

    // happy "smile" cut: erase an ellipse from the bottom, clipped to this eye's column
    float cut = s.cut * (1.0f - blink);
    if (cut > 0.5f) {
        int ry  = (int)(hi * 0.8f);
        int rx  = wi / 2 + 3;
        int ecx = x0 + wi / 2;
        int ecy = y0 + hi + ry - (int)lroundf(cut);
        u8g2.setClipWindow(x0, 0, x0 + wi, 63);
        u8g2.setDrawColor(0);
        u8g2.drawFilledEllipse(ecx, ecy, rx, ry);
        u8g2.setDrawColor(1);
        u8g2.setMaxClipWindow();
    }

    // lid slope: erase a triangle from the top
    float tilt = s.tilt * (1.0f - blink);
    int t = (int)lroundf(fabsf(tilt));
    if (t >= 1) {
        bool innerIsRight = !isRight;           // the left eye's inner side is its right edge
        bool lowerRight = (tilt > 0) ? innerIsRight : !innerIsRight;
        int xl = x0, xr = x0 + wi - 1;
        u8g2.setDrawColor(0);
        if (lowerRight) u8g2.drawTriangle(xl, y0, xr, y0, xr, y0 + t);
        else            u8g2.drawTriangle(xl, y0, xr, y0, xl, y0 + t);
        u8g2.setDrawColor(1);
    }
}

static void render() {
    unsigned long now = millis();
    u8g2.clearBuffer();
    for (int i = 0; i < 2; i++) {
        EyeShape s = cur[i];
        float side = (i == 1) ? 1.0f : -1.0f;
        // slight perspective: the eye on the side we look towards gets a bit smaller
        s.h *= (1.0f - 0.12f * gazeX * side) * (1.0f - 0.25f * fabsf(gazeY));
        s.w *= (1.0f - 0.08f * gazeX * side);
        float cx = (i == 1 ? EYE_CX_R : EYE_CX_L) + gazeX * LOOK_X;
        float cy = EYE_CY + gazeY * LOOK_Y;
        drawEye(cx, cy, s, blinkAmount, i == 1);
    }

    if (mode == M_HAPPY) {
        for (int i = 0; i < 4; i++) {
            int offX = 0;
            int offY = 0;

            if (i == 0) { // Левое сердечко
                offX = (int)lroundf(sinf(now * 0.008f) * 1.5f);
                offY = (int)lroundf(cosf(now * 0.006f) * 1.0f);
            } else if (i == 1) { // Правое сердечко
                offX = (int)lroundf(sinf(now * 0.007f + 2.0f) * 1.5f);
                offY = (int)lroundf(sinf(now * 0.009f) * 1.5f);
            } else if (i == 2) { // Снизу слева
                offY = (int)lroundf(sinf(now * 0.012f) * 2.0f);
            } else if (i == 3) { // Снизу справа
                offX = (int)lroundf(cosf(now * 0.010f + 1.0f) * 1.5f);
                offY = (int)lroundf(sinf(now * 0.011f) * 2.0f);
            }

            drawMiniHeart(HEART_POS[i][0], HEART_POS[i][1], offX, offY);
        }
    }

    u8g2.sendBuffer();
}

// =====================================================================
//  Setup / loop
// =====================================================================
void setup() {
    u8g2.setBusClock(400000);
    u8g2.begin();
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);

    curiousSide = 0;
    setPose(P_NORMAL);
    cur[0] = pose[0];
    cur[1] = pose[1];

    unsigned long now = millis();
    lastFrame = now;
    nextLookAt = now + 1500;
    nextActionAt = now + random(3000, 6000);
    scheduleBlink(now);
}

void loop() {
    unsigned long now = millis();
    if (now - lastFrame < FRAME_MS) return;
    float dt = (float)(now - lastFrame);
    lastFrame = now;

    bool pressed = (digitalRead(BUTTON_PIN) == HIGH);
    if (pressed && !prevPressed && mode == M_IDLE) startHappy(now);
    prevPressed = pressed;

    if (mode == M_HAPPY) updateHappy(now, pressed);
    else                 updateIdle(now);

    updateLook(now);
    updateBlink(now);

    // targets = pose + gentle vertical bob ("breathing", or a bounce while happy)
    float bob = (mode == M_HAPPY && curPose == P_HAPPY) ? sinf(now * 0.011f) * 2.0f
                                                        : sinf(now * 0.0018f) * 0.6f;
    float aShape = 1.0f - expf(-dt / tauShape);
    float aGaze  = 1.0f - expf(-dt / tauGaze);
    for (int i = 0; i < 2; i++) {
        EyeShape t = pose[i];
        t.y += bob;
        approachShape(cur[i], t, aShape);
    }
    approach(gazeX, gazeTX, aGaze);
    approach(gazeY, gazeTY, aGaze);

    render();
}