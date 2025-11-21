#pragma once

#include <FastLED.h>
#include "..\Audio\BPMDetect.h"
#include "..\Materials\SimplexNoise.h"

#define LED_VOLTS       5
#define LEDPIN          14
#define NUM_LEDS        600
#define MAX_BRIGHTNESS  200

// ================================
// Fast inline utilities
// ================================
static inline float clampf(float v, float a, float b) {
    return (v < a) ? a : (v > b) ? b : v;
}

static inline uint8_t clampu8(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Fast exp approximation (for LED falloff) – matches original visually
static inline float fastExpFall(float x) {
    // polynomial approximation for e^(-x) on [0,5]
    return 1.0f / (1.0f + x + 0.48f * x * x);
}


// ================================
// LEDStrip CLASS
// ================================
class LEDStrip {
private:
    CRGB led[NUM_LEDS] __attribute__ ((aligned (16)));
    CRGB prevLed[NUM_LEDS] __attribute__ ((aligned (16)));

    ClockMicros microClock;
    RCA rca;
    BeatDetector bpm;

    // Noise materials
    RGBColor noiseSpectrum[3] = {
        RGBColor(0,   0, 255),
        RGBColor(255, 0, 0),
        RGBColor(0, 255, 0)
    };

    GradientMaterial gNoiseMat = GradientMaterial(3, noiseSpectrum, 2.0f, false);
    SimplexNoise sNoise;
    float currentHue = 0.0f;

    float LowMax = 0.0f;
    float MidMax = 0.0f;
    float HighMax = 0.0f;

    uint8_t currentBrightess = 1;
    uint8_t prevBrightness = 0;

    int lLow = 0;
    int HighestLowValue = 0;

    unsigned long prevmillis = 0;
    int ct = 0;
    int lgct = 0;

    int MultiplierRandY = 1;
    int MultiplierRandZ = 1;

    int RunningAverageDiff;
    uint8_t MinBright = 0;
    int rafilter[50];

    CRGB trans_white_pixel = CRGB(150,150,150);

    // ====================
    // Brightness Pulse
    // ====================
    struct BrightnessPulseData {
        bool active = false;
        float intensity = 0.0f;
        float attackAmount = 0.0f;
        float decayRate = 0.90f;
        uint32_t startTime = 0;
    };

    BrightnessPulseData brightnessPulse;

    // Pulse constants
    static constexpr float PULSE_ATTACK = 1.8f;
    static constexpr float PULSE_DECAY  = 0.92f;

    struct Pulse {
        CRGB color;
        float length;
        uint32_t startTime;
        uint32_t duration;
    };

    static const uint8_t MAX_PULSES = 20;
    Pulse pulses[MAX_PULSES];
    uint8_t pulseCount = 0;
    uint32_t pulseTimeoutDuration = 0;

    // ====================
    // Beat Event Struct
    // ====================
    struct BeatEvent {
        bool kick  = false;
        bool snare = false;
        bool hihat = false;
    };

    // ====================
    // Constructor
    // ====================
public:
    LEDStrip() :
        sNoise(random16(), &gNoiseMat)
    {
        MultiplierRandY = random(1,5);
        MultiplierRandZ = random(1,5);
        memset(prevLed, 0, sizeof(prevLed));
        memset(rafilter, 0, sizeof(rafilter));
    }

    // ====================
    // Initialization
    // ====================
    void Init(uint8_t min) {
        FastLED.addLeds<WS2812B, LEDPIN, GRB>(led, NUM_LEDS);
        FastLED.setBrightness(min);
        this->MinBright = min;
    }

    void setLED(uint16_t pixel, RGBColor color) {
        led[pixel].r = color.R;
        led[pixel].g = color.G;
        led[pixel].b = color.B;
    }

    void ManualShow() {
        FastLED.show();
    }

    double getBand(uint8_t band) {
        return rca.getBand(band);
    }

    // ================================
    // GrayScale Helper (optimized)
    // ================================
    inline float GrayScale(float R, float G, float B, float saturation) {
        float avg = (R + G + B) * 0.3333333f;
        float v = saturation * avg - (currentBrightess * 60.0f / 255.0f);
        return v;
    }

    // ============================================================
    // Brightness Pulse Start / Update
    // ============================================================
    inline void StartBrightnessPulse(float attack, float decayRate) {
        brightnessPulse.active = true;
        brightnessPulse.attackAmount = attack;
        brightnessPulse.decayRate = decayRate;

        float newInt = brightnessPulse.intensity + attack;
        brightnessPulse.intensity = (newInt > 1.0f ? 1.0f : newInt);
        brightnessPulse.startTime = millis();
    }

    inline float UpdateBrightnessPulse() {
        if (!brightnessPulse.active) return 0.0f;

        brightnessPulse.intensity *= brightnessPulse.decayRate;
        if (brightnessPulse.intensity < 0.001f) {
            brightnessPulse.intensity = 0.0f;
            brightnessPulse.active = false;
        }

        return brightnessPulse.intensity;
    }

    // ============================================================
    // Start Pulse
    // ============================================================
    inline void StartPulse(CRGB color, float length, uint32_t t_ms, uint16_t timeSince) {
        if (pulseCount >= MAX_PULSES) return;
        if (timeSince < 150) return;

        pulseTimeoutDuration = (length / NUM_LEDS) * t_ms;

        Pulse &p = pulses[pulseCount++];
        p.color     = color;
        p.length    = clampf(length * timeSince / 500.0f, 30.0f, 70.0f);
        p.startTime = millis();
        p.duration  = (uint32_t)clampf(t_ms * timeSince / 500.0f, 1300.0f, 4000.0f);
    }

    // ===============================
    // Beat Detection (cached bands)
    // ===============================
    BeatEvent detectBeats();

    // Forward declarations for Part 2
    void RenderPulses(uint8_t mode);
    void Simplex(float ratio);
    void SimplexWithAudioMod(float ratio, bool idleState);

    // ===============================
    // Main update (implemented in Part 3)
    // ===============================
    void Update(int command, float ratio);
};

// ==========================================================================
// BEAT DETECTION  (Optimized band caching + fewer float ops)
// ==========================================================================
LEDStrip::BeatEvent LEDStrip::detectBeats() {

    static constexpr float SMOOTH_LOW   = 0.90f;
    static constexpr float SMOOTH_MID   = 0.92f;
    static constexpr float SMOOTH_HIGH  = 0.93f;

    static constexpr float THRESH_LOW   = 1.25f;
    static constexpr float THRESH_TRANS = 1.05f;
    static constexpr float THRESH_SNARE = 1.00f;
    static constexpr float THRESH_HIHAT = 1.10f;

    static constexpr uint16_t COOLDOWN_KICK  = 30;
    static constexpr uint16_t COOLDOWN_SNARE = 500;
    static constexpr uint16_t COOLDOWN_HIHAT = 80;

    // Cached persistent EMA states
    static float avgLow  = 0.0f;
    static float avgMid  = 0.0f;
    static float avgHigh = 0.0f;
    static float avgTrans = 0.0f;

    static uint32_t lastKick = 0;
    static uint32_t lastSnare = 0;
    static uint32_t lastHihat = 0;

    static bool initialized = false;

    // Initial baseline
    if (!initialized) {
        avgLow  = rca.getBandAvg(0, 4);
        avgMid  = rca.getBandAvg(4, 10);
        avgHigh = rca.getBandAvg(11, 15);
        initialized = true;
    }

    // Cache all band reads ONCE (huge speedup)
    float lowNow  = rca.getBandAvg(0, 3);
    float midNow  = rca.getBandAvg(3, 8);
    float highNow = rca.getBandAvg(8, 15);
    float transNow = rca.getBandAvg(4, 8);

    // EMA smoothing
    avgLow  = avgLow  * SMOOTH_LOW  + lowNow  * (1.0f - SMOOTH_LOW);
    avgMid  = avgMid  * SMOOTH_MID  + midNow  * (1.0f - SMOOTH_MID);
    avgHigh = avgHigh * SMOOTH_HIGH + highNow * (1.0f - SMOOTH_HIGH);

    avgTrans = avgTrans * SMOOTH_MID + transNow * (1.0f - SMOOTH_MID);

    uint32_t now = millis();
    BeatEvent e;

    // -------------------------
    // Kick detection
    // -------------------------
    if (lowNow > avgLow * THRESH_LOW &&
        transNow > avgTrans * THRESH_TRANS &&
        (now - lastKick) > COOLDOWN_KICK)
    {
        e.kick = true;
        lastKick = now;
    }

    // -------------------------
    // Snare detection
    // -------------------------
    if (midNow > avgMid * THRESH_SNARE &&
        (now - lastSnare) > COOLDOWN_SNARE)
    {
        e.snare = true;
        lastSnare = now;
    }

    // -------------------------
    // Hi-hat detection
    // -------------------------
    if (highNow > avgHigh * THRESH_HIHAT &&
        (now - lastHihat) > COOLDOWN_HIHAT)
    {
        e.hihat = true;
        lastHihat = now;
    }

    return e;
}



// ==========================================================================
// RENDER PULSES  (Faster, branch-light, cache-friendly)
// ==========================================================================
void LEDStrip::RenderPulses(uint8_t mode) {

    uint32_t now = millis();
    if (pulseCount == 0) return;

    // Local to avoid repeated lookups
    const int lastIndex = NUM_LEDS - 1;

    for (uint8_t i = 0; i < pulseCount; i++) {

        Pulse &p = pulses[i];
        float progress = (float)(now - p.startTime) / (float)p.duration;

        if (progress >= 1.0f) {
            // Remove finished pulse
            for (uint8_t j = i; j < pulseCount - 1; j++)
                pulses[j] = pulses[j + 1];
            pulseCount--;
            i--;
            continue;
        }

        float headPos = progress * lastIndex;
        float length = p.length;

        // Fast local
        const CRGB color = p.color;

        // ============================
        // DRAW FUNCTION INLINE
        // ============================
        auto DrawPulse = [&](float hPos, float len, const CRGB &c, bool forward) {

            int start = (int)hPos;
            int end   = forward ? (int)(hPos - len) : (int)(hPos + len);
            int step  = forward ? -1 : +1;

            // Precompute scalars
            const float invLen = 1.0f / len;
            const float falloff = 3.5f;
            const float sharp   = 4.0f;

            for (int idx = start; idx != end + step; idx += step) {
                if (idx < 0 || idx >= NUM_LEDS) continue;

                float d = fabsf(hPos - idx) * invLen;
                if (d > 1.0f) continue;

                // Slightly faster than expf
                float intensity = fastExpFall(d * falloff);
                float colorBoost = fastExpFall(d * sharp);

                float finalI = intensity * (1.0f + 0.5f * colorBoost);

                // Final clamp
                finalI = (finalI < 0) ? 0 : (finalI > 1) ? 1 : finalI;

                CRGB &dst = led[idx];
                dst.r = clampu8(dst.r + (uint8_t)(c.r * finalI), 0, 255);
                dst.g = clampu8(dst.g + (uint8_t)(c.g * finalI), 0, 255);
                dst.b = clampu8(dst.b + (uint8_t)(c.b * finalI), 0, 255);
            }
        };

        // =======================================================
        // MODES
        // =======================================================
        switch (mode) {

        case 0:     // left → right
            DrawPulse(headPos, length, color, true);
            break;

        case 1: {   // right → left
            float hp = lastIndex - headPos;
            DrawPulse(hp, length, color, false);
            break;
        }

        case 2: {   // center → outward
            float center = lastIndex * 0.5f;
            float offset = headPos * 0.5f;

            DrawPulse(center - offset, length, color, false);
            DrawPulse(center + offset, length, color, true);
            break;
        }

        case 3: {   // ends → center
            float left  = headPos;
            float right = lastIndex - headPos;

            DrawPulse(left,  length, color, true);
            DrawPulse(right, length, color, false);
            break;
        }

        } // switch
    }
}



// ==========================================================================
// SIMPLEX  (Frame-based noise generation)
// ==========================================================================
void LEDStrip::Simplex(float ratio) {

    // Precompute only once
    const float angle = ratio * 6.283185f * 2.0f;
    const float x = 0.5f * sinf(angle);

    float linSweep = (ratio > 0.5f ? (1.0f - ratio) : ratio);
    float sShift   = linSweep * 0.004f + 0.005f;

    gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
    gNoiseMat.HueShift(ratio * 720.0f);
    sNoise.SetScale(Vector3D(sShift, sShift, sShift));
    sNoise.SetZPosition(x * 4.0f);

    // Main LED loop
    for (int i = 0; i < NUM_LEDS; i++) {
        led[i] = RGBColorToRgb(
            sNoise.GetRGB(Vector3D(i,i,i), {}, {}),
            1.0f
        );
    }
    memcpy(prevLed, led, sizeof(led));
}

// ==========================================================================
// SIMPLEX WITH AUDIO MODULATION  (Heavily optimized)
// ==========================================================================
void LEDStrip::SimplexWithAudioMod(float ratio, bool IdleState) {

    // Precompute sin variations
    const float ang1 = ratio * 6.283185f * 2.0f;
    const float ang2 = ratio * 6.283185f * 4.0f + MultiplierRandY * 50;
    const float ang3 = ratio * 6.283185f * 2.0f + MultiplierRandZ * 100;

    float x = 0.5f * sinf(ang1) + 0.5f;
    float y = 0.5f * sinf(ang2) + 0.5f;
    float z = 0.5f * sinf(ang3) + 0.5f;

    float linSweep = (ratio > 0.5f ? (1.0f - ratio) : ratio);

    // Noise configuration
    float sShift = IdleState
        ? (linSweep * 0.001f)
        : (linSweep * 0.00075f + 0.0015f);

    gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
    gNoiseMat.HueShift(currentHue + x * (IdleState ? 180.0f : 360.0f));
    sNoise.SetScale(Vector3D(sShift, sShift, sShift));
    sNoise.SetZPosition(x * 8.0f);

    // Cache band averages once
    float bandLow  = rca.getBandAvg(0,3);
    float bandMid  = rca.getBandAvg(4,10);
    float bandHigh = rca.getBandAvg(10,15);

    float Rmult = Mathematics::Map(clampf(lLow,  0.03f, LowMax),  0.01f, LowMax, 0.3f, 10.0f);
    float Gmult = Mathematics::Map(clampf(bandMid - 1, 0.03f, MidMax), 0.01f, MidMax, 0.5f, 6.0f);
    float Bmult = Mathematics::Map(clampf(bandHigh, 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);

    const float UMult = 0.7f;
    const uint8_t minB = MinBright;

    // ========================================
    // IDLE MODE (no audio)
    // ========================================
    if (IdleState) {

        FastLED.setBrightness(minB);
        const float sat = 0.8f;

        for (int i = 0; i < NUM_LEDS; i++) {

            RGBColor ref = sNoise.GetRGB(Vector3D(i,i,i), {}, {});
            CRGB pixel = RGBColorToRgb(ref, 1.0f);

            float gray = GrayScale(pixel.r, pixel.g, pixel.b, sat);

            pixel.r = clampu8((x * UMult * ref.R)*(1-sat) + gray, 60, 255);
            pixel.g = clampu8((y * UMult * ref.G)*(1-sat) + gray, 50, 255);
            pixel.b = clampu8((z * UMult * ref.B)*(1-sat) + gray, 60, 255);

            led[i] = pixel;
            prevLed[i] = pixel;
        }
        return;
    }

    // ========================================
    // ACTIVE AUDIO MODE
    // ========================================

    uint32_t now = millis();
    BeatEvent e = detectBeats();

    // Kick pulse
    static uint32_t prevKick = now;
    if (e.kick && now - prevKick > 30) {
        StartPulse(trans_white_pixel, 60, 1050, now - prevKick);
        prevKick = now;
    }

    // Snare brightness POP
    static uint32_t prevSnr = now;
    if (e.snare && now - prevSnr > 500) {
        StartBrightnessPulse(1.0f, 0.7f);
        prevSnr = now;
    }

    // =========================
    // Three-tier brightness models
    // =========================
    float lowDiff = LowMax - lLow;
    bool case1 = (lowDiff <= RunningAverageDiff * 2.55f) && (LowMax >= HighestLowValue / 2) && (bandLow > 10) && (bandLow > LowMax * 0.65f);
    bool case2 = (lowDiff > RunningAverageDiff * 2.5f) && (lowDiff < RunningAverageDiff * 3.1f) && (bandLow > 10) && (bandLow <= LowMax * 0.65f);

    // CASE 1
    if (case1) {
        FastLED.setBrightness(clampu8(currentBrightess * Rmult, minB, MAX_BRIGHTNESS));

        for (int i = 0; i < NUM_LEDS; i++) {

            RGBColor ref = sNoise.GetRGB(Vector3D(i,i,i), {}, {});
            CRGB pix;

            pix.r = clampu8((Rmult * ref.R * UMult) / 15, 0, MAX_BRIGHTNESS);
            pix.g = clampu8((Gmult * ref.G * UMult) / 15, 0, MAX_BRIGHTNESS);
            pix.b = clampu8((Bmult * ref.B * UMult) / 15, 0, MAX_BRIGHTNESS);

            // smoothing
            CRGB &old = prevLed[i];
            old.r += (pix.r - old.r) * 0.4f;
            old.g += (pix.g - old.g) * 0.4f;
            old.b += (pix.b - old.b) * 0.4f;

            led[i] = old;
        }
    }

    // CASE 2
    else if (case2) {

        FastLED.setBrightness(clampu8(minB + currentBrightess * Rmult / 15, minB, (2 * MAX_BRIGHTNESS) / 3));

        for (int i = 0; i < NUM_LEDS; i++) {
            RGBColor ref = sNoise.GetRGB(Vector3D(i,i,i), {}, {});
            CRGB pix;

            pix.r = clampu8((Rmult * ref.R * UMult)/25, 0, (2*MAX_BRIGHTNESS)/3);
            pix.g = clampu8((Gmult * ref.G * UMult)/25, 0, (2*MAX_BRIGHTNESS)/3);
            pix.b = clampu8((Bmult * ref.B * UMult)/25, 0, (2*MAX_BRIGHTNESS)/3);

            CRGB &old = prevLed[i];
            old.r += (pix.r - old.r) * 0.4f;
            old.g += (pix.g - old.g) * 0.4f;
            old.b += (pix.b - old.b) * 0.4f;

            led[i] = old;
        }
    }

    // FALLBACK / DIM STATE
    else {

        if (bandHigh > 20 || bandLow > 5 || bandMid > 2) {
            FastLED.setBrightness(
                clampu8(minB + currentBrightess * Rmult / 20, minB, MAX_BRIGHTNESS / 3)
            );
        }

        if (FastLED.getBrightness() > minB)
            FastLED.setBrightness(FastLED.getBrightness() - 1);

        const float sat = 0.8f;

        for (int i = 0; i < NUM_LEDS; i++) {

            RGBColor ref = sNoise.GetRGB(Vector3D(i,i,i), {}, {});
            CRGB pix = RGBColorToRgb(ref, 0.1f);

            float gray = GrayScale(pix.r, pix.g, pix.b, sat);

            // Red
            if (bandLow > 10)
                pix.r = clampu8(Rmult * ref.R * UMult, 0, MAX_BRIGHTNESS/2);
            else
                pix.r = clampu8((x*UMult*ref.R)*(1-sat) + gray, 0, MAX_BRIGHTNESS/2);

            // Green
            if (bandMid > 10)
                pix.g = clampu8(Gmult * ref.G * UMult, 0, MAX_BRIGHTNESS/2);
            else
                pix.g = clampu8((y*UMult*ref.G)*(1-sat) + gray, 0, MAX_BRIGHTNESS/2);

            // Blue
            if (bandHigh > 15)
                pix.b = clampu8(Bmult * ref.B * UMult, 0, MAX_BRIGHTNESS/2);
            else
                pix.b = clampu8((z*UMult*ref.B)*(1-sat) + gray, 0, MAX_BRIGHTNESS/2);

            // smoothing
            CRGB &old = prevLed[i];
            old.r += (pix.r - old.r) * 0.4f;
            old.g += (pix.g - old.g) * 0.4f;
            old.b += (pix.b - old.b) * 0.4f;

            led[i] = old;
        }
    }
}

// ==========================================================================
// UPDATE LOOP  (Highly optimized, final stage)
// ==========================================================================
void LEDStrip::Update(int command, float ratio)
{
    // Cache time ONCE
    const uint32_t now = millis();

    // ===========================
    // FFT CACHING
    // ===========================
    // Read only the bands we actually use.
    float bandLow  = rca.getBandAvg(0, 3);
    float bandMid  = rca.getBandAvg(4, 10);
    float bandHigh = rca.getBandAvg(10, 15);

    // Also cache the single-band low value.
    lLow = rca.getBand(0);

    // ===========================
    // UPDATE peak tracking
    // ===========================
    if (bandLow > LowMax)  LowMax  = bandLow;
    if (bandMid > MidMax)  MidMax  = bandMid;
    if (bandHigh > HighMax) HighMax = bandHigh;

    // Highest observed low for color shaping
    if (lLow > HighestLowValue)
        HighestLowValue = lLow;

    // ===========================
    // RUNNING AVERAGE FILTER
    // ===========================
    {
        int sum = 0;
        for (int i = 0; i < 49; i++) {
            rafilter[i] = rafilter[i + 1];
            sum += rafilter[i];
        }
        rafilter[49] = lLow;
        sum += lLow;

        RunningAverageDiff = (sum / 50);
    }

    // ===========================
    // IDLE MODE DETECTION
    // ===========================
    bool idle = (bandLow < 2 && bandMid < 2 && bandHigh < 2);

    // ===========================
    // BRIGHTNESS SMOOTHING
    // ===========================
    {
        float newBrightness = (bandLow + bandMid + bandHigh);
        newBrightness = clampf(newBrightness * 3.0f, (float)MinBright, (float)MAX_BRIGHTNESS);

        // Soft smoothing
        currentBrightess = (uint8_t)(currentBrightess * 0.7f + newBrightness * 0.3f);

        // Combine brightness pulse (snare) into brightness
        float pulseBoost = UpdateBrightnessPulse();
        if (pulseBoost > 0) {
            currentBrightess = clampu8(
                currentBrightess + (uint8_t)(pulseBoost * 120),
                MinBright,
                MAX_BRIGHTNESS
            );
        }
    }

    // ===========================
    // CLEAR ARRAY
    // ===========================
    // Only when pulses or simplex will redraw entire strip anyway.
    memset(led, 0, sizeof(led));


    // ===========================
    // MODE DISPATCH
    // ===========================
    switch (command) {

    case 0:
        Simplex(ratio);
        break;

    case 1:
        SimplexWithAudioMod(ratio, idle);
        break;

    default:
        // fallback: same as idle simplex
        Simplex(ratio);
        break;
    }


    // ===========================
    // PULSE RENDER
    // ===========================
    // Mode 2 chosen arbitrarily to match original aesthetic — left-to-right
    RenderPulses(2);


    // ===========================
    // FINAL SHOW
    // ===========================
    FastLED.show();

    // Store previous array for smoothing in next frame
    memcpy(prevLed, led, sizeof(led));
}
