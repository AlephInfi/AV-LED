#pragma once

#include "..\Utilities\Conversions.h"
#include "..\Device\RCAIn.h"
#include <stdint.h>

class BeatDetector {
public:
    BeatDetector(float attack = 0.6f, float release = 0.1f, float sensitivity = 1.4f)
        : attack(attack), release(release), sensitivity(sensitivity) {}

    bool process(float input) {
        // Envelope follower
        if(input > envelope)
            envelope += attack * (input - envelope);
        else
            envelope += release * (input - envelope);

        // Adaptive threshold
        float threshold = envelope * sensitivity;

        // Beat when input spikes above envelope
        bool beatNow = (input > threshold);

        // Debounce beats to avoid double triggers
        if(beatNow && !wasHigh) {
            wasHigh = true;
            return true;
        }
        if(!beatNow)
            wasHigh = false;

        return false;
    }

private:
    float envelope = 0.0f;
    float attack, release;
    float sensitivity;
    bool wasHigh = false;
};
