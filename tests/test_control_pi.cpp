#include "../cv58-boost-v14-forward-pi/control_pi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_fails = 0;

static void expect_true(bool cond, const char *msg) {
    if (!cond) {
        std::printf("FAIL: %s\n", msg);
        g_fails++;
    }
}

static void expect_near(float got, float want, float tol, const char *msg) {
    if (std::fabs(got - want) > tol) {
        std::printf("FAIL: %s  got=%.4f want=%.4f tol=%.4f\n", msg, got, want, tol);
        g_fails++;
    }
}

int main() {
    /* clamp */
    expect_near(boostClampf(3.0f, 0.0f, 2.0f), 2.0f, 1e-6f, "clamp high");
    expect_near(boostClampf(-1.0f, 0.0f, 2.0f), 0.0f, 1e-6f, "clamp low");
    expect_near(boostClampf(1.0f, 0.0f, 2.0f), 1.0f, 1e-6f, "clamp mid");

    /* slew */
    expect_near(boostApplySlew(10.0f, 0.0f, 3.0f, 5.0f), 3.0f, 1e-6f, "slew up cap");
    expect_near(boostApplySlew(0.0f, 10.0f, 3.0f, 5.0f), 5.0f, 1e-6f, "slew down cap");
    expect_near(boostApplySlew(4.0f, 3.0f, 3.0f, 5.0f), 4.0f, 1e-6f, "slew inside");

    /* PI: positive error must increase output and integrate */
    {
        float integ = 0.0f;
        float out = boostRunPI(1.0f, 8.0f, 35.0f, 0.02f, &integ, -20.0f, 25.0f);
        expect_true(out > 8.0f, "CC PI output includes integral on first unsaturated step");
        expect_near(integ, 35.0f * 1.0f * 0.02f, 1e-5f, "integral stores Ki*err*dt");
        expect_near(out, 8.0f + integ, 1e-5f, "unsaturated out = P+I");
    }

    /* anti-windup: saturated output must not grow the integral */
    {
        float integ = 0.0f;
        float out = boostRunPI(10.0f, 8.0f, 35.0f, 0.02f, &integ, -20.0f, 25.0f);
        expect_near(out, 25.0f, 1e-6f, "P+I saturates at outMax");
        expect_near(integ, 0.0f, 1e-6f, "integral frozen while saturated");
    }

    /* CC plant: duty climbs until Ibat ≈ Iref, then holds */
    {
        const float dt = 0.02f;
        const float iRef = 5.0f;
        const float plant_gain = 5.0f / 200.0f; /* ~5 A at duty 200 */
        float integ = 0.0f;
        float duty = 80.0f;
        float iBat = 0.0f;
        for (int n = 0; n < 400; n++) {
            float iErr = iRef - iBat;
            float dDuty = boostRunPI(iErr, 8.0f, 35.0f, dt, &integ, -20.0f, 25.0f);
            duty = boostApplySlew(boostClampf(duty + dDuty, 0.0f, 460.0f),
                                  duty, 3.0f, 5.0f);
            iBat = plant_gain * duty;
        }
        expect_near(iBat, iRef, 0.25f, "CC PI regulates plant to 5 A");
        expect_true(duty > 80.0f && duty < 460.0f, "CC duty stayed inside Forward Dmax");
    }

    /* CV outer PI: voltage above target must drive Iref down.
     * Firmware also leaks the voltage integrator (×0.85) on over-voltage. */
    {
        const float dt = 0.02f;
        float integ = 1.5f; /* leftover CC-era current request */
        float iReq = 0.0f;
        float first = 0.0f;
        for (int n = 0; n < 40; n++) {
            float vErr = 56.00f - 56.40f; /* 0.4 V over target */
            iReq = boostRunPI(vErr, 0.70f, 0.35f, dt, &integ, 0.0f, 3.0f);
            if (n == 0) first = iReq;
            integ *= 0.85f;
        }
        expect_true(first > 0.5f, "CV starts from leftover Iref");
        expect_true(iReq < 0.15f, "CV voltage PI + over-V leak bleeds Iref");
    }

    /* CV deadband-style zero error must not keep integrating */
    {
        float integ = 0.8f;
        float out = boostRunPI(0.0f, 0.70f, 0.35f, 0.02f, &integ, 0.0f, 3.0f);
        expect_near(integ, 0.8f, 1e-6f, "zero error leaves integral unchanged");
        expect_near(out, 0.8f, 1e-6f, "zero error output is the held integral");
    }

    if (g_fails) {
        std::printf("%d test(s) failed\n", g_fails);
        return 1;
    }
    std::printf("all PI host tests passed\n");
    return 0;
}
