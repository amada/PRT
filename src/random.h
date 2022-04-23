#pragma once

#include <random>



namespace prt
{

#if 1

class Random
{
public:
    Random() {
        m_state.a = rand() | 1;
    }

    float generateMinus1to1() {
        return 2.0f*generate() - 1.0f;
    }

    float generate() {
        auto u = xorshift32(&m_state);
        union {
            uint32_t u;
            float f;
        } ui;

        ui.u = (u & 0x007fffff) | 0x3f800000;

        return ui.f - 1.0f;
    }

private:
    // Using https://en.wikipedia.org/wiki/Xorshift
    struct xorshift32_state {
        uint32_t a;
    };

    uint32_t xorshift32(xorshift32_state *state)
    {
        auto x = state->a;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return state->a = x;
    }

    xorshift32_state m_state;
};

#else

class Random
{
public:
    Random() : m_gen(rand()), m_rand(0.0f, 1.0f) {
    }

    float generateMinus1to1() {
        return 2.0f*generate() - 1.0f;
    }

    float generate() {
        return m_rand(m_gen);
    }

private:
    std::mt19937 m_gen;
    std::uniform_real_distribution<float> m_rand;
};

#endif

} // namespace prt