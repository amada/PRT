#pragma once

#include <random>



namespace prt
{

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

} // namespace prt