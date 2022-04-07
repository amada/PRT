#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits>

#include "thread_pool.h"
#include "util.h"
#include "vecmath.h"
#include "triangle.h"
#include "bvh.h"

using namespace prt;

#define ASSERT_F_EQUAL(x, y) { if ((x) != (y)) { printf("Equal assertion failed (%f != %f) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}
#define ASSERT_F_EPEQUAL(x, y) { if (std::abs((x) - (y)) > 0.0001f){ printf("Epsillon equal assertion failed (%f != %f) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}

#define ASSERT_I_EQUAL(x, y) { if ((x) != (y)) { printf("Equal assertion failed (%d != %d) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}


void test_soa_vecmath()
{
    printf("test_soa_vecmath()\n");

    {
        SoaFloat f(0.0f);
        f = f.replaceLane(0, 1.0f);
        f = f.replaceLane(1, 2.0f);
        f = f.replaceLane(2, 3.0f);
        f = f.replaceLane(3, 4.0f);
        ASSERT_F_EQUAL(f.getLane(0), 1.0f);
        ASSERT_F_EQUAL(f.getLane(1), 2.0f);
        ASSERT_F_EQUAL(f.getLane(2), 3.0f);
        ASSERT_F_EQUAL(f.getLane(3), 4.0f);

        SoaFloat c(1.5f);
        SoaMask mask = f.lessThan(c);
        f = select(f, c, mask);
        ASSERT_F_EQUAL(f.getLane(0), 1.5f);
        ASSERT_F_EQUAL(f.getLane(1), 2.0f);
        ASSERT_F_EQUAL(f.getLane(2), 3.0f);
        ASSERT_I_EQUAL(mask.ballot(), 1);
    }

    {
        SoaMask m0 = SoaMask::initAllTrue();
        SoaMask m1 = SoaMask::initAllTrue();
        m0 = m0.computeNot();
        m1 = m0 & m1;
        ASSERT_I_EQUAL(m0.getLane(0), 0);
        ASSERT_I_EQUAL(m1.getLane(0), 0);
    }

    {
        SoaFloat n(-2.1f);
        SoaFloat d(12.9f);
        SoaFloat f = n/d;
        ASSERT_F_EPEQUAL(f.getLane(0), -2.1f/12.9f);
    }

    {
        SoaVector3f v(1.0f, 1.0f, 1.0f);
        SoaFloat len = length(v);
        ASSERT_F_EPEQUAL(len.getLane(0), std::sqrt(1.0f + 1.0f + 1.0f));
    }

    {
        SoaFloat f0(2.3f);
        ASSERT_F_EQUAL(f0.floor().getLane(0), 2.0f);
    }

    {
        SoaFloat f1(81.9f);
        ASSERT_F_EQUAL(f1.floor().getLane(0), 81.0f);
    }

    {
        SoaMask m(0x5);
        ASSERT_I_EQUAL(m.moveMask(), 0x5);
    }
}

void test_thread_pool()
{
    printf("test_thread_pool()\n");

    ThreadPool pool;

    pool.create(3);

    for (uint32_t i = 0; i < 16; i++) {
        pool.queue([i]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            printf("sleep %u\n", i);
        });
    }
    pool.waitAllTasksDone();
}


int main()
{
    test_soa_vecmath();
    test_thread_pool();

    return 0;
}
