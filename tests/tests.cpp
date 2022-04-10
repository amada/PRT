#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits>

#include "thread_pool.h"
#include "vecmath.h"
#include "triangle.h"
#include "bvh.h"

using namespace prt;

#define ASSERT_F_EQUAL(x, y) { if ((x) != (y)) { printf("Equal assertion failed (%f != %f) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}
#define ASSERT_F_EPEQUAL(x, y) { if (std::abs((x) - (y)) > 0.0001f){ printf("Epsillon equal assertion failed (%f != %f) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}

#define ASSERT_I_EQUAL(x, y) { if ((x) != (y)) { printf("Equal assertion failed (%d != %d) at line %d in %s\n", (x), (y), __LINE__, __FILE__); __builtin_trap(); }}


void test_vecmath()
{
    printf("test_vecmath()\n");

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

        ASSERT_I_EQUAL(mask.anyTrue(), true);

        mask = f.lessThan(-10.0f);

        ASSERT_I_EQUAL(mask.anyTrue(), false);
    }

    {
        SoaMask m0; m0.setAll(true);
        SoaMask m1; m1.setAll(true);
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

void test_triangle_intersection()
{
    printf("test_triangle_intersection()\n");

    {
        SoaVector3f v0(0.0f, 0.0f, 0.0f);
        SoaVector3f v1(1.0f, 0.0f, 0.0f);
        SoaVector3f v2(0.0f, 1.0f, 0.0f);
        SoaVector3f o(0.5f, 0.5f, -1.0f);
        SoaVector3f d(0.0f, 0.0f, 1.0f);

        SoaMask mask; mask.setAll(false);
        SoaMask swapXZ; swapXZ.setAll(false);
        SoaMask swapYZ; swapYZ.setAll(false);
        auto intr = intersectTriangle(mask, o, d, swapXZ, swapYZ, v0, v1, v2);

        ASSERT_F_EQUAL(intr.t.getLane(0), 1.0f);
//        ASSERT_F_EQUAL(t.getLane(0), 0.5f);
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
    test_vecmath();
    test_triangle_intersection();
    test_thread_pool();

    return 0;
}
