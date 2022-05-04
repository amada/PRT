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

// Apple M1 performance core clock
const static uint32_t kCpuClk = 3200u*1000u*1000u;


class ScopePerf
{
    using timer = std::chrono::steady_clock;
public:
    ScopePerf(int64_t* timing) : m_timing(timing) {
        m_start = timer::now();
    }

    ~ScopePerf() {
        auto end = timer::now();
        *m_timing = std::chrono::duration_cast<std::chrono::microseconds>(end - m_start).count();
    }

private:
    int64_t* m_timing;
    std::chrono::time_point<timer> m_start;
};


void test_vecmath()
{
    printf("test_vecmath()\n");

    {
        SoaFloat f(5.0f);
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
        SoaVector3f o(0.4f, 0.4f, -1.0f);
        SoaVector3f d(0.0f, 0.0f, 1.0f);

        SoaMask mask; mask.setAll(true);
        SoaMask swapXZ; swapXZ.setAll(false);
        SoaMask swapYZ; swapYZ.setAll(false);
        auto intr = intersectTriangle(mask, o, d, swapXZ, swapYZ, v0, v1, v2);

        ASSERT_F_EQUAL(intr.t.getLane(0), 1.0f);
    }
}


void print_perf_result(const char* name, float result, uint32_t num, int64_t us, int32_t lanes)
{
    float intrSec = num*1000.0f*1000.0f/us;
    float clkIntr = kCpuClk/intrSec;

    srand(result); // in order to avoid result from being optimized out
    printf("[%s] ", name);
    printf("%lldus %.1fM intersections/sec ", us, intrSec/1000/1000);
    printf("%.1f clk/intersection, %.2f intersections/clk\n", clkIntr, lanes/clkIntr);
}

void test_perf_n_tri()
{
    const uint32_t kNum = 16*1024*1024;
    SoaFloat sum(0.0f);

    int64_t us = 0;

    struct Triangle {
        SoaVector3f v0;
        SoaVector3f v1;
        SoaVector3f v2;
    };

    auto tris = new Triangle[kNum];
    for (uint32_t i = 0; i < kNum; i++) {
        auto& tri = tris[i];
        tri.v0 = SoaVector3f(i, 0.0f, 0.0f);
        tri.v1 = SoaVector3f(1.0f, 0.0f, 1.0f/(i + 1));
        tri.v2 = SoaVector3f(0.0f, i & 0xff, 0.0f);
    }

    {
        ScopePerf perf(&us);

        SoaMask mask; mask.setAll(true);
        SoaMask swapXZ; swapXZ.setAll(false);
        SoaMask swapYZ; swapYZ.setAll(false);

        for (uint32_t i = 0; i < kNum; i++) {
            auto& tri = tris[i];
            auto& v0 = tri.v0;
            auto& v1 = tri.v1;
            auto& v2 = tri.v2;

            SoaVector3f o(0.4f, 0.0f, -1.0f);
            SoaVector3f d(0.0f, 0.0f, 1.0f);

            auto intr = intersectTriangle(mask, o, d, swapXZ, swapYZ, v0, v1, v2);

            sum = sum + intr.t;
        }
    }
    delete[] tris;

    print_perf_result("N rays-triangle", sum.getLane(0), kNum, us, SoaConstants::kLaneCount);
}

void test_perf_n_bbox()
{
    const uint32_t kNum = 16*1024*1024;
    SoaFloat sum(0.0f);

    int64_t us = 0;

    auto bboxes = new BBox[kNum];
    for (uint32_t i = 0; i < kNum; i++) {
        auto& bbox = bboxes[i];
        bbox.lower = Vector3f(-1.0f, -i*0.2f, 0.0f);
        bbox.upper = Vector3f(1.0f, i*2.0f, i & 0xff + 1);
    }


    SoaVector3f o(0.4f, 0.0f, -1.0f);
    SoaVector3f d(0.0f, 0.0f, 1.0f);
    auto inv_d = 1.0f/d;

    {
        ScopePerf perf(&us);

        for (uint32_t i = 0; i < kNum; i++) {
            auto& bbox = bboxes[i];

            auto t = bbox.intersect(o, d, inv_d);

            sum = sum + t;
        }
    }
    delete[] bboxes;

    print_perf_result("N rays-bbox", sum.getLane(0), kNum, us, SoaConstants::kLaneCount);
}

void test_perf_tri()
{
    const uint32_t kNum = 16*1024*1024;
    float sum = 0.0f;

    int64_t us = 0;

    struct Triangle {
        Vector3f v0;
        Vector3f v1;
        Vector3f v2;
    };

    auto tris = new Triangle[kNum];
    for (uint32_t i = 0; i < kNum; i++) {
        auto& tri = tris[i];
        tri.v0 = Vector3f(i, 0.0f, 0.0f);
        tri.v1 = Vector3f(1.0f, 0.0f, 1.0f/(i + 1));
        tri.v2 = Vector3f(0.0f, i & 0xff, 0.0f);
    }

    {
        ScopePerf perf(&us);

        for (uint32_t i = 0; i < kNum; i++) {
            auto& tri = tris[i];
            auto& v0 = tri.v0;
            auto& v1 = tri.v1;
            auto& v2 = tri.v2;
            Vector3f o(0.4f, i & 0xff, -1.0f);
            Vector3f d(0.0f, 0.0f, 1.0f);

            auto intr = intersectTriangle(o, d, v0, v1, v2);

            sum = sum + intr.t;
        }
    }
    delete[] tris;

    print_perf_result("Ray-triangle", sum, kNum, us, 1);
}

void test_perf_bbox()
{
    const uint32_t kNum = 16*1024*1024;
    SoaFloat sum(0.0f);

    int64_t us = 0;

    auto bboxes = new BBox[kNum];
    for (uint32_t i = 0; i < kNum; i++) {
        auto& bbox = bboxes[i];
        bbox.lower = Vector3f(-1.0f, -i*0.2f, 0.0f);
        bbox.upper = Vector3f(1.0f, i*2.0f, i & 0xff + 1);
    }


    Vector3f o(0.4f, 0.0f, -1.0f);
    Vector3f d(0.0f, 0.0f, 1.0f);
    auto inv_d = 1.0f/d;

    {
        ScopePerf perf(&us);

        for (uint32_t i = 0; i < kNum; i++) {
            auto& bbox = bboxes[i];

            auto t = bbox.intersect(o, d, inv_d);

            sum = sum + t;
        }
    }
    delete[] bboxes;

    print_perf_result("Ray-bbox", sum.getLane(0), kNum, us, 1);

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

    test_perf_n_tri();
    test_perf_n_bbox();
    test_perf_tri();
    test_perf_bbox();

    return 0;
}
