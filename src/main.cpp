#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <thread>
#include <limits>
#include <algorithm>

#include "thread_pool.h"
#include "image.h"
#include "sample_models.h"
#include "camera.h"
#include "bvh.h"
#include "scene.h"
#include "gbuffer_visualizer.h"
#include "path_tracer.h"

using namespace prt;

#define DATA_DIR "../../data/"

void setupCornellBox(Scene& scene, Camera& camera, float& exposure, uint32_t width, uint32_t height)
{
    auto cbox = new Bvh;;
    cbox->build(SampleModels::getCornellBox(true));
    scene.add(cbox);

    bool withTeapot = true;
    if (withTeapot) {
        Material mat;
        mat.init();
        mat.diffuse = {0.9f, 0.9f, 0.9f};
        mat.reflectionType = ReflectionType::kSpecular;

        Mesh teapot;
        teapot.loadObj(DATA_DIR "teapot/teapot.obj", mat);

        auto pos = teapot.getPositionBuffer();

        for (uint32_t i = 0; i < teapot.getVertexCount(); i++) {
            //        float s = 0.3f;//1.0f;// 0.005f // bunny
            float s = 0.005f; // teapot
            pos[i] = s * pos[i] + Vector3f(-0.5f, 0.0f, 0.5f);
        }
        teapot.calculateVertexNormals();
        teapot.calculateBounds();
        auto teapotBvh = new Bvh;

        teapotBvh->build(std::move(teapot));
        scene.add(teapotBvh);
    }

    camera.create({0, 0.965, 2.6}, {0, 0, -1.0f}, width, height);
    exposure = 1.0f;
}

void setupSponza(Scene& scene, Camera& camera, float& exposure, uint32_t width, uint32_t height)
{
    Mesh sponza;
    sponza.loadObj(DATA_DIR "sponza/sponza.obj");
    sponza.calculateVertexNormals();

    auto sponzaBvh = new Bvh;;
    sponzaBvh->build(std::move(sponza));

    scene.setDirectionalLight(normalize(Vector3f(0.05f, 1.0f, 0.1f)), Vector3f(16.7f, 15.6f, 11.7f)); // sky.exr
//    scene.setInfiniteAreaLight(DATA_DIR "skylight-day.exr");

    scene.add(sponzaBvh);

    camera.create({-10, 320.0, 0.0}, {0.5, -0.162612f, -0.1}, width, height);
    exposure = 1.0f;
}

void setupSanMiguelLowPoly(Scene& scene, Camera& camera, float& exposure, uint32_t width, uint32_t height)
{
    Mesh sanMiguel;
    sanMiguel.loadObj(DATA_DIR "san-miguel/san-miguel-low-poly.obj");
    sanMiguel.calculateVertexNormals();

    auto sanMiguelBvh = new Bvh;
    sanMiguelBvh->build(std::move(sanMiguel));

    scene.setDirectionalLight(normalize(Vector3f(0.2f, 1.0f, 0.2f)), Vector3f(16.7f, 15.6f, 11.7f)); // sky.exr
//    scene.setInfiniteAreaLight(DATA_DIR "skylight-day.exr");

    scene.add(sanMiguelBvh);

    camera.create({9, 2.0, 10.2}, {0.4, 0.1, -1}, width, height);
    exposure = 1.0f;
}

void setupZeroDay(Scene& scene, Camera& camera, float& exposure, uint32_t width, uint32_t height)
{
    Mesh zeroDay;
    zeroDay.loadObj("../../../../data/zero-day-measure-seven/zero-day-measure-seven.obj");

    auto zeroDayBvh = new Bvh;
    zeroDayBvh->build(std::move(zeroDay));

    scene.add(zeroDayBvh);

    camera.create({-1, 0.0, 0.2}, {-1.5, -0.1, -1}, width, height);
    exposure = 64.0f;
}

void raytrace_scene(uint32_t width, uint32_t height, const char* imagePath, void (*setupFunc)(Scene&, Camera&, float&, uint32_t, uint32_t))
{
    ThreadPool threadPool;

    Scene scene;
    Camera camera;
    float exposure;

    scene.init();
    setupFunc(scene, camera, exposure, width, height);

    Image image(width, height, true, exposure);

    auto start = std::chrono::steady_clock::now();
    threadPool.create(0);

    const uint32_t kTileWidth = 16;
    const uint32_t kTileHeight = 16;
    const uint32_t kSamples = 64;

    TotalStats totalStats;
    totalStats.clear();

    uint32_t totalTasks = 0;

    for (uint32_t y = 0; y < height; y += kTileHeight) {
        auto y0 = y;
        auto y1 = std::min(y + kTileHeight, height - 1);

        for (uint32_t x = 0; x < width; x += kTileWidth) {
            auto x0 = x;
            auto x1 = std::min(x + kTileWidth, width - 1);

            threadPool.queue([&image, x0, y0, x1, y1, &scene, &camera, &totalStats]() {

#if 0
                GbufferVisualizer visualizer(GbufferVisualizer::Type::kMeshNormal);
                visualizer.TraceBlock(image, x0, y0, x1, y1, scene, camera);
#else
                PathTracer tracer;
                tracer.TraceBlock(image, x0, y0, x1, y1, scene, camera, kSamples);
#ifdef PRT_ENABLE_STATS
                totalStats.merge(tracer.getStats());
#else
                (void)totalStats;
#endif
#endif
            });

            totalTasks++;
        }
    }

    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto now = std::chrono::steady_clock::now();
        auto finishedTasks = totalTasks - threadPool.getTaskCount();
        auto finished = 100.0f*finishedTasks/totalTasks;
        auto elapsedMs = (float)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        auto blkSec = finishedTasks/elapsedMs*1000.0f;
        auto estimatedTotalMs = totalTasks*elapsedMs/finishedTasks;
        auto estimatedRemainMs = estimatedTotalMs - elapsedMs;

        fprintf(stderr, "\r%.3f%% done; %.3f blk/s; %.2fs remain estimated", finished, blkSec, estimatedRemainMs/1000.0f);
    } while (threadPool.getTaskCount() != 0);


    threadPool.waitAllTasksDone();
    printf("\n");
#ifdef PRT_ENABLE_STATS
    totalStats.print();
#endif


    auto end = std::chrono::steady_clock::now();

    auto ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    auto sec = uint32_t(ms)/1000;
    auto minute = sec/60;
    sec = sec - 60*minute;
    printf("%um%us (%.3fms) @%uspp\n", minute, sec, ms, kSamples);

    image.saveToPpm(imagePath);
}

int main(int argc, char** argv)
{
    raytrace_scene(1024, 1024, "cornell_box.ppm", setupCornellBox);
//    raytrace_scene(1600, 900, "san_miguel.ppm", setupSanMiguelLowPoly);
//    raytrace_scene(1600, 900, "sponza.ppm", setupSponza);
//    raytrace_scene(1600, 900, "zero_day.ppm", setupZeroDay);

    return 0;
}
