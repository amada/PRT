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
#include "diffuse_visualizer.h"
#include "normal_visualizer.h"
#include "path_tracer.h"

using namespace prt;

const uint32_t kWidth = 256*4;
const uint32_t kHeight = 256*4;


void setupCornellBox(Scene& scene, Camera& camera, bool withTeapot)
{
    auto cbox = new Bvh;;
    cbox->build(SampleModels::getCornellBox(true));
    scene.add(cbox);

    if (withTeapot) {
        Mesh teapot;
        teapot.loadObj("../../data/teapot/teapot.obj",
                       {.diffuse = Vector3f{0.9f, 0.9f, 0.9f}, .reflectionType = ReflectionType::kSpecular});

        auto pos = teapot.getPositionBuffer();

        for (uint32_t i = 0; i < teapot.getVertexCount(); i++) {
            //        float s = 0.3f;//1.0f;// 0.005f // bunny
            float s = 0.005f; // teapot
            pos[i] = s * pos[i] + Vector3f(-0.5f, 0.0f, 0.5f);
        }
        teapot.calculateVertexNormals();
        auto teapotBvh = new Bvh;

        teapotBvh->build(std::move(teapot));
        scene.add(teapotBvh);
    }

    camera.create({0, 0.965, 2.6}, {0, 0, -1.0f}, kWidth, kHeight);
}

void setupSponza(Scene& scene, Camera& camera)
{
    Mesh sponza;
    sponza.loadObj("../../data/sponza/sponza.obj");
    sponza.calculateVertexNormals();

    auto sponzaBvh = new Bvh;;
    sponzaBvh->build(std::move(sponza));

    scene.setDirectionalLight(normalize(Vector3f(0.05f, 1.0f, 0.1f)), Vector3f(16.7f, 15.6f, 11.7f)); // sky.exr

    scene.add(sponzaBvh);

    camera.create({-10, 320.0, 0.0}, {0.5, -0.162612f, -0.1}, kWidth, kHeight); // Sponza
}

void setupSanMiguelLowPoly(Scene& scene, Camera& camera)
{
    Mesh sanMiguel;
    sanMiguel.loadObj("../../data/san-miguel/san-miguel-low-poly.obj");
    sanMiguel.calculateVertexNormals();

    auto sanMiguelBvh = new Bvh;
    sanMiguelBvh->build(std::move(sanMiguel));

    scene.setDirectionalLight(normalize(Vector3f(0.2f, 1.0f, 0.2f)), Vector3f(16.7f, 15.6f, 11.7f)); // sky.exr
    scene.add(sanMiguelBvh);

    camera.create({10, 2.0, 10.2}, {0, 0.01, -1}, kWidth, kHeight); // San Miguel
}

void raytrace_scene()
{
    Image image(kWidth, kHeight);

    ThreadPool threadPool;

    Scene scene;
    Camera camera;

    scene.init();
    setupCornellBox(scene, camera, true);
//    setupSponza(scene, camera);
//    setupSanMiguelLowPoly(scene, camera);

    auto start = std::chrono::steady_clock::now();
    threadPool.create(10);

    const uint32_t kTileWidth = 16;
    const uint32_t kTileHeight = 16;
    const uint32_t kSamples = 32;

    uint32_t totalTasks = 0;

    for (uint32_t y = 0; y < kHeight; y += kTileHeight) {
        auto y0 = y;
        auto y1 = std::min(y + kTileHeight, kHeight - 1);

        for (uint32_t x = 0; x < kWidth; x += kTileWidth) {
            auto x0 = x;
            auto x1 = std::min(x + kTileWidth, kWidth - 1);

            threadPool.queue([&image, x0, y0, x1, y1, &scene, &camera]() {
#if 0
//                DiffuseVisualizer visualizer;
                NormalVisualizer visualizer;
                visualizer.TraceBlock(image, x0, y0, x1, y1, scene, camera);
#else
                PathTracer tracer;
                tracer.TraceBlock(image, x0, y0, x1, y1, scene, camera, kSamples);
#endif
            });

            totalTasks++;
        }
    }

    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto now = std::chrono::steady_clock::now();
        auto finishedTasks = totalTasks - threadPool.getTaskCount();
        auto finished = 100.0f*finishedTasks/totalTasks;
        auto elapsedMs = (float)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        auto blkSec = finishedTasks/elapsedMs*1000.0f;
        auto estimatedTotalMs = totalTasks*elapsedMs/finishedTasks;
        auto estimatedRemainMs = estimatedTotalMs - elapsedMs;

        fprintf(stderr, "\r%.3f%% done; %.3f blk/s; %.2fs remain estimated", finished, blkSec, estimatedRemainMs/1000.0f);
    } while (threadPool.getTaskCount() != 0);

    printf("\n");

    threadPool.waitAllTasksDone();
    auto end = std::chrono::steady_clock::now();

    auto ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    auto sec = uint32_t(ms)/1000;
    auto minute = sec/60;
    sec = sec - 60*minute;
    printf("%um%us (%.3fms) @%uspp\n", minute, sec, ms, kSamples);

    image.SaveToPpm("cornell_box.ppm");
}

int main(int argc, char** argv)
{
    raytrace_scene();

    return 0;
}
