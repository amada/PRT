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
#include "path_tracer.h"

using namespace prt;

const uint32_t kWidth = 256*4;
const uint32_t kHeight = 256*4;


void raytrace_cornell_box()
{
    Image image(kWidth, kHeight);
 
    ThreadPool threadPool;

    bool withBunny = true;

    Bvh cbox;
    cbox.build(SampleModels::getCornellBox(true));

    Bvh bunny;
#if 0
    bunny.Build(SampleModels::GetBunny(Vector3f(0.0f), {
        .diffuse = Vector3f{0.5f, 0.5f, 0.5f},
//        .diffuse = Vector3f{0.8f, 0.8f, 0.8f},
        .reflectionType = ReflectionType::kDiffuse
        });
#endif

#if 0
    bunny.build(SampleModels::getTeapot(
        Vector3f(-0.5f, 0.0f, 0.5f), {
        .diffuse = Vector3f{0.9f, 0.9f, 0.9f},
        .reflectionType = ReflectionType::kSpecular
        }));
#endif
    Scene scene;
    scene.Add(&cbox);
    if (withBunny) {
        scene.Add(&bunny);
    }


    auto start = std::chrono::steady_clock::now();
    threadPool.create(16);

    Camera camera({0, 0.95, 2.8}, {0, -0.042612f, -1}, kWidth, kHeight);
    // camera dir should be normalized

    const uint32_t kTileWidth = 16;
    const uint32_t kTileHeight = 16;
    const uint32_t kSamples = 64*2;

    uint32_t totalTasks = 0;

    for (uint32_t y = 0; y < kHeight; y += kTileHeight) {
        auto y0 = y;
        auto y1 = std::min(y + kTileHeight, kHeight - 1);

        for (uint32_t x = 0; x < kWidth; x += kTileWidth) {
            auto x0 = x;
            auto x1 = std::min(x + kTileWidth, kWidth - 1);

            threadPool.queue([&image, x0, y0, x1, y1, &scene, &camera]() {
                PathTracer tracer;
                tracer.TraceBlock(image, x0, y0, x1, y1, scene, camera, kSamples);
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
    raytrace_cornell_box();

    return 0;
}