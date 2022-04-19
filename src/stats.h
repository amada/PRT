#pragma once

#include <atomic>

namespace prt
{

struct Stats
{
    uint64_t nodesTraversed;
    uint64_t primsTraversed;
    uint64_t raysTraced;

    void clear() {
        nodesTraversed = 0;
        primsTraversed = 0;
        raysTraced = 0;
    }

    void merge(const Stats& other) {
        nodesTraversed += other.nodesTraversed;
        primsTraversed += other.primsTraversed;
        raysTraced += other.raysTraced;
    }
};

struct TotalStats
{
    std::atomic<uint64_t> totalNodesTraversed;
    std::atomic<uint64_t> totalPrimsTraversed;
    std::atomic<uint64_t> totalRaysTraced;

    void clear() {
        totalNodesTraversed = 0;
        totalPrimsTraversed = 0;
        totalRaysTraced = 0;
    }

    void merge(const Stats& other) {
        totalNodesTraversed.fetch_add(other.nodesTraversed);
        totalPrimsTraversed.fetch_add(other.primsTraversed);
        totalRaysTraced.fetch_add(other.raysTraced);
    }

    void print() {
        const float kToM = 1.0f/1000000.0f;
        printf("Nodes traversed: %.3fM(%llu)\n", totalNodesTraversed.load()*kToM, totalNodesTraversed.load());
        printf("Prims traversed: %.3fM(%llu)\n", totalPrimsTraversed.load()*kToM, totalPrimsTraversed.load());
        printf("Rays traced: %.3fM(%llu)\n", totalRaysTraced.load()*kToM, totalRaysTraced.load());
    }
};


} // namespace prt
