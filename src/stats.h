#pragma once

#include <atomic>

#include "log.h"

namespace prt
{

struct Stats
{
    uint64_t nodesTraversed;
    uint64_t primsTraversed;
    uint64_t raysTraced;
    uint64_t occludedTraced;
    uint64_t triTested;

    void clear() {
        nodesTraversed = 0;
        primsTraversed = 0;
        raysTraced = 0;
        occludedTraced = 0;
        triTested = 0;
    }

    void merge(const Stats& other) {
        nodesTraversed += other.nodesTraversed;
        primsTraversed += other.primsTraversed;
        raysTraced += other.raysTraced;
        occludedTraced += other.occludedTraced;
        triTested += other.triTested;
    }
};

struct TotalStats
{
    std::atomic<uint64_t> totalNodesTraversed;
    std::atomic<uint64_t> totalPrimsTraversed;
    std::atomic<uint64_t> totalRaysTraced;
    std::atomic<uint64_t> totalOccludedTraced;
    std::atomic<uint64_t> totalTriTested;;

    void clear() {
        totalNodesTraversed = 0;
        totalPrimsTraversed = 0;
        totalRaysTraced = 0;
        totalOccludedTraced = 0;
        totalTriTested = 0;
    }

    void merge(const Stats& other) {
        totalNodesTraversed.fetch_add(other.nodesTraversed);
        totalPrimsTraversed.fetch_add(other.primsTraversed);
        totalRaysTraced.fetch_add(other.raysTraced);
        totalOccludedTraced.fetch_add(other.occludedTraced);
        totalTriTested.fetch_add(other.triTested);
    }

    void print() {
        const float kToM = 1.0f/1000000.0f;
        logPrintf(LogLevel::kInfo, "Nodes traversed: %.3fM(%llu)\n", totalNodesTraversed.load()*kToM, totalNodesTraversed.load());
        logPrintf(LogLevel::kInfo, "Prims traversed: %.3fM(%llu)\n", totalPrimsTraversed.load()*kToM, totalPrimsTraversed.load());
        logPrintf(LogLevel::kInfo, "Rays traced: %.3fM(%llu)\n", totalRaysTraced.load()*kToM, totalRaysTraced.load());
        logPrintf(LogLevel::kInfo, "Occluded traced: %.3fM(%llu)\n", totalOccludedTraced.load()*kToM, totalOccludedTraced.load());
        logPrintf(LogLevel::kInfo, "Tri tested: %.3fM(%llu)\n", totalTriTested.load()*kToM, totalTriTested.load());
    }
};


} // namespace prt
