#pragma once

namespace prt
{

struct Stats
{
    uint64_t nodesTraversed;
    uint64_t primsTraversed;

    void clear() {
        nodesTraversed = 0;
        primsTraversed = 0;
    }

    void merge(const Stats& other) {
        nodesTraversed += other.nodesTraversed;
        primsTraversed += other.primsTraversed;
    }
};


} // namespace prt
