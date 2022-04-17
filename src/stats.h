#pragma once

namespace prt
{

struct Stats
{
    uint64_t nodesTraversed;

    void clear() {
        nodesTraversed = 0;
    }

    void merge(const Stats& other) {
        nodesTraversed += other.nodesTraversed;
    }
};


} // namespace prt
