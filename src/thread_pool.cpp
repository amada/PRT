#include "thread_pool.h"

namespace prt
{

void ThreadPool::create(int32_t threadCount)
{
    if (threadCount <= 0) {
        uint32_t cpuThreads = std::thread::hardware_concurrency() + threadCount;
        if (cpuThreads == 0) {
            cpuThreads = 8;
            printf("Number of CPU threads is unknown; use %u threads\n", cpuThreads);
        } else {
            printf("%u CPU threads are available for ThreadPool\n", cpuThreads);
        }
        threadCount = cpuThreads;
    }

    m_alive = true;
    m_threadCount = threadCount;
    if (m_workers) delete[] m_workers;
    m_workers = new std::thread[threadCount];
    for (uint32_t i = 0; i < m_threadCount; i++) {
        m_workers[i] = std::thread([this]() { this->workerThreadRun(); });
    }
}

ThreadPool::~ThreadPool()
{
    if (m_alive) {
        waitAllTasksDone();
        m_alive = false;
    }
    delete[] m_workers;
    m_workers = nullptr;
}

void ThreadPool::queue(Task task)
{
    std::unique_lock<std::mutex> guard(m_mutex);

    m_tasks.push_back(task);
    m_cond.notify_one();
}

void ThreadPool::waitAllTasksDone()
{
    if (!m_alive)
        return;
    {
        std::unique_lock<std::mutex> guard(m_mutex);
        m_alive = false;
    }
    m_cond.notify_all();

    for (uint32_t i = 0; i < m_threadCount; i++) {
//    for (auto& worker : m_workers) {
        auto& worker = m_workers[i];
        worker.join();
    }
}

void ThreadPool::workerThreadRun()
{
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> guard(m_mutex);

            while (m_tasks.empty() && m_alive) {
                m_cond.wait(guard);
            }

            if (m_tasks.empty()) {
                break;
            } else {
                task = m_tasks.back();
                m_tasks.pop_back();
            }
        }

        task();
    }
}

} // namespace prt