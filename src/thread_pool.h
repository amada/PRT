#include <deque>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <thread>

namespace prt
{

class ThreadPool
{
public:
    typedef std::function<void()> Task;
//    ThreadPool();
    virtual ~ThreadPool();

    void create(uint32_t threadCount);
    void queue(Task task);
    void waitAllTasksDone();
    size_t getTaskCount() const { return m_tasks.size(); } // not thread safe

private:
    void workerThreadRun();

    std::deque<Task> m_tasks;
    std::mutex m_mutex;
    std::condition_variable m_cond;
    std::thread* m_workers = nullptr;
    bool m_alive = false;
    uint32_t m_threadCount = 0;
};

}
