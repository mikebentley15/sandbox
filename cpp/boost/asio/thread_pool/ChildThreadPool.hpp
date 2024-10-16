#include <boost/asio/thread_pool.hpp>

class ChildThreadPool {
public:
    ChildThreadPool(boost::thread_pool &parent, std::size_t threadCount)
    {
        for (size_t i = 0; i < threadCount; ++i)
        {
            boost::asio::post(parent, [this]() { pool_.attach(); });
        }
    }
private:
    boost::thread_pool pool_;
};
