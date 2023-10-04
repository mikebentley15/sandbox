#include <utility>

/// RAII-style class to call an arbitrary function upon destruction for cleanup.
/// However, you must ensure that the given function does not throw any exceptions.
template <typename Func>
class ScopeCleaner {
public:
    ScopeCleaner(Func func) : func_(std::move(func)) {}
    ~ScopeCleaner() noexcept { func_(); }
private:
    Func func_;
};
