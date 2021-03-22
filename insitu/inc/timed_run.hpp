#ifndef insitu_TIMED_RUN_HPP
#define insitu_TIMED_RUN_HPP

/*

    Shamelessly stolen from Alex Che's brilliant answer on Stack Overflow:
    https://stackoverflow.com/questions/40550730/how-to-implement-timeout-for-function-in-c

*/

#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

template <typename TF, typename TDuration, class... TArgs>
std::result_of_t<TF && (TArgs && ...)> run_with_timeout(TF&& f,
                                                        TDuration timeout,
                                                        TArgs&&... args)
{
    using R = std::result_of_t<TF && (TArgs && ...)>;
    std::packaged_task<R(TArgs...)> task(f);
    auto future = task.get_future();
    std::thread thr(std::move(task), std::forward<TArgs>(args)...);
    if (future.wait_for(timeout) != std::future_status::timeout)
    {
        thr.join();
        return future.get();    // this will propagate exception from f() if any
    }
    else
    {
        thr.detach();    // we leave the thread still running, non-ideal
        throw std::runtime_error("Timeout");
    }
}

template <typename TF, typename TDuration, class... TArgs>
void try_run_with_timeout(TF&& f, TDuration timeout, TArgs&&... args)
{
    try
    {
        run_with_timeout(f, timeout, std::forward<TArgs>(args)...);
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
}

#endif    // insitu_TIMED_RUN_HPP
