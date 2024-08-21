#pragma once

#include <deque>
#include <mutex>

namespace core
{
    namespace common
    {
        template <typename MsgType>
        struct ThreadSafeDeque
        {
            std::deque<MsgType> deque;
            std::mutex mtx;
        };
    }
}