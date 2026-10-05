// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_COROUTINE_H
#define GARNET_COROUTINE_H

#include <utility>
#include <coroutine>
#include <span>

#include "FreeRTOS.h"
#include "task.h"

#include "base/defs.h"
#include "base/panic.h"
#include "base/static_queue.h"

namespace coro
{

struct FrameStore
{
    std::span<std::byte> buf;
};

template<size_t SizeBytes>
struct alignas(std::max_align_t) FrameBuffer
{
    std::byte buf[SizeBytes]{};
    FrameStore as_store()
    {
        return FrameStore{std::span<std::byte>{buf, SizeBytes}};
    }

    [[nodiscard]] constexpr size_t size() const {
        return SizeBytes;
    }
};


using ConditionPredF = bool(void*);

class PollAsync
{
public:
    GARNET_NON_COPYABLE(PollAsync)

    struct promise_type
    {
        void *pred_ctx_{nullptr};
        ConditionPredF *pred_fn_{nullptr};
        TickType_t wake_tick_{0};
        bool condition_met_{false};
        bool waiting_{false};

        PollAsync get_return_object()
        {
            return PollAsync{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        void return_void() noexcept {}

        void unhandled_exception() noexcept
        {
            panic::panic("bug: unhandled exception in coroutine. this is not expected.");
        }

        void suspend_for(TickType_t ticks, ConditionPredF *pred, void *ctx)
        {
            pred_fn_ = pred;
            pred_ctx_ = ctx;

            if (ticks == portMAX_DELAY)
                wake_tick_ = portMAX_DELAY;
            else
                wake_tick_ = xTaskGetTickCount() + ticks;

            condition_met_ = false;
            waiting_ = true;
        }

        [[nodiscard]] bool is_waiting() const
        {
            return waiting_;
        }

        [[nodiscard]] bool check_condition() const
        {
            if (pred_fn_)
                return pred_fn_(pred_ctx_);
            return false;
        }

        [[nodiscard]] bool is_timeout(TickType_t now) const
        {
            if (wake_tick_ == portMAX_DELAY)
                return false;
            return static_cast<int32_t>(now - wake_tick_) >= 0;
        }

        void set_condition_met(bool met)
        {
            condition_met_ = met;
        }

        [[nodiscard]] bool was_condition_met() const
        {
            return condition_met_;
        }

        void clear_wait()
        {
            waiting_ = false;
            pred_ctx_ = nullptr;
            pred_fn_ = nullptr;
        }

        static void *operator new(size_t size, const FrameStore& store, auto&&...)
        {
            GCHECK(!store.buf.empty() && "invalid coroutine frame buffer");
            GCHECK(size <= store.buf.size() && "provided coroutine frame size too small");

            return store.buf.data();
        }

        // no-op on delete, since the frame is allocated statically, or managed by
        // the caller explicitly.
        static void operator delete(void *ptr, size_t) {}
    };
    using Handle = std::coroutine_handle<promise_type>;

    enum class PollStatus
    {
        kEmpty,
        kPending,
        kYielded,
        kFinished
    };

    constexpr PollAsync() = default;
    explicit PollAsync(const Handle& handle) : handle_(handle) {}

    PollAsync(PollAsync&& other) noexcept
        : handle_(std::exchange(other.handle_, nullptr)) {}
    PollAsync& operator=(PollAsync&& other) noexcept
    {
        if (this != &other)
        {
            if (handle_)
                handle_.destroy();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    void abort()
    {
        if (handle_)
        {
            handle_.destroy();
            handle_ = nullptr;
        }
    }

    [[nodiscard]] bool is_running() const
    {
        return handle_ && !handle_.done();
    }

    PollStatus poll(TickType_t now) const
    {
        if (!handle_)
            return PollStatus::kEmpty;
        if (handle_.done())
            return PollStatus::kFinished;

        auto& promise = handle_.promise();
        if (promise.is_waiting())
        {
            if (promise.check_condition())
            {
                // condition was met, wake up the coroutine
                promise.set_condition_met(true);
                promise.clear_wait();
                handle_.resume();
            }
            else if (promise.is_timeout(now))
            {
                promise.set_condition_met(false);
                promise.clear_wait();
                handle_.resume();
            }
            else
                return PollStatus::kPending;
        }
        else
        {
            // If the coroutine is not waiting anything,
            // continue its execution.
            handle_.resume();
        }
        if (handle_.done())
            return PollStatus::kFinished;

        return PollStatus::kYielded;
    }

private:
    Handle handle_;
};

enum class Cond
{
    kTimeout,
    kMet
};

template<typename Predicate>
struct AwaitDelayOrCondition
{
    TickType_t ticks_;
    Predicate pred_;
    PollAsync::Handle handle_{nullptr};
    bool immediate_ready_{false};
    // if the awaiter is immediately ready, what result should it return
    // in `await_resume()`?
    Cond immediate_result_{Cond::kTimeout};

    AwaitDelayOrCondition(TickType_t ticks, Predicate&& pred)
        : ticks_(ticks), pred_(std::forward<Predicate>(pred))
    {
        if (pred_())
        {
            // condition has been met, do not suspend anything,
            // since the condition has higher priority than the timeout event.
            immediate_ready_ = true;
            immediate_result_ = Cond::kMet;
        }
        else if (ticks_ == 0)
        {
            // condition has not been met
            immediate_ready_ = true;
            immediate_result_ = Cond::kTimeout;
        }
    }

    bool await_ready() const noexcept { return immediate_ready_; }

    void await_suspend(PollAsync::Handle handle) noexcept
    {
        handle_ = handle;
        handle_.promise().suspend_for(ticks_, +[](void *ctx) {
            return (*static_cast<Predicate*>(ctx))();
        }, &pred_);
    }

    Cond await_resume() const noexcept
    {
        if (immediate_ready_)
            return immediate_result_;
        return handle_.promise().was_condition_met() ?
            Cond::kMet : Cond::kTimeout;
    }
};


template<typename T, size_t QueueLength>
struct AwaitQueueReceive
{
    using Queue = StaticQueue<T, QueueLength>;

    TickType_t ticks_;
    PollAsync::Handle handle_{nullptr};
    Queue& queue_;
    T& out_;
    bool immediate_ready_{false};
    Cond immediate_result_{Cond::kTimeout};

    AwaitQueueReceive(TickType_t ticks, Queue& queue, T& out)
        : ticks_(ticks), queue_(queue), out_(out)
    {
        if (queue_.receive(out_, pdMS_TO_TICKS(0)))
        {
            immediate_ready_ = true;
            immediate_result_ = Cond::kMet;
        }
        else if (ticks_ == 0)
        {
            immediate_ready_ = true;
            immediate_result_ = Cond::kTimeout;
        }
    }

    bool await_ready() const noexcept { return immediate_ready_; }

    void await_suspend(PollAsync::Handle handle) noexcept
    {
        handle_ = handle;
        handle_.promise().suspend_for(ticks_, +[](void *ctx) {
            auto *self = static_cast<AwaitQueueReceive*>(ctx);
            return self->queue_.receive(self->out_, pdMS_TO_TICKS(0));
        }, this);
    }

    Cond await_resume() const noexcept
    {
        if (immediate_ready_)
            return immediate_result_;
        return handle_.promise().was_condition_met() ?
            Cond::kMet : Cond::kTimeout;
    }
};

struct AwaitDelay
{
    TickType_t ticks_;
    PollAsync::Handle handle_{nullptr};

    explicit AwaitDelay(TickType_t ticks) : ticks_(ticks) {}

    bool await_ready() const noexcept { return ticks_ == 0; }

    void await_suspend(PollAsync::Handle handle) noexcept
    {
        handle_ = handle;
        handle_.promise().suspend_for(ticks_, nullptr, nullptr);
    }

    void await_resume() const noexcept {}
};

template<typename Predicate>
auto delay_or_condition(TickType_t ticks, Predicate&& pred)
{
    return AwaitDelayOrCondition<Predicate>{ticks, std::forward<Predicate>(pred)};
}

template<typename T, size_t QueueLength>
auto queue_recv(TickType_t ticks, StaticQueue<T, QueueLength>& queue, T& out)
{
    return AwaitQueueReceive(ticks, queue, out);
}

template<typename Predicate>
auto wait_condition(Predicate&& pred)
{
    return AwaitDelayOrCondition<Predicate>{portMAX_DELAY, std::forward<Predicate>(pred)};
}

inline auto delay(TickType_t ticks)
{
    return AwaitDelay{ticks};
}

} // namespace coro

#endif //GARNET_COROUTINE_H
