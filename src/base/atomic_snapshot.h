// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_ATOMIC_SNAPSHOT_H
#define GARNET_ATOMIC_SNAPSHOT_H

#include <utility>
#include <atomic>
#include <type_traits>
#include <limits>
#include <cstring>
#include <cassert>
#include <optional>

#include "FreeRTOS.h"
#include "task.h"

#include "base/defs.h"
#include "base/priorities.h"
#include "base/panic.h"


// Architecture assumption:
//  Cortex-M0/M0+/M3/M4 single core
//  Thus, store/load with relaxed memory order + `std::atomic_signal_fence(acquire/release)`
//  can be used instead of store/load with acquire/release memory order, which eliminates
//  the unnecessary overhead of `dmb` instruction.
//

// A double-buffered atomic wrapper for any trivially copyable type `T`.
// You are forced to provide the priorities to avoid misuse.
// Note that this is a strict SPSC data structure.
template<typename T>
class AtomicSnapshot
{
    static_assert(std::is_trivially_copyable_v<T>, "`T` must be trivially copyable");

public:
    GARNET_NON_COPYABLE(AtomicSnapshot)
    GARNET_NON_MOVABLE(AtomicSnapshot)

    explicit AtomicSnapshot(const T& initial_value)
        : associated_(false)
        , front_idx_(0)
        , bufs_{initial_value, initial_value} {}

    template<Priority READER_PRI>
    class Reader
    {
    public:
        GARNET_NON_COPYABLE(Reader)

        constexpr static Priority kPriority = READER_PRI;

        Reader() : parent_(nullptr) {}
        Reader(Reader&& other) noexcept : parent_(other.parent_) {
            other.parent_ = nullptr;
        }
        Reader& operator=(Reader&& other) noexcept {
            if (this != &other)
            {
                parent_ = other.parent_;
                other.parent_ = nullptr;
            }
            return *this;
        }

        [[nodiscard]] T load() const {
            GCHECK(parent_ && "invalid operation");

            const uint8_t front_idx = parent_->front_idx_.load(std::memory_order_relaxed);
            std::atomic_signal_fence(std::memory_order_acquire);
            return parent_->bufs_[front_idx];
        }

    private:
        friend class AtomicSnapshot;
        explicit Reader(AtomicSnapshot *parent) : parent_(parent) {}

        AtomicSnapshot *parent_;
    };

    template<Priority WRITER_PRI>
    class Writer
    {
    public:
        GARNET_NON_COPYABLE(Writer)

        constexpr static Priority kPriority = WRITER_PRI;

        Writer() : parent_(nullptr) {}
        Writer(Writer&& other) noexcept : parent_(other.parent_) {
            other.parent_ = nullptr;
        }
        Writer& operator=(Writer&& rhs) noexcept {
            if (this != &rhs)
            {
                parent_ = rhs.parent_;
                rhs.parent_ = nullptr;
            }
            return *this;
        }

        void store(const T& value)
        {
            GCHECK(parent_ && "invalid operation");

            const uint8_t front_idx = parent_->front_idx_.load(std::memory_order_relaxed);
            const uint8_t back_idx = front_idx ^ 1;
            parent_->bufs_[back_idx] = value;

            std::atomic_signal_fence(std::memory_order_release);
            parent_->front_idx_.store(back_idx, std::memory_order_relaxed);
        }

        template<typename...ArgsT>
        void emplace(ArgsT&&... args)
        {
            GCHECK(parent_ && "invalid operation");
            const uint8_t front_idx = parent_->front_idx_.load(std::memory_order_relaxed);
            const uint8_t back_idx = front_idx ^ 1;
            parent_->bufs_[back_idx] = T{std::forward<ArgsT>(args)...};

            std::atomic_signal_fence(std::memory_order_release);
            parent_->front_idx_.store(back_idx, std::memory_order_relaxed);
        }

        template<typename Func>
        void modify(Func&& mutator)
        {
            GCHECK(parent_ && "invalid operation");
            const uint8_t front_idx = parent_->front_idx_.load(std::memory_order_relaxed);
            const uint8_t back_idx = front_idx ^ 1;

            parent_->bufs_[back_idx] = parent_->bufs_[front_idx];
            mutator(parent_->bufs_[back_idx]);

            std::atomic_signal_fence(std::memory_order_release);
            parent_->front_idx_.store(back_idx, std::memory_order_relaxed);
        }

    private:
        friend class AtomicSnapshot;
        explicit Writer(AtomicSnapshot *parent) : parent_(parent) {}

        AtomicSnapshot* parent_;
    };

    template<Priority READER_PRI, Priority WRITER_PRI>
    [[nodiscard]] std::pair<Reader<READER_PRI>, Writer<WRITER_PRI>> make_transfer_pair()
    {
        static_assert(READER_PRI > WRITER_PRI,
            "reader must have higher priority than writer");
        GCHECK(!associated_ && "AtomicSnapshot cannot be associated more than once");

        associated_ = true;
        return {Reader<READER_PRI>{this}, Writer<WRITER_PRI>{this}};
    }

private:
    bool associated_;
    std::atomic<uint8_t> front_idx_;
    T bufs_[2];
};


// Seqlock, single producer multiple readers, while reader priority MUST lower than writer's.
// Does NOT block writer but may block readers.
template<typename T, Priority WRITER_PRI>
class SingleCoreSeqlock
{
    static_assert(std::is_trivially_copyable_v<T>, "`T` must be trivially copyable");

public:
    GARNET_NON_COPYABLE(SingleCoreSeqlock)
    GARNET_NON_MOVABLE(SingleCoreSeqlock)

    explicit SingleCoreSeqlock(const T& initial_value) : seq_(0), val_(initial_value) {}

    void store(const T& value)
    {
        val_ = value;

        std::atomic_signal_fence(std::memory_order_release);
        seq_.store(seq_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }

    template<Priority PRI>
    [[nodiscard]] std::optional<T> load(TickType_t timeout) const
    {
        static_assert(PriorityTraits<PRI>::is_task(), "irq handlers must not call this function");
        static_assert(PRI < WRITER_PRI, "reader priority must lower than writer's");

        T local;
        uint32_t s1, s2;
        TickType_t start_tick = xTaskGetTickCount(), dt = 0;

        do
        {
            s1 = seq_.load(std::memory_order_relaxed);
            std::atomic_signal_fence(std::memory_order_acq_rel);

            // Strictly, this is undefined behavior because of data race condition
            // defined by cpp standard... but I think there are no safe alternatives
            // without introducing extra overhead.
            std::memcpy(&local, &val_, sizeof(local));

            std::atomic_signal_fence(std::memory_order_acq_rel);
            s2 = seq_.load(std::memory_order_relaxed);

            dt = xTaskGetTickCount() - start_tick;

            // If `timeout == 0`, the caller expecting a non-blocking load, we only
            // try once. If we got dirty data, there is no need to try again.
        } while (s1 != s2 && dt < timeout);

        if (s1 == s2)
            return local;
        return std::nullopt;
    }

    // Only try once, completely lock-free
    template<Priority PRI>
    [[nodiscard]] std::optional<T> try_load() const
    {
        static_assert(PRI < WRITER_PRI, "reader priority must lower than writer's");

        T local;
        const uint32_t s1 = seq_.load(std::memory_order_relaxed);
        std::atomic_signal_fence(std::memory_order_acq_rel);

        std::memcpy(&local, &val_, sizeof(local));

        std::atomic_signal_fence(std::memory_order_acq_rel);
        const uint32_t s2 = seq_.load(std::memory_order_relaxed);

        if (s1 == s2)
            return local;
        return std::nullopt;
    }

    template<Priority PRI>
    [[nodiscard]] bool try_load(T& local) const
    {
        static_assert(PRI < WRITER_PRI, "reader priority must lower than writer's");

        const uint32_t s1 = seq_.load(std::memory_order_relaxed);
        std::atomic_signal_fence(std::memory_order_acq_rel);

        std::memcpy(&local, &val_, sizeof(local));

        std::atomic_signal_fence(std::memory_order_acq_rel);
        const uint32_t s2 = seq_.load(std::memory_order_relaxed);

        return s1 == s2;
    }

private:
    std::atomic<uint32_t> seq_;
    T val_;
};

#endif //GARNET_ATOMIC_SNAPSHOT_H
