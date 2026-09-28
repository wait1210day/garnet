// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_HERMES_H
#define GARNET_HERMES_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <type_traits>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "base/err.h"
#include "base/static_queue.h"
#include "base/panic.h"

namespace hermes
{

template<uint16_t IdV, typename PayloadT_>
struct Topic
{
    static constexpr uint16_t kId = IdV;
    using PayloadT = PayloadT_;

    static_assert(std::is_trivially_copyable_v<PayloadT>, "requires trivially copyable payload");
    static_assert(std::is_standard_layout_v<PayloadT>, "requires standard layout payload");
};

template<typename TopicT>
concept TopicDef = requires {
    { TopicT::kId } -> std::convertible_to<uint16_t>;
    typename TopicT::PayloadT;
} && std::is_trivially_copyable_v<typename TopicT::PayloadT>;

struct MessageHeader
{
    uint16_t topic;
    TaskHandle_t from_task;
    TickType_t timestamp_ticks;
};

template<TopicDef TopicT>
struct Message
{
    using TopicType = TopicT;
    using PayloadT = typename TopicT::PayloadT;

    MessageHeader header;
    PayloadT payload;
};

namespace detail
{

enum class DeliveryMode : uint8_t
{
    kQueue,
    kOverwrite,
};

struct SubscriptionNode
{
    QueueHandle_t queue = nullptr;
    SubscriptionNode* next = nullptr;
    bool used = false;
    DeliveryMode mode = DeliveryMode::kQueue;
};

template<size_t MaxTopics>
struct TopicTable
{
    std::array<SubscriptionNode*, MaxTopics> heads{};
};

} // namespace detail

template<TopicDef TopicT, size_t QUEUE_DEPTH>
class Queue final : public StaticQueue<Message<TopicT>, QUEUE_DEPTH>
{
public:
    using StaticQueue<Message<TopicT>, QUEUE_DEPTH>::StaticQueue;
};

template<TopicDef TopicT>
class OverwriteQueue final : public StaticQueue<Message<TopicT>, 1>
{
public:
    using StaticQueue<Message<TopicT>, 1>::StaticQueue;
};

template<size_t MaxTopics = 16, size_t MaxSubscriptions = 16, size_t MaxPayloadBytes = 128>
class Bus
{
public:
    constexpr Bus() = default;

    void init()
    {
        taskENTER_CRITICAL();
        for (auto& node : sub_pool_)
        {
            node = {};
        }
        for (auto& head : topics_.heads)
        {
            head = nullptr;
        }
        taskEXIT_CRITICAL();
    }

    template<TopicDef TopicT, size_t QueueDepth>
    [[nodiscard]] ErrStatus sub(Queue<TopicT, QueueDepth>& queue)
    {
        static_assert(TopicT::kId < MaxTopics, "topic id exceeds bus topic table size");
        static_assert(sizeof(typename TopicT::PayloadT) <= MaxPayloadBytes,
                      "payload type exceeds bus max payload size");
        return sub_impl(TopicT::kId, queue.native_handle(), detail::DeliveryMode::kQueue);
    }

    template<TopicDef TopicT>
    [[nodiscard]] ErrStatus sub_overwrite(OverwriteQueue<TopicT>& queue)
    {
        static_assert(TopicT::kId < MaxTopics, "topic id exceeds bus topic table size");
        static_assert(sizeof(typename TopicT::PayloadT) <= MaxPayloadBytes,
                      "payload type exceeds bus max payload size");
        return sub_impl(TopicT::kId, queue.native_handle(), detail::DeliveryMode::kOverwrite);
    }

    template<TopicDef TopicT>
    [[nodiscard]] ErrStatus pub(const typename TopicT::PayloadT& payload)
    {
        static_assert(TopicT::kId < MaxTopics, "topic id exceeds bus topic table size");
        static_assert(sizeof(typename TopicT::PayloadT) <= MaxPayloadBytes,
                      "payload type exceeds bus max payload size");

        auto* curr = topics_.heads[TopicT::kId];
        if (curr == nullptr)
            return ErrStatus::kOk;

        Message<TopicT> msg{
            .header = MessageHeader{
                .topic = TopicT::kId,
                .from_task = xTaskGetCurrentTaskHandle(),
                .timestamp_ticks = xTaskGetTickCount(),
            },
            .payload = payload,
        };

        while (curr != nullptr)
        {
            // NOLINTNEXTLINE
            switch (curr->mode)
            {
            case detail::DeliveryMode::kQueue:
                static_cast<void>(xQueueSend(curr->queue, &msg, 0));
                break;
            case detail::DeliveryMode::kOverwrite:
                static_cast<void>(xQueueOverwrite(curr->queue, &msg));
                break;
            }
            curr = curr->next;
        }

        return ErrStatus::kOk;
    }

    template<TopicDef TopicT>
    [[nodiscard]] std::expected<void, ErrStatus> pub_expected(const typename TopicT::PayloadT& payload)
    {
        auto err = pub<TopicT>(payload);
        if (err != ErrStatus::kOk)
            return std::unexpected(err);
        return {};
    }

private:
    [[nodiscard]] detail::SubscriptionNode* alloc_free_sub_node()
    {
        for (auto& node : sub_pool_)
        {
            if (!node.used)
            {
                node.used = true;
                node.next = nullptr;
                node.queue = nullptr;
                node.mode = detail::DeliveryMode::kQueue;
                return &node;
            }
        }
        return nullptr;
    }

    [[nodiscard]] ErrStatus sub_impl(uint16_t topic, QueueHandle_t queue, detail::DeliveryMode mode)
    {
        if (topic >= MaxTopics || queue == nullptr)
            return ErrStatus::kInvalidArg;

        taskENTER_CRITICAL();

        auto*& head = topics_.heads[topic];
        for (auto* curr = head; curr != nullptr; curr = curr->next)
        {
            if (curr->queue == queue)
            {
                taskEXIT_CRITICAL();
                return ErrStatus::kOk;
            }
        }

        auto* node = alloc_free_sub_node();
        if (node == nullptr)
        {
            taskEXIT_CRITICAL();
            return ErrStatus::kNoMem;
        }

        node->queue = queue;
        node->mode = mode;
        node->next = head;
        head = node;

        taskEXIT_CRITICAL();
        return ErrStatus::kOk;
    }

    std::array<detail::SubscriptionNode, MaxSubscriptions> sub_pool_{};
    detail::TopicTable<MaxTopics> topics_{};
};

Bus<>& get_default_bus();

} // namespace hermes

#endif //GARNET_HERMES_H
