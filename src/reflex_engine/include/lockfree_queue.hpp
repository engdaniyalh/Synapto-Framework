/**
 * @file lockfree_queue.hpp
 * @brief Cache-aligned, Lock-Free Single-Producer Single-Consumer (SPSC) Ring Buffer.
 *
 * Designed for high-frequency deterministic data transfer between hard real-time
 * threads (SCHED_FIFO 100Hz control loop) and non-real-time threads (OPC UA and UDP network workers).
 *
 * Enforces 64-byte cache line alignment to prevent false sharing and uses explicit
 * memory orders (acquire/release) to ensure data visibility without locks.
 */

#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <optional>
#include <new>

template <typename T, size_t Capacity = 1024>
class SPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Queue Capacity must be a power of 2");

private:
    std::array<T, Capacity> buffer_{};

    /** Separate head and tail into independent 64-byte cache lines to eliminate false sharing */
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};

public:
    SPSCQueue() noexcept : head_(0), tail_(0) {}

    ~SPSCQueue() = default;
    SPSCQueue(const SPSCQueue&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;
    SPSCQueue(SPSCQueue&&) = delete;
    SPSCQueue& operator=(SPSCQueue&&) = delete;

    /**
     * @brief Pushes an item into the queue (Single-Producer thread only).
     * 
     * @param item Data element to enqueue.
     * @return true if successfully inserted, false if queue is full.
     */
    bool push(const T& item) noexcept {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (current_tail + 1) & (Capacity - 1);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue full
        }

        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pops an item by reference (Single-Consumer thread only).
     * 
     * @param item Output destination reference.
     * @return true if item was extracted, false if queue is empty.
     */
    bool pop(T& item) noexcept {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Queue empty
        }

        item = buffer_[current_head];
        head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    /**
     * @brief Pops an item returning std::optional (Single-Consumer thread only).
     * 
     * @return std::optional<T> containing the item if present, std::nullopt otherwise.
     */
    std::optional<T> pop() noexcept {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Queue empty
        }

        T item = buffer_[current_head];
        head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
        return item;
    }

    /**
     * @brief Checks if the queue is empty.
     */
    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Returns the fixed capacity of the queue.
     */
    [[nodiscard]] constexpr size_t capacity() const noexcept {
        return Capacity;
    }

    /**
     * @brief Calculates approximate count of available elements.
     */
    [[nodiscard]] size_t size() const noexcept {
        const size_t h = head_.load(std::memory_order_relaxed);
        const size_t t = tail_.load(std::memory_order_relaxed);
        if (t >= h) {
            return t - h;
        }
        return Capacity - (h - t);
    }
};
