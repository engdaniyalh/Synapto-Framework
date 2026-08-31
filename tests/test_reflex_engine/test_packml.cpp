/**
 * @file test_packml.cpp
 * @brief Google Test suite for ISA-TR88.00.02 PackML state transitions & SPSC Queue.
 */

#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include "packml_sm.hpp"
#include "lockfree_queue.hpp"

TEST(PackMLTest, InitialStateIsStopped) {
    PackMLStateMachine sm(PackMLState::STOPPED, 5);
    EXPECT_EQ(sm.getState(), PackMLState::STOPPED);
}

TEST(PackMLTest, ValidActingWaitingSequence) {
    constexpr int kActingTicks = 5;
    PackMLStateMachine sm(PackMLState::STOPPED, kActingTicks);

    // STOPPED -> RESETTING -> IDLE
    EXPECT_TRUE(sm.processCommand(PackMLCommand::RESET));
    EXPECT_EQ(sm.getState(), PackMLState::RESETTING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::IDLE);

    // IDLE -> STARTING -> EXECUTE
    EXPECT_TRUE(sm.processCommand(PackMLCommand::START));
    EXPECT_EQ(sm.getState(), PackMLState::STARTING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::EXECUTE);

    // EXECUTE -> HOLDING -> HELD
    EXPECT_TRUE(sm.processCommand(PackMLCommand::HOLD));
    EXPECT_EQ(sm.getState(), PackMLState::HOLDING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::HELD);

    // HELD -> UNHOLDING -> EXECUTE
    EXPECT_TRUE(sm.processCommand(PackMLCommand::UNHOLD));
    EXPECT_EQ(sm.getState(), PackMLState::UNHOLDING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::EXECUTE);

    // EXECUTE -> STOPPING -> STOPPED
    EXPECT_TRUE(sm.processCommand(PackMLCommand::STOP));
    EXPECT_EQ(sm.getState(), PackMLState::STOPPING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::STOPPED);
}

TEST(PackMLTest, EmergencyAbortOverride) {
    constexpr int kActingTicks = 5;
    PackMLStateMachine sm(PackMLState::STOPPED, kActingTicks);

    // Transition to EXECUTE
    sm.forceState(PackMLState::EXECUTE);
    EXPECT_EQ(sm.getState(), PackMLState::EXECUTE);

    // Emergency ABORT command
    EXPECT_TRUE(sm.processCommand(PackMLCommand::ABORT));
    EXPECT_EQ(sm.getState(), PackMLState::ABORTING);

    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::ABORTED);

    // Recovery: ABORTED -> CLEARING -> STOPPED
    EXPECT_TRUE(sm.processCommand(PackMLCommand::CLEAR));
    EXPECT_EQ(sm.getState(), PackMLState::CLEARING);
    for (int i = 0; i < kActingTicks; ++i) {
        sm.step();
    }
    EXPECT_EQ(sm.getState(), PackMLState::STOPPED);
}

TEST(PackMLTest, RejectsInvalidTransitions) {
    PackMLStateMachine sm(PackMLState::STOPPED, 5);

    // Cannot START directly from STOPPED
    EXPECT_FALSE(sm.processCommand(PackMLCommand::START));
    EXPECT_EQ(sm.getState(), PackMLState::STOPPED);

    // Cannot HOLD from STOPPED
    EXPECT_FALSE(sm.processCommand(PackMLCommand::HOLD));
    EXPECT_EQ(sm.getState(), PackMLState::STOPPED);
}

TEST(LockFreeQueueTest, SPSCThroughputAndZeroLoss) {
    SPSCQueue<uint64_t, 1024> queue;
    constexpr uint64_t kMessageCount = 1'000'000ULL;

    std::thread producer([&]() {
        for (uint64_t i = 0; i < kMessageCount; ++i) {
            while (!queue.push(i)) {
                std::this_thread::yield();
            }
        }
    });

    std::vector<uint64_t> received;
    received.reserve(kMessageCount);

    std::thread consumer([&]() {
        uint64_t val = 0;
        while (received.size() < kMessageCount) {
            if (queue.pop(val)) {
                received.push_back(val);
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    ASSERT_EQ(received.size(), kMessageCount);
    for (uint64_t i = 0; i < kMessageCount; ++i) {
        EXPECT_EQ(received[i], i);
    }
}

TEST(LockFreeQueueTest, CapacityBounds) {
    constexpr size_t kCap = 16;
    SPSCQueue<int, kCap> queue;

    EXPECT_TRUE(queue.empty());

    // Fill queue to capacity (usable slots = Capacity - 1 in SPSC)
    for (size_t i = 0; i < kCap - 1; ++i) {
        EXPECT_TRUE(queue.push(static_cast<int>(i)));
    }

    // Next push should fail as queue is full
    EXPECT_FALSE(queue.push(999));

    // Pop one item
    int val = -1;
    EXPECT_TRUE(queue.pop(val));
    EXPECT_EQ(val, 0);

    // Now push should succeed
    EXPECT_TRUE(queue.push(999));
}
