#pragma once
#include <stdint.h>
#include "C1NativeTileLK.h"
// V2C C1-B bounded single-inflight asynchronous visual FRONT END.
// Host-portable std::thread; Android NDK-compatible in principle, not yet
// integrated with Camera/Encoder. No future images ever enqueued.
#ifdef _WIN32
#define C1BA_EXPORT extern "C" __declspec(dllexport)
#else
#define C1BA_EXPORT extern "C" __attribute__((visibility("default")))
#endif
struct C1BAsyncWorker;
enum C1BSubmitCode {
    C1B_SUBMITTED=0, C1B_BUSY=1, C1B_STOPPED=2,
    C1B_ORDER_FAIL=3, C1B_DEADLINE_EXPIRED=4,
    C1B_BAD_ARGS=-1
};
enum C1BPollCode {
    C1B_READY=0, C1B_PENDING=1, C1B_LATE=2,
    C1B_NO_RESULT=3, C1B_CANCELLED=4, C1B_POLL_BAD_ARGS=-1
};
struct C1BAsyncStats {
    uint64_t submitted,busy,rejected_order,rejected_deadline,completed,
        ready_on_time,late,cancelled,destroyed_pending;
    int64_t last_seen_source,last_seen_sensor_ts;
};
C1BA_EXPORT uint64_t c1b_async_monotonic_ns();
C1BA_EXPORT C1BAsyncWorker* c1b_async_create(int diagnostic_delay_ms);
C1BA_EXPORT int c1b_async_submit(C1BAsyncWorker* worker,
    int64_t source_id,int64_t sensor_ts_ns,uint64_t absolute_deadline_ns,
    const uint8_t* prev_gray,const uint8_t* curr_gray,
    const uint8_t* prev_valid,const uint8_t* curr_valid);
C1BA_EXPORT int c1b_async_poll(C1BAsyncWorker* worker,int64_t expected_source_id,
    C1BVisual* result);
C1BA_EXPORT int c1b_async_cancel(C1BAsyncWorker* worker,int64_t source_id);
C1BA_EXPORT int c1b_async_stats(C1BAsyncWorker* worker,C1BAsyncStats* stats);
C1BA_EXPORT void c1b_async_destroy(C1BAsyncWorker** worker);
