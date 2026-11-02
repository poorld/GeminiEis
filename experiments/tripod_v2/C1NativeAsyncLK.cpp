#include "C1NativeAsyncLK.h"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

using Clock=std::chrono::steady_clock;
static constexpr size_t FrameBytes=320u*180u;
static constexpr size_t JobBytes=FrameBytes*4u;
uint64_t c1b_async_monotonic_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count());
}
struct C1BAsyncWorker {
    std::mutex mu;
    std::condition_variable cv;
    std::thread thread;
    bool stop=false,has_pending=false,working=false,ready=false;
    bool cancelled_current=false;
    int diag_delay_ms=0;
    int64_t job_id=0,job_sensor_ts=0;
    uint64_t job_deadline=0,done_ns=0;
    std::array<uint8_t,JobBytes> frames{};
    C1BVisual finished{};
    C1BAsyncStats stats{};
    explicit C1BAsyncWorker(int delay):diag_delay_ms(delay){
        stats.last_seen_source=-1;
        thread=std::thread([this] {workerLoop();});
    }
    void workerLoop(){
        while(true){
            std::array<uint8_t,JobBytes> local{};
            int64_t id=0;
            {
                std::unique_lock<std::mutex> lock(mu);
                cv.wait(lock,[&]{return stop||has_pending;});
                if(stop && !has_pending)break;
                local=frames;
                id=job_id;
                has_pending=false;
                working=true;
            }
            if(diag_delay_ms>0)
                std::this_thread::sleep_for(std::chrono::milliseconds(diag_delay_ms));
            C1BVisual out{};
            c1b_visual_pair(local.data(),local.data()+FrameBytes,
                local.data()+FrameBytes*2,local.data()+FrameBytes*3,
                320,180,320,&out);
            const uint64_t done=c1b_async_monotonic_ns();
            {
                std::lock_guard<std::mutex> lock(mu);
                working=false;
                ++stats.completed;
                if(stop ||cancelled_current || id!=job_id) {
                    ++stats.destroyed_pending;
                    ready=false;
                } else {
                    finished=out;
                    done_ns=done;
                    ready=true;
                }
            }
        }
    }
};
C1BAsyncWorker* c1b_async_create(int diagnostic_delay_ms){
    if(diagnostic_delay_ms<0||diagnostic_delay_ms>2000)return nullptr;
    try{return new C1BAsyncWorker(diagnostic_delay_ms);}catch(...){return nullptr;}
}
int c1b_async_submit(C1BAsyncWorker* w,
    int64_t source_id,int64_t sensor_ts_ns,uint64_t absolute_deadline_ns,
    const uint8_t* prev_gray,const uint8_t* curr_gray,
    const uint8_t* prev_valid,const uint8_t* curr_valid) {
    if(!w||!prev_gray||!curr_gray||!prev_valid||!curr_valid||
       source_id<0||sensor_ts_ns<=0)return C1B_BAD_ARGS;
    std::lock_guard<std::mutex> lock(w->mu);
    if(w->stop)return C1B_STOPPED;
    if(source_id<=w->stats.last_seen_source ||
       sensor_ts_ns<=w->stats.last_seen_sensor_ts) {
        ++w->stats.rejected_order;
        return C1B_ORDER_FAIL;
    }
    // Mark this SourceID as seen, even when dropped due to backpressure.
    w->stats.last_seen_source=source_id;
    w->stats.last_seen_sensor_ts=sensor_ts_ns;
    if(absolute_deadline_ns<=c1b_async_monotonic_ns()){
        ++w->stats.rejected_deadline;
        return C1B_DEADLINE_EXPIRED;
    }
    if(w->working||w->has_pending||w->ready) {
        ++w->stats.busy;
        return C1B_BUSY;
    }
    w->job_id=source_id;
    w->job_sensor_ts=sensor_ts_ns;
    w->job_deadline=absolute_deadline_ns;
    w->cancelled_current=false;
    w->ready=false;
    std::memcpy(w->frames.data(),prev_gray,FrameBytes);
    std::memcpy(w->frames.data()+FrameBytes,curr_gray,FrameBytes);
    std::memcpy(w->frames.data()+2*FrameBytes,prev_valid,FrameBytes);
    std::memcpy(w->frames.data()+3*FrameBytes,curr_valid,FrameBytes);
    w->has_pending=true;
    ++w->stats.submitted;
    w->cv.notify_one();
    return C1B_SUBMITTED;
}
int c1b_async_poll(C1BAsyncWorker* w,int64_t expected_source_id,
                   C1BVisual* result) {
    if(!w||!result)return C1B_POLL_BAD_ARGS;
    std::lock_guard<std::mutex> lock(w->mu);
    // SourceID / sensorTS must be *exact*; never reuse a previous frame's
    // LK output as current, even if the previous worker eventually finishes.
    if(expected_source_id!=w->job_id)return C1B_NO_RESULT;
    const uint64_t now=c1b_async_monotonic_ns();
    if(w->ready) {
        w->ready=false;
        if(w->cancelled_current || w->done_ns>w->job_deadline ||
           now>w->job_deadline) {
            w->cancelled_current=true;
            ++w->stats.late;
            return C1B_LATE;
        }
        *result=w->finished;
        ++w->stats.ready_on_time;
        return C1B_READY;
    }
    if(w->cancelled_current)return C1B_CANCELLED;
    if(now>w->job_deadline){
        w->cancelled_current=true;
        ++w->stats.late;
        return C1B_LATE;
    }
    return C1B_PENDING;
}
int c1b_async_cancel(C1BAsyncWorker* w,int64_t source_id) {
    if(!w)return -1;
    std::lock_guard<std::mutex> lock(w->mu);
    if(source_id!=w->job_id)return 1;
    if(w->cancelled_current)return 0;
    w->cancelled_current=true;
    w->ready=false;
    if(w->has_pending)w->has_pending=false;
    ++w->stats.cancelled;
    return 0;
}
int c1b_async_stats(C1BAsyncWorker* w,C1BAsyncStats* out) {
    if(!w||!out)return -1;
    std::lock_guard<std::mutex> lock(w->mu);
    *out=w->stats;
    return 0;
}
void c1b_async_destroy(C1BAsyncWorker** ptr){
    if(!ptr||!*ptr)return;
    C1BAsyncWorker* w=*ptr;
    *ptr=nullptr;
    {
        std::lock_guard<std::mutex> lock(w->mu);
        w->stop=true;
        w->has_pending=false;
        w->cancelled_current=true;
        w->ready=false;
    }
    w->cv.notify_all();
    if(w->thread.joinable())w->thread.join();
    delete w;
}
