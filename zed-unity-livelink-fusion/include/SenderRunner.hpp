#ifndef  __SENDER_RUNNER_HDR__
#define __SENDER_RUNNER_HDR__

#include <sl/Fusion.hpp>

#include <thread>

class SenderRunner {

public:
    SenderRunner();
    ~SenderRunner();

    void loadInit(sl::String initFile);
    bool open(sl::InputType);
    void start();
    void stop();

private:
    sl::Camera zed;
    sl::InitParameters init_params;
    sl::PositionalTrackingParameters positional_tracking_parameters;
    sl::BodyTrackingParameters body_tracking_parameters;
    sl::BodyTrackingRuntimeParameters body_runtime_parameters;
    void work();
    std::thread runner;
    bool running;
};

#endif // ! __SENDER_RUNNER_HDR__
