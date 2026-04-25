#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>

#include <matrix/matrix/math.hpp>

class GravityTransformModule : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    static Descriptor desc;

    GravityTransformModule();
    ~GravityTransformModule() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    int print_status() override;

private:
    void Run() override;

    static constexpr float GRAVITY_WORLD_X = 0.f;
    static constexpr float GRAVITY_WORLD_Y = 0.f;
    static constexpr float GRAVITY_WORLD_Z = -9.81f;

    static constexpr float EPSILON_RECONSTRUCTION = 1e-4f;
    static constexpr float EPSILON_NORM           = 1e-4f;
    static constexpr float EPSILON_ORTHOGONALITY  = 1e-5f;

    static constexpr uint32_t SCHEDULE_INTERVAL_US = 10000;

    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
};
