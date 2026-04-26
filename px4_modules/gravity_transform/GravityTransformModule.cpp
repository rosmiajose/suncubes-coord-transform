#include "GravityTransformModule.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

ModuleBase::Descriptor GravityTransformModule::desc{task_spawn, custom_command, print_usage};

GravityTransformModule::GravityTransformModule() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

GravityTransformModule::~GravityTransformModule()
{
    ScheduleClear();
}

int GravityTransformModule::task_spawn(int argc, char *argv[])
{
    GravityTransformModule *instance = new GravityTransformModule();
    if (instance) {
        desc.object.store(instance);
        desc.task_id = task_id_is_work_queue;
        instance->ScheduleOnInterval(SCHEDULE_INTERVAL_US);
        return PX4_OK;
    }
    PX4_ERR("alloc failed");
    return PX4_ERROR;
}

void GravityTransformModule::Run()
{
    vehicle_attitude_s att;
    if (!_vehicle_attitude_sub.update(&att)) {
        return;
    }

    const matrix::Vector3f g_world(GRAVITY_WORLD_X, GRAVITY_WORLD_Y, GRAVITY_WORLD_Z);
    const matrix::Quatf q(att.q);
    const matrix::Dcmf R(q);

    const matrix::Vector3f g_body = R * g_world;
    const matrix::Vector3f g_reconstructed = R.transpose() * g_body;

    const matrix::SquareMatrix<float, 3> RRt = R * R.transpose();
    const matrix::SquareMatrix<float, 3> I = matrix::eye<float, 3>();
    const matrix::SquareMatrix<float, 3> ortho_err_mat = RRt - I;

    float ortho_error_sq = 0.f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ortho_error_sq += ortho_err_mat(i, j) * ortho_err_mat(i, j);
        }
    }
    const float orthogonality_error = sqrtf(ortho_error_sq);
    const float norm_g_body = g_body.norm();
    const float norm_g_world = g_world.norm();
    const float norm_error = fabsf(norm_g_body - norm_g_world);
    const float reconstruction_error = (g_reconstructed - g_world).norm();

    const bool ok = (orthogonality_error < EPSILON_ORTHOGONALITY)
            && (norm_error < EPSILON_NORM)
            && (reconstruction_error < EPSILON_RECONSTRUCTION);

    // Only log once per second (every 100th cycle at 100 Hz)
    static uint32_t log_counter = 0;
    log_counter++;
    if (log_counter % 100 != 0) {
        return;
    }

    // Log results using PX4 logging system
    PX4_INFO("ts:%" PRIu64 " g_body:[%.3f,%.3f,%.3f] recon:[%.3f,%.3f,%.3f] err:%.1e %s",
             att.timestamp,
             (double)g_body(0), (double)g_body(1), (double)g_body(2),
             (double)g_reconstructed(0), (double)g_reconstructed(1), (double)g_reconstructed(2),
             (double)reconstruction_error, ok ? "OK" : "FAIL");
}

int GravityTransformModule::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int GravityTransformModule::print_status()
{
    PX4_INFO("Running on work queue, interval: %u us", SCHEDULE_INTERVAL_US);
    return 0;
}

int GravityTransformModule::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Real-time gravity vector coordinate transformation module (out-of-tree).
Uses PX4 logging system for output.
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("gravity_transform", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int gravity_transform_main(int argc, char *argv[])
{
    return ModuleBase::main(GravityTransformModule::desc, argc, argv);
}
