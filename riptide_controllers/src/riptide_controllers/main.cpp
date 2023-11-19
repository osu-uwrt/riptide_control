#include <vector>
#include <cstdlib>
#include <iostream>
#include <cstring>

// #define NO_OPTIMIZE __attribute__((optimize("O0")))
#define DEGREES_OF_FREEDOM 6
#define STATE_ARRAY_SIZE (DEGREES_OF_FREEDOM * 2)

static double profiles[DEGREES_OF_FREEDOM][10];
static double profile[10] = {0};
static bool initialized = false;
static double
    stored_vmax[DEGREES_OF_FREEDOM],
    stored_amax[DEGREES_OF_FREEDOM],
    stored_jmax[DEGREES_OF_FREEDOM],
    stored_time_step;


//this is a good test to see if linking working
extern "C" void profiler_hello(double *data, int rows, int cols, double *out)
{
    std::cout << "Hello wold from profiler! Have " << rows << " rows and " << cols << " items to process" << std::endl;

    for(int i = 0; i < rows * cols; i++)
    {
        // std::cout << data[i];
        out[i] = data[i] + 1;
    }

    std::cout << std::endl;
}


extern "C" void profiler_init(
    double vmax[DEGREES_OF_FREEDOM], 
    double amax[DEGREES_OF_FREEDOM], 
    double jmax[DEGREES_OF_FREEDOM], 
    double timeStep)
{
    initialized = true;
}


extern "C" bool profiler_initialized()
{
    return initialized;
}


extern "C" void profiler_set_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double timeStep)
{
    memcpy(stored_vmax, vmax, DEGREES_OF_FREEDOM);
    memcpy(stored_amax, amax, DEGREES_OF_FREEDOM);
    memcpy(stored_jmax, jmax, DEGREES_OF_FREEDOM);
    stored_time_step = timeStep;
}


extern "C" void profiler_get_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double *time_step)
{
    memcpy(vmax, stored_vmax, DEGREES_OF_FREEDOM);
    memcpy(amax, stored_amax, DEGREES_OF_FREEDOM);
    memcpy(jmax, stored_jmax, DEGREES_OF_FREEDOM);
    *time_step = stored_time_step;
}


extern "C" int profiler_calculate_new_profile(
    double current_state[STATE_ARRAY_SIZE], 
    double target_state[STATE_ARRAY_SIZE])
{
    // for(int i = 0; i < DEGREES_OF_FREEDOM; i++)
    // {
    //     for(int j = 0; j < 10; j++)
    //     {
    //         profiles[i][j] = rand();
    //     }
    // }

    return 10;
}


extern "C" void profiler_get_profile(int index, double *out)
{
    // memcpy(out, profiles[index], 10);
    memcpy(out, profile, 10);
}
