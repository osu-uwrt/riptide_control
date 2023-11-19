#include <vector>
#include <cstdlib>
#include <iostream>
#include <cstring>

// #define NO_OPTIMIZE __attribute__((optimize("O0")))
#define DEGREES_OF_FREEDOM 6
#define STATE_ARRAY_SIZE (DEGREES_OF_FREEDOM * 2)

double profiles[DEGREES_OF_FREEDOM][10];


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

}


extern "C" void profiler_set_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double timeStep)
{

}


extern "C" void profiler_calculate_new_profile(
    double current_state[STATE_ARRAY_SIZE], 
    double target_state[STATE_ARRAY_SIZE])
{
    for(int i = 0; i < DEGREES_OF_FREEDOM; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            profiles[i][j] = rand();
        }
    }
}


extern "C" void profiler_get_profile(int index, double *out, int *length)
{
    memcpy(out, profiles[index], 10);
    *length = 10;
}
