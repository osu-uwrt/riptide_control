//
// MATLAB Interface for C++ motion profiler
//

#include <iostream>
#include <dlfcn.h>
#include <mex.h>

//
// MACROS
//
#define DL_CHECK(expr, ret_if_bad) \
    do \
    { \
        if(!(expr)) \
        { \
            std::cerr << #expr " failed; dlerror(): " << dlerror() << std::endl; \
            return ret_if_bad; \
        } \
    } while(0)


//
// PARAMETERS
//
#define LIB_SO_NAME                         "libmotion_profiler.so"
#define PROFILER_HELLO_FUNC_NAME            "profiler_hello"
#define PROFILER_INIT_FUNC_NAME             "profiler_init"
#define PROFILER_SET_CONSTRAINTS_FUNC_NAME  "profiler_set_constraints"
#define PROFILER_COMPUTE_FUNC_NAME          "profiler_calculate_new_profile"
#define PROFILER_GET_FUNC_NAME              "profiler_get_profile"
#define DEGREES_OF_FREEDOM                  6
#define STATE_ARRAY_SIZE                    (DEGREES_OF_FREEDOM * 2)

//
// FUNCTION TYPES
//
typedef void (*profiler_hello_func)(
    double *data,
    int rows,
    int cols,
    double *out);

typedef void (*profiler_init_func)(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step);

typedef void (*profiler_compute_func)(
    double current_state[STATE_ARRAY_SIZE],
    double target_state[STATE_ARRAY_SIZE]);

typedef double* (*profiler_get_func)(
    int index);


//
// FUNCTIONS
//
static void *profiler_lib;
static profiler_hello_func profiler_hello;
static profiler_init_func profiler_init;
static profiler_init_func profiler_set_constraints;
static profiler_compute_func profiler_compute;
static profiler_get_func profiler_get;

//
// FUNCTION IMPLEMENTATIONS
//
bool lib_init()
{
    //attempt to open the shared library
    DL_CHECK(profiler_lib = dlopen(LIB_SO_NAME, RTLD_NOW), false);

    //attempt to assign functions
    DL_CHECK(profiler_hello = (profiler_hello_func) dlsym(profiler_lib, PROFILER_HELLO_FUNC_NAME), false);
    DL_CHECK(profiler_init = (profiler_init_func) dlsym(profiler_lib, PROFILER_INIT_FUNC_NAME), false);
    DL_CHECK(profiler_set_constraints = (profiler_init_func) dlsym(profiler_lib, PROFILER_SET_CONSTRAINTS_FUNC_NAME), false);
    DL_CHECK(profiler_compute = (profiler_compute_func) dlsym(profiler_lib, PROFILER_COMPUTE_FUNC_NAME), false);
    DL_CHECK(profiler_get = (profiler_get_func) dlsym(profiler_lib, PROFILER_GET_FUNC_NAME), false);
    
    return true;
}


//this function only exists for terminal output
bool try_lib_init()
{
    if(!profiler_lib)
    {
        if(!lib_init())
        {
            std::cerr << "Failed to open motion profiling library at " LIB_SO_NAME << std::endl;
            return false;
        }

        std::cout << "Successfully opened motion profiling library at " LIB_SO_NAME << std::endl;
    }

    return true;
}


extern "C" void call_profiler_hello(double *mat, int rows, int cols, double *out)
{
    if(!try_lib_init()) return;
    DL_CHECK(profiler_hello,);
    std::cout << "Calling hello with " << rows << " rows and " << cols << " cols" << std::endl;
    profiler_hello(mat, rows, cols, out);
}


extern "C" void call_profiler_init(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step)
{
    if(!try_lib_init()) return;
    DL_CHECK(profiler_init,);
    profiler_init(vmax, amax, jmax, time_step);
}


extern "C" void call_profiler_set_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step)
{
    if(!try_lib_init()) return;
    DL_CHECK(profiler_set_constraints,);
    profiler_set_constraints(vmax, amax, jmax, time_step);
}


extern "C" void call_profiler_compute(
    double current_state[STATE_ARRAY_SIZE],
    double target_state[STATE_ARRAY_SIZE])
{
    if(!try_lib_init()) return;
    DL_CHECK(profiler_compute,);
    profiler_compute(current_state, target_state);
}


extern "C" double *call_profiler_get(int index)
{
    if(!try_lib_init()) return {};
    DL_CHECK(profiler_get, {});
    return profiler_get(index);
}
