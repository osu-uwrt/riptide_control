//
// MATLAB Interface for C++ motion profiler
//

#include <iostream>
#include <dlfcn.h>
#include <cstring>
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
#define PROFILER_INITIALIZED_FUNC_NAME      "profiler_initialized"
#define PROFILER_SET_CONSTRAINTS_FUNC_NAME  "profiler_set_constraints"
#define PROFILER_GET_CONSTRAINTS_FUNC_NAME  "profiler_get_constraints"
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

typedef bool (*profiler_bool_func)();

typedef void (*profiler_init_func)(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step);

typedef void (*profiler_get_constraints_func)(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double *time_step);

typedef int (*profiler_compute_func)(
    double current_state[STATE_ARRAY_SIZE],
    double target_state[STATE_ARRAY_SIZE]);

typedef void (*profiler_get_profile_func)(
    int index,
    double *out);


//
// FUNCTIONS
//
static void *profiler_lib;
static profiler_hello_func profiler_hello;
static profiler_init_func profiler_init;
static profiler_bool_func profiler_initialized;
static profiler_init_func profiler_set_constraints;
static profiler_get_constraints_func profiler_get_constraints;
static profiler_compute_func profiler_compute;
static profiler_get_profile_func profiler_get_profile;


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
    DL_CHECK(profiler_initialized = (profiler_bool_func) dlsym(profiler_lib, PROFILER_INITIALIZED_FUNC_NAME), false);
    DL_CHECK(profiler_set_constraints = (profiler_init_func) dlsym(profiler_lib, PROFILER_SET_CONSTRAINTS_FUNC_NAME), false);
    DL_CHECK(profiler_get_constraints = (profiler_get_constraints_func) dlsym(profiler_lib, PROFILER_GET_CONSTRAINTS_FUNC_NAME), false);
    DL_CHECK(profiler_compute = (profiler_compute_func) dlsym(profiler_lib, PROFILER_COMPUTE_FUNC_NAME), false);
    DL_CHECK(profiler_get_profile = (profiler_get_profile_func) dlsym(profiler_lib, PROFILER_GET_FUNC_NAME), false);
    
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


void call_profiler_init(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step)
{
    DL_CHECK(profiler_init,);
    profiler_init(vmax, amax, jmax, time_step);
}


bool call_profiler_initialized()
{
    DL_CHECK(profiler_initialized, false);
    return profiler_initialized;
}


void call_profiler_set_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step)
{
    DL_CHECK(profiler_set_constraints,);
    profiler_set_constraints(vmax, amax, jmax, time_step);
}


void call_profiler_get_constraints(
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double *time_step)
{
    DL_CHECK(profiler_get_constraints,);
    profiler_get_constraints(vmax, amax, jmax, time_step);
}


int call_profiler_compute(
    double current_state[STATE_ARRAY_SIZE],
    double target_state[STATE_ARRAY_SIZE])
{
    DL_CHECK(profiler_compute, 0);
    return profiler_compute(current_state, target_state);
}


extern "C" int profiler_invoke(
    double current_state[STATE_ARRAY_SIZE],
    double target_state[STATE_ARRAY_SIZE],
    double vmax[DEGREES_OF_FREEDOM],
    double amax[DEGREES_OF_FREEDOM],
    double jmax[DEGREES_OF_FREEDOM],
    double time_step)
{
    if(!try_lib_init()) return 0;

    //initialize library if needed
    if(!call_profiler_initialized())
    {
        call_profiler_init(vmax, amax, jmax, time_step);
    }

    //ensure that parameters are up to date
    double
        current_vmax[DEGREES_OF_FREEDOM],
        current_amax[DEGREES_OF_FREEDOM],
        current_jmax[DEGREES_OF_FREEDOM],
        current_time_step;
    
    call_profiler_get_constraints(current_vmax, current_amax, current_jmax, &current_time_step);

    if(
        memcmp(current_vmax, vmax, DEGREES_OF_FREEDOM) != 0
        || memcmp(current_amax, amax, DEGREES_OF_FREEDOM) != 0
        || memcmp(current_jmax, jmax, DEGREES_OF_FREEDOM) != 0
        || current_time_step != time_step)
    {
        call_profiler_set_constraints(vmax, amax, jmax, time_step);
    }

    //compute motion profile
    return call_profiler_compute(current_state, target_state);
}


extern "C" void profiler_get(int index, double *out)
{
    if(!try_lib_init()) return;
    DL_CHECK(profiler_get_profile,);
    profiler_get_profile(index, out);
}
