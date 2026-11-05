#define _POSIX_C_SOURCE 200809L
/* Batch the acados stage API calls while retaining the generated solver. */
#include <time.h>
#include "acados_solver_slash_mpcc.h"

static double elapsed(const struct timespec *start, const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec)
        + (double)(end->tv_nsec - start->tv_nsec) * 1e-9;
}

/* All arrays are row-major stage arrays; x and u use normalized OCP units.
 * timings[0:3] are input, solve and output durations in seconds.
 */
static int step_impl(
    slash_mpcc_solver_capsule *capsule, int horizon,
    const double *x_guess, const double *u_guess, const double *parameters,
    const double *lower_h, const double *upper_h,
    double *x_solution, double *u_solution, double *timings,
    int *solver_status, const double *first_lower_u,
    const double *first_upper_u)
{
    if (!capsule || horizon != SLASH_MPCC_N || !x_guess || !u_guess ||
        !parameters || !lower_h || !upper_h || !x_solution ||
        !u_solution || !timings || !solver_status)
        return -1;

    struct timespec t0, t1, t2, t3;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int i = 0; i <= horizon; ++i) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims,
                        capsule->nlp_out, capsule->nlp_in, i,
                        "x", (void *)(x_guess + SLASH_MPCC_NX * i));
        if (slash_mpcc_acados_update_params(capsule, i,
                (double *)(parameters + 4 * i), 4) != 0)
            return 100 + i;
        if (i > 0) {
            if (ocp_nlp_constraints_model_set(capsule->nlp_config,
                    capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
                    i, "lh", (void *)(lower_h + i)) != 0)
                return 200 + i;
            if (ocp_nlp_constraints_model_set(capsule->nlp_config,
                    capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
                    i, "uh", (void *)(upper_h + i)) != 0)
                return 300 + i;
        }
        if (i < horizon)
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims,
                            capsule->nlp_out, capsule->nlp_in, i,
                            "u", (void *)(u_guess + 3 * i));
    }
    if (ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
            capsule->nlp_in, capsule->nlp_out, 0, "lbx",
            (void *)x_guess) != 0)
        return 400;
    if (ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
            capsule->nlp_in, capsule->nlp_out, 0, "ubx",
            (void *)x_guess) != 0)
        return 401;
    if (first_lower_u && first_upper_u) {
        if (ocp_nlp_constraints_model_set(capsule->nlp_config,
                capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
                0, "lbu", (void *)first_lower_u) != 0)
            return 500;
        if (ocp_nlp_constraints_model_set(capsule->nlp_config,
                capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
                0, "ubu", (void *)first_upper_u) != 0)
            return 501;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    *solver_status = slash_mpcc_acados_solve(capsule);
    clock_gettime(CLOCK_MONOTONIC, &t2);
    for (int i = 0; i <= horizon; ++i) {
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims,
                        capsule->nlp_out, i, "x", x_solution + SLASH_MPCC_NX * i);
        if (i < horizon)
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims,
                            capsule->nlp_out, i, "u", u_solution + 3 * i);
    }
    clock_gettime(CLOCK_MONOTONIC, &t3);
    timings[0] = elapsed(&t0, &t1);
    timings[1] = elapsed(&t1, &t2);
    timings[2] = elapsed(&t2, &t3);
    return 0;
}

int slash_mpcc_c_batch_step(
    slash_mpcc_solver_capsule *capsule, int horizon,
    const double *x_guess, const double *u_guess, const double *parameters,
    const double *lower_h, const double *upper_h,
    double *x_solution, double *u_solution, double *timings,
    int *solver_status)
{
    return step_impl(capsule, horizon, x_guess, u_guess, parameters,
                     lower_h, upper_h, x_solution, u_solution, timings,
                     solver_status, 0, 0);
}

/* Keep all future stage limits at the prediction mesh interval, but use the
 * actual feedback interval for the command that is applied now. */
int slash_mpcc_c_batch_step_timed(
    slash_mpcc_solver_capsule *capsule, int horizon,
    const double *x_guess, const double *u_guess, const double *parameters,
    const double *lower_h, const double *upper_h,
    const double *first_lower_u, const double *first_upper_u,
    double *x_solution, double *u_solution, double *timings,
    int *solver_status)
{
    if (!first_lower_u || !first_upper_u) return -1;
    return step_impl(capsule, horizon, x_guess, u_guess, parameters,
                     lower_h, upper_h, x_solution, u_solution, timings,
                     solver_status, first_lower_u, first_upper_u);
}
