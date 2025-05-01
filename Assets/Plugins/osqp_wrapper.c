#include "osqp.h"
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

EXPORT_API void SolveCBF(float* position, float* velocity, float* neighbors, int num_neigh, float CBF_DS, float CBF_C); {
    // If the Boid has no neighbors skip CBF calculation
	if (num_neigh == 0) {
		return;
	}

	c_float P_x[2] = {1.0, 1.0};
    c_int P_nnz  = 2;
    c_int P_r[2] = {0, 1};
    c_int P_c[3] = {0, 1, 2};
    c_float q[2]   = {-1 * velocity[0], -1 * velocity[1]};
    c_float A_x[num_neigh * 2];
    c_int A_nnz  = num_neigh * 2;
    c_int A_r[num_neigh * 2];
    c_int A_c[3] = {0, num_neigh, num_neigh * 2};
    c_float l[num_neigh];
	c_float u[num_neigh];
	c_int n = 2;
    c_int m = num_neigh;

	for (int i = 0; i < num_neigh; i++) {
        A_x[i] = (position[0] - neighbors[i * 3]);
        A_x[i + num_neigh] = (position[1] - neighbors[i * 3 + 1]);
        A_r[i] = i;
        A_r[i + num_neigh] = i;

        float h = (
			pow(position[0] - neighbors[i * 3], 2) + 
			pow(position[1] - neighbors[i * 3 + 1], 2) - 
			(CBF_DS * CBF_DS)
		);

        l[i] = -1 * CBF_C * h;
        u[i] = INT_MAX; // We don't have a true upper limit, so make it max
	}

	// Exitflag
	c_int exitflag = 0;

  	// Workspace structures
  	OSQPWorkspace *work;
  	OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  	OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

  	// Populate data
  	if (data) {
  	  data->n = n;
  	  data->m = m;
  	  data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_r, P_c);
  	  data->q = q;
  	  data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_r, A_c);
  	  data->l = l;
  	  data->u = u;
  	}

	// Define solver settings as default
  	if (settings) osqp_set_default_settings(settings);
	settings->verbose = 0;
	settings->polish = 1;
	settings->eps_abs = 1E-12;
	settings->eps_rel = 1E-12;
	settings->eps_prim_inf = 1E-12;
	settings->eps_dual_inf = 1E-12;

  	// Setup workspace
  	exitflag = osqp_setup(&work, data, settings);
	if (exitflag != 0) {
		printf("ERROR: Failed OSQP setup with flag %d\n", (int)exitflag);
	}

  	// Solve Problem
  	osqp_solve(work);
	
	velocity[0] = work->solution->x[0];
	velocity[1] = work->solution->x[1];

	// free all osqp structures/memory
	osqp_cleanup(work);
	if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
	c_free(settings);
}
