#include <windows.h>
#include "osqp.h"
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

EXPORT_API void SolveCBF(OSQPFloat* position, OSQPFloat* velocity, OSQPFloat* neighbors, int num_neigh, OSQPFloat CBF_DS, OSQPFloat CBF_C) {
    // If the Boid has no neighbors skip CBF calculation
	if (num_neigh == 0) {
		return;
	}
	
	OSQPInt n = 2;
    OSQPInt m = num_neigh;
	
	OSQPFloat P_x[2] = {1.0, 1.0};
	OSQPInt P_nnz  = 2;
    OSQPInt P_r[2] = {0, 1};
    OSQPInt P_c[3] = {0, 1, 2};
    
	OSQPFloat q[2]   = {-1 * velocity[0], -1 * velocity[1]};
    OSQPFloat* A_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * m * 2);
    OSQPInt A_nnz  = m * 2;
    OSQPInt* A_r = (OSQPInt*)malloc(sizeof(OSQPInt) * m * 2);
    OSQPInt A_c[3] = {0, m, m * 2};
    OSQPFloat* l = (OSQPFloat*)malloc(sizeof(OSQPFloat) * m);
	OSQPFloat* u = (OSQPFloat*)malloc(sizeof(OSQPFloat) * m);

	for (OSQPInt i = 0; i < m; i++) {
        A_x[i] = (position[0] - neighbors[i * 3]);
        A_x[i + m] = (position[1] - neighbors[i * 3 + 1]);
        A_r[i] = i;
        A_r[i + m] = i;

        OSQPFloat h = (
			pow(position[0] - neighbors[i * 3], 2) + 
			pow(position[1] - neighbors[i * 3 + 1], 2) - 
			(CBF_DS * CBF_DS)
		);

        l[i] = -1 * CBF_C * h;
        u[i] = INT_MAX;
	}

	OSQPInt exitflag = 0;

	OSQPSolver *solver;

	OSQPCscMatrix* P = OSQPCscMatrix_new(n, n, P_nnz, P_x, P_r, P_c);
	OSQPCscMatrix* A = OSQPCscMatrix_new(m, n, A_nnz, A_x, A_r, A_c);

	OSQPSettings *settings = OSQPSettings_new();
	settings->verbose = 0;
	settings->polishing = 1;
	settings->eps_abs = 1E-12;
	settings->eps_rel = 1E-12;
	settings->eps_prim_inf = 1E-12;
	settings->eps_dual_inf = 1E-12;
  	
  	// Setup solver
  	exitflag = osqp_setup(&solver, P, q, A, l, u, m, n, settings);

  	// Solve Problem
  	if(!exitflag) exitflag = osqp_solve(solver);
	
	velocity[0] = solver->solution->x[0];
	velocity[1] = solver->solution->x[1];

	// free all osqp structures/memory
	osqp_cleanup(solver);
	OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
	OSQPSettings_free(settings);
}
