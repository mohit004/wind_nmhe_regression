/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmhe_auxiliary_functions.h"

#include <stdio.h>

real_t* nmhe_getVariablesX( )
{
	return nmheVariables.x;
}

real_t* nmhe_getVariablesU( )
{
	return nmheVariables.u;
}

#if NMHE_NY > 0
real_t* nmhe_getVariablesY( )
{
	return nmheVariables.y;
}
#endif

#if NMHE_NYN > 0
real_t* nmhe_getVariablesYN( )
{
	return nmheVariables.yN;
}
#endif

real_t* nmhe_getVariablesX0( )
{
#if NMHE_INITIAL_VALUE_FIXED
	return nmheVariables.x0;
#else
	return 0;
#endif
}

/** Print differential variables. */
void nmhe_printDifferentialVariables( )
{
	int i, j;
	printf("\nDifferential variables:\n[\n");
	for (i = 0; i < NMHE_N + 1; ++i)
	{
		for (j = 0; j < NMHE_NX; ++j)
			printf("\t%e", nmheVariables.x[i * NMHE_NX + j]);
		printf("\n");
	}
	printf("]\n\n");
}

/** Print control variables. */
void nmhe_printControlVariables( )
{
	int i, j;
	printf("\nControl variables:\n[\n");
	for (i = 0; i < NMHE_N; ++i)
	{
		for (j = 0; j < NMHE_NU; ++j)
			printf("\t%e", nmheVariables.u[i * NMHE_NU + j]);
		printf("\n");
	}
	printf("]\n\n");
}

/** Print ACADO code generation notice. */
void nmhe_printHeader( )
{
	printf(
		"\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n"
		"Copyright (C) 2008-2015 by Boris Houska, Hans Joachim Ferreau,\n" 
		"Milan Vukov and Rien Quirynen, KU Leuven.\n"
	);
	
	printf(
		"Developed within the Optimization in Engineering Center (OPTEC) under\n"
		"supervision of Moritz Diehl. All rights reserved.\n\n"
		"ACADO Toolkit is distributed under the terms of the GNU Lesser\n"
		"General Public License 3 in the hope that it will be useful,\n"
		"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
		"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n"
		"GNU Lesser General Public License for more details.\n\n"
	);
}

#if !(defined _DSPACE)
#if (defined _WIN32 || defined _WIN64) && !(defined __MINGW32__ || defined __MINGW64__)

void nmhe_tic( nmhe_timer* t )
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}

real_t nmhe_toc( nmhe_timer* t )
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (real_t)t->freq.QuadPart);
}


#elif (defined __APPLE__)

void nmhe_tic( nmhe_timer* t )
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}

real_t nmhe_toc( nmhe_timer* t )
{

    uint64_t duration; /* elapsed time in clock cycles*/
    
    t->toc = mach_absolute_time();
    duration = t->toc - t->tic;
    
    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (real_t)duration / 1e9;
}

#else

#if __STDC_VERSION__ >= 199901L
/* C99 mode */

/* read current time */
void nmhe_tic( nmhe_timer* t )
{
	gettimeofday(&t->tic, 0);
}

/* return time passed since last call to tic on this timer */
real_t nmhe_toc( nmhe_timer* t )
{
	struct timeval temp;
	
	gettimeofday(&t->toc, 0);
    
	if ((t->toc.tv_usec - t->tic.tv_usec) < 0)
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_usec = 1000000 + t->toc.tv_usec - t->tic.tv_usec;
	}
	else
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_usec = t->toc.tv_usec - t->tic.tv_usec;
	}
	
	return (real_t)temp.tv_sec + (real_t)temp.tv_usec / 1e6;
}

#else
/* ANSI */

/* read current time */
void nmhe_tic( nmhe_timer* t )
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}


/* return time passed since last call to tic on this timer */
real_t nmhe_toc( nmhe_timer* t )
{
	struct timespec temp;
    
	clock_gettime(CLOCK_MONOTONIC, &t->toc);	
    
	if ((t->toc.tv_nsec - t->tic.tv_nsec) < 0)
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_nsec = 1000000000+t->toc.tv_nsec - t->tic.tv_nsec;
	}
	else
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}
	
	return (real_t)temp.tv_sec + (real_t)temp.tv_nsec / 1e9;
}

#endif /* __STDC_VERSION__ >= 199901L */

#endif /* (defined _WIN32 || _WIN64) */

#endif
