/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  a library for finding the maximum clique of a graph     		   */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Bharath Pattabiraman and Md. Mostofa Ali Patwary               		   */
/*            EECS Department, Northwestern University                       		   */
/*            email: {bpa342,mpatwary}@eecs.northwestern.edu                 		   */
/*                                                                           		   */
/*   Copyright, 2014, Northwestern University			             		   */
/*   See COPYRIGHT notice in top-level directory.                            		   */
/*                                                                           		   */
/*   Please site the following publication if you use this package:           		   */
/*   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2, 	   	   */
/*   Wei-keng Liao, and Alok Choudhary.	 					   	   */
/*   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with           	   */
/*   Applications to Overlapping Community Detection"				  	   */
/*   http://arxiv.org/abs/1411.7460 		 					   */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "findClique.h"

namespace FMC {
bool fexists(const char *filename)
{
	ifstream ifile(filename);
	return ifile.is_open();
}

double wtime() // returns wall time in seconds
{
	timeval tv;
	gettimeofday(&tv, NULL);
	return (double)tv.tv_sec + (double)tv.tv_usec/1000000;
}

void usage(char *argv0)
{
	const char *params =
		"Usage: %s [options...] inputfile\n"
		"OPTIONS:\n"
		"\t-t algorithm type\t: 0 for exact, 1 for heuristic(default)\n"
		"\t-l input lower bound\t: intial max clique size e.g. default 0\n"
		"\t-p print clique\t\t: This parameter if want to print the max clique (only for exact algorithm)\n";
	fprintf(stderr, params, argv0);
	exit(-1);
}

int getDegree(vector<int>* ptrVtx, int idx)
{
	return ( (*ptrVtx)[idx+1] - (*ptrVtx)[idx] );
}
}