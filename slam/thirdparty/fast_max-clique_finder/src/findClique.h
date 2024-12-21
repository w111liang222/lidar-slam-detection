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

#ifndef FINDCLIQUE_H_INCLUDED
#define FINDCLIQUE_H_INCLUDED

#include "graphIO.h"
#include <cstddef>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <numeric>
#include <algorithm>
using namespace std;

#ifdef _DEBUG
int DEBUG=1;
#endif

namespace FMC {
    
//Function Definitions
bool fexists(const char *filename);
double wtime();
void usage(char *argv0);
int getDegree(vector<int>* ptrVtx, int idx);
void print_max_clique(vector<int>& max_clique_data);

int maxClique( CGraphIO& gio, int l_bound, vector<int>& max_clique_data );
void maxCliqueHelper( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

int maxCliqueHeu( CGraphIO& gio );
int maxCliqueHeu(CGraphIO& gio, vector<int>& max_clique_data);
void maxCliqueHelperHeu( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

}
#endif 
