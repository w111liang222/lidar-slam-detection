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
int pruned1;
int pruned2;
int pruned3;
int pruned5;

/* Algorithm 2: CLIQUE: Recursive Subroutine of algorithm 1. */
void maxCliqueHelper( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter )
{
	int iPos, index = 0, maxClq_prev;
	vector <int>* ptrVertex = gio.GetVerticesPtr();
	vector <int>* ptrEdge = gio.GetEdgesPtr();
	vector <int> U_new;
   U_new.reserve(gio.GetVertexCount());

	if( U->size() == 0  )
	{
		if( sizeOfClique > maxClq )
		{
			maxClq = sizeOfClique;
         max_clique_data_inter.clear();
		}
		return;
	}

	while( U->size() > 0 )
	{
		//Old Pruning
		if( sizeOfClique + U->size() <= maxClq )
			return;

		index = U->back();
		U->pop_back();

		// Loop over neighbrs of v_index.
		for(int j = (*ptrVertex)[index]; j < (*ptrVertex)[index + 1]; j++ )
			//Pruning 5
			if( getDegree(ptrVertex, (*ptrEdge)[j]) >=  maxClq )
			{
				// Loop over U.
				for(int i = 0; i < U->size(); i++)
				{
					if( (*ptrEdge)[j] == (*U)[i] )
						U_new.push_back( (*ptrEdge)[j]);
				}
			}
			else
				pruned3++;

      maxClq_prev = maxClq;

      maxCliqueHelper( gio, &U_new, sizeOfClique + 1, maxClq, max_clique_data_inter );

      if(maxClq > maxClq_prev)
         max_clique_data_inter.push_back(index);

		U_new.clear();
	}
}

/* Algorithm 1: MAXCLIQUE: Finds maximum clique of the given graph */
int maxClique( CGraphIO& gio, int l_bound, vector<int>& max_clique_data )
{
	vector <int>* ptrVertex = gio.GetVerticesPtr();
	vector <int>* ptrEdge = gio.GetEdgesPtr();
	vector <int> U;
   U.reserve(gio.GetVertexCount());
   vector<int> max_clique_data_inter;
   max_clique_data_inter.reserve(gio.GetVertexCount());
   max_clique_data.reserve(gio.GetVertexCount());
	int maxClq = l_bound;
   int prev_maxClq;

	//cout << "Computing Max Clique... with lower bound " << maxClq << endl;
	pruned1 = 0;
	pruned2 = 0;
	pruned3 = 0;
	pruned5 = 0;

	//Bit Vector to track if vertex has been considered previously.
	int *bitVec = new int[gio.GetVertexCount()];
	memset(bitVec, 0, gio.GetVertexCount() * sizeof(int));

	for(int i = gio.GetVertexCount()-1; i >= 0; i--)
	{
		bitVec[i] = 1;
      prev_maxClq = maxClq;

		U.clear();
		//Pruning 1
		if( getDegree(ptrVertex, i) < maxClq)
		{
			pruned1++;
			continue;
		}

		for( int j = (*ptrVertex)[i]; j < (*ptrVertex)[i + 1]; j++ )
		{	
			//Pruning 2
			if(bitVec[(*ptrEdge)[j]] != 1)
			{
				//Pruning 3
				if( getDegree(ptrVertex, (*ptrEdge)[j]) >=  maxClq )			
					U.push_back((*ptrEdge)[j]);
				else
					pruned3++;
			}
			else 
				pruned2++;
		}

		maxCliqueHelper( gio, &U, 1, maxClq, max_clique_data_inter );

      if(maxClq > prev_maxClq)
      {
         max_clique_data_inter.push_back(i);
         max_clique_data = max_clique_data_inter;
		}
		max_clique_data_inter.clear();
	}

   delete [] bitVec;
   max_clique_data_inter.clear();

#ifdef _DEBUG
	cout << "Pruning 1 = " << pruned1 << endl;
	cout << "Pruning 2 = " << pruned2 << endl;
	cout << "Pruning 3 = " << pruned3 << endl;
	cout << "Pruning 5 = " << pruned5 << endl;
#endif

		return maxClq;
}

void print_max_clique(vector<int>& max_clique_data)
{
   //cout << "Maximum clique: ";
   for(int i = 0; i < max_clique_data.size(); i++)
      cout << max_clique_data[i] + 1 << " ";
   cout << endl;
}
}