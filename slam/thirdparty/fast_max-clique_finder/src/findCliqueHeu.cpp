/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  a library for finding the maximum clique from a graph     		   */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Md. Mostofa Ali Patwary and Bharath Pattabiraman             		   */
/*            EECS Department, Northwestern University                       		   */
/*            email: {mpatwary,bpa342}@eecs.northwestern.edu                 		   */
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
#include <cstddef>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>

namespace FMC {
int maxDegree, maxClq;

/* Algorithm 2: MaxCliqueHeu: A heuristic to find maximum clique */
int maxCliqueHeu(CGraphIO& gio)
{
	vector <int>* p_v_i_Vertices = gio.GetVerticesPtr();
	vector <int>* p_v_i_Edges = gio.GetEdgesPtr();

	//srand(time(NULL));

	maxDegree = gio.GetMaximumVertexDegree();
	int maxClq = - 1, u, icc;
	vector < int > v_i_S;
	vector < int > v_i_S1;
	v_i_S.resize(maxDegree + 1, 0);
	v_i_S1.resize(maxDegree + 1, 0);

	int iPos, iPos1, iCount, iCount1;

	int notComputed = 0;

	// compute the max clique for each vertex
	for(int iCandidateVertex = 0; iCandidateVertex < p_v_i_Vertices->size() - 1; iCandidateVertex++)
	{
		// Pruning 1
		if(maxClq > ((*p_v_i_Vertices)[iCandidateVertex + 1] - (*p_v_i_Vertices)[iCandidateVertex]))
		{
			notComputed++;
			continue;
		}
		
		iPos = 0;		
		v_i_S[iPos++] = iCandidateVertex;
		int z, imdv = iCandidateVertex, imd = (*p_v_i_Vertices)[iCandidateVertex + 1] - (*p_v_i_Vertices)[iCandidateVertex];

		int iLoopCount = (*p_v_i_Vertices)[iCandidateVertex + 1];
		for(int j = (*p_v_i_Vertices)[iCandidateVertex]; j < iLoopCount; j++)
		{
			// Pruning 3
			if(maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[j] + 1] - (*p_v_i_Vertices)[(*p_v_i_Edges)[j]]))
				v_i_S[iPos++] = (*p_v_i_Edges)[j];
		}

		icc = 0;
		
		while(iPos > 0)
		{
			int imdv1 = -1, imd1 = -1;

			icc++;

			// generate a random number x from 0 to iPos -1
			// aasign that imdv = x and imd = d(x)
			//imdv = v_i_S[rand() % iPos];
			imdv = v_i_S[iPos-1];
			imd = (*p_v_i_Vertices)[imdv + 1] - (*p_v_i_Vertices)[imdv];

			iPos1 = 0;

			for(int j = 0; j < iPos; j++)
			{
				iLoopCount = (*p_v_i_Vertices)[imdv + 1];
				for(int k = (*p_v_i_Vertices)[imdv]; k < iLoopCount; k++)
				{
					// Pruning 5
					if(v_i_S[j] == (*p_v_i_Edges)[k] && maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[k] + 1] - (*p_v_i_Vertices)[(*p_v_i_Edges)[k]]))
					{
						v_i_S1[iPos1++] = v_i_S[j]; // calculate the max degree vertex here
						
						break;
					}
				}
			}

			for(int j = 0; j < iPos1; j++)
				v_i_S[j] = v_i_S1[j];
			
			iPos = iPos1;

			imdv = imdv1;
			imd = imd1;			
		}
	
		if(maxClq < icc)
			maxClq = icc;
			
	}

	return maxClq;
}

/* Algorithm 2: MaxCliqueHeu: A heuristic to find maximum clique */
/* Modified by Josh Mangelson to return possible Max clique */
int maxCliqueHeu(CGraphIO& gio, vector<int>& max_clique_data)
{
	vector <int>* p_v_i_Vertices = gio.GetVerticesPtr();
	vector <int>* p_v_i_Edges = gio.GetEdgesPtr();

    vector <int> max_clique_data_inter;
    max_clique_data_inter.reserve(gio.GetVertexCount());
    max_clique_data.reserve(gio.GetVertexCount());

	//srand(time(NULL));

	maxDegree = gio.GetMaximumVertexDegree();
	int maxClq = - 1, u, icc;
	vector < int > v_i_S;
	vector < int > v_i_S1;
	v_i_S.resize(maxDegree + 1, 0);
	v_i_S1.resize(maxDegree + 1, 0);

	int iPos, iPos1, iCount, iCount1;

	int notComputed = 0;

	// compute the max clique for each vertex
	for(int iCandidateVertex = 0; iCandidateVertex < p_v_i_Vertices->size() - 1; iCandidateVertex++)
	{
      max_clique_data_inter.clear ();

      //      std::cout << iCandidateVertex << " ---------------\n";
        // Pruning 1
		if(maxClq > ((*p_v_i_Vertices)[iCandidateVertex + 1] - (*p_v_i_Vertices)[iCandidateVertex]))
		{
			notComputed++;
			continue;
		}
		
		iPos = 0;		
		v_i_S[iPos++] = iCandidateVertex;
		int z, imdv = iCandidateVertex, imd = (*p_v_i_Vertices)[iCandidateVertex + 1] - (*p_v_i_Vertices)[iCandidateVertex];

		int iLoopCount = (*p_v_i_Vertices)[iCandidateVertex + 1];
		for(int j = (*p_v_i_Vertices)[iCandidateVertex]; j < iLoopCount; j++)
		{
			// Pruning 3
			if(maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[j] + 1] - (*p_v_i_Vertices)[(*p_v_i_Edges)[j]]))
				v_i_S[iPos++] = (*p_v_i_Edges)[j];
		}

        // std::cout << iPos << std::endl;
        // for(int x = 0; x < v_i_S.size(); x++)
        //   std::cout << v_i_S[x] << ", ";
        // std::cout << std::endl << "------------" << std::endl;
        
        
		icc = 0;
        max_clique_data_inter.push_back(iCandidateVertex);
		
		while(iPos > 0)
		{
			int imdv1 = -1, imd1 = -1;

			icc++;

			// generate a random number x from 0 to iPos -1
			// aasign that imdv = x and imd = d(x)
			//imdv = v_i_S[rand() % iPos];
			imdv = v_i_S[iPos-1];
			imd = (*p_v_i_Vertices)[imdv + 1] - (*p_v_i_Vertices)[imdv];

			iPos1 = 0;

			for(int j = 0; j < iPos; j++)
			{
				iLoopCount = (*p_v_i_Vertices)[imdv + 1];
				for(int k = (*p_v_i_Vertices)[imdv]; k < iLoopCount; k++)
				{
					// Pruning 5
					if(v_i_S[j] == (*p_v_i_Edges)[k] && maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[k] + 1] - (*p_v_i_Vertices)[(*p_v_i_Edges)[k]]))
					{
						v_i_S1[iPos1++] = v_i_S[j]; // calculate the max degree vertex here
						
						break;
					}
				}
			}


            // for(int x = 0; x < v_i_S.size(); x++)
            //   std::cout << v_i_S[x] << ", ";
            // std::cout << std::endl;
            // std::cout << "v_i_S1: ";
            // for(int x = 0; x < iPos1; x++)
            //   std::cout << v_i_S1[x] << ", ";
            // std::cout << std::endl;
            // std::cout << imdv << ", " << imd << std::endl;
            // std::cout << iPos << ", " << iPos1 << std::endl;
            
			for(int j = 0; j < iPos1; j++)
				v_i_S[j] = v_i_S1[j];

            if (iPos1 != 0) {
              //              max_clique_data_inter.push_back(iPos);
              max_clique_data_inter.push_back(imdv);;
            }

            // for(int x=0; x < max_clique_data_inter.size();x++){
            //   std::cout << max_clique_data_inter[x] << ", ";
            // }
            // std::cout << std::endl;

            
			iPos = iPos1;

			imdv = imdv1;
			imd = imd1;			
		}
	
		if(maxClq < icc){
			maxClq = icc;
            max_clique_data = max_clique_data_inter;
        }
	}

	return maxClq;
}

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v)
{
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}

}