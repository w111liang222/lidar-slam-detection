/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  an I/O library for reading a graph    			 	   */                                                   
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

#ifndef _graphIO_
#define _graphIO_

#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <float.h>
#include <string.h>

#define LINE_LENGTH 256

using namespace std;

namespace FMC {
typedef std::vector<int> IntVector;

class CGraphIO
{
public:

	CGraphIO(){ }
	virtual ~CGraphIO();

	bool readGraph(string s_InputFile, float connStrength = -DBL_MAX);
	string getFileExtension(string fileName);
	bool ReadMatrixMarketAdjacencyGraph(string s_InputFile, float connStrength = -DBL_MAX);
	bool ReadMeTiSAdjacencyGraph(string s_InputFile);
	void CalculateVertexDegrees();

	int GetVertexCount(){ return m_vi_Vertices.size() - 1; }
	int GetEdgeCount(){ return m_vi_Edges.size()/2; }
	int GetMaximumVertexDegree(){ return m_i_MaximumVertexDegree; }
	int GetMinimumVertexDegree(){ return m_i_MinimumVertexDegree; }
	double GetAverageVertexDegree(){ return m_d_AverageVertexDegree; }
	string GetInputFile(){ return m_s_InputFile; }
	
	vector <int>* GetVerticesPtr(){ return &m_vi_Vertices; }
	vector <int>* GetEdgesPtr(){ return &m_vi_Edges; }

public:
	int m_i_MaximumVertexDegree;
	int m_i_MinimumVertexDegree;
	double m_d_AverageVertexDegree;

	string 	m_s_InputFile;

	vector<int> 	m_vi_Vertices;
	vector<int>   m_vi_OrderedVertices;
	vector<int> 	m_vi_Edges;
	vector<double> 	m_vd_Values;
};
}
#endif
