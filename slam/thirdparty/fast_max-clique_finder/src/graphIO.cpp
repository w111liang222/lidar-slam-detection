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

#include "graphIO.h"

namespace FMC {
CGraphIO::~CGraphIO()
{
	m_vi_Vertices.clear();
	m_vi_OrderedVertices.clear();
	m_vi_Edges.clear();
	m_vd_Values.clear();
}

bool CGraphIO::readGraph(string s_InputFile, float connStrength)
{
	string fileExtension = getFileExtension(s_InputFile);		
	if(fileExtension == "mtx")
	{
		// matrix market format
		return ReadMatrixMarketAdjacencyGraph(s_InputFile, connStrength);
	}
	else if(fileExtension == "gr")
	{
		// gr format
		return ReadMeTiSAdjacencyGraph(s_InputFile);
	}
	else
		return false;	
}

bool CGraphIO::ReadMatrixMarketAdjacencyGraph(string s_InputFile, float connStrength)
{
	istringstream in2;
	string line="";
	map<int,vector<int> > nodeList;
	map<int,vector<double> > valueList;
	int col=0, row=0, rowIndex=0, colIndex=0;
	int entry_counter = 0, num_of_entries = 0;
	double value;

	ifstream in (s_InputFile.c_str());	
	if(!in) 
	{
		cout<<m_s_InputFile<<" not Found!"<<endl;
		return false;
	}

	char data[LINE_LENGTH];
	char banner[LINE_LENGTH];
	char mtx[LINE_LENGTH];
	char crd[LINE_LENGTH];
	char data_type[LINE_LENGTH];
	char storage_scheme[LINE_LENGTH];
	char* p;
	bool b_getValue = true;
	int num_upper_triangular = 0;

	// read the banner
	getline(in, line);
	strcpy(data, line.c_str());

	if (sscanf(data, "%s %s %s %s %s", banner, mtx, crd, data_type, storage_scheme) != 5)
	{    
		cout << "Matrix file banner is missing!!!" << endl;
		return false;
	}

	// intersted about the forth part
	for (p=data_type; *p!='\0'; *p=tolower(*p),p++);

	if (strcmp(data_type, "pattern") == 0)
		b_getValue = false;

	getline(in, line);
	while(line.size()>0&&line[0]=='%') //ignore comment line
		getline(in,line);

	in2.str(line);
	in2 >> row >> col >> num_of_entries;

	if(row!=col) 
	{
		cout<<"* WARNING: GraphInputOutput::ReadMatrixMarketAdjacencyGraph()"<<endl;
		cout<<"*\t row!=col. This is not a square matrix. Can't process."<<endl;
		return false;
	}

	while(!in.eof() && entry_counter<num_of_entries) //there should be (num_of_entries+1) lines in the input file (excluding the comments)
	{
		getline(in,line);
		entry_counter++;

		if(line!="")
		{
			in2.clear();
			in2.str(line);

			in2 >> rowIndex >> colIndex >> value;
			rowIndex--;
			colIndex--;

			if(rowIndex < 0 || rowIndex >= row)
				cout << "Something wrong rowIndex " << rowIndex << " row " << row << endl;

			if(colIndex < 0 || colIndex >= col)
				cout << "Something wrong colIndex " << colIndex << " col " << col << endl;

			if(rowIndex == colIndex)
			{
				continue;
			}

			// This is to handle directed graphs. If the edge is already present, skip. If not add.
			int exists=0;
			for(int k=0; k<nodeList[rowIndex].size(); k++) {
				if(colIndex == nodeList[rowIndex][k]) {
					exists = 1;
					break;
				}
			}

			if(exists==1) {
				num_upper_triangular++;
			} else {
				if(b_getValue)
				{
					if(value > connStrength)
					{
						nodeList[rowIndex].push_back(colIndex);
						nodeList[colIndex].push_back(rowIndex);
					}
				} 
				else 
				{
					nodeList[rowIndex].push_back(colIndex);
					nodeList[colIndex].push_back(rowIndex);
				}

				if(b_getValue && value > connStrength) 
				{
					valueList[rowIndex].push_back(value);
					valueList[colIndex].push_back(value);
				}
			}
		}
	}

	//cout << "No. of upper triangular pruned: " << num_upper_triangular << endl;
	m_vi_Vertices.push_back(m_vi_Edges.size());

	for(int i=0;i < row; i++) 
	{
		m_vi_Edges.insert(m_vi_Edges.end(),nodeList[i].begin(),nodeList[i].end());
		m_vi_Vertices.push_back(m_vi_Edges.size());
	}

	if(b_getValue) 
	{
		for(int i=0;i<row; i++) 
		{
			m_vd_Values.insert(m_vd_Values.end(),valueList[i].begin(),valueList[i].end());
		}
	}

	nodeList.clear();
	valueList.clear();
	CalculateVertexDegrees();
	return true;
}


bool CGraphIO::ReadMeTiSAdjacencyGraph(string s_InputFile)
{
	return true;
}

void CGraphIO::CalculateVertexDegrees()
{
		int i_VertexCount = m_vi_Vertices.size() - 1;

		m_i_MaximumVertexDegree =  -1;
		m_i_MinimumVertexDegree = -1;

		for(int i = 0; i < i_VertexCount; i++)
		{
			int i_VertexDegree = m_vi_Vertices[i + 1] - m_vi_Vertices[i];

			if(m_i_MaximumVertexDegree < i_VertexDegree)
			{
				m_i_MaximumVertexDegree = i_VertexDegree;
			}

			if(m_i_MinimumVertexDegree == -1)
			{
				m_i_MinimumVertexDegree = i_VertexDegree;
			}
			else if(m_i_MinimumVertexDegree > i_VertexDegree)
			{
				m_i_MinimumVertexDegree = i_VertexDegree;
			}
		}

		m_d_AverageVertexDegree = (double)m_vi_Edges.size()/i_VertexCount;

		return;
}

string CGraphIO::getFileExtension(string fileName)
{
	string::size_type result;
	string fileExtension = "";

	//1. see if the fileName is given in full path
	/*result = fileName.rfind("/", fileName.size() - 1);
	  if(result != string::npos)
	  {
	//found the path (file prefix)
	//get the path, including the last DIR_SEPARATOR
	path = fileName.substr(0,result+1);
	//remove the path from the fileName
	fileName = fileName.substr(result+1);
	}
	*/

	//2. see if the fileName has file extension. For example ".mtx"
	result = fileName.rfind('.', fileName.size() - 1);
	if(result != string::npos)
	{
		//found the fileExtension
		//get the fileExtension excluding the '.'
		fileExtension = fileName.substr(result+1);
		//remove the fileExtension from the fileName
		//fileName = fileName.substr(0,result);	
	}

	//3. get the name of the input file
	//name = fileName;

	return fileExtension;
}
}