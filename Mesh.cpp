	#include "Mesh.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.

	// make a copy of the bind vertices as the current vertices
	ifstream infile(filename);
	std::string line;
	while(std::getline(infile,line)){
		std::istringstream iss(line);
		float x, y, z; string header;
		iss >> header >> x >> y >> z;
		if (header=="v"){
			bindVertices.push_back(Vector3f(x,y,z));
		}else if (header=="f"){
			Tuple3u face;
			face[0]=x;
			face[1]=y;
			face[2]=z;
			faces.push_back(face);
		}else{
			std::cout << "Header mismatched!\n";
		}
	}
	infile.close();
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
	for (unsigned i=0; i<faces.size(); i++){
		glBegin(GL_TRIANGLES);
		Vector3f cv0,cv1,cv2,n;
		cv0 = currentVertices[faces[i][0]-1];
		cv1 = currentVertices[faces[i][1]-1];
		cv2 = currentVertices[faces[i][2]-1];
		n = (Vector3f::cross(cv1-cv0,cv2-cv0)).normalized();
		glNormal3d(n[0],n[1],n[2]); glVertex3d(cv0[0],cv0[1],cv0[2]);
		glNormal3d(n[0],n[1],n[2]); glVertex3d(cv1[0],cv1[1],cv1[2]);		
		glNormal3d(n[0],n[1],n[2]); glVertex3d(cv2[0],cv2[1],cv2[2]);
		glEnd();
	}
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	ifstream infile(filename);
	std::string line;
	float wght;
	while(std::getline(infile,line)){
		std::istringstream iss(line);
		std::vector<float> wght_array;
		wght_array.push_back(0.f);
		for (unsigned i=0; i<=16; i++){
			iss >> wght;
			wght_array.push_back(wght);
		}
		attachments.push_back(wght_array);
	}
	infile.close();
}
