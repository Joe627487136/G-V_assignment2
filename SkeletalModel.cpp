#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
	ifstream infile(filename);
	std::string line;
	while(std::getline(infile,line)){
		std::istringstream iss(line);
		float x, y, z; int parent_index;
		iss >> x >> y >> z >> parent_index;
		Matrix4f translation = Matrix4f::translation(x,y,z);
		Joint * joint = new Joint;
		joint -> transform = translation;
		m_joints.push_back(joint);

		if (parent_index == -1){
			m_rootJoint = joint;
		}else{
			Joint * parent = m_joints[parent_index];
			parent -> children.push_back(joint);
		}
	}
	infile.close();
}

void recursive_draw_ChildrenJoints(Joint * joint, MatrixStack * m_matrixStack){
	m_matrixStack->push(joint->transform);
	glLoadMatrixf(m_matrixStack->top());
	glutSolidSphere(0.025f, 12, 12);

	vector<Joint*> children = joint->children;
	for (unsigned i=0; i<children.size(); i++){
		recursive_draw_ChildrenJoints(children[i],m_matrixStack);
	}
	m_matrixStack->pop();
	glLoadMatrixf(m_matrixStack->top());
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
	recursive_draw_ChildrenJoints(m_rootJoint,&m_matrixStack);
}

void recursive_draw_Skeleton(Joint * joint, MatrixStack * m_matrixStack){
	Matrix4f translation = Matrix4f::translation(0,0,0.5);
	m_matrixStack->push(joint->transform);
	vector<Joint*> children = joint->children;
	for (unsigned i=0; i<children.size(); i++){
		Joint * child = joint->children[i];
		Vector3f z = child->transform.getCol(3).xyz();
		Matrix4f scale = Matrix4f::scaling(0.025, 0.025, z.abs());
		Vector3f y = Vector3f::cross(z, Vector3f(0,0,1)).normalized();
		Vector3f x = Vector3f::cross(y, z).normalized();
		Matrix4f rotation = Matrix4f(Vector4f(x,0),Vector4f(y,0),Vector4f(z.normalized(),0),Vector4f(0,0,0,1));
		m_matrixStack->push(rotation);
		m_matrixStack->push(scale);
		m_matrixStack->push(translation);
		glLoadMatrixf(m_matrixStack->top());
		glutSolidCube(1.0f);
		m_matrixStack->pop();
		m_matrixStack->pop();
		m_matrixStack->pop();
		recursive_draw_Skeleton(child,m_matrixStack);
	}
	m_matrixStack->pop();
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	recursive_draw_Skeleton(m_rootJoint,&m_matrixStack);
}


void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	Joint * joint = m_joints[jointIndex];
	Matrix3f r_transform = Matrix3f::rotateX(rX)*Matrix3f::rotateY(rY)*Matrix3f::rotateZ(rZ);
	joint->transform.setSubmatrix3x3(0,0,r_transform);
}

void recursive_computeBindWorldToJointTransforms(Joint* joint,MatrixStack* m_matrixStack){
	m_matrixStack->push(joint->transform);
	joint->bindWorldToJointTransform = m_matrixStack->top().inverse();
	vector<Joint*> children = joint->children;
	for (unsigned i=0; i<children.size();i++){
		recursive_computeBindWorldToJointTransforms(children[i],m_matrixStack);
	}
	m_matrixStack->pop();
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
	m_matrixStack.clear();
	recursive_computeBindWorldToJointTransforms(m_rootJoint,&m_matrixStack);
}

void recursive_updateCurrentJointToWorldTransforms(Joint* joint,MatrixStack* m_matrixStack){
	m_matrixStack->push(joint->transform);
	joint->currentJointToWorldTransform = m_matrixStack->top();
	vector<Joint*> children = joint->children;
	for (unsigned i=0; i<children.size();i++){
		recursive_updateCurrentJointToWorldTransforms(children[i],m_matrixStack);
	}
	m_matrixStack->pop();
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
	m_matrixStack.clear();
	recursive_updateCurrentJointToWorldTransforms(m_rootJoint,&m_matrixStack);
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.

	for (unsigned i=0; i<m_mesh.bindVertices.size();i++){
		Vector3f bind_vertice = m_mesh.bindVertices[i];
		vector<float> wght = m_mesh.attachments[i];
		Vector3f sum = Vector3f(0.f,0.f,0.f);
		for (unsigned j=0; j<m_joints.size()-1; j++){
			Joint* joint = m_joints[j+1];
			Vector3f tbp = (joint->currentJointToWorldTransform * (joint->bindWorldToJointTransform*Vector4f(bind_vertice,1))).xyz();
			sum+=wght[j]*tbp;
		}
		m_mesh.currentVertices[i]=sum;
	}
}

