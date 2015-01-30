#pragma once

#define _CRT_SECURE_NO_DEPRECATE
//Some Windows Headers (For Time, IO, etc.)
#include <windows.h>
#include <mmsystem.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>


// Assimp includes

#include <assimp/cimport.h> // C importer
#include <assimp/scene.h> // collects data
#include <assimp/postprocess.h> // various extra operations
#include <stdio.h>
#include <math.h>
#include <vector> // STL dynamic memory.
#include <string>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4, glm::ivec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp> // glm::value_ptr

#include "RigidBody.h"
#include "AntTweakBar.h"

using namespace std;

class RigidBodyManager
{
public:

	RigidBodyManager();

	std::vector<RigidBody> bodies;
	int num;
	float drag_coeff;

	void addRigidBody(RigidBody body);
	void addRigidBody(Mesh mesh);
	void addRigidBody(const char* filename);


	void addTBar(TwBar *bar);

	void load_mesh();
	void draw(GLuint spID);



	void update(float dt);

	void reset();

	bool checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, int &hit_target, glm::vec3 &hit_pos);

	void drawBSpheres(GLuint spID);

	void checkCollisions();

};
