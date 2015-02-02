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
	
	string collision_method;

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

	void drawCollisionBoxes(GLuint spID);
	void drawBSpheres(GLuint spID);
	void drawAABBs(GLuint spID);

	void checkCollisionsSphere();
	void checkCollisionsAABB();

	struct SPpoint
	{
		int id;
		float point;
		bool start;

		SPpoint *prev;
		SPpoint *next;

		SPpoint()
		{
			id = NULL;
			point = NULL;
			start = false;
			prev = NULL;
			next = NULL;
		}

		SPpoint(int _id, float _point, bool _start)
		{
			id = _id;
			point = _point;
			start = _start;
			prev = NULL;
			next = NULL;
		}

		bool operator > (SPpoint &other)
		{
			return point > other.point;
		}

		bool operator< (SPpoint &other)
		{
			return point < other.point;
		}	
	};

	struct SPpointList
	{
		SPpoint *head;
		SPpoint *tail;

		SPpointList()
		{
			head = NULL;
			tail = NULL;
		}

		void addToEnd(SPpoint &p)
		{
			if(head == NULL)
			{
				head = &p;
				tail = &p;
				p.prev = NULL;
				p.next = NULL;
			}
			else
			{
				tail->next = &p;
				p.prev = tail;
				p.next = NULL;
				tail = &p;
			}
		}

		void addToStart(SPpoint &p)
		{
			if(head == NULL)
			{
				head = &p;
				tail = &p;
				p.prev = NULL;
				p.next = NULL;
			}
			else
			{
				head->prev = &p;
				p.prev = NULL;
				p.next = head;
				head = &p;
			}
		}
	};
	
	std::vector<SPpoint> unsortedx;
	std::vector<SPpoint> unsortedy;
	std::vector<SPpoint> unsortedz;

	SPpointList xlist;
	SPpointList ylist;
	SPpointList zlist;


	void createUSlists();
	void updateUSlists();
	void createSPlists();
	void updateSPlists();
	void insertionSort(SPpointList &list);

	void checkCollisionsAABBSweepPrune();


	struct CollisionPair
	{
		RigidBody *body1, *body2;

		CollisionPair()
		{
			body1 = body2 = NULL;
		}
		CollisionPair(RigidBody &_body1, RigidBody &_body2)
		{
			body1 = &_body1;
			body2 = &_body2;
		}

	};

	std::vector<CollisionPair> collision_pairs;

	void clearCollisions();
	void updateCollisions();

};
