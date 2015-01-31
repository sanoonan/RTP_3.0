#include "RigidBodyManager.h"

using namespace std;



RigidBodyManager :: RigidBodyManager()

{
	num = 0;
	drag_coeff = 0;
}


void RigidBodyManager :: addRigidBody(RigidBody body)
{
	num++;
	body.id = num;
	body.drag_coeff = drag_coeff;
	bodies.push_back(body);
}

void RigidBodyManager :: addRigidBody(Mesh mesh)
{
	RigidBody body(mesh);
	addRigidBody(body);
}


void RigidBodyManager :: addRigidBody(const char* filename)
{
	RigidBody body(filename);
	addRigidBody(body);
}


void RigidBodyManager :: addTBar(TwBar *bar)
{
	TwAddVarRO(bar, "Drag Coefficient", TW_TYPE_FLOAT, &drag_coeff, "");
	TwAddVarRO(bar, "Number of Bodies", TW_TYPE_INT8, &num, "");

}

void RigidBodyManager :: load_mesh()
{
	for(int i=0; i<num; i++)
		bodies[i].load_mesh();
}



void RigidBodyManager :: draw(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].draw(spID);
}


void RigidBodyManager :: update(float dt)
{
	for(int i=0; i<num; i++)
		bodies[i].update(dt);
}

void RigidBodyManager :: reset()
{
	for(int i=0; i<num; i++)
		bodies[i].reset();
}


bool RigidBodyManager :: checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, int &hit_target, glm::vec3 &hit_pos)
{
	float min_hit_dist;
	glm::vec3 min_hit_pos;
	int min_hit_target;
	bool hit = false;

	for(int i=0; i < num; i++)
	{
		glm::vec3 hit_position;
		if(bodies[i].checkRayHit(ray_origin, p1, p2, hit_position))
		{
			float hit_dist = glm::distance(ray_origin, hit_position);

			if(!hit)
			{
				hit = true;
				min_hit_pos = hit_position;
				min_hit_dist = hit_dist;
				min_hit_target = i;
			}
			else
			{
				if(hit_dist < min_hit_dist)
				{
					min_hit_dist = hit_dist;
					min_hit_pos = hit_position;
					min_hit_target = i;
				}
			}
		}
	}

	if(hit)
	{
		hit_target = min_hit_target;
		hit_pos = min_hit_pos;
	}

	return hit;
}

void RigidBodyManager :: drawBSpheres(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].drawBSphere(spID);
}


void RigidBodyManager :: checkCollisions()
{
	for(int i=0; i<num; i++)
		bodies[i].collision = false;

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
		{
			if(i != j)
			{
				if(bodies[i].b_sphere.checkCollision(bodies[j].b_sphere))
				{
					bodies[i].collision = true;
					bodies[j].collision = true;
				}
			}
		}
}

