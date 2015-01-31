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

	createUSlists();
}

void RigidBodyManager :: createUSlists()
{
	int size = 0;
	for(int i=0; i<num; i++)
	{

		SPpoint p = SPpoint(bodies[i].id, bodies[i].aabb.vmin.x, true);
		unsortedx.push_back(p);

		unsortedx.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.x, false));

		unsortedy.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmin.y, true));

		unsortedy.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.y, false));

		unsortedz.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmin.z, true));

		unsortedz.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.z, false));
	}

	

	createSPlists();
}

void RigidBodyManager :: updateUSlists()
{
	for(int i=0; i<num*2; i++)
	{
		int id = unsortedx[i].id;
		bool start = unsortedx[i].start;
		if(start)
			unsortedx[i].point = bodies[id-1].aabb.vmin.x;
		else
			unsortedx[i].point = bodies[id-1].aabb.vmax.x;

		id = unsortedy[i].id;
		start = unsortedy[i].start;
		if(start)
			unsortedy[i].point = bodies[id-1].aabb.vmin.y;
		else
			unsortedy[i].point = bodies[id-1].aabb.vmax.y;

		id = unsortedz[i].id;
		start = unsortedz[i].start;
		if(start)
			unsortedz[i].point = bodies[id-1].aabb.vmin.z;
		else
			unsortedz[i].point = bodies[id-1].aabb.vmax.z;

	}
}

void RigidBodyManager :: createSPlists()
{
	for(int i=0; i<num*2; i++)
	{
		xlist.addToEnd(unsortedx[i]);
		ylist.addToEnd(unsortedy[i]);
		zlist.addToEnd(unsortedz[i]);
	}

	updateSPlists();
}

void RigidBodyManager :: updateSPlists()
{
	insertionSort(xlist);
	insertionSort(ylist);
	insertionSort(zlist);
}

void RigidBodyManager :: insertionSort(SPpointList &list)
{
	if(list.head == NULL)
		return;


	SPpointList new_list;
	SPpoint head = *list.head;

	new_list.addToStart(*list.head);

	SPpoint current_safe = *head.next;
	SPpoint *current = head.next;
	

	while(current != NULL)
	{
		SPpoint *check = new_list.tail;

		while(check != NULL)
		{
			if(current->point > check->point)
			{
				if(check->next != NULL)
				{
					current->next = check->next;
					check->next->prev = current;
				}
				else
				{
					current->next = NULL;
					new_list.tail = current;
				}

				current->prev = check;
				check->next = current;

				break;
			}
			else
				check = check->prev;
		}

		if(check == NULL)
			new_list.addToStart(*current);

		current = current_safe.next;

		if(current != NULL)
			current_safe = *current_safe.next;
	}

	list = new_list;
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

	updateUSlists();
	updateSPlists();
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

void RigidBodyManager :: drawAABBs(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].drawAABB(spID);
}


void RigidBodyManager :: checkCollisionsSphere()
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

void RigidBodyManager :: checkCollisionsAABB()
{
	for(int i=0; i<num; i++)
		bodies[i].collision = false;

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
		{
			if(i != j)
			{
				if(bodies[i].aabb.checkCollision(bodies[j].aabb))
				{
					bodies[i].collision = true;
					bodies[j].collision = true;
				}
			}
		}
}


void RigidBodyManager :: checkCollisionsAABBSweepPrune()
{
	for(int i=0; i<num; i++)
		bodies[i].collision = false;


	std::vector<std::vector<int>> pairs(num);
	for(int i=0; i<num; i++)
		pairs[i].resize(num);

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
			pairs[i][j] = 0;


	std::vector<int> active_list;

	SPpoint *current = xlist.head;
	int num_active = 0;

	while(current != NULL)
	{
		int id = current->id-1;
		if(current->start)
		{
			active_list.push_back(id);

			if(active_list.size() > 1)
			{
				for(int i=0; i<active_list.size()-1; i++)
				{
					pairs[id][active_list[i]]++;
					pairs[active_list[i]][id]++;
				}
			}
		}
		else
		{
			for(int i=0; i<active_list.size(); i++)
				if(active_list[i] == id)
					active_list.erase(active_list.begin()+i);
		}


		if(current == xlist.tail)
			current = ylist.head;
		else if(current == ylist.tail)
			current = zlist.head;
		else
			current = current->next;
	}


	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
			if(pairs[i][j] >= 3)
			{
				bodies[i].collision = true;
				bodies[j].collision = true;
			}
}