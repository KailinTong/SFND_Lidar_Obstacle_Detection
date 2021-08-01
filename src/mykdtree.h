/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef PLAYBACK_KDTREE_H
#define PLAYBACK_KDTREE_H

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id, const uint dimension)
	{
		if (*node == nullptr)
			*node = new Node(point, id);
		else
		{
			uint cd = depth % dimension;

			if(point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id, dimension);
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id, dimension);
			}
			
		}
		
	}

	void insert(std::vector<float> point, int id, const uint dimension)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id, dimension);
	}



	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids, const uint dimension)
	{
		if(node != nullptr)
		{

			if((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)))
			{
			    if(dimension == 3)
                {
                    if(node->point[2]>=(target[2]-distanceTol)&&node->point[2]<=(target[2]+distanceTol)){
                        float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
                        if(distance <= distanceTol)
                            ids.emplace_back(node->id);
                    }
                }
			    else{
                    float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
                    if(distance <= distanceTol)
                        ids.emplace_back(node->id);
			    }

			}

			if( target[depth%dimension] - node->point[depth%dimension] <  distanceTol)
				searchHelper(target, node->left, depth+1, distanceTol, ids, dimension);
			if( target[depth%dimension] - node->point[depth%dimension]  > -distanceTol )
				searchHelper(target, node->right, depth+1, distanceTol, ids, dimension);
		}
	}


 
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol, const uint dimension)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids, dimension);


		return ids;
	}

};


#endif //PLAYBACK_KDTREE_H






