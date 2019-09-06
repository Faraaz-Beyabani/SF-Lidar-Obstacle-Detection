/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once
#include "../../render/render.h"


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
	: root(NULL)
	{}
  
  	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
    {
     	if(*node == nullptr)
        {
          	*node = new Node(point, id);
        }
      	else
        {
          	int dim = depth % 2;
          
          	if(point[dim] < (*node)->point[dim])
            {
             	insertHelper(&(*node)->left, depth + 1, point, id);
            }
            else
            {
							insertHelper(&(*node)->right, depth + 1, point, id);
            }
        }
    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}
  
	std::vector<int> searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol)
	{
			std::vector<int> ids;
			float leftBound = target[0] - distanceTol;
			float rightBound = target[0] + distanceTol;
			float topBound = target[1] + distanceTol;
			float bottomBound = target[1] - distanceTol;
		
			if(node != nullptr)
			{
				if((node->point[0] >= leftBound && node->point[0] <= rightBound) && (node->point[1] >= bottomBound && node->point[1] <= topBound))
					{
							float distance = sqrt((pow(node->point[0] - target[0], 2)) + (pow(node->point[1] - target[1], 2)));
							if(distance <= distanceTol)
							{
								ids.push_back(node->id); 
							}
					}
				
					if(target[depth%2] - distanceTol < node->point[depth%2])
					{
						std::vector<int> leftIDs = searchHelper(target, node->left, depth + 1, distanceTol);
							ids.insert(ids.end(), leftIDs.begin(), leftIDs.end());
					}
					if(target[depth%2] + distanceTol > node->point[depth%2])
					{
						std::vector<int> rightIDs = searchHelper(target, node->right, depth + 1, distanceTol);
							ids.insert(ids.end(), rightIDs.begin(), rightIDs.end());
					}
			}
		
			return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      	ids = searchHelper(target, root, 0, distanceTol);
		return ids;
	}
	

};




