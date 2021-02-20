/* \author Aaron Brown */
// Quiz on implementing kd tree

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
	Node* root; //Pointer to the first node

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root,0,point,id); 

	}

	// A helper that help insert the node and carry out different checking
	// Insert based on x or y depending on the dimension given
	void insertHelper(Node *& node, int depth, std::vector<float> point, int id)
	{
		// Check if the root node or if child node has been reached in recursion (null)
		if (node == NULL){
			//Create and assign to the current node next pointer
			node = new Node(point, id);
		}

		//Otherwise just do some different recursion
		else
		{
			//Generalise to a remainder problem and can easily be extends to more d in the future
			int depthRemainder = depth%2;
			if (point.at(depthRemainder) < node->point.at(depthRemainder))
				//To the left it is
				insertHelper(node->left,depth+1,point, id);
			else
				insertHelper(node->right,depth+1,point,id);

		}		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




