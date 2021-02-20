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
		searchHelper(root,0,target,distanceTol,ids);

		return ids;
	}

	// A search helper to iterate and save the answers
	void searchHelper(Node *& node, int depth, std::vector<float> target, float distanceTol, std::vector<int> & ids){
		//If the node reached is NULL, we have search all over the tree, time to go up
		if (node == NULL){
			cout << "Reached a leaf node, time to recurse up ^" << endl;
		}

		else{
			// If we haven't reach the end of the tree:
			// First Attempt methodology: 
			// L2 norm is calculated everytime a node is visited => expensive due to square root
			// If the target node is smaller than the slice of current node, check only one side of it =>
			// Even we choose 2 if phase for iterate left and right, only 1 branch will be visited (in short this is okay for finding where to insert)

			// Our goal for this part is however to find all the points that is nearest to our target within a boundary region.
			// Boundary is a box and if let say both extreme side of the box belongs to 2 different splitting region, we need to evaluate both branch of the tree.
			// Use box checking as a cheaper algorithm than quickly go caluclate L2 Norm as this would be cheaper
			
			//Attemp 2:
			//Firstly, decide if it is worth it to do an L2 norm calculation
			//Check if the coordinate of node visited is within the box region
			if ((node->point[0] >= target[0]- distanceTol || node->point[0] <= target[0] + distanceTol) && (node->point[1] >= target[1]- distanceTol || node->point[1] <= target[1] + distanceTol)){
				// Now justify to go into this expensive operation (making sure it is inside a circle within the square and not on the square edge only)
				cout << "Visiting node: " << node->id << endl;
				float sumSquaredDiff = 0;
				for (int i = 0; i < target.size(); ++i){
					sumSquaredDiff += pow((target[i]-node->point[i]),2);
				}

				float euclidDist = sqrt(sumSquaredDiff);

				if (euclidDist <= distanceTol){
					cout << "Inserting as distance "<< euclidDist << " is within tolerance "<< distanceTol << endl;
					ids.push_back(node->id);
				}
			}
			
			// Then regardless of belongs to the circle, square or not at all, narrow down or visit both the search space to the sides of the x or y-split
			//which contain our box.

			int depthRemainder = depth % 2;
			if (target.at(depthRemainder) - distanceTol < node->point.at(depthRemainder))
				//To the left it is
				searchHelper(node->left,depth+1,target, distanceTol, ids);
			if (target.at(depthRemainder) + distanceTol > node->point.at(depthRemainder))
				searchHelper(node->right,depth+1,target,distanceTol, ids);

			//Done then will just wrap up to root and terminated

		}
	}
	

};




