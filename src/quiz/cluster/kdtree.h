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
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		helpinsert(root,point,0,id); 
	}
	void helpinsert(Node* &root, std::vector<float> point, int depth, int id){
     	if(root==NULL){
         	root = new Node(point,id);
          	return;
        }
      	else{
         	int cd = depth%2;
          	if(point[cd]>root->point[cd]){
             	helpinsert(root->right,point,depth+1,id); 
            }
          	else{
             	helpinsert(root->left,point,depth+1,id); 
            }
        }
    }
	// return a list of point ids in the tree that are within distance of target
  	void helpsearch(Node* root, std::vector<float> target, float distanceTol,int depth,std::vector<int> &ids){
     if(root==NULL){
      	return; 
     }
     if(root->point[0]>=target[0]-distanceTol && root->point[0]<=target[0]+distanceTol && root->point[1]>=target[1]-distanceTol && root->point[1]<=target[1]+distanceTol){
       float deltaX = root->point[0]-target[0];
       float deltaY = root->point[1]-target[1];
       float dist = sqrt(deltaX*deltaX+deltaY*deltaY);
       if(dist<=distanceTol){
        	ids.push_back(root->id); 
       }

     }
     if(target[depth%2]-distanceTol < root->point[depth%2]){
      	helpsearch(root->left,target,distanceTol,depth+1,ids); 
     }
     if(target[depth%2]+distanceTol > root->point[depth%2]){
       helpsearch(root->right,target,distanceTol,depth+1,ids); 
     }
      
    }
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      	helpsearch(root,target,distanceTol,0,ids);
		return ids;
	}
	

};




