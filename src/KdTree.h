#ifndef KDTREE_H_
#define KDTREE_H_

#include<vector>
#include<stdlib.h>
#include<math.h>

struct Node{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;
    Node(std::vector<float> point,int id):point(point),id(id),left(nullptr),right(nullptr){}
};

struct KdTree{
    Node* root;
    KdTree(){ root = nullptr;}

    void helpinsert(Node* &root,int depth,std::vector<float> point, int id){
        if(root==nullptr){
            root = new Node(point,id);
            return;
        }
        if(point[depth%3]>root->point[depth%3]){
            helpinsert(root->right,depth+1,point,id);
        }
        else{
            helpinsert(root->left,depth+1,point,id);
        }
    }

    void insert(std::vector<float> point, int id){
        helpinsert(root,0,point,id);
    }

    void helpsearch(std::vector<float> target,Node* root,int depth,float distanceTol,std::vector<int> &closest ){
        if(root==nullptr){
            return;
        }
        if(abs(target[0]-root->point[0])<=distanceTol&&abs(target[1]-root->point[1])<=distanceTol&&abs(target[2]-root->point[2])<=distanceTol)
        {
            float deltaX = target[0]-root->point[0];
            float deltaY = target[1]-root->point[1];
            float deltaZ = target[2]-root->point[2];
            if(sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ)<=distanceTol){
                closest.push_back(root->id);
            }
        }
        if(target[depth%3]-distanceTol<root->point[depth%3]){
            helpsearch(target,root->left,depth+1,distanceTol,closest);
        }
        if(target[depth%3]+distanceTol>root->point[depth%3]){
            helpsearch(target,root->right,depth+1,distanceTol,closest);
        }

    }
    std::vector<int> search(std::vector<float> target,float distanceTol){
        std::vector<int> closest;
        helpsearch(target,root,0,distanceTol,closest);
        return closest;
    }
    
};



#endif
