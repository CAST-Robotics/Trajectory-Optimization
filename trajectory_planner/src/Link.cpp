#include "headers/Link.h"

 _Link::_Link(short int ID, string name,short int parentID, short int numChilds, short int* childID, double length){
    this->ID_= ID;
    this->name_ = name;
    this->parentID_ = parentID;
    childID_ = new short int[numChilds];
    for (int i = 0; i < numChilds ; ++i)
        childID_[i] = childID[i];
    this->length_ = length;
 }

 short int _Link::getID(){
    return this->ID_;
 }

double _Link::q(){
    return this->q_;
}

void _Link::q(double config){
    this->q_ = config;
}

short int* _Link::getChildID(){
    return this->childID_;
}

short int _Link::getParentID(){
    return this->parentID_;
}