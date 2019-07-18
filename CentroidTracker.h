
#ifndef CENTROIDTRACKER_H
#define CENTROIDTRACKER_H

#include <map>
#include "SceneObject.h"
#include <vector>
#include <string>

class CentroidTracker
{
  private:
    int maxDisappeared_;
    int nextObjectID_;
    int minAppeared_;
    std::map <int, struct SceneObject*> object_list_;
    // std::map <int, struct SceneObject*> disappeared_list_;
    void deregister_object_ (int objectId);
    int register_object_ (std::vector <float>& object, std::string label);
    std::vector<std::vector<float>> compute_dist(std::vector<std::vector<float>> &object);
    float compute_euc_dist(struct SceneObject* curr_objects, std::vector<float> &new_objects);

    // std::map<int, std::pair<double, double>> disappeared_;
    // std::map<int, std::pair<double, double>> objects_;

  public:
    CentroidTracker();
    std::map<int, int>  update(std::vector<std::vector<float>> &object);
    void print_test();
    // std::map<int, std::pair<double, double>> get_objects_list();
};

#endif
