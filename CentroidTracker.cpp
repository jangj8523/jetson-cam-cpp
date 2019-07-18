

#include "CentroidTracker.h"
#include "SceneObject.h"
#include <math.h>
#include <iostream>
#include <functional>
#include <set>
#include <algorithm>
#include <map>

// const int default_maxDisappeared = 40;
const int default_startObjectId = 0;


struct comp{
	template<typename T>
	bool operator()(const T& l, const T& r) const
	{
		if (l.second != r.second)
			return l.second < r.second;

		return l.first < r.first;
	}
};

CentroidTracker::CentroidTracker() :
  // maxDisappeared_(default_maxDisappeared),
  nextObjectID_(default_startObjectId)
{}


void CentroidTracker::print_test() {
  printf("Printing for test\n");
  printf ("ALLOCATE memory\n");
  std::vector <float> sample {1.0, 1.0, 11, 2, 24};
  std::string name = "work";
  register_object_(sample, name);
  printf("NOW DEALLOCATE MEMORY\n");
  deregister_object_(0);

}

int CentroidTracker::register_object_(std::vector<float>& object, std::string label) {
  struct SceneObject *newObject = (struct SceneObject *) malloc (sizeof(struct SceneObject));
  newObject->x_left = object[0];
  newObject->x_right = object[1];
  newObject->y_left = object[2];
  newObject->y_right = object[3];
	newObject->last_depth = object[4];
  newObject->first_appearance_time = object[5];
  newObject->frame_till_last_appearance = 0;
  newObject->object_label = label;
  newObject->object_id = nextObjectID_;
  object_list_[nextObjectID_] = newObject;
	return nextObjectID_++;
;
}

void CentroidTracker::deregister_object_(int object_id) {
  struct SceneObject * entry = object_list_[object_id];
  object_list_.erase (object_id);
  free(entry);
}


float CentroidTracker::compute_euc_dist(struct SceneObject* curr_objects, std::vector<float> &new_objects) {
    float curr_mid_x = (curr_objects->x_left + curr_objects->x_right)/2;
    float curr_mid_y = (curr_objects->y_left + curr_objects->y_right)/2;
		float curr_depth = curr_objects->last_depth;

    float new_mid_x = (new_objects[0] + new_objects[2])/2;
    float new_mid_y = (new_objects[1] + new_objects[3])/2;
		float new_depth = new_objects[4];

    return sqrt(pow(curr_mid_x - new_mid_x, 2) + pow(curr_mid_y - new_mid_y, 2) + pow(curr_depth - new_depth, 2));

}

// std::vector<std::vector<float>> CentroidTracker::compute_dist (std::vector<std::vector<float>> &object) {
//   int curr_length = object_list_.size();
//   int new_length = object.size();
//
//   for (int i = 0; i < curr_length; i++) {
//     std::vector<float> curr_wrt_new;
//     for (int j = 0; j < new_length; j++) {
//         float distance = compute_euc_dist(object_list_[i] , object[j]);
//         curr_wrt_new
//     }
//   }
// }

std::map<int, int> CentroidTracker::update(std::vector<std::vector<float>> &object) {
  std::map<int, struct SceneObject*>::iterator obj_it;
  std::vector<std::vector<float>>::iterator vec_vec_it;
	std::map<int, int> object_id_with_new;
  // std::map<int, struct SceneObject*>::iterator disappear_it;

  if (object.empty()) {
    for ( obj_it = object_list_.begin(); obj_it != object_list_.end(); obj_it++ ) {
      int key = obj_it->first;
      struct SceneObject* obj = obj_it->second;
      obj->frame_till_last_appearance++;

      if (obj->frame_till_last_appearance > maxDisappeared_) { deregister_object_(obj_it->first); }
    }
    return object_id_with_new;
  }

  printf("here we are!\n");
  /*
  TODO: Replace the hardcode pedestrian
  When the Object List is empty and not empty;
  */

	std::set<int> unused_curr;
	std::set<int> unused_new;



  if (object_list_.empty()) {
    for (int i = 0; i < object.size(); i++) {
      int id = register_object_(object[i], "Pedestrian");
			object_id_with_new[i] = id;
      printf("registered\n");
    }
  } else {
      printf("NOT EMPTY\n");
      int curr_length = object_list_.size();
      int new_length = object.size();
      std::map<float, std::pair<int, int>, std::greater<int>> euc_dist;

      int curr_index = 0;
      for ( obj_it = object_list_.begin(); obj_it != object_list_.end(); obj_it++) {
        for (int j = 0; j < new_length; j++) {
            float distance = compute_euc_dist(obj_it->second, object[j]);
            euc_dist[distance] = std::make_pair(obj_it->first, j);
						unused_curr.insert(obj_it->first);
						unused_new.insert(j);
        }
        curr_index += 1;
      }
      /*
      SORTING THE MAP euc_dist
      */
      std::set<std::pair<float, std::pair<int,int>>, comp> set(euc_dist.begin(), euc_dist.end());
			std::set<int> used_current_id;
			std::set<int> used_new_id;

			for (auto const &pair: set) {
    	   std::cout << "WOW DEGUBE {" << pair.first << "," << pair.second.first << ", " << pair.second.second << '}' << '\n';
				 int curr_object_id = pair.second.first;
				 int new_object_index= pair.second.second;
				 bool curr_isUsed = (std::find(used_current_id.begin(), used_current_id.end(), curr_object_id) != used_current_id.end());
				 bool new_isUsed = (std::find(used_new_id.begin(), used_new_id.end(), new_object_index) != used_new_id.end());
				 /*
				 If the id of the current object or the index of the new object is used,
				 then just skip.
				 */
				 if(curr_isUsed || new_isUsed) {
					 continue;
				 }

				 /*
				 	update the object_list_ with new data of the object
					and add the object to the used list
				 */

				 object_list_[curr_object_id]->x_left = object[new_object_index][0];
				 object_list_[curr_object_id]->y_left = object[new_object_index][1];
				 object_list_[curr_object_id]->x_right = object[new_object_index][2];
				 object_list_[curr_object_id]->y_right = object[new_object_index][3];
				 object_list_[curr_object_id]->last_depth = object[new_object_index][4];
				 object_list_[curr_object_id]->last_appearance_time = object[new_object_index][5];
				 object_list_[curr_object_id]->frame_till_last_appearance = 0;

				 /*
	 				Remove the new object and the current object id from the
					unused list
	 			*/
				 unused_new.erase(new_object_index);
				 unused_curr.erase(curr_object_id);
				 object_id_with_new[new_object_index] = curr_object_id;

    	}


			/*
			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared
			*/
			if (object_list_.size() >= object.size()) {
				printf("SOMETHING DISAPPEARED\n");
				for (int unused_id : unused_curr) {
					object_list_[unused_id]->frame_till_last_appearance++;
					if (object_list_[unused_id]->frame_till_last_appearance > maxDisappeared_)
						deregister_object_(unused_id);
				}
			} else {
				/*
				# otherwise, if the number of input centroids is greater
				# than the number of existing object centroids we need to
				# register each new input centroid as a trackable object
				*/
				printf("NEW REGISTRATION\n");

				for (int unused_index : unused_new) {
					int i = register_object_(object[unused_index], "pedestrian");
					object_id_with_new[unused_index] = i;
				}
			}
	}
	printf ("OUTSIDE REUT\n");
	return object_id_with_new;
}

//
// std::map<int, std::pair<double, double>> CentroidTracker::get_objects_list() {
//   return objects_;
// }
