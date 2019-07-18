
#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

struct SceneObject {
  double first_appearance_time;
  int frame_till_last_appearance;
  int last_appearance_time; 

  /*
  POSITION VARIABLES
  */
  double x_left;
  double y_left;
  double x_right;
  double y_right;
  double last_angle;
  double last_depth;

  /*
  Miscellaneous Flags
  */
  bool tracking_flag;
  int object_id;
  std::string object_label;
  double reliability_score;
};

#endif
