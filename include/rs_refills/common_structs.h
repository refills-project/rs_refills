#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <tf/tf.h>
#include <opencv2/opencv.hpp>

namespace rs
{

struct Facing
{

  std::string gtin;
  std::string dan;
  std::string productId;
  std::string facingId;

  double width;
  double height;

  tf::Stamped<tf::Pose> leftSeparator, rightSeparator;

  cv::Rect imageLoc;
  cv::Mat mask;

  struct obj_dims
  {
    double w, h, d;
  } productDims;

  enum class ShelfType {HANGING=0, STANDING};
  ShelfType shelfType;

};

const static std::vector<std::string> shelf_type_strings = {"HANGING", "STANDING"};

std::string getTextFromEnum(int enum_val)
{
  return shelf_type_strings[enum_val];
}

int enumFromString(std::string value)
{
  std::vector<std::string>::const_iterator it = std::find(shelf_type_strings.begin(), shelf_type_strings.end(), value);
  if(it != shelf_type_strings.end())
    return static_cast<int>(it - shelf_type_strings.begin());
  else
    return 0;
}
}

#endif // COMMON_STRUCTS_H
