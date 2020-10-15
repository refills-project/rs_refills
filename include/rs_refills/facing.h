#pragma once

struct Facing {

  struct obj_dims {
       double w, h, d;
  } productDims;

  double width;
  double height;
  enum class ShelfType {HANGING, STANDING};
  ShelfType shelfType;

  tf::Stamped<tf::Pose> leftSeparator, rightSeparator, topRightCorner;
  std::string gtin;
  std::string dan;
  std::string productId;
  std::string facingId;

  cv::Rect rect, rect_hires_;
  cv::Mat mask;
};
