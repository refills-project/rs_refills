#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/filter.h>

#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;


class ShelfDetector : public DrawingAnnotator
{


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  pcl::PointCloud<pcl::Boundary> boundaries_;

public:

  ShelfDetector():DrawingAnnotator(__func__)
  {
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    cas.get(VIEW_CLOUD,*cloud_);
    cas.get(VIEW_NORMALS,*normals_);

    boundaries_.points.clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_no_nan (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointIndicesPtr indices(new pcl::PointIndices());
    pcl::removeNaNFromPointCloud(*cloud_,*cloud_no_nan,indices->indices);

    pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (cloud_);
    est.setInputNormals (normals_);
    est.setIndices(indices);
    est.setRadiusSearch (0.02);   // 2cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
//    est.compute (boundaries_);


    outInfo("Cloud size: " << cloud_->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "cloud";

    double pointSize=1.0;

    for ( size_t i=0; i<boundaries_.points.size();++i )
    {
        outInfo(boundaries_.points.at(i));
//        cloud_->points[index].rgba =0x00ff00;
    }
    if(firstRun)
    {
      visualizer.addPointCloud(cloud_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ShelfDetector)
