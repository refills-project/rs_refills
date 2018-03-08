#include <uima/api.hpp>

#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/boundary.h>
#include <pcl/features/organized_edge_detection.h>

#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;


class ShelfDetector : public DrawingAnnotator
{

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  std::vector<pcl::PointIndices> label_indices_;
  std::vector<pcl::PointIndicesPtr> line_inliers_;

  int min_line_inliers_;
public:

  ShelfDetector(): DrawingAnnotator(__func__), min_line_inliers_(50)
  {
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_filtered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();


  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("min_line_inliers", min_line_inliers_);
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void projectPointOnPlane(pcl::PointXYZRGBA &pt, const std::vector<float> &plane_model)
  {
    assert(plane_model.size() == 4);
    cv::Point3f normal(plane_model[0], plane_model[1], plane_model[2]);
    float planeDist = plane_model[3];
    cv::Point3f point(pt.x,pt.y,pt.z);
    float pointDist = point.dot(normal);
    float t = planeDist + pointDist;
    cv::Point3f projected = point - normal * t;
    pt.x = projected.x;pt.z = projected.z;pt.y = projected.y;
  }

  void projectPointCloudOnPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<float> &plane_model)
  {
    for(auto &p: cloud->points)
      projectPointOnPlane(p,plane_model);
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");

    rs::StopWatch clock;
    label_indices_.clear();
    line_inliers_.clear();

    rs::SceneCas cas(tcas);
    cas.get(VIEW_CLOUD, *cloud_);
    cas.get(VIEW_NORMALS, *normals_);
    rs::Scene scene = cas.getScene();
    tf::StampedTransform camToWorld,worldToCam;
    rs::conversion::from(scene.viewPoint.get(),camToWorld);

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(camToWorld, eigenTransform);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_);
    sor.setKeepOrganized(true);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.05);
    sor.filter(*cloud_filtered_);

    pcl::OrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
    oed.setInputNormals(normals_);
    oed.setInputCloud(cloud_filtered_);
    oed.setDepthDisconThreshold(0.05);
    oed.setMaxSearchNeighbors(0.03);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY);
    pcl::PointCloud<pcl::Label> labels;

    oed.compute(labels, label_indices_);
    if(label_indices_[0].indices.size() == 0)
    {
      outWarn("No NaN boundaries found. Existing annotator");
      return UIMA_ERR_NONE;
    }

    pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
    outInfo("Before filter: " << cloud_filtered_->points.size());
    ei.setInputCloud(cloud_filtered_);
    //this is the bullshit of PCL...one algo returns PointIndices next algo want a f'in pointer
    ei.setIndices(boost::make_shared<pcl::PointIndices>(label_indices_[0]));
    ei.setKeepOrganized(true);
    ei.filterDirectly(cloud_filtered_);

    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud_filtered_);
    vg.setLeafSize(0.02, 0.02, 0.02);
    vg.filter(*cloud_filtered_);

    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_filtered_, *edge_cloud, eigenTransform);
//    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_filtered_, *cloud_filtered_, eigenTransform);
//    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_, *cloud_, eigenTransform);

    std::vector<float> xz_plane{0.0,1.0,0.0,0.5};
    projectPointCloudOnPlane(edge_cloud,xz_plane);

//    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr edge_cloud = cloud_filtered_->makeShared();

    //TODO what should be a stop criteria here?
    int count = 0;
    while(count++ < 5)
    {

      //lines parallel to the X-AXES (THIS CAN CHANGE)
      pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGBA>::Ptr
      model_pl(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGBA> (edge_cloud));
      model_pl->setAxis(Eigen::Vector3f(1.0, 0, 0));

      pcl::search::Search<pcl::PointXYZRGBA>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGBA>);
      search->setInputCloud(edge_cloud);
      model_pl->setSamplesMaxDist(0.07, search);
      model_pl->setEpsAngle(2 * M_PI / 180);

      pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model_pl);
      ransac.setDistanceThreshold(0.01);

      ransac.computeModel();
      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      ransac.getInliers(inliers->indices);

      float avg_y=0;
      std::for_each(inliers->indices.begin(), inliers->indices.end(), [&avg_y,this](int n){
        avg_y += this->cloud_filtered_->points[n].y;
      }
      );
      avg_y = avg_y/inliers->indices.size();
      float ssd = 0;
      std::for_each(inliers->indices.begin(), inliers->indices.end(), [avg_y,&ssd,this](int n){
        ssd += (this->cloud_filtered_->points[n].y - avg_y)*(this->cloud_filtered_->points[n].y - avg_y);
      }
      );

      float var = std::sqrt(ssd/(inliers->indices.size()));


      //the variance on y needs to be small
      if(inliers->indices.size() > min_line_inliers_ && var < 0.012)
      {
        outInfo("variance is : "<< var);
        outInfo("Line inliers found: " << inliers->indices.size());
        line_inliers_.push_back(inliers);
        ei.setInputCloud(edge_cloud);
        ei.setIndices(inliers);
        ei.setNegative(true);
        ei.setKeepOrganized(true);
        ei.filterDirectly(edge_cloud);
      }
      else{
        outWarn("variance was: "<<var);
        outWarn("inliers was:  "<< inliers->indices.size());
      }
    }

    outInfo("Cloud size: " << cloud_->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "cloud";
    outInfo("Cloud Filtered size: " << cloud_filtered_->points.size());
    double pointSize = 10.0;
    double pointSize2 = pointSize/4.0;
    for(int i = 0; i < line_inliers_.size(); ++i)
    {
      for(int j = 0; j < line_inliers_[i]->indices.size(); ++j)
      {
          cloud_filtered_->points[line_inliers_[i]->indices[j]].rgba = rs::common::colors[i];
          cloud_filtered_->points[line_inliers_[i]->indices[j]].a = 255;
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_, "original");
      visualizer.addPointCloud(cloud_filtered_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "original");
    }
    else
    {
      visualizer.updatePointCloud(cloud_, "original");
      visualizer.updatePointCloud(cloud_filtered_, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "original");
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ShelfDetector)
