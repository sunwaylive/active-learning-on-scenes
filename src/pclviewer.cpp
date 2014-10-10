#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Scene Segmentation");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  if(pcl::io::loadPCDFile<PointT>("../data/s7.pcd", *cloud) == -1){
    PCL_ERROR("Could't read file! \n");
    return;
  }
  
  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  //viewer->addCoordinateSystem(1.0f, 0, 0, 0, 0);
  ui->qvtkWidget->update ();
}

void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");
  std::vector<int> inliers;
  pcl::PointCloud<PointT>::Ptr deskplane(new pcl::PointCloud<PointT>);
  
  pcl::SampleConsensusModelPlane<PointT>::Ptr
    model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud));
  pcl::RandomSampleConsensus<PointT> ransac(model_p);
  ransac.setDistanceThreshold(.01);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // double original_height = 0.0f;
  // int original_total = cloud->size();
  // for(size_t i = 0; i < original_total; ++i){
  //   original_height += cloud->points[i].y;
  // }
  // std::cout<<original_height / original_total <<std::endl;
  
  pcl::copyPointCloud<PointT>(*cloud, inliers, *deskplane);
  //remove outlier
  pcl::PointCloud<PointT>::Ptr clean_deskplane(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(deskplane);  
  sor.setMeanK (500);
  sor.setStddevMulThresh(1.0);
  sor.filter(*clean_deskplane);
  pcl::io::savePCDFileASCII("../data/clean_plane.pcd", *clean_deskplane);
  
  float x_max = -1000, x_min = 1000;
  for(int i = 0; i < clean_deskplane->points.size(); ++i){
    x_max = std::max(x_max, clean_deskplane->points[i].x);
    x_min = std::min(x_min, clean_deskplane->points[i].x);
  }
  cout<<"x_min: " << x_min <<endl;
  cout<<"x_max: " << x_max <<endl;
  viewer->updatePointCloud (clean_deskplane, "cloud");
  ui->qvtkWidget->update ();
  return;
  
  //remove point under the y min
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
  range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
                                                                   new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, x_max)));

  pcl::ConditionalRemoval<PointT> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);;
  condrem.setKeepOrganized(true);
  condrem.filter(*cloud_filtered);
  
  
  //remove outlier
   pcl::PointCloud<PointT>::Ptr clean_scene(new pcl::PointCloud<PointT>);
  // pcl::StatisticalOutlierRemoval<PointT> sor;
  // sor.setInputCloud(cloud_filtered);  
  // sor.setMeanK (500);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*clean_scene);
  
  viewer->updatePointCloud (clean_scene, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::RGBsliderReleased ()
{
  // Set the new color
  for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
