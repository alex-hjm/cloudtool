#include "segmentation.h"



std::vector<Cloud::Ptr>
Segmentation::PlaneSACSegmentation(const Cloud::Ptr &cloud,double distance_weightconst,double threshold,nt max_iterations,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    Cloud::Ptr cloud_segmented (new Cloud);
    ModelCoefficients::Ptr cofes(new ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointXYZRGBN,PointXYZRGBN> sacsegn;

    sacsegn.setModelType(pcl::SacModel::SACMODEL_NORMAL_PLANE);
    sacsegn.setMethodType(pcl::SAC_RANSAC);
    sacsegn.setNormalDistanceWeight(distance_weightconst);
    sacsegn.setDistanceThreshold(threshold);
    sacsegn.setMaxIterations(max_iterations);
    sacsegn.setOptimizeCoefficients(true);
    sacsegn.setNumberOfThreads(12);
    sacsegn.setInputCloud(cloud);
    sacsegn.setInputNormals(cloud);
    sacsegn.segment(*inliers,*cofes);

    pcl::ExtractIndices<PointXYZRGBN> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filter(*cloud_segmented);
    segmented_clouds.push_back(cloud_segmented);
    emit result(segmented_clouds,time.toc());
}

std::vector<Cloud::Ptr>
Segmentation::EuclideanClusterExtraction(const Cloud::Ptr &cloud, double tolerance, int min_cluster_size, int max_cluster_size)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::EuclideanClusterExtraction<PointXYZRGBN> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        Cloud::Ptr cloud_cluster (new Cloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        segmented_clouds.push_back(cloud_cluster);
        j++;
    }
    emit result(segmented_clouds,time.toc());
}

std::vector<Cloud::Ptr>
Segmentation::RegionGrowing(const Cloud::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,bool CurvatureTest,
                            float SmoothnessThreshold,float CurvatureThreshold, int NumberOfNeighbours)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::RegionGrowing<PointXYZRGBN, PointXYZRGBN> reg;

    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (max_cluster_size);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(smoothMode);
    reg.setSmoothnessThreshold(SmoothnessThreshold/180*M_PI);
    reg.setCurvatureTestFlag(CurvatureTest);
    reg.setCurvatureThreshold(CurvatureThreshold);
    reg.setNumberOfNeighbours(NumberOfNeighbours);
    reg.setInputCloud(cloud);
    reg.setInputNormals(cloud);

    reg.extract (cluster_indices);
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        Cloud::Ptr cloud_cluster (new Cloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        segmented_clouds.push_back(cloud_cluster);
        j++;
    }
    emit result(segmented_clouds,time.toc());
}
