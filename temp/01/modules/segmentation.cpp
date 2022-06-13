#include "segmentation.h"

Segmentation::Segmentation(QObject *parent) : QObject(parent)
{

}

void
Segmentation::SACSegmentation(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                    int max_iterations,double min_radius,double max_radius,bool optimize,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    Cloud::Ptr cloud_segmented (new Cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointXYZRGBN> sacseg;
    sacseg.setModelType (model_type);
    sacseg.setMethodType(method_type);
    sacseg.setDistanceThreshold(threshold);
    sacseg.setMaxIterations(max_iterations);
    sacseg.setOptimizeCoefficients(optimize);
    sacseg.setRadiusLimits(min_radius,max_radius);
    sacseg.setNumberOfThreads(12);
    sacseg.setInputCloud(cloud);
    sacseg.segment(*inliers,*cofes);

    pcl::ExtractIndices<PointXYZRGBN> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filter(*cloud_segmented);
    segmented_clouds.push_back(cloud_segmented);
    emit result(segmented_clouds,time.toc());
}

void
Segmentation::SACSegmentationFromNormals(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                          int max_iterations,double min_radius,double max_radius,bool optimize,bool negative,double distance_weightconst)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    Cloud::Ptr cloud_segmented (new Cloud());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointXYZRGBN,PointXYZRGBN> sacsegn;

    sacsegn.setModelType(model_type);
    sacsegn.setMethodType(method_type);
    sacsegn.setDistanceThreshold(threshold);
    sacsegn.setMaxIterations(max_iterations);
    sacsegn.setOptimizeCoefficients(optimize);
    sacsegn.setRadiusLimits(min_radius,max_radius);
    sacsegn.setNormalDistanceWeight(distance_weightconst);
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

void
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

void
Segmentation::RegionGrowing(const Cloud::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,
                                  bool CurvatureTest, bool ResidualTest, float SmoothnessThreshold, float ResidualThreshold,
                                  float CurvatureThreshold, int NumberOfNeighbours)
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
    reg.setResidualTestFlag(ResidualTest);
    reg.setResidualThreshold(ResidualThreshold);
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

void
Segmentation::RegionGrowingRGB(const Cloud::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,
                                     bool CurvatureTest, bool ResidualTest, float SmoothnessThreshold, float ResidualThreshold,
                                     float CurvatureThreshold, int NumberOfNeighbours,float pointcolorthresh,float regioncolorthresh,
                                     float distancethresh,int nghbr_number)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::RegionGrowingRGB<PointXYZRGBN, PointXYZRGBN> regrgb;

    regrgb.setMinClusterSize (min_cluster_size);
    regrgb.setMaxClusterSize (max_cluster_size);
    regrgb.setSmoothModeFlag(smoothMode);
    regrgb.setSmoothnessThreshold(SmoothnessThreshold/180*M_PI);
    regrgb.setCurvatureTestFlag(CurvatureTest);
    regrgb.setCurvatureThreshold(CurvatureThreshold);
    regrgb.setResidualTestFlag(ResidualTest);
    regrgb.setResidualThreshold(ResidualThreshold);
    regrgb.setNumberOfNeighbours(NumberOfNeighbours);

    regrgb.setPointColorThreshold(pointcolorthresh);
    regrgb.setRegionColorThreshold(regioncolorthresh);
    regrgb.setDistanceThreshold(distancethresh);
    regrgb.setNumberOfRegionNeighbours(nghbr_number);

    regrgb.setSearchMethod (tree);
    regrgb.setInputCloud(cloud);
    regrgb.setInputNormals(cloud);
    regrgb.extract (cluster_indices);

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

void
Segmentation::MinCutSegmentation(const Cloud::Ptr &cloud,const Eigen::Vector3f &center,double sigma,double radius,
                                       double weight,int neighbour_number)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    Cloud::Ptr foreground_points(new Cloud);
    PointXYZRGBN point;
    point.x = center[0];
    point.y = center[1];
    point.z = center[2];
    foreground_points->points.push_back(point);

    pcl::MinCutSegmentation<PointXYZRGBN> mseg;
    mseg.setInputCloud (cloud);
    mseg.setSigma(sigma);
    mseg.setRadius(radius);
    mseg.setSourceWeight(weight);
    mseg.setSearchMethod(tree);
    mseg.setForegroundPoints(foreground_points);
    mseg.setNumberOfNeighbours(neighbour_number);
    mseg.extract(cluster_indices);
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

void
Segmentation::DonSegmentation(const Cloud::Ptr &cloud,double mean_radius, double scale1,double scale2,double threshold,
                                    double segradius,int minClusterSize ,int maxClusterSize)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    scale1*=mean_radius;
    scale2*=mean_radius;
    segradius*=mean_radius;

    pcl::NormalEstimationOMP<PointXYZRGBN, PointN> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    CloudN::Ptr normals_small_scale (new CloudN);

    ne.setRadiusSearch (scale1);
    ne.setNumberOfThreads(12);
    ne.compute (*normals_small_scale);

    CloudN::Ptr normals_large_scale (new CloudN);
    ne.setRadiusSearch (scale2);
    ne.setNumberOfThreads(12);
    ne.compute (*normals_large_scale);

    CloudN::Ptr doncloud (new CloudN);
    copyPointCloud (*cloud, *doncloud);

    pcl::DifferenceOfNormalsEstimation<PointXYZRGBN, PointN, PointN> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    don.computeFeature (*doncloud);
    pcl::ConditionOr<PointN>::Ptr range_cond;
    range_cond->addComparison (pcl::FieldComparison<PointN>::ConstPtr (new pcl::FieldComparison<PointN>("curvature", pcl::ComparisonOps::GT, threshold)));

    pcl::ConditionalRemoval<PointN> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);
    pcl::PointCloud<PointN>::Ptr doncloud_filtered (new pcl::PointCloud<PointN>);
    condrem.filter (*doncloud_filtered);
    doncloud = doncloud_filtered;

    pcl::search::KdTree<PointN>::Ptr segtree (new pcl::search::KdTree<PointN>);
    segtree->setInputCloud (doncloud);

    pcl::EuclideanClusterExtraction<PointN> ecc;
    ecc.setClusterTolerance (segradius);
    ecc.setMinClusterSize (minClusterSize);
    ecc.setMaxClusterSize (maxClusterSize);
    ecc.setSearchMethod (segtree);
    ecc.setInputCloud (doncloud);
    ecc.extract (cluster_indices);

    int n = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        Cloud::Ptr cloud_cluster (new Cloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        segmented_clouds.push_back(cloud_cluster);
        n++;
    }
    emit result(segmented_clouds,time.toc());
}

void
Segmentation::MorphologicalFilter(const Cloud::Ptr &cloud,int max_window_size,float slope,float max_distance,float initial_distance,
                                        float cell_size,float base,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Cloud::Ptr cloud_segmented (new Cloud());
    pcl::ProgressiveMorphologicalFilter<PointXYZRGBN> pmf;

    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(max_window_size);
    pmf.setSlope(slope);
    pmf.setMaxDistance(max_distance);
    pmf.setInitialDistance(initial_distance);
    pmf.setCellSize(cell_size);
    pmf.setBase(base);
    pmf.extract(inliers->indices);

    pcl::ExtractIndices<PointXYZRGBN> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filter(*cloud_segmented);

    segmented_clouds.push_back(cloud_segmented);
    emit result(segmented_clouds,time.toc());
}

void
Segmentation::SupervoxelClustering(const Cloud::Ptr &cloud,float voxel_resolution,float seed_resolution,float color_importance,
                                         float spatial_importance,float normal_importance, bool camera_transform)
{
    pcl::console::TicToc time;
    time.tic();
    std::vector<Cloud::Ptr> segmented_clouds;
    CloudXYZRGBA::Ptr cloud_xyzrgba(new CloudXYZRGBA);
    pcl::copyPointCloud(*cloud,*cloud_xyzrgba);

    pcl::SupervoxelClustering<PointXYZRGBA> super(voxel_resolution,seed_resolution);
    std::map <uint32_t, pcl::Supervoxel<PointXYZRGBA>::Ptr> supervoxel_clusters;

    if (camera_transform)
        super.setUseSingleCameraTransform (false);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    super.setInputCloud (cloud_xyzrgba);
    super.extract (supervoxel_clusters);

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end ();){
        uint32_t supervoxel_label = label_itr->first;
        pcl::Supervoxel<PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);
        //VoxelClouds
        Cloud::Ptr supervoxel_cloud(new Cloud);
        pcl::copyPointCloud(*supervoxel->voxels_,*supervoxel_cloud);
        segmented_clouds.push_back(supervoxel_cloud);
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }
    emit result(segmented_clouds,time.toc());
}





