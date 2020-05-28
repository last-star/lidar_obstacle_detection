// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //create the filtering object
    pcl::VoxelGrid<PointT> sor;
    //����һ���µ��ڴ��������ڴ���˲���ĵ�������
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);//����������ĳߴ�
    sor.filter(*cloudFiltered);//���˲�������ݴ�ŵ����濪�ٵ��ڴ�ռ���

    //ROI
    //ɾ��һЩ���������Զ�ĵ�
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    //pcl::CropBox�����˸����������ڵĵ�������
    //����һ���������ݽ�ȡ�Ķ���
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    //remove ego car roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    //��¼���˲��������
    roof.filter(indices);

    //��ȡ�ָ����ⲿ�㣬ɾ�������ڣ��򵽳������ĵ�
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point : indices)
        inliers-> indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);//���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
    extract.setNegative(true);//������ȡ���
    extract.filter(*cloudRegion);//��ȡ����洢��cloudRegion

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    for(int index : inliers-> indices)
        planeCloud-> points.push_back(cloud-> points[index]);
    //Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.


    //ransac3d(pcl�����ض���RANSAC�������Ե���)
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    //��������������
    while(maxIterations --)
    {
        std::unordered_set<int> inliers;
        while(inliers.size() < 3)
            inliers.insert(rand()%(cloud-> points.size()));
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliers.begin();
        x1 = cloud-> point[*itr].x;
        y1 = cloud-> point[*itr].y;
        z1 = cloud-> point[*itr].z;
        itr ++;
        x2 = cloud-> point[*itr].x;
        y2 = cloud-> point[*itr].y;
        z2 = cloud-> point[*itr].z;
        itr ++;
        x3 = cloud-> point[*itr].x;
        y3 = cloud-> point[*itr].y;
        z3 = cloud-> point[*itr].z;

        float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        float D = -(i * x1 + j * y1 + k * z1);
        for(int index = 0; index < cloud-> points.size(); index ++)
        {
            if(inliers.count(index) > 0)
                continue;
            PointT point = cloud-> points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            //��������еĵ㵽���ƽ��ľ���
            float d = fabs(i * x4 + j * y4 + k * z4 + D)/sqrt(i * i + j * j + k * k)
            //����������С����С�������ƣ������ƽ�����
            if(d < distanceThreshold)
                inliers.insert(index);
        }
        if(inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
    for(int index = 0; index < cloud-> points.size(); index ++)
    {
        if(inliersResult.count(index))
        {
            inliers2-> indices.push_back(index);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree-> setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideaClusterExtraction<PointT> ec;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);//�������̶�
    //��Ⱥ�ĵ�����min��max
    ec.setMinClusterSize(minSize);//�޳���С�Ŀ����������ĵ�
    ec.setMaxClusterSize(maxSize);//�ֽ�ǳ���ļ�Ⱥ�����������Ⱥ�ص���������ݲ���԰������Ǹ��õؽ��������.
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);//get all clusters indices

    //for each cluster indice
    for(pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster-> points.push_back(cloud-> points[index]);
        cloudCluster-> width = cloudCluster-> points.size();
        cloudCluster-> height = 1;
        cloudCluster-> is_dense = true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);
	for(auto id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(),false);
    for(size_t idx=0;idx< cloud->points.size(); ++idx)
        {
            if(processed[idx] == false)
            {
                std::vector<int> cluster_idx;
                typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
                clusterHelper(idx,cloud, cluster_idx, processed, tree, distanceTol);
                if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
                {
                    for(int i=0; i<cluster_idx.size(); i++)
                    {
                        PointT point;
                        point = cloud->points[cluster_idx[i]];
                        cloudCluster->points.push_back(point);
                    }
                    cloudCluster->width = cloudCluster->points.size();
                    cloudCluster->height = 1;
                    clusters.push_back(cloudCluster);
                }
                else
                {
                    for(int i=1;i<cluster_idx.size();i++)
                    {
                        processed[cluster_idx[i]] = false;
                    }
                }
            }
        }
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
