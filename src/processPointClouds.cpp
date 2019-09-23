// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    pcl::VoxelGrid<PointT> vg;

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    Eigen::Vector4f roofMin = {-1.5,-1.7,-1,1};
    Eigen::Vector4f roofMax = {2.6,1.7,-0.4,1};
    roof.setMin(roofMin);
    roof.setMax(roofMax);
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr plane{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr obstacle{new pcl::PointCloud<PointT>};
    for(int index : inliers->indices){
        plane->points.push_back(cloud->points[index]);
    }
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle,plane);
    return segResult;
}
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
  	int pointSize = cloud->points.size();
	srand(time(NULL));
	while(maxIterations--){
		int firstPoint = rand()%pointSize;
      	int secondPoint = rand()%pointSize;
      	int thirdPoint = rand()%pointSize;
      	
      	std::unordered_set<int> inliers;
      	double x1 = cloud->points[firstPoint].x;
      	double y1 = cloud->points[firstPoint].y;
      	double z1 = cloud->points[firstPoint].z;
       	double x2 = cloud->points[secondPoint].x;
      	double y2 = cloud->points[secondPoint].y;
      	double z2 = cloud->points[secondPoint].z;
       	double x3 = cloud->points[thirdPoint].x;
      	double y3 = cloud->points[thirdPoint].y;
      	double z3 = cloud->points[thirdPoint].z;
      
      	double A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
      	double B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
      	double C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
      	double D = -(A*x1 + B*y1 + C*z1);
        double sqr = sqrt(A*A + B*B + C*C);
      
      	for(int i = 0; i<pointSize; i++){
         	double dist = abs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + D)/sqr;
          if(dist<=distanceTol){
           	inliers.insert(i); 
          }
        }
      	if(inliers.size()> inliersResult.size()){
         	inliersResult = inliers; 
        }
    }

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = Ransac(cloud,maxIterations,distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr cloudInliers{new pcl::PointCloud<PointT>};
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers{new pcl::PointCloud<PointT>};

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    if(cloudInliers->points.size()==0){
        std::cout<<"can not estimate the plane model from the given point cloud"<<std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair(cloudOutliers,cloudInliers);
    return segResult;
}

/*

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    typename pcl::SACSegmentation<PointT> seg;
    //seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RRANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers->indices.size()==0){
        std::cout<<"can not estimate the plane model from the given point cloud"<<std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
*/
template<typename PointT>
KdTree* ProcessPointClouds<PointT>::buildKdTree(typename pcl::PointCloud<PointT>::Ptr cloud){
    KdTree* tree = new KdTree();
    for(int i = 0; i<cloud->points.size(); i++){
        std::vector<float> curr;
        curr.push_back(cloud->points[i].x);
        curr.push_back(cloud->points[i].y);
        curr.push_back(cloud->points[i].z);
        tree->insert(curr,i);
    }
    return tree;
}
template <typename PointT>
void ProcessPointClouds<PointT>::helpcluster(KdTree* tree, typename pcl::PointCloud<PointT>::Ptr cloud, int pos,float clusterTol,std::vector<bool> &processed,std::vector<int> &cluster )
{
    processed[pos] = true;
    cluster.push_back(pos);
    std::vector<float> target;
    target.push_back(cloud->points[pos].x);
    target.push_back(cloud->points[pos].y);
    target.push_back(cloud->points[pos].z);
    std::vector<int> nearPoint = tree->search(target,clusterTol);
    for(int &index : nearPoint){
        if(!processed[index]){
             helpcluster(tree,cloud,index,clusterTol,processed,cluster);
        }
    }

}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = buildKdTree(cloud);
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->points.size(),false);
    for(int i = 0; i<cloud->points.size();i++){
        if(!processed[i]){
            std::vector<int> cluster;

            helpcluster(tree,cloud,i,clusterTolerance,processed,cluster);

            if(cluster.size()>=minSize&&cluster.size()<=maxSize){
                clusters.push_back(cluster);
            }
        }
    }
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
    for(auto &indices: clusters){
        typename pcl::PointCloud<PointT>::Ptr clusterCloud {new typename pcl::PointCloud<PointT>};
        for(int &index : indices){
            clusterCloud->points.push_back(cloud->points[index]);
        }
        clusterClouds.push_back(clusterCloud);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusterClouds.size() << " clusters" << std::endl;
    return clusterClouds;
}
/*
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud (cloud);
  	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance); // 2cm
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);
	for(pcl::PointIndices getindices : cluster_indices){
      typename pcl:: PointCloud<PointT> ::Ptr cloudCluster { new pcl:: PointCloud<PointT> };
      for(int index : getindices.indices){
       	cloudCluster->points.push_back(cloud->points[index]); 
      }
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

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