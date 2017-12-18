
#include <scanRegistration.hpp>
#include <laserOdometry.hpp>
#include <laserMapping.hpp>



#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>




using namespace pcl;
using namespace std;

int main(){

	
	int i;
	char filename[50] = {0};

	
 	
       


    scanRegistration scanner;
	laserOdometry odometrier;
	laserMapping mapper;



	 
	for (i=0;i<1 ;i++){
		
		if (i<10) 
			snprintf(filename,sizeof(filename),"/home/xiesc/09/velodyne/00000%d.bin",i);
			else if (10<=i &&i<100)
			snprintf(filename,sizeof(filename),"/home/xiesc/09/velodyne/0000%d.bin",i);
			else if (100<=i && i <1000)
			snprintf(filename,sizeof(filename),"/home/xiesc/09/velodyne/000%d.bin",i); 
			else if (1000<=i && i <10000)
			snprintf(filename,sizeof(filename),"/home/xiesc/09/velodyne/00%d.bin",i); 
			
		
		

		// load point cloud
		fstream input(filename, ios::in | ios::binary);
		if(!input.good()){
			cerr << "Could not read file: " << filename << endl;
			exit(EXIT_FAILURE);
		}
		input.seekg(0, ios::beg);

		pcl::PointCloud<PointXYZI> points ;
        points.clear();
		int j;
		for (j=0; input.good() && !input.eof(); j++) {
			PointXYZI point;
			input.read((char *) &point.x, 3*sizeof(float));
			input.read((char *) &point.intensity, sizeof(float));
			points.push_back(point);
		}

		input.close();
        
        // pcl::io::savePCDFileASCII ("/home/xiesc/test.pcd", points);
        
		// cout << "Read KTTI point cloud with " << j << " belong to " <<filename<<","<<i <<endl;
        
		scanRegistrationBack scanValueBack;
        laserOdometryBack odometryValueBack;
		laserMappingBack mappingBackValue;

        scanValueBack = scanner.laserCloudHandler(points);
		odometryValueBack = odometrier.laserOdometryHandler(scanValueBack);
		mappingBackValue = mapper.laserMappingHandler(odometryValueBack);
        scanner.test_print();
	
    
    	// // Save DoN features
		// writer.write<PointXYZI> (outfile, *points, false);
		
	}






    
    
    return 0;


}