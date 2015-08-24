
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
using namespace std;

bool next_iteration = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;

  PCL_ERROR ("Presseed \n");
}

int user_data; 
    
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) 
{ 
    viewer.setBackgroundColor (1.0, 0.5, 1.0); 
    pcl::PointXYZ o; 
    o.x = 1.0; 
    o.y = 0; 
    o.z = 0; 
    viewer.addSphere (o, 0.25, "sphere", 0); 
    std::cout << "i only run once" << std::endl; 
    
} 
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer) 
{ 
    static unsigned count = 0; 
    std::stringstream ss; 
    ss << "Once per viewer loop: " << count++; 
    viewer.removeShape ("text", 0); 
    viewer.addText (ss.str(), 200, 300, "text", 0); 
    
    //FIXME: possible race condition here: 
    user_data++; 
} 

int main (int argc, char** argv)
{
	
    
    
	/* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);*/

	//if (pcl::io::loadPCDFile<pcl::PointXYZ> ("scan_007.pcd", *cloud_in) == -1 || pcl::io::loadPCDFile<pcl::PointXYZ> ("scan_008.pcd", *cloud_out) == -1) //* load the file
	//{
	//	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	//	return 0;
	//}

	

	// Fill in the CloudIn data
	 // cloud_in->width    = 5;
	 // cloud_in->height   = 1;
	 // cloud_in->is_dense = false;
	 // cloud_in->points.resize (cloud_in->width * cloud_in->height);
	 // for (size_t i = 0; i < cloud_in->points.size (); ++i)
	 // {
		//cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		//cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		//cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	 // }
	 // std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
		//  << std::endl;
	 // for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
		//  cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		//  cloud_in->points[i].z << std::endl;
	 // *cloud_out = *cloud_in;
	 // std::cout << "size:" << cloud_out->points.size() << std::endl;
	 // for (size_t i = 0; i < cloud_in->points.size (); ++i)
		//cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	 // std::cout << "Transformed " << cloud_in->points.size () << " data points:"
		//  << std::endl;

	/* pcl::visualization::CloudViewer viewer("My Test Cloud");
	 viewer.showCloud(cloud_out);*/
//	// Register keyboard callback :
//  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
//
//  int iterations = 0 ;
//  // Display the visualiser
//  while (!viewer.wasStopped ())
//  {
//
//    // The user pressed "space" :
//    if (next_iteration)
//    {
//      // The Iterative Closest Point algorithm
//      /*time.tic ();
//      icp.align (*cloud_icp);
//      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;*/
//
//    
//        printf ("\033[11A");  // Go up 11 lines in terminal output.
////        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
//        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//       
////        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//        //viewer.updatePointCloud (cloud_in, cloud_out, "cloud_icp_v2");  
//		
//    }
//    next_iteration = false;
//  }

	  
	   /*for(;;){
		   if(next_iteration){
				viewer.showCloud(cloud_in);
		   }else
			   viewer.showCloud(cloud_out);
	   }*/
//	   viewer.showCloud(cloud_out);
	   /*while (!viewer.wasStopped())
	   {
	   }*/
	/*else{
		std::cout << "Loaded "
				<< cloud->width * cloud->height
				<< " data points from test_pcd.pcd with the following fields: "
				<< std::endl;
		for (size_t i = 0; i < cloud->points.size (); ++i)
		std::cout << " x " << cloud->points[i].x
					<< " y "    << cloud->points[i].y
					<< "  z"    << cloud->points[i].z << std::endl;
	}*/

	// Copying Pcd Data to these objects.
     pcl::PointCloud<pcl::PointXYZ>::Ptr data_input (new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr data_target (new pcl::PointCloud<pcl::PointXYZ>);
	 if (pcl::io::loadPCDFile<pcl::PointXYZ> ("scan_007.pcd", *data_input) == -1 || pcl::io::loadPCDFile<pcl::PointXYZ> ("scan_008.pcd", *data_target) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return 0;
	}
	  pcl::visualization::CloudViewer viewer("My Test Cloud");
	  viewer.showCloud(data_input);
	
	  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	  icp.setInputCloud(data_input);
	  icp.setInputTarget(data_target);
	  pcl::PointCloud<pcl::PointXYZ> ::Ptr Final;
	  double fitness = 1245;
	  Eigen::Matrix4f transform;
	  //Matrix4x4 transform;
	  while(fitness > 2){
		  icp.align(*Final);
		  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		  icp.getFitnessScore() << std::endl;
		  std::cout << icp.getFinalTransformation() << std::endl;
		  transform = icp.getFinalTransformation();
		  fitness = icp.getFitnessScore();


		  // Executing the transformation
		  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
		  // You can either apply transform_1 or transform_2; they are the same
		  pcl::transformPointCloud (*Final, *transformed_cloud, transform);

		   viewer.showCloud(transformed_cloud);

		   *Final = *transformed_cloud;

//		  pcl::transformPointCloud (Final, transform, transform);
	  }
	 
	  int f = 0 ; 
	  cin >>f;
	  return (0);
	
}

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//bool next_iteration = false;
//
//void
//print4x4Matrix (const Eigen::Matrix4d & matrix)
//{
//  printf ("Rotation matrix :\n");
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
//  printf ("Translation vector :\n");
//  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
//}
//
//void
//keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
//                       void* nothing)
//{
//  if (event.getKeySym () == "space" && event.keyDown ())
//    next_iteration = true;
//}
//
//int
//main (int argc,
//      char* argv[])
//{
//  // The point clouds we will be using
//  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
//  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
//  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
//
//  // Checking program arguments
//  //if (argc < 2)
//  //{
//  //  printf ("Usage :\n");
//  //  printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
//  //  PCL_ERROR ("Provide one ply file.\n");
//  //  return (-1);
//  //}
//
//  int iterations = 1;  // Default number of ICP iterations
//  //if (argc > 2)
//  //{
//  //  // If the user passed the number of iteration as an argument
//  //  iterations = atoi (argv[2]);
//  //  if (iterations < 1)
//  //  {
//  //    PCL_ERROR ("Number of initial iterations must be >= 1\n");
//  //    return (-1);
//  //  }
//  //}
//
//  pcl::console::TicToc time;
//  time.tic ();
//  if (pcl::io::loadPLYFile ("bunny.ply", *cloud_in) < 0)
//  {
//    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
//	int ff = 0 ; 
//	cin>>ff;
//    return (-1);
//  }
//  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
//
//  // Defining a rotation matrix and translation vector
//  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
//
//  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//  double theta = M_PI / 8;  // The angle of rotation in radians
//  transformation_matrix (0, 0) = cos (theta);
//  transformation_matrix (0, 1) = -sin (theta);
//  transformation_matrix (1, 0) = sin (theta);
//  transformation_matrix (1, 1) = cos (theta);
//
//  // A translation on Z axis (0.4 meters)
//  transformation_matrix (2, 3) = 0.4;
//
//  // Display in terminal the transformation matrix
//  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//  print4x4Matrix (transformation_matrix);
//
//  // Executing the transformation
//  pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
//  *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
//
//  // The Iterative Closest Point algorithm
//  time.tic ();
//  pcl::IterativeClosestPoint<PointT, PointT> icp;
//  icp.setMaximumIterations (iterations);
//  icp.setInputSource (cloud_icp);
//  icp.setInputTarget (cloud_in);
//  icp.align (*cloud_icp);
//  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
//  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
//
//  if (icp.hasConverged ())
//  {
//    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
//    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//    transformation_matrix = icp.getFinalTransformation ().cast<double>();
//    print4x4Matrix (transformation_matrix);
//  }
//  else
//  {
//    PCL_ERROR ("\nICP has not converged.\n");
//		int ff = 0 ; 
//	cin>>ff;
//
//    return (-1);
//  }
//
//  // Visualization
//  pcl::visualization::PCLVisualizer viewer ("ICP demo");
//  // Create two verticaly separated viewports
//  int v1 (0);
//  int v2 (1);
//  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//
//  // The color we will be using
//  float bckgr_gray_level = 0.0;  // Black
//  float txt_gray_lvl = 1.0 - bckgr_gray_level;
//
//  // Original point cloud is white
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
//                                                                             (int) 255 * txt_gray_lvl);
//  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
//
//  // Transformed point cloud is green
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
//  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
//
//  // ICP aligned point cloud is red
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
//  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
//
//  // Adding text descriptions in each viewport
//  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
//  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
//
//  std::stringstream ss;
//  ss << iterations;
//  std::string iterations_cnt = "ICP iterations = " + ss.str ();
//  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
//
//  // Set background color
//  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
//  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
//
//  // Set camera position and orientation
//  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//  viewer.setSize (1280, 1024);  // Visualiser window size
//
//  // Register keyboard callback :
//  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
//
//  // Display the visualiser
//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//
//    // The user pressed "space" :
//    if (next_iteration)
//    {
//      // The Iterative Closest Point algorithm
//      time.tic ();
//      icp.align (*cloud_icp);
//      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;
//
//      if (icp.hasConverged ())
//      {
//        printf ("\033[11A");  // Go up 11 lines in terminal output.
//        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
//        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
//
//        ss.str ("");
//        ss << iterations;
//        std::string iterations_cnt = "ICP iterations = " + ss.str ();
//        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//      }
//      else
//      {
//        PCL_ERROR ("\nICP has not converged.\n");
//			int ff = 0 ; 
//	cin>>ff;
//
//        return (-1);
//      }
//    }
//    next_iteration = false;
//  }
//
//  	int ff = 0 ; 
//	cin>>ff;
//
//  return (0);
//}