
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <signal.h>
#include <string>

#include<opencv/cv.h>
#include <opencv2/opencv.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>

#include <libfreenect2/config.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
#include "viewer.h"
#endif


//Point cloud is inverted in y and z directions
cv:: Point TL;
cv:: Point BR;
cv:: Mat img,hsv,binary;  
cv::Mat AllignedRGB(424, 512, CV_8UC3); //RGB Image created for RGB analysis
bool protonect_shutdown = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

//projects depth map to create a pointcloud
pcl::PointCloud<pcl::PointXYZRGBA> PointCloud(libfreenect2::Frame* undistorted, libfreenect2::Frame* registered)
{ //currently failing
	std::cout<<"Point cloud function running" << std::endl;
	//or pixel xi=0..511, yi=0..423, (xu, yu) = ((xi+0.5-cx)/fx, (yi+0.5-cy)/fy), 
	//then the point coordinates you are looking for is (xu*undistorted[512*yi+xi], yu*undistorted[512*yi+xi], undistorted[512*yi+xi]) 
	//and the color is registered[512*yi+xi]

	float* undistorted_data = (float *)undistorted->data;
	float* registered_data = (float *)registered->data;
	float fx =  366.012;
	float  fy =  366.012;
	float  cx  = 256.948;
	float  cy =  209.061;
	int i = 0;


	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	cloud.width    = 512;
	cloud.height   = 424;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);
	
	std::cout<<"Point cloud function running" << std::endl;
	for (int xi = 0; xi < registered->width; xi++){
		for (int yi = 0; yi<registered->height; yi++){
			float xu = ((xi+0.5-cx)/fx);
			float yu = (yi+0.5-cy)/fy;
			float xWorld = xu*undistorted_data[512*yi+xi];
			float yWorld = yu*undistorted_data[512*yi+xi];
			float zWorld = undistorted_data[512*yi+xi];
			float RGB = registered_data[512*yi+xi]; 
			cloud.points[i].x = xWorld / 1000;
			cloud.points[i].y = yWorld / 1000;
			cloud.points[i].z = zWorld / 1000;
			cloud.points[i].rgb = RGB;
			Eigen::Vector3i rgb = cloud.points[i].getRGBVector3i();
			AllignedRGB.at<cv::Vec3b>(yi,xi)[0] = rgb[2];
			AllignedRGB.at<cv::Vec3b>(yi,xi)[1] = rgb[1];
			AllignedRGB.at<cv::Vec3b>(yi,xi)[2] = rgb[0];
			i++;}
	}
	/*pcl::PointCloud<pcl::PointXYZRGBA> newCloud;
	  std::vector<int> second (1,1);   
	  pcl::removeNaNFromPointCloud(cloud, newCloud,second); */
	pcl::io::savePLYFile("/home/kinect2/Dropbox/data/Trial36/KinectPointCloud.ply", cloud,true);

	std::cerr << "Saved " << cloud.points.size () << " data points to KinectPointCloud.ply." << std::endl;
	for (size_t i = 0; i < 50; ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " " << cloud.points[i].rgb <<std::endl;

	std::cout<<"Point cloud function done" << std::endl;
	cv::imwrite("/home/kinect2/Dropbox/data/Trial36/Alligned.jpg",AllignedRGB); //save the alligned RGB image
	return cloud;
}

//detects stain
cv::Point* redDetection(cv::Point points[]){ //detects largest contour and return boungding box of the contour 
	
	cv::cvtColor(AllignedRGB, hsv, CV_BGR2HSV);  //using globally defined AllignedRGB image from point cloud
	cv::imwrite("/home/kinect2/Dropbox/data/Trial36/HSV.jpg", hsv);	
	////get binary image 
	//cv::inRange(hsv, cv::Scalar(157, 72, 156), cv::Scalar(180, 169, 255), binary);
	cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), binary); 
	cv::medianBlur(binary,binary,3);
	cv::dilate(binary,binary,cv::Mat(),cv::Point(-1,-1),1);	
	cv::imwrite("/home/kinect2/Dropbox/data/Trial36/binary.jpg", binary);	
	////find contours from binary image  
	
	std::vector<std::vector<cv::Point> > contours;  
	cv::findContours(binary, contours, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1); //find contours  
	std::vector<double> areas(contours.size());  
	//find largest contour area  
	for(int i = 0; i < contours.size(); i++)  
	{  
		areas[i] = cv::contourArea(cv::Mat(contours[i])); } 

	//get index of largest contour  
	double max;  
	cv::Point maxPosition;  
	cv::minMaxLoc(cv::Mat(areas),0,&max,0,&maxPosition);  
	//draw largest contour.  
	cv::drawContours(binary, contours, maxPosition.y, cv::Scalar(255), CV_FILLED);  
	cv::imwrite("/home/kinect2/Dropbox/data/Trial36/contours.jpg", binary);
	
	//cv::imshow("Binary", binary);
	cv::waitKey(0);
	////draw bounding rectangle around largest contour  

	cv::Point center;  
	cv::Rect r;  
	if (contours.size() >= 1)  
	{  
		r = cv::boundingRect(contours[maxPosition.y]);  
		cv::rectangle(AllignedRGB, r.tl(),r.br(), CV_RGB(255, 0, 0), 3, 8, 0); //draw rectangle  
		cv::imwrite("/home/kinect2/Dropbox/data/Trial36/DetectedStain.jpg", AllignedRGB);
	}  
	//Get Bounding Box Corners
	points[0] = r.tl(); //top left
	points[1] = cv::Point(r.x ,r.y - r.height); //bottom left
	points[2] = cv::Point(r.x + r.width,r.y ); //top right
	points[3] = r.br(); //bottom right
	//cout << r.height << "acualt height" << endl;

	//get centroid  
	center.x = r.x + (r.width/2);  
	center.y= r.y + (r.height/2); 
	points[4] = center;
		
	return points;
	
} 


//path planner code
pcl::PointCloud<pcl::PointXYZRGBA> SegmentAndTransform( cv::Point points[], pcl::PointCloud<pcl::PointXYZRGBA> cloud){ //segements the bounding box and return that point in 3D space
	
cv::Point TopLeft = points[0];
cv::Point BotLeft = points[1];
cv::Point TopRight = points[2];
cv::Point BotRight = points[3];
int stepSize = 1;
int BBheight = abs(TopLeft.y - BotLeft.y)  ; // calculates height of bbox
int BBwidth = abs(BotRight.x - BotLeft.x) ; //calculates width of bbox
int BBPts = BBheight*BBwidth / (stepSize*stepSize);
pcl::PointCloud<pcl::PointXYZRGBA> testCloud;
testCloud.width = BBwidth;
testCloud.height = BBheight;
testCloud.is_dense = false;
testCloud.points.resize (testCloud.width * testCloud.height);
cv::Point PatchPts[BBPts]; //filled color coordinates of boudning box
float xWorld[BBPts];
float yWorld[BBPts];
float zWorld[BBPts];
int counter = 0;


for (int i = 0; i < BBheight; i = i+stepSize){ //increment by 1 row 1 col in pixel coordinates
	
	if(i%2==0){
		for (int j = 0; j < BBwidth; j = j+stepSize){ //increment row
			if (j % 2 == 0)		
				PatchPts[counter] = cv::Point(TopLeft.x + j,TopLeft.y + i);
			counter++;
		}
	}
	else{
		for (int j = BBwidth-1; j >= 0; j = j-stepSize){ //increment row
			if (j % 2 == 0)		
				PatchPts[counter] = cv::Point(TopLeft.x + j,TopLeft.y + i);
			counter++;
		}
	}	

}

//overlay  each color coordinates to x,y,z in the pointcloud

int pTransform;
for (int i = 0; i<BBPts;i++){
	pTransform = (cloud.height*PatchPts[i].x) + (PatchPts[i].y); //Gives index of Color pixel (x,y) to Point Cloud array position i 
	

	xWorld[i] = cloud.points[pTransform].x; //Probably will not work due to KDtree structure
	yWorld[i] = cloud.points[pTransform].y;
	zWorld[i] = cloud.points[pTransform].z;
	testCloud.points[i].x = cloud.points[pTransform].x*1000;
	testCloud.points[i].y = cloud.points[pTransform].y*1000;
	testCloud.points[i].z = cloud.points[pTransform].z*1000;
	testCloud.points[i].rgb = cloud.points[pTransform].rgb;
} 
std::cout << "Top Left" << cloud.points[((cloud.height*TopLeft.x) + (TopLeft.y))].x << "," << cloud.points[((cloud.height*TopLeft.x) + (TopLeft.y))].y << " , " << cloud.points[((cloud.height*TopLeft.x) + (TopLeft.y))].z <<std::endl; 
std::cout << "Top Right" << cloud.points[(cloud.height*TopRight.x) + (TopRight.y)].x << "," << cloud.points[(cloud.height*TopRight.x) + (TopRight.y)].y << " , " << cloud.points[(cloud.height*TopRight.x) + (TopRight.y)].z <<std::endl; 
std::cout << "Bottom Right" << cloud.points[(cloud.height*BotRight.x) + (BotRight.y)].x << "," << cloud.points[(cloud.height*BotRight.x) + (BotRight.y)].y << " , " << cloud.points[(cloud.height*BotRight.x) + (BotRight.y)].z <<std::endl; 
std::cout << "Bottom Left" << cloud.points[(cloud.height*BotLeft.x) + (BotLeft.y)].x << "," << cloud.points[(cloud.height*BotLeft.x) + (BotLeft.y)].y << " , " << cloud.points[(cloud.height*BotLeft.x) + (BotLeft.y)].z <<std::endl; 

 //output detected kinect pts to text file;

std:: ofstream x,y,z;
x.open("xK.txt");
y.open("yK.txt");
z.open("zK.txt");

for (int i = 0; i< BBwidth*BBheight ;i++)
{
    if(x.is_open() && y.is_open() && z.is_open()){
	x << xWorld[i] << ",";
	y << yWorld[i] << ",";
	z << zWorld[i] << ",";	
     }
        
 }


x.close();
y.close();
z.close(); 

pcl::io::savePLYFile("/home/kinect2/Dropbox/data/Trial36/RedDetected.ply", testCloud,true);
std::cout << "Saved RED Point cloud" << std::cout;
return testCloud;

}

//trabsforms points from kinect frame to kuka frame
void transformKinect2Kuka(pcl::PointCloud<pcl::PointXYZRGBA> detectedCloud){ //using wrong point cloud
	
	cv::Point3d KukaPts[detectedCloud.width * detectedCloud.height];
	pcl::PointCloud<pcl::PointXYZRGBA> testCloud;
	testCloud.width = detectedCloud.width;
	testCloud.height = detectedCloud.height;
	testCloud.is_dense = false;
	testCloud.points.resize (testCloud.width * testCloud.height);

	double r11 = -0.9993;
	double r12 = -0.0219;
	double r13 = -0.0318;
	double r21 = 0.0222;
	double r22 = -0.9997 ;
	double r23 = -0.0096;
	double r31 =  0.0316 ;
	double r32 = 0.0103;
	double r33 = -0.9994;

	double Tx =  367.4;
 	double Ty =  12.7;
	double Tz = 1269.1;

	double transformData[] = {r11,r12,r13,Tx,r21,r22,r23,Ty,r31,r32,r33,Tz,0,0,0,1};
	cv::Mat TransformationMat = cv::Mat(4, 4, CV_64F, &transformData);

	for (int i = 1; i<detectedCloud.width * detectedCloud.height;i++) //corrects for any lost data
	{
		if((abs(detectedCloud.points[i].x) == 0) && (abs(detectedCloud.points[i].y) == 0) && (abs(detectedCloud.points[i].z) == 0)){
			detectedCloud.points[i].x = (detectedCloud.points[i-1].x + detectedCloud.points[i+1].x) / 2;
			detectedCloud.points[i].y = (detectedCloud.points[i-1].y + detectedCloud.points[i+1].y) / 2;
			detectedCloud.points[i].z = (detectedCloud.points[i-1].z + detectedCloud.points[i+1].z) / 2;
		}

	}

	for (int i =0; i <detectedCloud.width * detectedCloud.height;i++){ //transformation 
		double tempdata[] = {detectedCloud.points[i].x  ,detectedCloud.points[i].y ,detectedCloud.points[i].z  ,1};
		cv::Mat temp(4,1,CV_64F,&tempdata);
		//cout << (temp) << endl;
		cv::Mat result = TransformationMat*(temp);
		//cout << "this is result " << result << endl;
		//cout << "Val 1: " <<result.at<double>(1,0) << endl;

		cv::Point3d ptPrime(result.at<double>(0,0),result.at<double>(1,0),result.at<double>(2,0));

		KukaPts[i] = ptPrime; //save in array
		testCloud.points[i].x = ptPrime.x ;
		testCloud.points[i].y = ptPrime.y  ;
		testCloud.points[i].z = ptPrime.z ;
		testCloud.points[i].rgb = detectedCloud.points[i].rgb;
	}

 // Output points 



std:: ofstream n,x,y,z,points;
n.open("/home/kinect2/Dropbox/data/Trial36/n.txt");
x.open("/home/kinect2/Dropbox/data/Trial36/x.txt");
y.open("/home/kinect2/Dropbox/data/Trial36/y.txt");
z.open("/home/kinect2/Dropbox/data/Trial36/z.txt");
points.open("/home/kinect2/Dropbox/points.txt");
for (int i = 0; i< detectedCloud.width * detectedCloud.height ;i++)
{
    if( x.is_open() && y.is_open() && z.is_open()){
	x << KukaPts[i].x << ",";
	y << KukaPts[i].y << ",";
	z << KukaPts[i].z << ",";	
     }
        
 }

if(n.is_open())
{
	n << detectedCloud.width* detectedCloud.height;
	}


n.close();
x.close();
y.close();
z.close();

//for java
points << "double[] x = {";
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1;i++)
{
	points << KukaPts[i].x << ",";       
 } 
points << KukaPts[detectedCloud.width * detectedCloud.height - 1].x << "};" << std::endl;
points << "double[] y = {" ;
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1;i++)
{
	points << KukaPts[i].y << ",";       
 } 
points << KukaPts[detectedCloud.width * detectedCloud.height - 1].y << "};" << std::endl;
points << "double[] z = {" ;
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1;i++)
{
	points << KukaPts[i].z << ",";       
 } 
points << KukaPts[detectedCloud.width * detectedCloud.height - 1].z << "};" << std::endl;

//for matlab
points << " x = [";
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1;i++)
{
	points << KukaPts[i].x << ",";       
 } 
points << KukaPts[detectedCloud.width * detectedCloud.height - 1].x << "];" << std::endl;
points << " y = [" ;
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1;i++)
{
	points << KukaPts[i].y << ",";       
 } 
points <<  KukaPts[detectedCloud.width * detectedCloud.height - 1].y << "];" << std::endl;
points << " z = [" ;
for (int i = 0; i< detectedCloud.width * detectedCloud.height - 1 ;i++)
{
	points << KukaPts[i].z << ",";       
 } 
points << KukaPts[detectedCloud.width * detectedCloud.height - 1].z << "];" << std::endl;



points.close();


pcl::io::savePLYFile("KukaTransform.ply", testCloud,true);
}



int main(int argc, char *argv[])
{
  	
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "cpu")
    {
      if(!pipeline)
        pipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
 
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  Viewer viewer;
  viewer.initialize();
#endif
 cv::Mat bgrMat;

int counter  = 0;

  while(counter < 1)
  {
    //defining frames
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    //defining Mats
    cv::Mat color(rgb->height, rgb->width, CV_8UC4, rgb->data);	
    cv::Mat mDepth(depth->height, depth->width, CV_32FC1, depth->data);
    mDepth  / 4500.0f;
    cv::imwrite("/home/kinect2/Dropbox/data/Trial36/Depth.jpg", mDepth);
    cv::imwrite("/home/kinect2/Dropbox/data/Trial36/Before.jpg",color);
    
    //registration
    registration->apply(rgb, depth, &undistorted, &registered);
    
    //detecting stain
    pcl::PointCloud<pcl::PointXYZRGBA> thisCloud = PointCloud(&undistorted, &registered);
    cv::Point bboxPoints[5];
    redDetection(bboxPoints);
    
    //outputting some data on screen 
    int Transform = (thisCloud.height*bboxPoints[4].x) + (bboxPoints[4].y);
    std::stringstream ss,ss1,ss2;
    ss << thisCloud.points[Transform].x;
    std::string str1 = ss.str();
    ss1 << thisCloud.points[Transform].y;
    std::string str2 = ss1.str();
    ss2 << thisCloud.points[Transform].z;
    std::string str3 = ss2.str();
    std::string pointText = (str1) + ","+  (str2) +  ","+  (str3);
    cv::putText(AllignedRGB,pointText, cv::Point(bboxPoints[4].x,bboxPoints[4].y),
    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
    std::cout << thisCloud.points[Transform].x << " , " << thisCloud.points[Transform].y << ", " << thisCloud.points[Transform].z << std::endl;
    cv::circle(AllignedRGB, cv::Point(bboxPoints[4].x,bboxPoints[4].y),1,cv::Scalar(255,0,0),5,8,0);
    cv::rectangle(AllignedRGB,bboxPoints[0],bboxPoints[3],cv::Scalar(0,0,0),1,8,0); //tL and Br
    
    //showing the detection image
    cv::imshow("Calib",AllignedRGB); 
    cv::waitKey(0);
    
    //exporting data and transforming points
    pcl::PointCloud<pcl::PointXYZRGBA> redCloud = SegmentAndTransform( bboxPoints, thisCloud);
    transformKinect2Kuka(redCloud);

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
   //viewer.addFrame("RGB", rgb);
   //viewer.addFrame("ir", ir);
   //viewer.addFrame("depth", depth);
   //viewer.addFrame("registered", &registered);

   
    protonect_shutdown = viewer.render();
#else
    protonect_shutdown = true;
#endif

    listener.release(frames);

   counter++;
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }
	

//  cv::imshow("Color Image", bgra);
  //cv::waitKey(0);
  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete registration;

  return 0;
}
