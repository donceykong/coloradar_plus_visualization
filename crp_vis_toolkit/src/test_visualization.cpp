#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iomanip>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageMapper3D.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <crp_vis_toolkit/pointcloud.h>
#include <crp_vis_toolkit/pose.h>

int currentFileIndex = 1;  // Start with the first point cloud file
const int FileInc = 10;
double cloudTimestamp;

const std::string camera_config_path = CAMERA_CONFIG_PATH;
const std::string dataset_path = DATASET_PATH;

int run = 0;
std::string env = "c4c_garage";
std::string env_dir = dataset_path + env + "_run" +  std::to_string(run) + "/";

std::string groundtruth_dir = env_dir + "groundtruth/";
std::string lidar_dir = env_dir + "lidar/";
std::string lidar_timestamp_path = lidar_dir + "timestamps.txt";


PointCloud pointCloudHandler;  // Instance of PointCloud class
std::map<double, int> cloudIndexTimestampMap;  // Lidar timestamps

std::map<double, Pose> poseMap;  // Ground truth poses

// Initialize PCL Visualizer
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr accumCloud(new pcl::PointCloud<pcl::PointXYZI>);

// Images
vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
vtkSmartPointer<vtkImageActor> imageActor = vtkSmartPointer<vtkImageActor>::New();  // Create an Image Actor to display the image

void updatePointCloudDisplay(int currentFileIndex) {
    viewer->removePointCloud("current_scan");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "current_scan");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "current_scan");

    std::string accum_cloud_name = "accumulated_ds_cloud_" + std::to_string(currentFileIndex);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> accumulated_ds_intensity_distribution(downsampledCloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(downsampledCloud, accumulated_ds_intensity_distribution, accum_cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, accum_cloud_name);

    imageActor->GetMapper()->SetInputData(vtkImage);
}

bool setImage(int fileIndex) {
    int run = 0;
    std::string env = "c4c_garage";
    std::string env_dir = dataset_path + env + "_run" + std::to_string(run) + "/";
    std::string camera_dir = env_dir + "camera/";
    std::string filename = camera_dir + "images/rgb/camera_rgb_image_" + std::to_string(fileIndex) + ".bin";

    // Binary file parameters
    int width = 1280;       // Image width
    int height = 800;       // Image height
    int numChannels = 3;    // Number of channels (RGB image)

    // Read binary file
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return -1;
    }

    // Allocate memory for the image data
    size_t dataSize = width * height * numChannels;
    std::vector<unsigned char> imageData(dataSize);
    file.read(reinterpret_cast<char*>(imageData.data()), dataSize);
    file.close();

    if (!file) {
        std::cerr << "Error: Failed to read the complete file!" << std::endl;
        return -1;
    }

    // Wrap the binary data into a vtkImageData object
    vtkImage->SetDimensions(width, height, 1);
    vtkImage->AllocateScalars(VTK_UNSIGNED_CHAR, numChannels); // Assuming 8-bit channels

    // Flip the image vertically
    for (int row = 0; row < height / 2; ++row) {
        for (int col = 0; col < width * numChannels; ++col) {
            std::swap(
                imageData[row * width * numChannels + col],
                imageData[(height - row - 1) * width * numChannels + col]
            );
        }
    }

    // Copy the data into vtkImageData
    unsigned char* vtkImageBuffer = static_cast<unsigned char*>(vtkImage->GetScalarPointer());
    std::memcpy(vtkImageBuffer, imageData.data(), dataSize);

    return true;
}

// Handles keyboard events for navigation
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    std::cout << std::fixed << std::setprecision(12);
    if (event.keyDown()) {
        if (event.getKeySym() == "Up") {
            currentFileIndex += FileInc;

            std::string pointcloudPath = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
            if (pointCloudHandler.setCurrentCloud(pointcloudPath, cloud) &&
                pointCloudHandler.setCurrentTimestamp(currentFileIndex, cloudIndexTimestampMap, cloudTimestamp) &&
                setImage(currentFileIndex)
                ) {

                Pose interpolatedPose = PoseHandler::interpolatePose(cloudTimestamp, poseMap);
                pointCloudHandler.transform(cloud, interpolatedPose);
                pointCloudHandler.downsample(cloud, downsampledCloud, 0.1f);

                updatePointCloudDisplay(currentFileIndex);

                Eigen::Affine3f transform = PoseHandler::poseToAffine3f(interpolatedPose);
                viewer->removeCoordinateSystem("base_frame");
                viewer->addCoordinateSystem(3.0, transform, "base_frame");

                std::cout << "Loaded file: lidar_pointcloud_" << currentFileIndex << ".bin with timestamp: " << cloudTimestamp << std::endl;
            }
        } else if (event.getKeySym() == "Down") {
            if (currentFileIndex > 1) {
                currentFileIndex -= FileInc;
                
                std::string pointcloudPath = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
                if (pointCloudHandler.setCurrentCloud(pointcloudPath, cloud) &&
                    pointCloudHandler.setCurrentTimestamp(currentFileIndex, cloudIndexTimestampMap, cloudTimestamp)) {

                    Pose interpolatedPose = PoseHandler::interpolatePose(cloudTimestamp, poseMap);
                    pointCloudHandler.transform(cloud, interpolatedPose);
                    pointCloudHandler.downsample(cloud, downsampledCloud, 0.1f);

                    updatePointCloudDisplay(currentFileIndex);

                    Eigen::Affine3f transform = PoseHandler::poseToAffine3f(interpolatedPose);
                    viewer->removeCoordinateSystem("base_frame");
                    viewer->addCoordinateSystem(3.0, transform, "base_frame");

                    std::cout << "Loaded file: lidar_pointcloud_" << currentFileIndex << ".bin with timestamp: " << cloudTimestamp << std::endl;
                }
            }
        }

        if (event.getKeySym() == "s") {
            viewer->saveCameraParameters(camera_config_path);
            std::cout << "Camera parameters saved." << std::endl;
        } else if (event.getKeySym() == "l") {
            viewer->loadCameraParameters(camera_config_path);
            std::cout << "Camera parameters loaded." << std::endl;
        }
    }
}

int main() {
    if (PoseHandler::loadPoses(groundtruth_dir, poseMap)) {
        std::cout << "Successfully loaded " << poseMap.size() << " poses." << std::endl;
    } else {
        std::cerr << "Failed to load poses." << std::endl;
        return 1;
    }

    if (pointCloudHandler.loadTimestamps(lidar_timestamp_path, cloudIndexTimestampMap)) {
        std::cout << "Successfully loaded " << cloudIndexTimestampMap.size() << " lidar timestamps." << std::endl;
    } else {
        std::cerr << "Failed to load lidar timestamps." << std::endl;
        return 1;
    }

    std::string pointcloudPath = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(currentFileIndex) + ".bin";
    if (!(pointCloudHandler.setCurrentCloud(pointcloudPath, cloud) &&
          pointCloudHandler.setCurrentTimestamp(currentFileIndex, cloudIndexTimestampMap, cloudTimestamp))) {
        std::cerr << "Failed to load initial point cloud." << std::endl;
        return 1;
    }

    if (!setImage(currentFileIndex)) {
        return 1;
    }
    
    pointCloudHandler.downsample(cloud, downsampledCloud, 0.1f);

    updatePointCloudDisplay(currentFileIndex);

    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(2.0);
    viewer->initCameraParameters();

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());


    // For image data
    imageActor->GetMapper()->SetInputData(vtkImage);
    vtkSmartPointer<vtkRenderer> imageRenderer = vtkSmartPointer<vtkRenderer>::New();   // Create a Renderer for the Image
    vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow(); // Get the VTK Render Window from the PCL Visualizer
    imageRenderer->AddActor(imageActor);
    renderWindow->AddRenderer(imageRenderer); // Add the image renderer to the render window

    // Adjust viewport for PCL and image
    viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->SetViewport(0.5, 0.0, 1.0, 1.0); // PCL on the right
    imageRenderer->SetViewport(0.0, 0.0, 0.5, 1.0); // Image on the left


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}
