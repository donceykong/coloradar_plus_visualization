#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

int currentFileIndex = 1;  // Start with the first point cloud file
int FileInc = 10;  // Start with the first point cloud file
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

bool loadPointCloud(int fileIndex, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    int run = 0;
    std::string env = "c4c_garage";
    std::string env_dir = env + "_run" +  std::to_string(run) + "/";
    std::string lidar_dir = env_dir + "lidar/";
    std::string filename = lidar_dir + "pointclouds/lidar_pointcloud_" + std::to_string(fileIndex) + ".bin";
    std::ifstream inputFile(filename, std::ios::binary);

    if (!inputFile) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return false;
    }

    // Move to the end of the file to determine the file size
    inputFile.seekg(0, std::ios::end);
    std::streamsize fileSize = inputFile.tellg();
    inputFile.seekg(0, std::ios::beg);

    // Calculate the number of points (each point has 4 floats: X, Y, Z, I)
    size_t numPoints = fileSize / (4 * sizeof(float));
    std::vector<float> pointCloudData(numPoints * 4);

    // Read the data into the vector
    if (!inputFile.read(reinterpret_cast<char*>(pointCloudData.data()), fileSize)) {
        std::cerr << "Error: Could not read the file!" << std::endl;
        return false;
    }

    inputFile.close();
    
    // Clear the existing point cloud and populate it with new data
    cloud->clear();
    for (size_t i = 0; i < pointCloudData.size(); i += 4) {
        pcl::PointXYZI point;
        point.x = pointCloudData[i];
        point.y = pointCloudData[i + 1];
        point.z = pointCloudData[i + 2];
        point.intensity = pointCloudData[i + 3];
        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return true;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);

    if (event.keyDown()) {
        if (event.getKeySym() == "Up") {
            currentFileIndex += FileInc;
            if (loadPointCloud(currentFileIndex, cloud)) {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
                viewer->updatePointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "sample cloud");
                std::cout << "Loaded file: lidar_pointcloud_" << currentFileIndex << ".bin" << std::endl;
            }
        } else if (event.getKeySym() == "Down") {
            if (currentFileIndex > 1) {
                currentFileIndex -= FileInc;
                if (loadPointCloud(currentFileIndex, cloud)) {
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
                    viewer->updatePointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "sample cloud");
                    std::cout << "Loaded file: lidar_pointcloud_" << currentFileIndex << ".bin" << std::endl;
                }
            }
        }

        if (event.getKeySym() == "s") {
            viewer->saveCameraParameters("camera_params.txt");
            std::cout << "Camera parameters saved." << std::endl;
        } else if (event.getKeySym() == "l") {
            viewer->loadCameraParameters("camera_params.txt");
            std::cout << "Camera parameters loaded." << std::endl;
        }
    }
}

int main() {
    // Load the initial point cloud
    if (!loadPointCloud(currentFileIndex, cloud)) {
        return 1;  // Exit if the initial point cloud could not be loaded
    }

    // Create a PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);

    // Add point cloud with intensity colormap
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Register keyboard callback
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    // Main loop for visualization
    while (!viewer->wasStopped()) {
        viewer->spin();
    }

    return 0;
}
