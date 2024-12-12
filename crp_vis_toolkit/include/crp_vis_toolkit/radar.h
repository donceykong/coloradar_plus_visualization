#ifndef CRP_VIS_TOOLKIT_RADAR_H_
#define CRP_VIS_TOOLKIT_RADAR_H_

#pragma once

#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <crp_vis_toolkit/pose.h>  // For the Pose struct

namespace Radar {

    struct HeatmapParams {
        int minRadarRange;
        float intensityThreshold;
        int numAzimuthBins;
        int numElevationBins;
        int numRangeBins;
        float range_bin_width;
        std::vector<float> azimuthBins;
        std::vector<float> elevationBins;
    };

    // Set heatmap parameters
    void setHeatmapParams(HeatmapParams& params);

    // Convert polar to Cartesian coordinates
    Eigen::Vector3f polarToCartesian(int r_bin, int az_bin, int el_bin, const HeatmapParams& params);

    // Precompute heatmap points
    pcl::PointCloud<pcl::PointXYZI>::Ptr precomputeHeatmapPoints(const HeatmapParams& params);

    // Assign heatmap data to the precomputed point cloud
    bool assignHeatmapData(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl, const std::vector<float>& radarHeatmapData, const HeatmapParams& params);

    // Load radar point cloud from heatmap
    bool setCurrentCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, HeatmapParams& params);

}; // namespace Radar

#endif // CRP_VIS_TOOLKIT_RADAR_H_
