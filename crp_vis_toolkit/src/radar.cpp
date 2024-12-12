#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Geometry>

#include <crp_vis_toolkit/radar.h>

// TODO: Use calib file to set below 
void Radar::setHeatmapParams(HeatmapParams& params) {
    params.minRadarRange = 10;
    params.intensityThreshold = 0.002;
    params.numAzimuthBins = 128;
    params.numElevationBins = 32;
    params.numRangeBins = 128;
    params.range_bin_width = 0.0592943951488;

    params.azimuthBins = {-1.3669834137, -1.30120515823, -1.24833345413, -1.20276021957, -1.16203439236, -1.12482368946, 
                           -1.0903083086, -1.0579417944, -1.02733767033, -0.99821138382, -0.970345854759, -0.943571031094, 
                           -0.917750835419, -0.892774283886, -0.868549585342, -0.844999492168, -0.822058618069, -0.799670696259, 
                           -0.777787148952, -0.756365478039, -0.73536837101, -0.714762806892, -0.694519400597, -0.674611747265, 
                           -0.655016183853, -0.635711312294, -0.616677582264, -0.597897350788, -0.579354345798, -0.561033666134, 
                           -0.542921543121, -0.525005221367, -0.50727301836, -0.489713847637, -0.472317516804, -0.455074489117, 
                           -0.437975734472, -0.421012848616, -0.404177844524, -0.387463241816, -0.37086185813, -0.354366987944, 
                           -0.337972193956, -0.321671336889, -0.305458545685, -0.289328247309, -0.273275017738, -0.257293701172, 
                           -0.241379350424, -0.225527092814, -0.209732279181, -0.193990409374, -0.178297072649, -0.162647992373, 
                           -0.147039011121, -0.131466031075, -0.115925058722, -0.100412160158, -0.0849234685302, -0.0694551765919, 
                           -0.0540035180748, -0.0385647527874, -0.0231351815164, -0.0077111152932, 0.0077111152932, 0.0231351815164, 
                           0.0385647527874, 0.0540035180748, 0.0694551765919, 0.0849234685302, 0.100412160158, 0.115925058722, 
                           0.131466031075, 0.147039011121, 0.162647992373, 0.178297072649, 0.193990409374, 0.209732279181, 
                           0.225527092814, 0.241379350424, 0.257293701172, 0.273275017738, 0.289328247309, 0.305458545685, 
                           0.321671336889, 0.337972193956, 0.354366987944, 0.37086185813, 0.387463241816, 0.404177844524, 
                           0.421012848616, 0.437975734472, 0.455074489117, 0.472317516804, 0.489713847637, 0.50727301836, 
                           0.525005221367, 0.542921543121, 0.561033666134, 0.579354345798, 0.597897350788, 0.616677582264, 
                           0.635711312294, 0.655016183853, 0.674611747265, 0.694519400597, 0.714762806892, 0.73536837101, 
                           0.756365478039, 0.777787148952, 0.799670696259, 0.822058618069, 0.844999492168, 0.868549585342, 
                           0.892774283886, 0.917750835419, 0.943571031094, 0.970345854759, 0.99821138382, 1.02733767033, 
                           1.0579417944, 1.0903083086, 1.12482368946, 1.16203439236, 1.20276021957, 1.24833345413, 
                           1.30120515823, 1.3669834137};

    params.elevationBins = {-1.27362585068, -1.10726797581, -0.98413258791, -0.880573093891, -0.788668692112, -0.704597592354, 
                             -0.626161694527, -0.551952362061, -0.480995953083, -0.412579834461, -0.346157461405, -0.281292319298, 
                             -0.21762278676, -0.154838755727, -0.0926650241017, -0.0308490488678, 0.0308490488678, 0.0926650241017, 
                             0.154838755727, 0.21762278676, 0.281292319298, 0.346157461405, 0.412579834461, 0.480995953083, 
                             0.551952362061, 0.626161694527, 0.704597592354, 0.788668692112, 0.880573093891, 0.98413258791, 
                             1.10726797581, 1.27362585068};
}

Eigen::Vector3f Radar::polarToCartesian(int r_bin, int az_bin, int el_bin, const Radar::HeatmapParams& params) {
    Eigen::Vector3f point;

    float range = r_bin * params.range_bin_width;
    float cos_el = std::cos(params.elevationBins[el_bin]);
    float sin_el = std::sin(params.elevationBins[el_bin]);
    float cos_az = std::cos(params.azimuthBins[az_bin]);
    float sin_az = std::sin(params.azimuthBins[az_bin]);

    point.x() = range * cos_el * cos_az;
    point.y() = range * cos_el * sin_az;
    point.z() = range * sin_el;

    return point;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Radar::precomputeHeatmapPoints(const Radar::HeatmapParams& params) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZI>);

    for (int rangeIdx = 0; rangeIdx < params.numRangeBins - params.minRadarRange; ++rangeIdx) {
        for (int azIdx = 0; azIdx < params.numAzimuthBins; ++azIdx) {
            for (int elIdx = 0; elIdx < params.numElevationBins; ++elIdx) {
                pcl::PointXYZI point;
                Eigen::Vector3f cartesian = polarToCartesian(rangeIdx + params.minRadarRange, azIdx, elIdx, params);
                point.x = cartesian.x();
                point.y = cartesian.y();
                point.z = cartesian.z();
                point.intensity = 0.0f; // Placeholder
                pcl->points.push_back(point);
            }
        }
    }

    pcl->width = pcl->points.size();
    pcl->height = 1;
    pcl->is_dense = true;

    return pcl;
}

bool Radar::assignHeatmapData(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl, const std::vector<float>& radarHeatmapData, const Radar::HeatmapParams& params) {
    size_t expectedSize = params.numElevationBins * params.numAzimuthBins * params.numRangeBins * 2;

    if (radarHeatmapData.size() != expectedSize) {
        std::cerr << "Error: Radar heatmap data size mismatch!" << std::endl;
        return false;
    }

    if (pcl->points.size() != params.numElevationBins * params.numAzimuthBins * (params.numRangeBins - params.minRadarRange)) {
        std::cerr << "Error: Point cloud size mismatch with heatmap dimensions!" << std::endl;
        return false;
    }

    float intensityMin = std::numeric_limits<float>::max();
    float intensityMax = std::numeric_limits<float>::lowest();

    // Step 1: Find intensityMin and intensityMax
    for (size_t i = 0; i < radarHeatmapData.size(); i += 2) {
        float intensity = radarHeatmapData[i];
        intensityMin = std::min(intensityMin, intensity);
        intensityMax = std::max(intensityMax, intensity);
    }

    if (intensityMin == intensityMax) {
        std::cerr << "Error: All intensity values are the same. Cannot normalize." << std::endl;
        return false;
    }

    // Step 2: Update intensities in pcl->points
    size_t idx = 0;
    float total_intensity = 0.0f;
    size_t valid_points = 0;
    float validIntensityMin = std::numeric_limits<float>::max();
    float validIntensityMax = std::numeric_limits<float>::lowest();
    for (int rangeIdx = 0; rangeIdx < params.numRangeBins - params.minRadarRange; ++rangeIdx) {
        for (int azIdx = 0; azIdx < params.numAzimuthBins; ++azIdx) {
            for (int elIdx = 0; elIdx < params.numElevationBins; ++elIdx, ++idx) {
                if (idx >= pcl->points.size()) {
                    std::cerr << "Error: Index out of bounds for point cloud." << std::endl;
                    return false;
                }

                size_t heatmapIdx = ((elIdx * params.numAzimuthBins + azIdx) * params.numRangeBins + (rangeIdx + params.minRadarRange)) * 2;
                float raw_intensity = radarHeatmapData[heatmapIdx];
                float normalized_intensity = (raw_intensity - intensityMin) / (intensityMax - intensityMin);

                if (normalized_intensity >= params.intensityThreshold and abs(pcl->points[idx].z) < 0.1) {
                    pcl->points[idx].intensity = raw_intensity; //normalized_intensity;
                    // pcl->points[idx].z = 0.0f; // Set z to 0 for all points
                    total_intensity += normalized_intensity;
                    validIntensityMin = std::min(validIntensityMin, raw_intensity);
                    validIntensityMax = std::max(validIntensityMax, raw_intensity);
                    valid_points++;
                } else {
                    pcl->points[idx].intensity = -1.0f; // Mark as invalid
                    // pcl->points[idx].z = 0.0f; // Set z to 0 for all points
                }
            }
        }
    }

    if (valid_points == 0) {
        std::cerr << "No points meet the intensity threshold after normalization!" << std::endl;
        return false;
    }

    float average_intensity = total_intensity / valid_points;
    std::cout << "Average Intensity = " << average_intensity << std::endl;

    pcl->points.erase(
        std::remove_if(pcl->points.begin(), pcl->points.end(), [](const pcl::PointXYZI& point) {
            return point.intensity < 0.0f; // Remove invalid points
        }),
        pcl->points.end()
    );

    // Step 3: Normalize valid intensities
    for (auto& point : pcl->points) {
        if (point.intensity >= 0.0f) { // Normalize only valid points
            point.intensity = (point.intensity - validIntensityMin) / (validIntensityMax - validIntensityMin);
        }
    }

    // Set cloud dimensions
    pcl->width = pcl->points.size();
    pcl->height = 1;

    return true;
}


bool Radar::setCurrentCloud(const std::string& filename, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                            Radar::HeatmapParams& params) {
    std::ifstream inputFile(filename, std::ios::binary);
    if (!inputFile) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    inputFile.seekg(0, std::ios::end);
    std::streamsize fileSize = inputFile.tellg();
    inputFile.seekg(0, std::ios::beg);

    std::vector<float> radarHeatmapData(fileSize / sizeof(float));
    if (!inputFile.read(reinterpret_cast<char*>(radarHeatmapData.data()), fileSize)) {
        std::cerr << "Error: Could not read the entire file " << filename << std::endl;
        return false;
    }
    inputFile.close();

    // Assign heatmap data to the precomputed points
    if (!Radar::assignHeatmapData(cloud, radarHeatmapData, params)) {
        std::cerr << "Error: Failed to assign heatmap data" << std::endl;
        return false;
    }

    return true;
}