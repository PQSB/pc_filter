#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

#include "pc_filter/pc_filter_lib.hpp"

// ros2 includes
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

// pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <cxxopts.hpp>

// De las funciones temporales
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

// #include <visualization_msgs/msg/marker_array.hpp>
// #include <tf2/LinearMath/Quaternion.h>

constexpr double MAX_DIST_DEFAULT = -1.0;

const std::unordered_map<std::string, int> semantic_kitti_classes = {
    // Static
    {"unlabeled", 0}, {"outlier", 1}, {"car", 10}, {"bicycle", 11}, {"bus", 13}, {"motorcycle", 15},
    {"on-rails", 16}, {"truck", 17}, {"other-vehicle", 20},

    // Static
    {"person", 30},{"bicyclist", 31},{"motorcyclist", 32},

    // Terrain and road
    {"road", 40}, {"parking", 44}, {"sidewalk", 48}, {"other-ground", 49},

    // Vegetation and structures
    {"building", 50}, {"fence", 51}, {"other-structure", 52}, {"lane-marking", 60},
    {"vegetation", 70}, {"trunk", 71}, {"terrain", 72},

    // Objects
    {"pole", 80}, {"traffic-sign", 81}, {"other-object", 99},

    // Dynamic classes
    {"moving-car", 252}, {"moving-bicyclist", 253}, {"moving-person", 254},
    {"moving-motorcyclist", 255}, {"moving-on-rails", 256}, {"moving-bus", 257},
    {"moving-truck", 258}, {"moving-other-vehicle", 259}
};

namespace fs = std::filesystem;

struct FovCalibration
{
    Eigen::Matrix<float, 3, 4> P2;
    Eigen::Matrix4f Tr_velo_to_cam;

    int image_width{};
    int image_height{};
};

struct Config
{
    // Neccesary
    fs::path pc_dir;
    fs::path rosbag_out;
    fs::path ts_file;

    // Input cloud topic
    std::string pc_topic;
    bool export_input_pc = false;

    // fov filter
    fs::path fov_file;
    bool use_fov_filter = false;
    FovCalibration fov_params;

    // Images
    bool use_images = false;
    fs::path img_dir;
    std::string img_topic;

    // 3D-MOOD detections
    bool use_detections = false;
    fs::path det_dir;
    double det_score{};
    double det_max_dist = MAX_DIST_DEFAULT;
    std::string det_topic;
    std::unordered_set<std::string> det_classes;

    // Semantic segmentation
    bool use_seg = false;
    fs::path seg_dir;
    std::string seg_topic;
    std::unordered_set<std::string> seg_classes;
    double seg_max_dist = MAX_DIST_DEFAULT;

    bool no_confirm;

    // To export just the filtered point clouds directly to a directory
    bool only_clouds;
    fs::path clouds_out_dir;
};

pcl::PointCloud<pcl::PointXYZI>::Ptr
semantic_seg_filter(
  const std::unordered_set<std::string>& classes2filter,
  double seg_max_dist, 
  pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  fs::path lbl_path,
  const std::vector<int64_t> & original_indices)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Open lbl file (.label)
    std::ifstream label_file(lbl_path, std::ios::binary);
    if (!label_file.is_open()) {
        throw std::runtime_error("[ERROR] Cannot open semantic segmentation lbl file: " + lbl_path.string());
    }

    std::unordered_set<uint16_t> numeric_ids_to_remove;

    // Change classes to filter from string to the numeric id
    for (const std::string& name : classes2filter) {
        auto it = semantic_kitti_classes.find(name);
        if (it != semantic_kitti_classes.end()) {
            numeric_ids_to_remove.insert(it->second);
        } else {
            throw std::runtime_error("[ERROR] ID " + name + "is not in the provided ones");
        }
    }

    std::vector<uint16_t> all_labels;
    uint32_t temp_label; // lbl file size

    while (label_file.read(reinterpret_cast<char*>(&temp_label), sizeof(uint32_t))) {
        // Extract just the class id bits
        all_labels.push_back(temp_label & 0xFFFF);
    }

    filtered_cloud->reserve(input_cloud->size());

    double max_dist_sq = seg_max_dist * seg_max_dist;
    double dist_sq;

    for (size_t i = 0; i < input_cloud->size(); ++i) {
        // Recover the original index the point had before the FOV filter
        int orig_idx = original_indices[i];

        if (seg_max_dist != MAX_DIST_DEFAULT){
            const auto & point = input_cloud->points[i];
            // To avoid using sqrt
            dist_sq = (point.x * point.x) + (point.y * point.y);
            if (dist_sq > max_dist_sq) {
                filtered_cloud->push_back(input_cloud->points[i]);
                continue;
            }
        }

        if (orig_idx < 0 || orig_idx >= static_cast<int>(all_labels.size())) {
            throw std::runtime_error("[ERROR] Index " + std::to_string(orig_idx) + "out of lbl labels file range");
        }

        uint16_t semantic_class = all_labels[orig_idx];

        // Check if the point is valid and add it in case it is
        if (numeric_ids_to_remove.find(semantic_class) == numeric_ids_to_remove.end()) {
            filtered_cloud->push_back(input_cloud->points[i]);
        }
    }

    return filtered_cloud;
}

FovCalibration
getFovFromFile(const fs::path& fov_file_path)
{
    std::ifstream fov_file(fov_file_path);
    if (!fov_file.is_open()) {
        throw std::runtime_error(
            "[ERROR] Can't open fov filter calib file: " + fov_file_path.string());
    }

    FovCalibration calib;

    // Read images width and height
    if (!(fov_file >> calib.image_width >> calib.image_height)) {
        throw std::runtime_error(
            "[ERROR] File doesn't contains image dimensions: " + fov_file_path.string());
    }

    // Read P2 (3x4)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            fov_file >> calib.P2(r, c);
        }
    }

    // Read Tr (3x4 - Formato KITTI omitiendo la última fila)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            fov_file >> calib.Tr_velo_to_cam(r, c);
        }
    }

    if (!fov_file) {
        throw std::runtime_error(
            "[ERROR] The file was terminated prematurely: " + fov_file_path.string());
    }

    // Fill Tr last row since it's 4x4
    calib.Tr_velo_to_cam(3, 0) = 0.0f;
    calib.Tr_velo_to_cam(3, 1) = 0.0f;
    calib.Tr_velo_to_cam(3, 2) = 0.0f;
    calib.Tr_velo_to_cam(3, 3) = 1.0f;

    return calib;
}

std::vector<Detection>
loadDetections(
    const fs::path & det_file,
    double det_score_threshold,
    double det_max_dist,
    const std::unordered_set<std::string>& allowed_classes)
{
    std::ifstream file(det_file);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open detections file: " + det_file.string());
    }

    std::vector<Detection> detections;

    std::string line;

    double det_max_dist_sq = det_max_dist * det_max_dist;

    while (std::getline(file, line))
    {
        if (line.empty()) {continue;}

        std::stringstream ss(line);

        Detection det;

        if (!(ss >> det.category >> det.x >> det.y >> det.z >> det.w
            >> det.l >> det.h >> det.ry >> det.score)) {continue;}

        if (det.score < det_score_threshold) {continue;}

        // Check if the filter is active
        if (det_max_dist != MAX_DIST_DEFAULT){
            // To avoid using sqrt
            double dist_sq = (det.x * det.x) + (det.y * det.y);
            if (dist_sq > det_max_dist_sq) {continue;}
        }

        if (!allowed_classes.empty() &&
            allowed_classes.find(det.category) == allowed_classes.end()) {continue;}

        detections.push_back(std::move(det));
    }

    return detections;
}

/*
FUNCIÓN FILTRADO TEMPORAL A FALTA DE MODIFICAR LA DE LA LIBRERÍA
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr
filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<Detection>& detections)
{
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    if (!cloud || cloud->empty())
    {
        return filtered_cloud;
    }

    if (detections.empty())
    {
        return cloud;
    }

    pcl::IndicesPtr indices_to_keep(new std::vector<int>);

    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);

    for (const auto& det : detections)
    {
        if (det.l <= 0.0f || det.w <= 0.0f || det.h <= 0.0f){continue;}

        Eigen::Vector4f min_pt(
            -det.l / 2.0f, -det.w / 2.0f, -det.h / 2.0f, 1.0f);

        Eigen::Vector4f max_pt(
            det.l / 2.0f, det.w / 2.0f, det.h / 2.0f,1.0f);

        Eigen::Vector3f translation(det.x, det.y, det.z);
        Eigen::Vector3f rotation(0.0f, 0.0f, det.ry);

        crop.setMin(min_pt);
        crop.setMax(max_pt);

        crop.setTranslation(translation);
        crop.setRotation(rotation);

        std::vector<int> inside_indices;

        crop.filter(inside_indices);

        indices_to_keep->insert(
            indices_to_keep->end(), inside_indices.begin(), inside_indices.end());
    }

    // If there are no points that fall within the detections, return the entire cloud
    if (indices_to_keep->empty())
    {
        return cloud;
    }

    std::sort(indices_to_keep->begin(), indices_to_keep->end());

    indices_to_keep->erase(
        std::unique(
            indices_to_keep->begin(),
            indices_to_keep->end()),
        indices_to_keep->end());

    pcl::ExtractIndices<pcl::PointXYZI> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_keep);

    extract.setNegative(true);

    extract.filter(*filtered_cloud);

    return filtered_cloud;
}

std::vector<int64_t>
loadTimestamps(const fs::path& ts_file)
{

    std::ifstream file(ts_file);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open timestamp file: " + ts_file.string());
    }

    std::vector<int64_t> timestamps_ns;

    std::string line;
    double timestamp_sec;

    while (std::getline(file, line))
    {
        // Ignore emepty lines
        if (line.empty()) continue; 

        // Transform text into double
        timestamp_sec = std::stod(line);

        // Get the time in nanoseconds
        timestamps_ns.push_back(static_cast<int64_t>(timestamp_sec * 1e9));
    }

    // In case the timestamps file is empty
    if (timestamps_ns.empty())
    {
        throw std::runtime_error("Timestamp file is empty: " + ts_file.string());
    }

    return timestamps_ns;
}

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<int64_t>>
loadPointCloudXYZI(
    const fs::path& filepath, const bool use_fov, const FovCalibration& calib)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // Stores the index of the original position of each point within the binary file (.bin) before applying any filters.
    std::vector<int64_t> correspondence;

    // Check if the file exists
    if (!fs::exists(filepath)) {
        std::cerr << "[WARN] File does not exist: " << filepath << "\n";
        return {cloud, correspondence};
        // throw std::runtime_error("[ERROR] File does not exist: " + filepath.string());
    }

    // Get the total size of the file
    const size_t file_size = fs::file_size(filepath);
    const size_t point_size = 4 * sizeof(float); // XYZI = 16 bytes

    if (file_size == 0) {
        std::cerr << "[ERROR] Empty file: " << filepath << "\n";
        return {cloud, correspondence};
        // throw std::runtime_error("[ERROR] Empty file: " + filepath.string());
    }

    if (file_size % point_size != 0) {
        std::cerr << "[ERROR] Invalid .bin file size for XYZI format: "
                  << file_size << " bytes\n";
        return {cloud, correspondence};
        // throw std::runtime_error("[ERROR] Invalid .bin file size for XYZI format: " + 
        //                          std::to_string(file_size) + " bytes en " + filepath.string());
    }

    // Calculate the number of points of the point cloud
    const size_t num_points = file_size / point_size;

    std::vector<float> raw(num_points * 4);

    // Open the file
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open file: " << filepath << "\n";
        // return [cloud, correspondence];
        return {cloud, correspondence};
    }

    // comprobar si puedo usar fread
    if (!file.read(reinterpret_cast<char*>(raw.data()), file_size)) {
        std::cerr << "[ERROR] Could not read full file\n";
        return {cloud, correspondence};;
    }

    cloud->points.reserve(num_points);
    correspondence.reserve(num_points);
    // for (size_t i = 0; i < num_points; ++i) {
    //     cloud->points[i].x = raw[i * 4 + 0];
    //     cloud->points[i].y = raw[i * 4 + 1];
    //     cloud->points[i].z = raw[i * 4 + 2];
    //     cloud->points[i].intensity = raw[i * 4 + 3];
    // }

    Eigen::Vector4f X_velo;
    Eigen::Vector4f X_cam;
    Eigen::Vector3f uvw;

    pcl::PointXYZI pt;

    float x, y, z, intensity, u, v;
    for (size_t i = 0; i < num_points; ++i) {
        x = raw[i * 4 + 0];
        y = raw[i * 4 + 1];
        z = raw[i * 4 + 2];
        intensity = raw[i * 4 + 3];

        if (use_fov) {
            X_velo << x, y, z, 1.0f;

            X_cam = calib.Tr_velo_to_cam * X_velo;

            // If point is behind the camera, ignore it
            if (X_cam.z() <= 0.0f) {
                continue;
            }   
        
            // Proyection
            uvw = calib.P2 * X_cam;

            u = uvw.x() / uvw.z();
            v = uvw.y() / uvw.z();

            if (u < 0 || u >= calib.image_width ||
                v < 0 || v >= calib.image_height)
            {
                continue;
            }

        }

        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = intensity;
    
        cloud->points.push_back(pt);
        correspondence.push_back(static_cast<int64_t>(i));
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return {cloud, correspondence};
}

void
print_summary(const Config& cfg)
{
    std::cout << "\n=====================================\n";
    std::cout << "CONFIGURATION SUMMARY\n";
    std::cout << "=====================================\n";

    std::cout << "[OUTPUT MODE]\n";
    if (cfg.only_clouds) {
        std::cout << "  Mode   : DIRECT FILES EXPORT (.bin)\n";
        std::cout << "  output : " << cfg.clouds_out_dir << "\n\n";
    } else {
        std::cout << "  Mode   : ROSBAG EXPORT\n";
        std::cout << "  output : " << cfg.rosbag_out << "\n\n";
    }

    std::cout << "[POINTCLOUDS]\n";
    std::cout << "  dir   : " << cfg.pc_dir << "\n";
    std::cout << "  topic : " << cfg.pc_topic << "\n";
    std::cout << "  timestamps file : " << cfg.ts_file << "\n\n";

    if (cfg.use_fov_filter)
    {
        std::cout << "[FOV FILTER] ENABLED\n";
        std::cout << "  fov calib file : " << cfg.fov_file<< "\n\n";
    }
    else
    {
        std::cout << "[FOV FILTER] DISABLED\n";
        std::cout << "  Reason: fov_filter not provided -> module ignored\n\n";
    }

    if (cfg.use_images)
    {
        std::cout << "[IMAGES] ENABLED\n";
        std::cout << "  dir   : " << cfg.img_dir << "\n";
        std::cout << "  topic : " << cfg.img_topic << "\n\n";
    }
    else
    {
        std::cout << "[IMAGES] DISABLED\n";
        std::cout << "  Reason: img_dir not provided -> module ignored\n\n";
    }

    if (cfg.use_detections)
    {
        std::cout << "[DETECTIONS] ENABLED\n";
        std::cout << "  dir            : " << cfg.det_dir << "\n";
        std::cout << "  score          : " << cfg.det_score << "\n";
        std::cout << "  topic          : " << cfg.det_topic << "\n";
        std::cout << "  max_dist       : " << cfg.det_max_dist << "\n";
        std::cout << "  classes        : ";

        for (const auto& c : cfg.det_classes)
            std::cout << c << " ";

        std::cout << "\n\n";
    }
    else
    {
        std::cout << "[DETECTIONS] DISABLED\n";
        std::cout << "  Reason:\n";
        if (!cfg.det_dir.empty())
        {
            std::cout << "   - det_dir provided but module incomplete\n";
        }
        else
        {
            std::cout << "   - det_dir not provided -> module ignored\n";
        }

        if (!cfg.det_classes.empty() ||
            !cfg.det_topic.empty())
        {
            std::cout
                << "   - partial detection args were ignored\n";
        }
        std::cout << "\n";
    }
    if (cfg.use_seg)
    {
        std::cout << "[SEMANTIC SEGMENTATION] ENABLED\n";
        std::cout << "  dir            : " << cfg.seg_dir << "\n";
        std::cout << "  topic          : " << cfg.seg_topic << "\n";
        std::cout << "  max_dist       : " << cfg.seg_max_dist << "\n";
        std::cout << "  classes        : " ;

        for (const auto& c : cfg.seg_classes)
            std::cout << c << " "; 
    }
    else
    {
        std::cout << "[SEMANTIC SEGMENTATION] DISABLED\n";
        std::cout << "  Reason:\n";
        if (!cfg.seg_dir.empty())
        {
            std::cout << "   - seg_dir provided but module incomplete\n";
        }
        else
        {
            std::cout << "   - seg_dir not provided -> module ignored\n";
        }

        if (!cfg.seg_classes.empty() ||
            !cfg.seg_topic.empty())
        {
            std::cout
                << "   - partial semantic segmentation args were ignored\n";
        }
    }

    std::cout << "\n=====================================\n\n";
}

void
validate(const Config& cfg)
{
    bool has_rosbag = !cfg.rosbag_out.empty();
    bool has_clouds = !cfg.clouds_out_dir.empty();

    if (has_rosbag && has_clouds) {
        throw std::invalid_argument("Conflicting arguments: Cannot provide both 'rosbag_out' and 'clouds_out_dir'");
    }
    if (!has_rosbag && !has_clouds) {
        throw std::invalid_argument("Missing output argument: You must provide either 'rosbag_out' or 'clouds_out_dir'");
    }

    // Needed
    if (!fs::exists(cfg.pc_dir) || !fs::is_directory(cfg.pc_dir))
        throw std::invalid_argument("pc_dir invalid");

    if (cfg.only_clouds) {
        if (cfg.clouds_out_dir.empty()) {
            throw std::invalid_argument("clouds_out_dir must be provided when only_clouds is enabled");
        }

        if (!cfg.ts_file.empty()) {
            throw std::invalid_argument("ts_file is not supported in only_clouds mode (timestamps are only needed for rosbag output)");
        }

        if (fs::exists(cfg.clouds_out_dir) && !fs::is_directory(cfg.clouds_out_dir)) {
            throw std::invalid_argument("clouds_out_dir must be a directory path");
        }


        if (!cfg.use_detections && !cfg.use_seg) {
            throw std::invalid_argument("only_clouds mode requires enabling either detections (use_detections) or segmentation (use_seg)");
        }
        if (cfg.use_images) {
            throw std::invalid_argument("Images (image_dir) are not supported in only_clouds mode");
        }

        // ROS topics are not used since no rosbag is generated
        if (!cfg.pc_topic.empty() || !cfg.img_topic.empty() || !cfg.det_topic.empty() || !cfg.seg_topic.empty()) {
            throw std::invalid_argument("ROS topics (pc_topic, img_topic, det_topic, seg_topic) cannot be used in only_clouds mode");
        }

    } else {
        if (cfg.rosbag_out.empty()) {
            throw std::invalid_argument("rosbag_out must be provided when only_clouds is not enabled");
        }
        if (!fs::exists(cfg.ts_file) || !fs::is_regular_file(cfg.ts_file)) {
            throw std::invalid_argument("ts_file is invalid or does not exist");
        }

        if (!cfg.export_input_pc && !cfg.use_images && !cfg.use_detections && !cfg.use_seg) {
            throw std::invalid_argument("No data modules enabled. At least one must be active to generate output");
        }

        if (cfg.export_input_pc && cfg.pc_topic.empty()) {
            throw std::invalid_argument("pc_topic is necessary to publish input point clouds");
        }
    }

    // Images related
    if (cfg.use_images) {
        if (!fs::exists(cfg.img_dir) || !fs::is_directory(cfg.img_dir))
            throw std::invalid_argument("img_dir is invalid or does not exist");

        if (cfg.img_topic.empty())
            throw std::invalid_argument("img_topic is necessary if images are used");
    }

    // 3D Detections
    if (cfg.use_detections)
    {
        if (!fs::exists(cfg.det_dir) || !fs::is_directory(cfg.det_dir))
            throw std::invalid_argument("det_dir is invalid or does not exist");

        if (cfg.det_score < 0.0 || cfg.det_score > 1.0)
            throw std::invalid_argument("det_score is out of range [0.0, 1.0]");

        if (cfg.det_classes.empty())
            throw std::invalid_argument("Classes to filter must be provided for detections");

        if (!cfg.only_clouds && cfg.det_topic.empty())
            throw std::invalid_argument("det_topic is necessary to publish filtered point clouds");

        if (cfg.det_max_dist != MAX_DIST_DEFAULT && cfg.det_max_dist < 0.0)
            throw std::invalid_argument("det_max_dist must be >= 0.0");
    }

    // Semantic Segmentation
    if (cfg.use_seg)
    {
        if (!fs::exists(cfg.seg_dir) || !fs::is_directory(cfg.seg_dir))
            throw std::invalid_argument("seg_dir is invalid or does not exist");

        if (cfg.seg_classes.empty())
            throw std::invalid_argument("Classes to filter must be provided for segmentation");

        if (!cfg.only_clouds && cfg.seg_topic.empty())
            throw std::invalid_argument("seg_topic is necessary to publish filtered point clouds");

        if (cfg.seg_max_dist != MAX_DIST_DEFAULT && cfg.seg_max_dist < 0.0)
            throw std::invalid_argument("seg_max_dist must be >= 0.0");
    }
}

Config
parse_arguments(int argc, char* argv[])
{
    Config cfg;

    cxxopts::Options options(
        argv[0],
        "Generate a rosbag using exported data");

    options.add_options()

        // Input point clouds
        ("p,pc_dir", "Input point clouds folder path (REQUIRED)", cxxopts::value<fs::path>(cfg.pc_dir))
        ("ts_file", "Timestamps file needed for rosbag messages headers (Required ONLY for rosbag output)", cxxopts::value<fs::path>(cfg.ts_file))
        ("pc_topic", "Input point clouds topic", cxxopts::value<std::string>(cfg.pc_topic))

        ("fov_filter", "Preprocessed file generated with prepare_fov_filter_calib_file.py", cxxopts::value<fs::path>())

        // Output Modes (Mutually exclusive)
        ("rosbag_out", "Output rosbag path (Selects rosbag output mode and cannot be used with only_clouds_out)", cxxopts::value<fs::path>(cfg.rosbag_out))
        ("clouds_out_dir", "Output directory to save just the filtered clouds, no rosbag (Selects direct file output and cannot be used with rosbag_out)", cxxopts::value<fs::path>())

        // Images
        ("i,img_dir", "Input images folder path", cxxopts::value<fs::path>())

        ("img_topic", "Input images topic (Required if img_dir is used)", cxxopts::value<std::string>())

        // Detector filter parameters
        ("det_dir", "Detections folder path [Detector argument]", cxxopts::value<fs::path>())
        ("det_score", "Minimum score threshold [Detector argument]", cxxopts::value<double>())
        ("det_topic", "Filtered point clouds topic name [Detector argument]", cxxopts::value<std::string>())
        ("det_max_dist", "Maximum detection distance [Detector argument]", cxxopts::value<double>())
        ("det_classes", "Classes to filter (c1,c2,c3,...) [Detector argument]", cxxopts::value<std::vector<std::string>>())

        // Semantic segmentation filter parameters
        ("seg_dir", "Semantic segmentation labels folder path [Segmentation argument]", cxxopts::value<fs::path>())
        ("seg_topic", "Filtered point clouds topic name (Required ONLY for rosbag output) [Segmentation argument]", cxxopts::value<std::string>())
        ("seg_classes", "ID of the classes to filter (id1,id2,id3,...) [Segmentation argument]", cxxopts::value<std::vector<std::string>>())
        ("seg_max_dist", "Maximum point distance to filter[Segmentation argument]", cxxopts::value<double>())

        // Skips argument user confirmation (to allow fast executions)
        ("no_confirm", "Skips the user arguments confirmation (NOTE: pure flag; any provided value using '=' will be ignored and treated as true")

        ("h,help", "Show help");

    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
        std::cout << options.help() << "\n";
        exit(EXIT_SUCCESS);
    }

    cfg.no_confirm = result.count("no_confirm") > 0;

    if (result.count("clouds_out_dir")) {
        cfg.only_clouds = true;
        cfg.clouds_out_dir = result["clouds_out_dir"].as<fs::path>();
    }

    if (result.count("pc_topic"))
    {
        cfg.export_input_pc = true;
        cfg.pc_topic = result["pc_topic"].as<std::string>();
    }

    if (result.count("fov_filter"))
    {
        cfg.use_fov_filter = true;
        cfg.fov_file = result["fov_filter"].as<fs::path>();

        // Add check of fov_file before filling FovCalibration
        if (!fs::exists(cfg.fov_file) || !fs::is_regular_file(cfg.fov_file))
        throw std::invalid_argument("fov_file is invalid or does not exist");

        // Fill the FovCalibration struct
        cfg.fov_params = getFovFromFile(cfg.fov_file);
    }

    if (result.count("img_dir"))
    {
        cfg.use_images = true;
        cfg.img_dir = result["img_dir"].as<fs::path>();

        if (result.count("img_topic"))
            cfg.img_topic = result["img_topic"].as<std::string>();
    }

    if (result.count("det_dir"))
    {
        cfg.use_detections = true;
        cfg.det_dir = result["det_dir"].as<fs::path>();

        if (result.count("det_score")) {
            cfg.det_score = result["det_score"].as<double>();
        }
        
        if (result.count("det_topic")) {
            cfg.det_topic = result["det_topic"].as<std::string>();
        }

        if (result.count("det_classes")) {
            auto class_vec = result["det_classes"].as<std::vector<std::string>>();
            cfg.det_classes = std::unordered_set<std::string>(class_vec.begin(), class_vec.end());
        }

        if (result.count("det_max_dist")) {
            cfg.det_max_dist = result["det_max_dist"].as<double>();
        }
    }

    if (result.count("seg_dir"))
    {
        cfg.use_seg = true;
        cfg.seg_dir = result["seg_dir"].as<fs::path>();

        if (result.count("seg_topic")) {
            cfg.seg_topic = result["seg_topic"].as<std::string>();
        }

        if (result.count("seg_classes")) {
            auto class_vec = result["seg_classes"].as<std::vector<std::string>>();
            cfg.seg_classes = std::unordered_set<std::string>(class_vec.begin(), class_vec.end());
        }

        if (result.count("seg_max_dist")) {
            cfg.seg_max_dist = result["seg_max_dist"].as<double>();
        }
    }

    return cfg;
}

std::vector<fs::path>
getPointCloudFiles(const fs::path& pc_dir)
{
    std::vector<fs::path> files;

    for (const auto& entry : fs::directory_iterator(pc_dir))
    {
        if (!entry.is_regular_file())
            continue;

        if (entry.path().extension() != ".bin")
            continue;

        files.push_back(entry.path());
    }

    // Since bin files have sequential names with the same number of digits
    std::sort(
        files.begin(),
        files.end());

    return files;
}

void
createTopics(rosbag2_cpp::Writer& writer, const Config& cfg)
{
    rosbag2_storage::TopicMetadata topic_info;
    topic_info.serialization_format = "cdr";

    if (cfg.export_input_pc) {
        topic_info.name = cfg.pc_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
        // std::cout << "[OK] " << cfg.pc_topic << " topic created succesfully\n";
    }

    if (cfg.use_detections) {
        topic_info.name = cfg.det_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
        // std::cout << "[OK] " << cfg.det_topic << " topic created succesfully\n";

        topic_info.name = "/det_boxes";
        topic_info.type = "visualization_msgs/msg/MarkerArray";
        topic_info.offered_qos_profiles = {};

    }

    if (cfg.use_images) {
        topic_info.name = cfg.img_topic;
        topic_info.type = "sensor_msgs/msg/Image";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
        // std::cout << "[OK] " << cfg.img_topic << " topic created succesfully\n";
    }

    std::cout << std::endl;
}

std::unordered_map<std::string, fs::path>
buildFileIndex(const fs::path& directory)
{
    std::unordered_map<std::string, fs::path> index;

    for (const auto& entry : fs::directory_iterator(directory))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }

        index.emplace(
            entry.path().stem().string(),
            entry.path());
    }

    return index;
}

void
writeImage(
    const fs::path& img_path,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns)
{
    cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);

    if (image.empty()) {
        throw std::runtime_error("Cannot load image: " + img_path.string());
    }

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    rclcpp::Time img_time(timestamp_ns);
    image_msg->header.frame_id = "cam_2";
    image_msg->header.stamp = img_time;

    writer.write<sensor_msgs::msg::Image>(*image_msg, topic, img_time);
}

void
writeCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns)
{
    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(*cloud,msg);

    rclcpp::Time cloud_time(timestamp_ns);

    msg.header.frame_id = "base_lidar";
    msg.header.stamp = cloud_time;

    writer.write<sensor_msgs::msg::PointCloud2>(msg, topic, cloud_time);
}

void
writeRosbag(
    rosbag2_cpp::Writer& writer,
    const Config& cfg,
    const std::vector<fs::path>& files,
    const std::vector<int64_t>& timestamps_ns)
{
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;


    std::unordered_map<std::string, fs::path> image_index;
    std::unordered_map<std::string, fs::path> detection_index;
    std::unordered_map<std::string, fs::path> seg_index;

    if (cfg.use_images)
    {
        image_index = buildFileIndex(cfg.img_dir);
    }

    if (cfg.use_detections)
    {
        detection_index = buildFileIndex(cfg.det_dir);
    }

    if (cfg.use_seg)
    {
        seg_index = buildFileIndex(cfg.seg_dir);
    }

    std::string frame_id;

    for (size_t i = 0; i < files.size(); ++i)
    {

        frame_id = files[i].stem().string();

        std::cout
            << "\rWritting rosbag messages ["
            << (i + 1)
            << "/"
            << files.size()
            << "]"
            << std::flush;

        if (cfg.use_images)
        {
            auto img_it = image_index.find(frame_id);

            if (img_it != image_index.end()) {
                writeImage(img_it->second.string(), cfg.img_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing image for frame: " + frame_id);
            }
        }

        auto [input_cloud, corr] = loadPointCloudXYZI(
            files[i], cfg.use_fov_filter, cfg.fov_params);

        if (!input_cloud || input_cloud->empty())
        {
            std::cerr
                << "\n[WARN] Empty cloud: "
                << files[i]
                << '\n';

            continue;
        }

        if (cfg.export_input_pc) {
            writeCloud(input_cloud, cfg.pc_topic, writer, timestamps_ns[i]);
        }

        if (cfg.use_seg) {
            auto seg_it = seg_index.find(frame_id);

            if (seg_it != seg_index.end()) {
                auto sk_filtered_cloud = semantic_seg_filter(cfg.seg_classes, cfg.seg_max_dist, input_cloud, seg_it->second.string(), corr);
                writeCloud(sk_filtered_cloud, cfg.seg_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing semantic segmentation lbl file for frame: " + frame_id);
            }
        }

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);

            if (det_it != detection_index.end()) {
                auto detections = loadDetections(det_it->second.string(), cfg.det_score, cfg.det_max_dist, cfg.det_classes);
                auto filtered_cloud = filterPointCloud(input_cloud, detections);
                // Filter the cloud first
                writeCloud(filtered_cloud, cfg.det_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing detections for frame: " + frame_id);
            }
        }

    }

    std::cout << '\n';
}

void
generate_rosbag(const Config& cfg,  const std::vector<int64_t>& timestamps_ns)
{
    auto point_cloud_files = getPointCloudFiles(cfg.pc_dir);

    // Make sure timestamps and point cloud files have 1:1 correspondence
    if (timestamps_ns.size() != point_cloud_files.size()) {
        throw std::runtime_error(
            "Number of point clouds (" +
            std::to_string(point_cloud_files.size()) +
            ") does not match number of timestamps (" +
            std::to_string(timestamps_ns.size()) + ")");
    }

    // Configure output rosbag parameters
    rosbag2_cpp::Writer writer;

    rosbag2_storage::StorageOptions out_storage;
    out_storage.uri = cfg.rosbag_out;
    out_storage.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions out_converter;
    out_converter.input_serialization_format  = "cdr";
    out_converter.output_serialization_format = "cdr";

    writer.open(out_storage, out_converter);

    // Create the topics of the new rosbag
    createTopics(writer, cfg);

    // Write data in the rosbag
    writeRosbag(writer, cfg, point_cloud_files, timestamps_ns);
}

void
savePointCloudXYZI(const fs::path& filepath, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath.string());
    }

    // Create the vector with the correct size
    std::vector<float> buffer(cloud->points.size() * 4);

    // Fill the vector by index
    size_t i = 0;
    for (const auto& pt : cloud->points) {
        buffer[i++] = pt.x;
        buffer[i++] = pt.y;
        buffer[i++] = pt.z;
        buffer[i++] = pt.intensity;
    }

    // Write the reult in file
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(float));

    file.close();
}

void
export_clouds_to_directory(const Config& cfg)
{
    auto point_cloud_files = getPointCloudFiles(cfg.pc_dir);

    fs::path det_out_dir;
    fs::path seg_out_dir;

    // Crear subcarpetas según los filtros activos
    if (cfg.use_detections) {
        det_out_dir = cfg.clouds_out_dir / "detections";
        fs::create_directories(det_out_dir);
    }
    if (cfg.use_seg) {
        seg_out_dir = cfg.clouds_out_dir / "segmentation";
        fs::create_directories(seg_out_dir);
    }

    std::unordered_map<std::string, fs::path> detection_index;
    std::unordered_map<std::string, fs::path> seg_index;

    if (cfg.use_detections) detection_index = buildFileIndex(cfg.det_dir);
    if (cfg.use_seg) seg_index = buildFileIndex(cfg.seg_dir);

    std::string frame_id;

    for (size_t i = 0; i < point_cloud_files.size(); ++i)
    {
        frame_id = point_cloud_files[i].stem().string();

        std::cout << "\rExporting point clouds [" << (i + 1) << "/" << point_cloud_files.size() << "]" << std::flush;

        auto [input_cloud, corr] = loadPointCloudXYZI(
            point_cloud_files[i], cfg.use_fov_filter, cfg.fov_params);

        if (!input_cloud || input_cloud->empty()) continue;

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);
            if (det_it != detection_index.end()) {
                auto detections = loadDetections(det_it->second.string(), cfg.det_score, cfg.det_max_dist, cfg.det_classes);
                auto filtered_cloud = filterPointCloud(input_cloud, detections);
                
                savePointCloudXYZI(det_out_dir / point_cloud_files[i].filename(), filtered_cloud);
            } else {
                throw std::runtime_error("Missing detections file for frame: " + frame_id);
            }
        }

        if (cfg.use_seg) {
            auto seg_it = seg_index.find(frame_id);
            if (seg_it != seg_index.end()) {
                auto sk_filtered_cloud = semantic_seg_filter(cfg.seg_classes, cfg.seg_max_dist, input_cloud, seg_it->second.string(), corr);
                
                savePointCloudXYZI(seg_out_dir / point_cloud_files[i].filename(), sk_filtered_cloud);
            } else {
                throw std::runtime_error("Missing semantic segmentation lbl file for frame: " + frame_id);
            }
        }
    }
    std::cout << '\n';
}

int main(int argc, char* argv[])
{
    try
    {
        auto cfg = parse_arguments(argc, argv);

        validate(cfg);

        if (!cfg.no_confirm) {
            print_summary(cfg);
            std::cout << "Type [y|Y] to continue, or [ANY KEY] to cancel: ";

            char input;
            std::cin.get(input);

            // if (!input.empty()) {
            //     std::cout << "\n[ABORTED] EXECUTION ABORTED\n";
            //     exit(EXIT_FAILURE);
            // }
            if (input != 'y' && input != 'Y') {
                std::cout << "\n[ABORTED] EXECUTION ABORTED\n";
                exit(EXIT_FAILURE);
            }
        }

        if (cfg.only_clouds) {
            std::cout << "\n[INFO] Exporting filtered clouds directly to directory: " << cfg.clouds_out_dir << "\n";
            export_clouds_to_directory(cfg);
            std::cout << "\n[OK] POINT CLOUDS EXPORTED SUCCESSFULLY\n";
        } else {
            std::vector<int64_t> timestamps = loadTimestamps(cfg.ts_file);
            generate_rosbag(cfg, timestamps);
            std::cout << "\n[OK] ROSBAG GENERATED SUCCESFULLY\n";
        }

        exit(EXIT_SUCCESS);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] " << e.what() << "\n";
        exit(EXIT_FAILURE);
    }
}
