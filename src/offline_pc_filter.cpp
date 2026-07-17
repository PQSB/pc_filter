#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <stdexcept>
#include <cstdlib>
#include <filesystem>

#include "cxxopts.hpp"
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include "pc_filter/pc_filter_lib.hpp"

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
        ("seg_classes", "ID of the classes to filter (id1,id2,id3,...) [Segmentation argument]", cxxopts::value<std::vector<uint16_t>>())
        ("seg_max_dist", "Maximum point distance to filter [Segmentation argument]", cxxopts::value<double>())

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
        cfg.fov_params = pc_filter::getFovFromFile(cfg.fov_file);
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
            auto id_vec = result["seg_classes"].as<std::vector<uint16_t>>();
            cfg.seg_classes = std::unordered_set<uint16_t>(id_vec.begin(), id_vec.end());
        }

        if (result.count("seg_max_dist")) {
            cfg.seg_max_dist = result["seg_max_dist"].as<double>();
        }
    }

    return cfg;
}

void
generate_rosbag(const Config& cfg,  const std::vector<int64_t>& timestamps_ns)
{
    auto point_cloud_files = pc_filter::getPointCloudFiles(cfg.pc_dir);

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
    pc_filter::createTopics(writer, cfg);

    // Write data in the rosbag
    pc_filter::writeRosbag(writer, cfg, point_cloud_files, timestamps_ns);
}

void
export_clouds_to_directory(const Config& cfg)
{
    auto point_cloud_files = pc_filter::getPointCloudFiles(cfg.pc_dir);

    fs::path det_out_dir;
    fs::path seg_out_dir;

    // Create subcfolders for the active filters
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

    if (cfg.use_detections) {
        detection_index = pc_filter::buildFileIndex(cfg.det_dir);
        if (detection_index.size() != point_cloud_files.size()) {
            throw std::runtime_error("Number of point clouds (" + std::to_string(point_cloud_files.size()) + 
                                     ") does not match number of detections (" + std::to_string(detection_index.size()) + ")");
        }
    }

    if (cfg.use_seg) {
        seg_index = pc_filter::buildFileIndex(cfg.seg_dir);
        if (seg_index.size() != point_cloud_files.size()) {
            throw std::runtime_error("Number of point clouds (" + std::to_string(point_cloud_files.size()) + 
                                     ") does not match number of semantic segmentation labels (" + std::to_string(seg_index.size()) + ")");
        }
    }

    std::string frame_id;

    for (size_t i = 0; i < point_cloud_files.size(); ++i)
    {
        frame_id = point_cloud_files[i].stem().string();

        std::cout << "\rExporting point clouds [" << (i + 1) << "/" << point_cloud_files.size() << "]" << std::flush;

        auto [input_cloud, corr] = pc_filter::loadPointCloudXYZI(
            point_cloud_files[i], cfg.use_fov_filter, cfg.fov_params);

        if (!input_cloud || input_cloud->empty()) continue;

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);
            if (det_it != detection_index.end()) {
                auto detections = pc_filter::loadDetections(det_it->second.string(), cfg.det_score, cfg.det_max_dist, cfg.det_classes);
                auto filtered_cloud = pc_filter::filterPointCloud(input_cloud, detections);
                
                pc_filter::savePointCloudXYZI(det_out_dir / point_cloud_files[i].filename(), filtered_cloud);
            } else {
                throw std::runtime_error("Missing detections file for frame: " + frame_id);
            }
        }

        if (cfg.use_seg) {
            auto seg_it = seg_index.find(frame_id);
            if (seg_it != seg_index.end()) {
                auto sk_filtered_cloud = pc_filter::semantic_seg_filter(cfg.seg_classes, cfg.seg_max_dist, input_cloud, seg_it->second.string(), corr);
                
                pc_filter::savePointCloudXYZI(seg_out_dir / point_cloud_files[i].filename(), sk_filtered_cloud);
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
            std::vector<int64_t> timestamps = pc_filter::loadTimestamps(cfg.ts_file);
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
