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

namespace fs = std::filesystem;

struct Config
{
    // Neccesary
    fs::path pc_dir;
    fs::path rosbag_out;
    fs::path ts_file;

    // Input cloud topic
    std::string pc_topic;
    bool export_input_pc = false;

    // Imágenes
    bool use_images = false;
    fs::path img_dir;
    std::string img_topic;

    // Detections
    bool use_detections = false;
    fs::path det_dir;
    double score{};
    std::string filtered_topic;
    std::unordered_set<std::string> classes;
};

/*
FUNCIÓN LEER DETECCIONES DE FICHERO TEMPORAL A FALTA DE MODIFICAR LA DE LA LIBRERÍA
*/
std::vector<Detection>
loadDetections(
    const std::string det_file,
    double score_threshold,
    const std::unordered_set<std::string>& allowed_classes)
{
    std::ifstream file(det_file);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open detections file: " + det_file);
    }

    std::vector<Detection> detections;

    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty()) {continue;}

        std::stringstream ss(line);

        Detection det;

        if (!(ss >> det.category >> det.x >> det.y >> det.z >> det.w
            >> det.l >> det.h >> det.ry >> det.score)) {continue;}

        if (det.score < score_threshold) {continue;}

        if (!allowed_classes.empty() &&
            allowed_classes.find(det.category) == allowed_classes.end()) {continue;}

        detections.push_back(std::move(det));
    }

    return detections;
}

/*
FUNCIÓN FILTRADO TEMPORAL A FALTA DE MODIFICAR LA DE LA LIBRERÍA
*/
int
filter_detections_from_cloud(
  const std::vector<Detection>& dets,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // 1. Filtros de seguridad para evitar caídas del programa
    if (dets.empty() || !cloud || cloud->empty()) {
        return -1;
    }

    // 2. Vector para acumular los índices de puntos a borrar de TODAS las cajas
    pcl::IndicesPtr indices_to_remove(new std::vector<int>);

    // 3. Configurar el objeto CropBox apuntando a nuestra nube de entrada
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);

    for (const auto& det : dets)
    {

        Eigen::Vector4f min_pt, max_pt;
        Eigen::Vector3f rotation;
        Eigen::Vector3f translation(det.x, det.y, det.z);

        // X=Largo, Y=Ancho, Z=Alto. Rotation in Z.
        min_pt = Eigen::Vector4f(-det.l / 2.0f, -det.w / 2.0f, -det.h / 2.0f, 1.0f);
        max_pt = Eigen::Vector4f( det.l / 2.0f,  det.w / 2.0f,  det.h / 2.0f, 1.0f);
   
        rotation = Eigen::Vector3f(0.0f, 0.0f, det.ry);

        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.setTranslation(translation);
        crop.setRotation(rotation);

        std::vector<int> inside;
        crop.filter(inside);

        indices_to_remove->insert(indices_to_remove->end(), inside.begin(), inside.end());
    }

    // 7. Si ninguna caja contenía puntos, terminamos el proceso de inmediato
    if (indices_to_remove->empty()) {
        // std::cerr << "Nothing filtered in the detections provided\n";
        //std::cout << "\n[Filtro] Terminado. No se encontraron puntos dentro de las cajas (0 filtrados)." << std::endl;
        cloud->clear();
        return 0;
    }

    std::sort(indices_to_remove->begin(), indices_to_remove->end());
    indices_to_remove->erase(std::unique(indices_to_remove->begin(), indices_to_remove->end()), indices_to_remove->end());

    // 9. Ejecutar ExtractIndices una sola vez para toda la nube
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_remove);
    extract.setNegative(false);  // true = borrar los puntos que están dentro de las cajas

    // Filtrar hacia un contenedor temporal y reasignar a la nube original
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    extract.filter(*cloud_filtered);
    //cloud->swap(*cloud_filtered);
    *cloud = *cloud_filtered; 

    return 0;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr
loadPointCloudXYZI(const std::string& filepath)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Check if the file exists
    if (!std::filesystem::exists(filepath)) {
        std::cerr << "[WARN] File does not exist: " << filepath << "\n";
        return cloud;
    }

    // Get the total size of the file
    const size_t file_size = std::filesystem::file_size(filepath);
    const size_t point_size = 4 * sizeof(float); // XYZ = 12 bytes

    if (file_size == 0) {
        std::cerr << "[ERROR] Empty file: " << filepath << "\n";
        return cloud;
    }

    if (file_size % point_size != 0) {
        std::cerr << "[ERROR] Invalid .bin file size for XYZI format: "
                  << file_size << " bytes\n";
        return cloud;
    }

    // Calculate the number of points of the point cloud
    const size_t num_points = file_size / point_size;

    std::vector<float> raw(num_points * 4);

    // Open the file
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open file: " << filepath << "\n";
        return cloud;
    }

    // comprobar si puedo usar fread
    if (!file.read(reinterpret_cast<char*>(raw.data()), file_size)) {
        std::cerr << "[ERROR] Could not read full file\n";
        return cloud;
    }

    cloud->points.resize(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = raw[i * 4 + 0];
        cloud->points[i].y = raw[i * 4 + 1];
        cloud->points[i].z = raw[i * 4 + 2];
        cloud->points[i].intensity = raw[i * 4 + 3];
    }

    cloud->width  = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

void
print_summary(const Config& cfg)
{
    std::cout << "\n=====================================\n";
    std::cout << "CONFIGURATION SUMMARY\n";
    std::cout << "=====================================\n";

    std::cout << "[POINTCLOUDS]\n";
    std::cout << "  dir   : " << cfg.pc_dir << "\n";
    std::cout << "  topic : " << cfg.pc_topic << "\n";
    std::cout << "  timestamps file : " << cfg.ts_file << "\n\n";

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
        std::cout << "  score          : " << cfg.score << "\n";
        std::cout << "  filtered_topic : " << cfg.filtered_topic << "\n";
        std::cout << "  classes        : ";

        for (const auto& c : cfg.classes)
            std::cout << c << " ";

        std::cout << "\n";
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

        if (!cfg.classes.empty() ||
            !cfg.filtered_topic.empty())
        {
            std::cout
                << "   - partial detection args were ignored\n";
        }
        std::cout << "\n";
    }
    std::cout << "=====================================\n\n";
}

void
validate(const Config& cfg)
{
    // Needed
    if (!fs::exists(cfg.pc_dir) || !fs::is_directory(cfg.pc_dir))
        throw std::invalid_argument("pc_dir invalid");

    if (cfg.rosbag_out.empty())
        throw std::invalid_argument("rosbag_out must be provided");

    if (!fs::exists(cfg.ts_file) || !fs::is_regular_file(cfg.ts_file))
        throw std::invalid_argument("ts_file invalid");

    if (cfg.export_input_pc)
    {
        if (cfg.pc_topic.empty())
            throw std::invalid_argument("pc_topic neccesary to publish input point clouds");
    }

    // Images related
    if (cfg.use_images)
    {
        if (!fs::exists(cfg.img_dir) || !fs::is_directory(cfg.img_dir))
            throw std::invalid_argument("img_dir invalid");

        if (cfg.img_topic.empty())
            throw std::invalid_argument("img_topic neccessary if images are used");
    }

    // Detections related
    if (cfg.use_detections)
    {
        if (!fs::exists(cfg.det_dir) || !fs::is_directory(cfg.det_dir))
            throw std::invalid_argument("det_dir invalid");

        if (cfg.score < 0.0 || cfg.score > 1.0)
            throw std::invalid_argument("score is out of range [0.0, 1.0]");

        if (cfg.filtered_topic.empty())
            throw std::invalid_argument("filtered_topic neccesary to publish filtered point clouds");

        if (cfg.classes.empty())
            throw std::invalid_argument("Classes to filter must be provided");
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
        ("ts_file", "Timestamps file needed for rosbag messages headers (REQUIRED)", cxxopts::value<fs::path>(cfg.ts_file))
        ("pc_topic", "Input point clouds topic", cxxopts::value<std::string>(cfg.pc_topic))

        // Output
        ("o,rosbag_out", "Output rosbag path (REQUIRED)", cxxopts::value<fs::path>(cfg.rosbag_out))

        // Imágenes
        ("i,img_dir", "Input images folder path", cxxopts::value<fs::path>())

        ("img_topic", "Input images topic", cxxopts::value<std::string>())

        // Filter
        ("d,det_dir", "Detections folder path", cxxopts::value<fs::path>())
        ("s,score", "Min score threshold", cxxopts::value<double>())
        ("filtered_topic", "Filtered point clouds topic name", cxxopts::value<std::string>())
        ("c,classes", "Classes to filter (c1,c2,c3,...)", cxxopts::value<std::vector<std::string>>())

        ("h,help", "Show help");

    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
        std::cout << options.help() << "\n";
        exit(EXIT_SUCCESS);
    }

    if (result.count("pc_topic"))
    {
        cfg.export_input_pc = true;
        cfg.pc_topic = result["pc_topic"].as<std::string>();
    }

    // Imágenes → activar módulo solo si existe dir
    if (result.count("img_dir"))
    {
        cfg.use_images = true;
        cfg.img_dir = result["img_dir"].as<fs::path>();

        if (!result.count("img_topic"))
            throw std::invalid_argument("img_dir requires img_topic");

        cfg.img_topic = result["img_topic"].as<std::string>();
    }

    // Detecciones → activar módulo solo si existe dir
    if (result.count("det_dir"))
    {
        cfg.use_detections = true;

        cfg.det_dir = result["det_dir"].as<fs::path>();

        if (!result.count("score") ||
            !result.count("filtered_topic") ||
            !result.count("classes"))
        {
            throw std::invalid_argument("det_dir requires score, filtered_topic and classes");
        }

        cfg.score = result["score"].as<double>();

        cfg.filtered_topic = result["filtered_topic"].as<std::string>();

        auto class_vec = result["classes"].as<std::vector<std::string>>();

        cfg.classes = std::unordered_set<std::string>(class_vec.begin(), class_vec.end()); 
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
        topic_info.name = cfg.filtered_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
        // std::cout << "[OK] " << cfg.filtered_topic << " topic created succesfully\n";
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
    const std::string& img_path,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns)
{
    cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);

    if (image.empty()) {
        throw std::runtime_error("Cannot load image: " + img_path);
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

    if (cfg.use_images)
    {
        image_index = buildFileIndex(cfg.img_dir);
    }

    if (cfg.use_detections)
    {
        detection_index = buildFileIndex(cfg.det_dir);
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

        auto input_cloud = loadPointCloudXYZI(files[i]);

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

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);

            if (det_it != detection_index.end()) {
                auto detections = loadDetections(det_it->second.string(), cfg.score, cfg.classes);
                // Filtrar la nube primero
                writeCloud(input_cloud, cfg.filtered_topic, writer, timestamps_ns[i]);

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

int main(int argc, char* argv[])
{
    try
    {
        auto cfg = parse_arguments(argc, argv);

        validate(cfg);
        print_summary(cfg);
        std::cout << "Press [ENTER] to continue, or [ANY KEY + ENTER] to cancel: ";

        std::string input;
        std::getline(std::cin, input);

        if (!input.empty()) {
            std::cout << "\n[ABORTED] EXECUTION ABORTED\n";
            exit(EXIT_FAILURE);
        }

        //std::cout << "[OK] configuration succesful\n";

        //std::cout << "[INFO] loading timesatmps..." << std::flush;
        std::vector<int64_t> timestamps = loadTimestamps(cfg.ts_file);
        //std::cout << "\r[OK] timestamps loaded succesfully\n";

        generate_rosbag(cfg, timestamps);

        std::cout << "\n[OK] ROSBAG GENERATED SUCCESFULLY\n";

        exit(EXIT_SUCCESS);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] " << e.what() << "\n";
        exit(EXIT_FAILURE);
    }
}
