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

#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

constexpr double MAX_DIST_DEFAULT = -1.0;

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
    fs::path m_det_dir;
    double m_score{};
    double m_max_dist = MAX_DIST_DEFAULT;
    std::string m_topic;
    std::unordered_set<std::string> m_classes;

    // Semantic kitti
    bool use_sk = false;
    fs::path sk_lbl_dir;
    std::string sk_topic;
    std::unordered_set<std::string> sk_classes;
};

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

visualization_msgs::msg::Marker
makeBoxMarker(const Detection& det, int64_t ts, int id) {
    visualization_msgs::msg::Marker m;

    m.header.frame_id = "base_lidar";
    m.header.stamp = rclcpp::Time(ts);

    m.ns = "crop_boxes";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = det.x;
    m.pose.position.y = det.y;
    m.pose.position.z = det.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, det.ry);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    m.scale.x = det.l;
    m.scale.y = det.w;
    m.scale.z = det.h;

    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.5f;
    
    // CAMBIO CLAVE: Controlado por DELETEALL, sin temporizadores reales
    m.lifetime = rclcpp::Duration::from_seconds(0); 
    
    return m;
}


/*
FUNCIÓN LEER DETECCIONES DE FICHERO TEMPORAL A FALTA DE MODIFICAR LA DE LA LIBRERÍA
*/
std::vector<Detection>
loadDetections(
    const std::string det_file,
    double m_score_threshold,
    double m_max_dist,
    const std::unordered_set<std::string>& allowed_classes)
{
    std::ifstream file(det_file);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open detections file: " + det_file);
    }

    std::vector<Detection> detections;

    std::string line;

    double m_max_dist_sq = m_max_dist * m_max_dist;

    while (std::getline(file, line))
    {
        if (line.empty()) {continue;}

        std::stringstream ss(line);

        Detection det;

        if (!(ss >> det.category >> det.x >> det.y >> det.z >> det.w
            >> det.l >> det.h >> det.ry >> det.score)) {continue;}

        if (det.score < m_score_threshold) {continue;}

        // Check if the filter is active
        if (m_max_dist != MAX_DIST_DEFAULT){
            // To avoid using sqrt
            double dist_sq = (det.x * det.x) + (det.y * det.y) + (det.z * det.z);
            if (dist_sq > m_max_dist_sq) {continue;}
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

    // Conservar únicamente los puntos
    // que están dentro de las detecciones
    extract.setNegative(false);

    extract.filter(*filtered_cloud);

    return filtered_cloud;
}

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
loadPointCloudXYZI(
    const fs::path& filepath, const bool use_fov, const FovCalibration& calib)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Check if the file exists
    if (!fs::exists(filepath)) {
        std::cerr << "[WARN] File does not exist: " << filepath << "\n";
        return cloud;
    }

    // Get the total size of the file
    const size_t file_size = fs::file_size(filepath);
    const size_t point_size = 4 * sizeof(float); // XYZI = 16 bytes

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

    cloud->points.reserve(num_points);

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
    }

    cloud->width  = cloud->points.size();
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

    std::cout << "[OUTPUT]\n";
    std::cout << "  output : " << cfg.rosbag_out << "\n\n";

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
        std::cout << "[3D-MOOD DETECTIONS] ENABLED\n";
        std::cout << "  dir            : " << cfg.m_det_dir << "\n";
        std::cout << "  score          : " << cfg.m_score << "\n";
        std::cout << "  topic : " << cfg.m_topic << "\n";
        std::cout << "  max_dist       : " << cfg.m_max_dist << "\n";
        std::cout << "  classes        : ";

        for (const auto& c : cfg.m_classes)
            std::cout << c << " ";

        std::cout << "\n";
    }
    else
    {
        std::cout << "[DETECTIONS] DISABLED\n";
        std::cout << "  Reason:\n";
        if (!cfg.m_det_dir.empty())
        {
            std::cout << "   - m_det_dir provided but module incomplete\n";
        }
        else
        {
            std::cout << "   - m_det_dir not provided -> module ignored\n";
        }

        if (!cfg.m_classes.empty() ||
            !cfg.m_topic.empty())
        {
            std::cout
                << "   - partial detection args were ignored\n";
        }
        std::cout << "\n";
    }
    if (cfg.use_sk)
    {
        std::cout << "[SEMANTIC KITTI] ENABLED\n";
        std::cout << "  dir            : " << cfg.sk_lbl_dir << "\n";
        std::cout << "  topic          : " << cfg.sk_topic << "\n";
        std::cout << "  classes        : " ;

        for (const auto& c : cfg.sk_classes)
            std::cout << c << " "; 
    }
    else
    {
        std::cout << "[SEMANTIC KITTI] DISABLED\n";
        std::cout << "  Reason:\n";
        if (!cfg.sk_lbl_dir.empty())
        {
            std::cout << "   - sk_lbl_dir provided but module incomplete\n";
        }
        else
        {
            std::cout << "   - sk_lbl_dir not provided -> module ignored\n";
        }

        if (!cfg.sk_classes.empty() ||
            !cfg.sk_topic.empty())
        {
            std::cout
                << "   - partial semantic kitti args were ignored\n";
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

    // 3D-MOOD Detections related
    if (cfg.use_detections)
    {
        if (!fs::exists(cfg.m_det_dir) || !fs::is_directory(cfg.m_det_dir))
            throw std::invalid_argument("m_det_dir invalid");

        if (cfg.m_score < 0.0 || cfg.m_score > 1.0)
            throw std::invalid_argument("m_score is out of range [0.0, 1.0]");

        if (cfg.m_topic.empty())
            throw std::invalid_argument("m_topic neccesary to publish filtered point clouds");

        if (cfg.m_classes.empty())
            throw std::invalid_argument("Classes to filter must be provided");
    }

    // Semantic kitti related
    if (cfg.use_sk)
    {
       if (!fs::exists(cfg.sk_lbl_dir) || !fs::is_directory(cfg.sk_lbl_dir))
            throw std::invalid_argument("sk_lbl_dir invalid");

        if (cfg.sk_topic.empty())
            throw std::invalid_argument("sk_topic neccesary to publish filtered point clouds");

        if (cfg.sk_classes.empty())
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

        ("fov_filter", "Preprocessed file generated with prepare_fov_filter_calib_file.py", cxxopts::value<fs::path>())

        // Output
        ("o,rosbag_out", "Output rosbag path (REQUIRED)", cxxopts::value<fs::path>(cfg.rosbag_out))

        // Images
        ("i,img_dir", "Input images folder path", cxxopts::value<fs::path>())

        ("img_topic", "Input images topic", cxxopts::value<std::string>())

        // 3D-MOOD filter parameters
        ("m_det_dir", "Detections folder path [3D-MOOD argument]", cxxopts::value<fs::path>())
        ("m_score", "Min score threshold [3D-MOOD argument]", cxxopts::value<double>())
        ("m_topic", "Filtered point clouds topic name [3D-MOOD argument]", cxxopts::value<std::string>())
        ("m_max_dist", "Maximum detection distance [3D-MOOD argument]", cxxopts::value<double>())
        ("m_classes", "Classes to filter (c1,c2,c3,...) [3D-MOOD argument]", cxxopts::value<std::vector<std::string>>())

        // Semantic kitti filter parameters
        ("sk_lbl_dir", "Semantic kitti labels folder path [Sem Kitti argument]", cxxopts::value<fs::path>())
        ("sk_topic", "Filtered point clouds topic name [Sem Kitti argument]", cxxopts::value<std::string>())
        ("sk_classes", "Classes to filter (c1,c2,c3,...) [Sem Kitti argument]", cxxopts::value<std::vector<std::string>>())

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

    if (result.count("fov_filter"))
    {
        cfg.use_fov_filter = true;
        cfg.fov_file = result["fov_filter"].as<fs::path>();

        // Add check of fov_file before filling FovCalibration
        if (!fs::exists(cfg.fov_file) || !fs::is_regular_file(cfg.fov_file))
        throw std::invalid_argument("fov_file invalid");

        // Fill the FovCalibration struct
        cfg.fov_params = getFovFromFile(cfg.fov_file);
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
    if (result.count("m_det_dir"))
    {
        cfg.use_detections = true;

        cfg.m_det_dir = result["m_det_dir"].as<fs::path>();

        if (!result.count("m_score") ||
            !result.count("m_topic") ||
            !result.count("m_classes"))
        {
            throw std::invalid_argument("m_det_dir requires m_score, m_topic and m_classes");
        }

        cfg.m_score = result["m_score"].as<double>();

        cfg.m_topic = result["m_topic"].as<std::string>();

        auto class_vec = result["m_classes"].as<std::vector<std::string>>();

        cfg.m_classes = std::unordered_set<std::string>(class_vec.begin(), class_vec.end());

        // Validate m_max_dist argument here since it has a default value
        if (result.count("m_max_dist")) {
            double input_dist = result["m_max_dist"].as<double>();

            if (input_dist >= 0.0) {
                cfg.m_max_dist = input_dist;
            } else {
                throw std::invalid_argument("m_max_dist is invalid [m_max_dist >= 0.0]");
            }
        }
    }

    if (result.count("m_det_dir"))
    {
       cfg.use_sk = true;

        cfg.sk_lbl_dir = result["sk_lbl_dir"].as<fs::path>();

        if (!result.count("sk_topic") ||
            !result.count("sk_classes"))
        {
            throw std::invalid_argument("sk_lbl_dir requires sk_topic and sk_classes");
        }

        cfg.sk_topic = result["sk_topic"].as<std::string>();

        auto class_vec = result["sk_classes"].as<std::vector<std::string>>();

        cfg.sk_classes = std::unordered_set<std::string>(class_vec.begin(), class_vec.end());
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
        topic_info.name = cfg.m_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
        // std::cout << "[OK] " << cfg.m_topic << " topic created succesfully\n";

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

    if (cfg.use_images)
    {
        image_index = buildFileIndex(cfg.img_dir);
    }

    if (cfg.use_detections)
    {
        detection_index = buildFileIndex(cfg.m_det_dir);
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

        auto input_cloud = loadPointCloudXYZI(
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

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);

            if (det_it != detection_index.end()) {
                auto detections = loadDetections(det_it->second.string(), cfg.m_score, cfg.m_max_dist, cfg.m_classes);
                auto filtered_cloud = filterPointCloud(input_cloud, detections);
                // Filtrar la nube primero
                writeCloud(filtered_cloud, cfg.m_topic, writer, timestamps_ns[i]);

                visualization_msgs::msg::MarkerArray marker_array;

                visualization_msgs::msg::Marker clean_marker;
                clean_marker.header.frame_id = "base_lidar";
                clean_marker.header.stamp = rclcpp::Time(timestamps_ns[i]);
                clean_marker.ns = "crop_boxes";
                clean_marker.action = visualization_msgs::msg::Marker::DELETEALL;
                marker_array.markers.push_back(clean_marker);


                int id_contador = 0;
                for (const auto& det : detections) {
                    visualization_msgs::msg::Marker box = makeBoxMarker(det, timestamps_ns[i], id_contador++);
                    marker_array.markers.push_back(box);
                }

                writer.write<visualization_msgs::msg::MarkerArray>(
                    marker_array, 
                    "/det_boxes", 
                    rclcpp::Time(timestamps_ns[i])
                );


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
