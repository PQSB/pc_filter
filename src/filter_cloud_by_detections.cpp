#include <iostream>
#include <string>
#include <getopt.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>

#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pc_filter/pc_filter_lib.hpp"

#include <Eigen/Dense>

constexpr double MIN_SCORE = 0.0;
constexpr double MAX_SCORE = 1.0;

int
load_sync_map(
    std::ifstream& csv_file,
    const std::string& key_column,
    const std::string& value_column,
    std::map<std::string, std::string>& sync_map)
{
    std::string line;

    // Read the header
    if (!std::getline(csv_file, line)) {
        std::cerr << "The provided file is empty\n";
        return -1;
    }

    std::stringstream header_ss(line);
    std::string col;
    std::unordered_map<std::string, int> col_index;
    int idx = 0;

    while (std::getline(header_ss, col, ',')) {
        col_index[col] = idx++;
    }

    // std::cout << "--- Cabecera procesada ---" << std::endl;
    // for (const auto& pair : col_index) {
    //     std::cout << "Columna: [" << pair.first << "] -> Indice: " << pair.second << std::endl;
    // }

    // Check if the columns of interest exist
    if (!col_index.count(key_column)) {
        return -1;
    }
    if (!col_index.count(value_column)) {
        return -1;
    }

    int key_idx = col_index[key_column];
    int value_idx = col_index[value_column];

    while (std::getline(csv_file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;

        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        if (tokens.size() <= static_cast<size_t>(std::max(key_idx, value_idx))) {
            continue;
        }

        std::string key = tokens[key_idx];
        std::string value = tokens[value_idx];

        sync_map[key] = value;


        // // --- PRINT 2: Ver procesamiento fila a fila ---
        // std::cout << "Procesado: Key=[" << key << "] -> Value=[" << value << "]" << std::endl;
    }

    // std::cout << "\n--- Estado final del Sync Map (" << sync_map.size() << " elementos) ---" << std::endl;
    // for (const auto& pair : sync_map) {
    //     std::cout << "  " << pair.first << " => " << pair.second << std::endl;
    // }

    return 0;
}

void
print_help(const char* prog_name)
{
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "Options:\n"
              << "  -s, --score <score>        Confidence detection threshold [0.00, 1.00]\n"
              << "  -d, --det_dir <path>       Path to detections directory\n"
              << "  -p, --pc_dir <path>        Path to point clouds directory\n"
              << "  -i, --rosbag_in <file>     Input rosbag\n"
              << "  -o, --rosbag_out <file>    Output rosbag\n"
              << "  -f, --sync_file <file>     Path to synchronization file\n"
              << "  -t, --tf_file <file>       Path to cam2lidar matrix\n"
              << "  -c, --classes <classes>    Adds the class to the target classes to filter\n"
              << "  -h, --help                 Show this help message\n";
}

// Función para leer la matriz de transformación adaptada a tu estilo de código
Eigen::Matrix4f
loadTransformationMatrix(const std::string& filepath) 
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Comprobar si el archivo existe usando filesystem
    if (!std::filesystem::exists(filepath)) {
        std::cerr << "The transformation file provided does not exist: " << filepath << "\n";
        exit(EXIT_FAILURE);
    }

    // Abrir el archivo de texto
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "File " << filepath << " cannot be opened\n";
        exit(EXIT_FAILURE);
    }

    // Leer los datos secuencialmente para rellenar la matriz de 4x4
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(file >> transform(i, j))) {
                std::cerr << "Error: Invalid format in transformation file. Expected a 4x4 matrix.\n";
                exit(EXIT_FAILURE);
            }
        }
    }

    return transform;
}

std::shared_ptr<std::vector<Detection>>
load_detections_from_file(
        const std::string& det_path,
        const float min_score,
        const std::unordered_set<std::string>& allowed_set,
        const std::optional<Eigen::Matrix4f>& transform_matrix = std::nullopt)
{
    auto detections = std::make_shared<std::vector<Detection>>();

    // Argument validation
    if (!std::filesystem::exists(det_path)) {
        std::cerr << "[WARN] Detections file: " << det_path << " does not exists\n";
        return detections;
    }

    std::ifstream file(det_path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Detections file: " << det_path << "can't be oppened\n";
        return detections;
    }

    float delta_yaw = 0.0f;
    if (transform_matrix.has_value()) {
        delta_yaw = std::atan2((*transform_matrix)(1, 0), (*transform_matrix)(0, 0));
    }

    std::string line;
    std::stringstream ss;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        ss.str(line);
        ss.clear();

        Detection det;

        if (ss >> det.category >> det.x >> det.y >> det.z
            >> det.w >> det.l >> det.h >> det.ry >> det.score) {
        
            if (det.score >= min_score && allowed_set.find(det.category) != allowed_set.end()) {

                if (transform_matrix.has_value()) {
                    // Crear el punto
                    Eigen::Vector4f point(det.x, det.y, det.z, 1.0f);
                    // Transformar el punto a coordenadas de lidar
                    Eigen::Vector4f transformed_point = (*transform_matrix) * point;

                    det.x = transformed_point.x();
                    det.y = transformed_point.y();
                    det.z = transformed_point.z();

                    det.ry += delta_yaw;
                    if (det.ry > M_PI)  det.ry -= 2.0f * M_PI;
                    if (det.ry < -M_PI) det.ry += 2.0f * M_PI;
                }

                detections->push_back(det);
            }
            
        }
    }

    return detections;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
loadPointCloudXYZ(const std::string& filepath) // Recibe la ruta del archivo binario como una referencia constante.
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Check if the file exists
    if (!std::filesystem::exists(filepath)) {
        std::cerr << "[WARN] File does not exist: " << filepath << "\n";
        return cloud;
    }

    // Get the total size of the file
    const size_t file_size = std::filesystem::file_size(filepath);
    const size_t point_size = 3 * sizeof(float); // XYZ = 12 bytes

    if (file_size == 0) {
        std::cerr << "[ERROR] Empty file: " << filepath << "\n";
        return cloud;
    }

    if (file_size % point_size != 0) {
        std::cerr << "[ERROR] Invalid .bin file size for XYZ format: "
                  << file_size << " bytes\n";
        return cloud;
    }

    // Calculate the number of points of the point cloud
    const size_t num_points = file_size / point_size;

    std::vector<float> raw(num_points * 3);

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
        cloud->points[i].x = raw[i * 3 + 0];
        cloud->points[i].y = raw[i * 3 + 1];
        cloud->points[i].z = raw[i * 3 + 2];
    }

    cloud->width  = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

void
filter_rosbag_point_clouds(
    const std::string& input_bag,
    const std::string& output_bag,
    const std::string target_topic,
    const std::string detections_folder,
    const std::string pc_folder,
    const float min_score,
    const std::vector<std::string>& allowed_classes,
    const std::map<std::string, std::string>& sync_map,
    const std::optional<Eigen::Matrix4f>& transform_matrix = std::nullopt)
{
    rosbag2_cpp::Reader reader;

    rosbag2_storage::StorageOptions in_storage;
    in_storage.uri = input_bag;
    in_storage.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions in_converter;
    in_converter.input_serialization_format  = "cdr";
    in_converter.output_serialization_format = "cdr";

    reader.open(in_storage, in_converter);

    rosbag2_cpp::Writer writer;

    rosbag2_storage::StorageOptions out_storage;
    out_storage.uri = output_bag;
    out_storage.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions out_converter;
    out_converter.input_serialization_format  = "cdr";
    out_converter.output_serialization_format = "cdr";

    writer.open(out_storage, out_converter);

    // Create topics in the new rosbag
    for (const auto& topic_info : reader.get_all_topics_and_types()) {
        writer.create_topic(topic_info);
    }

    // Convert allowed_classes a un unordered_set para búsquedas O(1)
    std::unordered_set<std::string> allowed_set;
    allowed_set.reserve(allowed_classes.size());
    for (const auto& c : allowed_classes) {
        allowed_set.insert(c);
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

    char spinner[] = {'|', '/', '-', '\\'};
    long i = 0;
    long interval = 100;

    std::cout << "[OK] Start generating rosbag with filtered point clouds\n";
    while (reader.has_next()) {
        auto msg = reader.read_next();
        const std::string& topic = msg->topic_name;

        if (topic == target_topic) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

            serializer.deserialize_message(&serialized_msg, &cloud_msg);

            uint64_t sec = cloud_msg.header.stamp.sec;
            uint64_t nanosec = cloud_msg.header.stamp.nanosec;

            uint64_t ts = (sec * 1000000000ULL) + nanosec;

            std::string ts_key = std::to_string(ts);

            if (sync_map.find(ts_key) == sync_map.end()) {
                std::cerr << "Can't find message with timestamp " << ts_key << " in the rosbag\n";
                continue;
            }

            std::string det_name = sync_map.at(ts_key);
            std::string det_path = detections_folder + "/" + det_name + ".txt";

            std::shared_ptr<std::vector<Detection>> detections;

            if (transform_matrix.has_value()) {
                detections = load_detections_from_file(det_path, min_score, allowed_set, transform_matrix);
            
            } else {
                detections = load_detections_from_file(det_path, min_score, allowed_set);
            }

            // Create full path to the detecions file
            if (detections->empty()) {
                //std::cout << "[INFO] No detections found in file " << det_name << std::endl;
                writer.write(msg);
                continue;
            } //else {
                //std::cout << "[INFO] " << detections->size() << " detections found in file " << det_name << std::endl;
            //}

            std::string pc_path = pc_folder + "/" + ts_key + ".bin";
            auto point_cloud = loadPointCloudXYZ(pc_path);

            if (point_cloud->empty()) {
                std::cout << "[INFO] Point cloud in file " << pc_path << " is empty\n";
                continue;
            }

            if (pc_filter::filter_detections_from_cloud(detections, point_cloud) < 0) {
                continue;
            }

            // Construir nuevo mensaje usando la nube de puntos filtrada
            sensor_msgs::msg::PointCloud2 new_cloud_msg;

            pcl::toROSMsg(*point_cloud, new_cloud_msg);
            new_cloud_msg.header = cloud_msg.header;

            // writer.write(new_cloud_msg, topic, rclcpp::Time(msg->recv_timestamp));
            writer.write<sensor_msgs::msg::PointCloud2>(new_cloud_msg, topic, rclcpp::Time(msg->recv_timestamp));
        } else {
            writer.write(msg);
        }

        if (++i % interval == 0) {
            std::cout << "\rGenerating... [" << spinner[(i / interval) % 4] << "]" << std::flush;
        }
    }

    std::cout << "\n[OK] New rosbag generated succesfully\n";
}


int
main(int argc, char* argv[]) {
    int opt;

    std::string det_dir, pc_dir, rosbag_in, rosbag_out, sync_file, tf_file;
    double score = -1.0;

    bool score_flag = false, det_flag = false, pc_flag = false, in_flag = false, out_flag = false, file_flag = false;

    std::vector<std::string> allowed_classes;

    static struct option long_options[] = {
        {"score", required_argument, 0, 's'},
        {"det_dir", required_argument, 0, 'd'},
        {"pc_dir", required_argument, 0, 'p'},
        {"rosbag_in", required_argument, 0, 'i'},
        {"rosbag_out", required_argument, 0, 'o'},
        {"sync_file", required_argument, 0, 'f'},
        {"classes", required_argument, 0, 'c'},
        {"tf_file", required_argument, 0, 't'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "hs:d:p:i:o:f:c:t:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 's':
                std::cout << "score: " << optarg << "\n";

                score = std::stod(optarg);
                if (score < MIN_SCORE || score > MAX_SCORE) {
                    std::cerr << "score value out of range [0.0 : 1.0]\n";
                    exit(EXIT_FAILURE);
                }

                score_flag = true;
                break;

            case 'd':
                std::cout << "det_dir: " << optarg << "\n";
                det_flag = true;
                det_dir = optarg;
                break;

            case 'p':
                std::cout << "pc_dir: " << optarg << "\n";
                pc_flag = true;
                pc_dir = optarg;
                break;

            case 'i':
                std::cout << "rosbag_in: " << optarg << "\n";
                in_flag = true;
                rosbag_in = optarg;
                break;

            case 'o':
                std::cout << "rosbag_out: " << optarg << "\n";
                out_flag = true;
                rosbag_out = optarg;
                break;

            case 'f':
                std::cout << "sync_file: " << optarg << "\n";
                file_flag = true;
                sync_file = optarg;
                break;

            case 'c':
                std::cout << "added class " << optarg << "\n";
                allowed_classes.emplace_back(optarg);
                break;

            case 't':
                std::cout << "Provided cam2lidar matrix file: " << optarg << "\n";
                tf_file = optarg;
                break;

            case 'h':
                std::cout << "Help\n";
                print_help(argv[0]);
                exit(EXIT_SUCCESS);

            case '?':
                std::cerr << "Error: Invalid \n";
                break;
        }
    }

    // Check if all the arguments were provided
    if (!(score_flag && det_flag && pc_flag && in_flag && out_flag && file_flag)) {
        std::cerr << "[ERROR] Missing arguments\n\n";
        print_help(argv[0]);
        exit(EXIT_FAILURE);
    }

    // Check if extra arguments were provided
    if (optind < argc) {
        std::cerr << "[ERROR] Unrecognized arguments\n";
        print_help(argv[0]);
        exit(EXIT_FAILURE);
    }

    std::ifstream file(sync_file);
    if (!file.is_open()) {
        std::cerr << "[ERROR] File " << sync_file << " cannot be opened\n";
        exit(EXIT_FAILURE);
    }

    if (!std::filesystem::exists(rosbag_in)) {
        std::cerr << "[ERROR] The input rosbag provided does not exists\n";
        exit(EXIT_FAILURE);
    }

    if (!tf_file.empty() && !std::filesystem::is_regular_file(tf_file)) {
        std::cerr << "[ERROR] The provided cam2file transform file is not valid\n";
        exit(EXIT_FAILURE);
    }

    if (allowed_classes.empty()) {
        std::cerr << "[ERROR] NO allowed classes provided, output rosbag would be equal to input rosbag\n";
        exit(EXIT_FAILURE);
    }

    std::cout << "-------------------------------------------\n";

    std::map<std::string, std::string> sync_map;

    std::cout << "[OK] Loading sync map...\n";
    if (load_sync_map(file, "/rslidar_points_timestamp", "/color/image_timestamp", sync_map) < 0) {
        exit(EXIT_FAILURE);
    }

    std::cout << "[OK] Sync map loaded succesfully\n";

    if (tf_file.empty()) {
        filter_rosbag_point_clouds(
            rosbag_in, rosbag_out, "/rslidar_points", det_dir, pc_dir, score, allowed_classes, sync_map);
    
    } else {
        Eigen::Matrix4f transfom_matrix;
        transfom_matrix = loadTransformationMatrix(tf_file);
        filter_rosbag_point_clouds(
            rosbag_in, rosbag_out, "/rslidar_points", det_dir, pc_dir, score, allowed_classes, sync_map, transfom_matrix);
    }

    //TODO
    // Añadir parámetros para las columnas a sincronizar y para el topic de las nubes de puntos a filtrar

    exit(EXIT_SUCCESS);
}
