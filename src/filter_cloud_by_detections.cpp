#include <unistd.h>
#include <iostream>
#include <string>
#include <getopt.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>

#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

constexpr double MIN_SCORE = 0.0;
constexpr double MAX_SCORE = 1.0;

struct Detection {
    std::string category;
    float x, y, z;
    float w, l, h;
    float ry;
    float score;
};

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

    std::cout << "--- Cabecera procesada ---" << std::endl;
    for (const auto& pair : col_index) {
        std::cout << "Columna: [" << pair.first << "] -> Indice: " << pair.second << std::endl;
    }

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

        if (tokens.size() <= std::max(key_idx, value_idx)) {
            continue;
        }

        std::string key = tokens[key_idx];
        std::string value = tokens[value_idx];

        sync_map[key] = value;


        // --- PRINT 2: Ver procesamiento fila a fila ---
        std::cout << "Procesado: Key=[" << key << "] -> Value=[" << value << "]" << std::endl;
    }

    std::cout << "\n--- Estado final del Sync Map (" << sync_map.size() << " elementos) ---" << std::endl;
    for (const auto& pair : sync_map) {
        std::cout << "  " << pair.first << " => " << pair.second << std::endl;
    }

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
              << "  -h, --help                 Show this help message\n";
}

std::shared_ptr<std::vector<Detection>>
load_detections_from_file(
        const std::string& det_path,
        const float min_score)
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

    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        Detection det;

        ss >> det.category
           >> det.x >> det.y >> det.z
           >> det.w >> det.l >> det.h
           >> det.ry
           >> det.score;

        if (!ss.fail() && det.score >= min_score) {
            detections->push_back(det);
        }
    }

    return detections;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
loadPointCloud(const std::string& filepath, const size_t channels)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Check if the file exists
    if (!std::filesystem::exists(filepath)) {
        std::cerr << "[WARN] Point cloud file: " << filepath << " does not exists\n";
        return cloud; 
    }

    // Check if there are 3 or more channels
    if (channels < 3) {
        std::cerr << "[WARN] Channel number must be >= 3\n";
        return cloud;
    }

    // Open the file
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Point cloud file: " << filepath << "can't be oppened\n";
        return cloud;
    }

    // Get the total size of the file
    const size_t file_size = std::filesystem::file_size(filepath);

    // Calculate the number of points of the point cloud
    const size_t num_points = file_size / (channels * sizeof(float));

    cloud->points.resize(num_points);

    std::vector<float> buffer(channels);

    for (size_t i = 0; i < num_points; ++i)
    {
        if (!file.read(reinterpret_cast<char*>(buffer.data()), channels * sizeof(float))) {
            throw std::runtime_error("Error leyendo punto " + std::to_string(i));
        }

        cloud->points[i].x = buffer[0];
        cloud->points[i].y = buffer[1];
        cloud->points[i].z = buffer[2];
    }

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = true;

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
    const std::map<std::string, std::string>& sync_map)
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

    while (reader.has_next()) {
        auto msg = reader.read_next();
        const std::string& topic = msg->topic_name;

        if (topic == target_topic) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

            serializer.deserialize_message(&serialized_msg, &cloud_msg);

            uint64_t sec = cloud_msg.header.stamp.sec;
            uint64_t nanosec = cloud_msg.header.stamp.nanosec;

            uint64_t ts = (sec * 1000000000ULL) + nanosec;

            std::string ts_key = std::to_string(ts);

            if (!sync_map.contains(ts_key)) {
                std::cerr << "Can't find message with timestamp " << ts_key << "in the rosbag\n";
                continue;
            }

            std::string det_name = sync_map.at(ts_key);
            std::string det_path = detections_folder + "/" + det_name;

            auto detections = load_detections_from_file(det_path, min_score);

            // Create full path to the detecions file
            if (detections->empty()) {
                std::cout << "[INFO] No detections found in file " << det_name  << std::endl;
                continue;
            }

            std::string pc_path = pc_folder + "/" + ts_key;
            auto point_cloud = loadPointCloud(pc_path, 3);

            if (point_cloud->empty()) {
                std::cout << "[INFO] Point cloud in file " << pc_path << " is empty\n";
                continue;
            }

            // cargar la nube de puntos del fichero correspondiente

            // llamar a la función para filtrar

            std::cout << "....";
        } else {
            writer.write(msg);
        }
    }

    std::cout << "[OK] Copia completada." << std::endl;
}


int
main(int argc, char* argv[]) {
    int opt;

    std::string det_dir, pc_dir, rosbag_in, rosbag_out, sync_file;
    double score = -1.0;

    bool score_flag = false, det_flag = false, pc_flag = false, in_flag = false, out_flag = false, file_flag = false;

    static struct option long_options[] = {
        {"score", required_argument, 0, 's'},
        {"det_dir", required_argument, 0, 'd'},
        {"pc_dir", required_argument, 0, 'p'},
        {"rosbag_in", required_argument, 0, 'i'},
        {"rosbag_out", required_argument, 0, 'o'},
        {"sync_file", required_argument, 0, 'f'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "hs:d:p:i:o:f:", long_options, nullptr)) != -1) {
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
        std::cerr << "Error: missing arguments\n\n";
        print_help(argv[0]);
        exit(EXIT_FAILURE);
    }

    // Check if extra arguments were provided
    if (optind < argc) {
        std::cerr << "Error: unrecognized arguments\n";
        print_help(argv[0]);
        exit(EXIT_FAILURE);
    }

    std::ifstream file(sync_file);
    if (!file.is_open()) {
        std::cerr << "File " << sync_file << " cannot be opened\n";
        exit(EXIT_FAILURE);
    }

    if (!std::filesystem::exists(rosbag_in)) {
        std::cerr << "The input rosbag provided does not exists\n";
        exit(EXIT_FAILURE);
    }

    std::map<std::string, std::string> sync_map;

    if (load_sync_map(file, "/rslidar_points_timestamp", "/color/image_timestamp", sync_map) < 0) {
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
