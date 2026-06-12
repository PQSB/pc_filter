#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

#include <cxxopts.hpp>

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
    std::vector<std::string> classes;
};

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

        cfg.classes =result["classes"].as<std::vector<std::string>>();
    }

    return cfg;
}

int main(int argc, char* argv[])
{
    try
    {
        auto cfg = parse_arguments(argc, argv);

        validate(cfg);
        print_summary(cfg);

        std::cout << "Configuration succesful\n";

        exit(EXIT_SUCCESS);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] " << e.what() << "\n";
        exit(EXIT_FAILURE);
    }
}
