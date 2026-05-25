#include "pc_filter/pc_filter_lib.hpp"

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>

void
pc_filter::filter_cloud_x(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg)
{
  std_msgs::msg::Header original_header = msg->header;

  // Convertir la nube de puntos a PCL
  // Convertir de PCLPointCloud2 a PointCloud<pcl::PointXYZ>
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
 
  // Filtrar la nube de puntos
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : *cloud)
  {
    if (point.x >= 0.0)
    {
      filtered_cloud->points.push_back(point);
    }
  }

  // Convertir la nube de puntos filtrada a sensor_msgs/PointCloud2
  pcl::toROSMsg(*filtered_cloud, filtered_msg);

  filtered_msg.header = original_header;
}

void
pc_filter::filter_cloud_fov_2d(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  sensor_msgs::msg::PointCloud2 &filtered_msg,
  float min_angle, float max_angle)
{
  std_msgs::msg::Header original_header = msg->header;

  // Convertir la nube de puntos a PCL
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  pcl::fromROSMsg(*msg, *cloud);
  // Filtrar la nube de puntos
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : *cloud)
  {

    float angle = std::atan2(point.y, point.x);

    if (angle > min_angle && angle < max_angle)
    {
      filtered_cloud->points.push_back(point);
    }
  }

  // Convertir la nube de puntos filtrada a sensor_msgs/PointCloud2
  pcl::toROSMsg(*filtered_cloud, filtered_msg);

  filtered_msg.header = original_header;
}

int
pc_filter::filter_detections_from_cloud(
  const std::shared_ptr<std::vector<Detection>>& dets,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const bool from_camera)
{
    // 1. Filtros de seguridad para evitar caídas del programa
    if (!dets || dets->empty() || !cloud || cloud->empty()) {
        return -1;
    }

    // 2. Vector para acumular los índices de puntos a borrar de TODAS las cajas
    pcl::IndicesPtr indices_to_remove(new std::vector<int>);

    // 3. Configurar el objeto CropBox apuntando a nuestra nube de entrada
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud);

    for (const auto& det : *dets)
    {

        Eigen::Vector4f min_pt, max_pt;
        Eigen::Vector3f rotation;
        Eigen::Vector3f translation(det.x, det.y, det.z);

        // Lidar case
        // CAMBIAR COORDENADAS TODO A LIDAR DIRECTAMENTE
        if (!from_camera) {
            // X=Largo, Y=Ancho, Z=Alto. Rotation in Z.
            min_pt = Eigen::Vector4f(-det.l / 2.0f, -det.w / 2.0f, -det.h / 2.0f, 1.0f);
            max_pt = Eigen::Vector4f( det.l / 2.0f,  det.w / 2.0f,  det.h / 2.0f, 1.0f);
            rotation = Eigen::Vector3f(0.0f, 0.0f, det.ry);
        
        // Camera case
        } else {
            // X=Ancho, Y=Alto, Z=Largo. Rotación en Y.
            min_pt = Eigen::Vector4f(-det.w / 2.0f, -det.h / 2.0f, -det.l / 2.0f, 1.0f);
            max_pt = Eigen::Vector4f( det.w / 2.0f,  det.h / 2.0f,  det.l / 2.0f, 1.0f);
            rotation = Eigen::Vector3f(0.0f, det.ry, 0.0f);
        }

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
        return 0;
    }

    std::cerr << "Found detections to filter\n";

    // 8. Optimización crítica: ordenar y eliminar índices duplicados 
    // Esto es vital por si hay cajas de detección que se solapan en el espacio
    std::sort(indices_to_remove->begin(), indices_to_remove->end());
    indices_to_remove->erase(std::unique(indices_to_remove->begin(), indices_to_remove->end()), indices_to_remove->end());

    // 9. Ejecutar ExtractIndices una sola vez para toda la nube
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_remove);
    extract.setNegative(true);  // true = borrar los puntos que están dentro de las cajas

    // Filtrar hacia un contenedor temporal y reasignar a la nube original
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_filtered);
    *cloud = *cloud_filtered; 

    // Faster
    // extract.filter(*cloud);

    return 0;
}
