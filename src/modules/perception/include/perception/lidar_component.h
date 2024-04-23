/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef PERCEPTION_LIDAR_COMPONENT_H
#define PERCEPTION_LIDAR_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include "ground_segmentation/ground_segmentation.h"

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace robocar::perception::lidar {
	class LidarComponent : public cycle::Service {
	public:
		LidarComponent(const cycle::Params& params);

		void serve() override;

	private:
		// params
		Eigen::Vector4f _roi_min_point;
    	Eigen::Vector4f _roi_max_point;
		double _leaf_size = 0.05;
		double _ec_tolerance = 0.35;
    	int _ec_min_size = 5;
    	int _ec_max_size = 2000;
		double _obj_min_height = 0.25;

		pcl::CropBox<pcl::PointXYZI> _cb;
		pcl::VoxelGrid<pcl::PointXYZI> _vg;
		pcl::search::KdTree<pcl::PointXYZI>::Ptr _tree;
		pcl::EuclideanClusterExtraction<pcl::PointXYZI> _ec;
		pcl::MomentOfInertiaEstimation<pcl::PointXYZI> _moi;
		std::unique_ptr<GroundSegmentation> _ground_segmentation;

		// publishers
		rclcpp::Publisher<msg::PointCloud>::SharedPtr _pub_non_ground;
		rclcpp::Publisher<msg::Objects3d>::SharedPtr _pub_objects3d;
		// subscriber
		rclcpp::Subscription<msg::PointCloud>::SharedPtr _sub_point_cloud;

		void on_point_cloud(msg::PointCloud pc);
	};
}

#endif // PERCEPTION_LIDAR_COMPONENT_H