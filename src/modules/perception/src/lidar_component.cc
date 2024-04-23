/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#include "perception/lidar_component.h"

#include <pcl_conversions/pcl_conversions.h>

using namespace robocar::perception::lidar;

LidarComponent::LidarComponent(const cycle::Params& params) : cycle::Service(params) {
	// params
	double roi_min_x = params.get("roi_min_x").to_double();
	double roi_max_x = params.get("roi_max_x").to_double();
	if (roi_max_x <= roi_min_x) {
        throw std::invalid_argument("'roi_max_x' must be greater than 'roi_min_x'");
    }
	double roi_min_y = params.get("roi_min_y").to_double();
	double roi_max_y = params.get("roi_max_y").to_double();
	if (roi_max_y <= roi_min_y) {
        throw std::invalid_argument("'roi_max_y' must be greater than 'roi_min_y'");
    }
	double roi_min_z = params.get("roi_min_z").to_double();
	double roi_max_z = params.get("roi_max_z").to_double();
	if (roi_max_z <= roi_min_z) {
        throw std::invalid_argument("'roi_max_z' must be greater than 'roi_min_z'");
    }
	double _leaf_size = params.get("leaf_size").to_double();
	if (_leaf_size <= 0.0) {
        throw std::invalid_argument("'leaf_size' must be > 0.0");
    }
	_ec_tolerance = params.get("ec_tolerance").to_double();
	if (_ec_tolerance <= 0.0) {
        throw std::invalid_argument("'ec_tolerance' must be > 0.0");
    }
	_ec_min_size = params.get("ec_min_size").to_int();
	if (_ec_min_size <= 0) {
        throw std::invalid_argument("'ec_min_size' must be > 0");
    }
    _ec_max_size = params.get("ec_max_size").to_int();
	if (_ec_max_size <= 0) {
        throw std::invalid_argument("'ec_max_size' must be > 0");
    }
	if (_ec_max_size <= _ec_min_size) {
        throw std::invalid_argument("'ec_max_size' must be greater than 'ec_min_size'");
    }
	_obj_min_height = params.get("obj_min_height").to_double();
	if (_obj_min_height <= 0.0) {
        throw std::invalid_argument("'obj_min_height' must be > 0.0");
    }

	// init ROI filter and downsampling
	_roi_min_point = Eigen::Vector4f(roi_min_x, roi_min_y, roi_min_z, 1);
    _roi_max_point = Eigen::Vector4f(roi_max_x, roi_max_y, roi_max_z, 1);
	_cb.setMin(_roi_min_point);
    _cb.setMax(_roi_max_point);
	_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);

	// init ground segmentation
	GroundSegmentationParams gs_params;
	_ground_segmentation = std::make_unique<GroundSegmentation>(gs_params);

	// init euclidean clustering
	_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
	_ec.setClusterTolerance(_ec_tolerance);
	_ec.setMinClusterSize(_ec_min_size);
	_ec.setMaxClusterSize(_ec_max_size);
	_ec.setSearchMethod(_tree);

	// publishers
	_pub_non_ground = this->create_publisher<msg::PointCloud>("perception/lidar/non_ground", 1);
	_pub_objects3d = this->create_publisher<msg::Objects3d>("perception/lidar/objects3d", 1);
	// subscriber
	_sub_point_cloud = this->create_subscription<msg::PointCloud>("sensors/points", 1,
																  std::bind(&LidarComponent::on_point_cloud,
																			this, std::placeholders::_1));
}

void LidarComponent::serve() {
	// do nothing for now
}

void LidarComponent::on_point_cloud(msg::PointCloud r_pc) {
	// convert to pcl
    auto pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::moveFromROSMsg(r_pc, *pc);
	if (pc->empty()) {
		return;
	}

	// remove ground
	auto indices = std::make_shared<std::vector<int>>();
	_ground_segmentation->segment(*pc, indices.get());
	auto non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	for (int i=0; i < indices->size(); i++) {
		if (indices->at(i) == 0)
			non_ground->push_back(pc->at(i));
	}
	if (non_ground->empty()) {
		return;
	}

	// ROI filter
    _cb.setInputCloud(non_ground);
    _cb.filter(*non_ground);
	if (non_ground->empty()) {
		return;
	}

	// downsampling
	//LOG_DEBUG("before " + std::to_string(non_ground->size()));
	_vg.setInputCloud (non_ground);
	_vg.filter(*non_ground);
	if (non_ground->empty()) {
		return;
	}
	//LOG_DEBUG("after " + std::to_string(non_ground->size()) + "\n");

	// euclidean clustering
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters;
	std::vector<pcl::PointIndices> cluster_indices;
	_ec.setInputCloud(non_ground);
	_ec.extract(cluster_indices);
	for (const auto& indices : cluster_indices) {
		auto cloud_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
		for (const auto& idx : indices.indices) {
			cloud_cluster->push_back((*non_ground)[idx]);
		}
		cloud_clusters.push_back(cloud_cluster);
	}

	// build objects from clusters
	msg::Objects3d objects;
    int index = 0;
    for(auto& cluster : cloud_clusters) {
		pcl::PointXYZI min_point, max_point, position;
		Eigen::Matrix3f rotational_matrix;
		Eigen::Vector3f dir_major, dir_middle, dir_minor;

		// compute oriented bounding boxes
		_moi.setInputCloud(cluster);
		_moi.compute();
		_moi.getOBB(min_point, max_point, position, rotational_matrix);
		_moi.getEigenVectors(dir_major, dir_middle, dir_minor);
		double x_min = min_point.x + position.x;
    	double y_min = min_point.y + position.y;
    	double z_min = min_point.z + position.z;
    	double x_max = max_point.x + position.x;
    	double y_max = max_point.y + position.y;
    	double z_max = max_point.z + position.z;

        if (z_max - z_min >= _obj_min_height) {
			msg::Object3d object;

			// header
			auto stamp = std::llround(pc->header.stamp / 1000.0);
			object.header.stamp = cycle::utils::unix_ms_to_ros_time(stamp);
			object.header.frame_id = pc->header.frame_id;
			// type
			object.type = OBJ_OBSTACLE;
			// ground center
			object.ground_center.x = (x_min + x_max) / 2;
			object.ground_center.y = (y_min + y_max) / 2;
			object.ground_center.z = z_min;
			// direction
			object.direction.x = dir_major(0);
			object.direction.y = dir_major(1);
			object.direction.z = dir_major(2);
			// dims
			object.dims.x = x_max - x_min;
			object.dims.y = y_max - y_min;
			object.dims.z = z_max - z_min;
			// velocity
			object.velocity.x = 0.0;
			object.velocity.y = 0.0;
			object.velocity.z = 0.0;

			// distances from ground center
        	double l_2 = object.dims.x / 2;
        	double w_2 = object.dims.y / 2;
        	double h = object.dims.z;
			// compute bbox points in object frame
        	std::vector<Eigen::Vector3d> bbox(8);
        	bbox[0] = {l_2, w_2, 0};
        	bbox[1] = {-l_2, w_2, 0};
        	bbox[2] = {-l_2, -w_2, 0};
        	bbox[3] = {l_2, -w_2, 0};
        	bbox[4] = {l_2, w_2, h};
        	bbox[5] = {-l_2, w_2, h};
        	bbox[6] = {-l_2, -w_2, h};
        	bbox[7] = {l_2, -w_2, h};

			// direction angle
        	double theta = atan2(object.direction.y, object.direction.x);
        	double cos_theta = cos(theta);
        	double sin_theta = sin(theta);
        	// rotation and translation into vehicle frame
        	for (int j=0; j < bbox.size(); j++) {
				msg::Vector3d point;
				point.x = (bbox[j][0] * cos_theta) - (bbox[j][1] * sin_theta)
						  + object.ground_center.x;
        		point.y = (bbox[j][0] * sin_theta) + (bbox[j][1] * cos_theta)
						  + object.ground_center.y;
				point.z = bbox[j][2] + object.ground_center.z;
        		object.points.push_back(point);
        	}

			// add object
			objects.objects.push_back(object);
			index++;
        }
    }

    // publish
	non_ground->header.stamp = pc->header.stamp;
	non_ground->header.frame_id = pc->header.frame_id;
	if (non_ground->size() > 0) {
		msg::PointCloud pc_msg;
		pcl::toROSMsg(*non_ground, pc_msg);
		_pub_non_ground->publish(pc_msg);
    }
	_pub_objects3d->publish(objects);
}