#pragma once
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <Eigen/Geometry>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Points;
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;

struct Ekf {
  typedef std::shared_ptr<Ekf> Ptr;
  int id;
  double dt;
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  PointsPtr tra_pts;
  rclcpp::Time last_update_stamp_;
  int cum, update_num;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;
  Eigen::Vector3d measurement_position;

  std::deque<Eigen::MatrixXd> InnoCov_list;

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 6);
    C.setZero(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 3) = dt;
    B(4, 4) = dt;
    B(5, 5) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 3) = 1;
    C(4, 4) = 1;
    C(5, 5) = 1;
    K = C;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt(0, 0) = 0.1;
    Qt(1, 1) = 0.1;
    Qt(2, 2) = 0.1;
    Qt(3, 3) = 0.1;
    Qt(4, 4) = 0.1;
    Qt(5, 5) = 0.1;
    Rt(0, 0) = 0.09;
    Rt(1, 1) = 0.09;
    Rt(2, 2) = 0.09;
    Rt(3, 3) = 0.4;
    Rt(4, 4) = 0.4;
    Rt(5, 5) = 0.4;
    x.setZero(6);
  }
  inline void predict(double elapsed) {
    dt = elapsed;
    A(0, 3) = A(1, 4) = A(2, 5) = dt;
    x = A * x;
    Sigma = A * Sigma * A.transpose() + Qt * (dt / 0.02);
    return;
  }
  inline void reset(const Eigen::Vector3d& z, const PointsPtr& tar_pts, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt, int id_) {
    x.head(3) = z;
    measurement_position = z;
    x.tail(3).setZero();
    Sigma.setZero();
    min_pt = minPt;
    max_pt = maxPt;
    tra_pts = tar_pts;
    last_update_stamp_ = rclcpp::Clock().now();
    cum = 10;
    update_num = 0;
    id = id_;
  }
  inline bool checkValid(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt, const PointsPtr& tar_pts) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    measurement_position = z1;
    Sigma = Sigma - K * C * Sigma;
    min_pt = minPt;
    max_pt = maxPt;
    tra_pts = tar_pts;
    last_update_stamp_ = rclcpp::Clock().now();
    update_num ++;
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};
