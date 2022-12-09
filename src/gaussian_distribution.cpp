// C++ includes
#include <math.h>

#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen includes
#include <Eigen/Dense>

class GaussianDensity {
 private:
  ros::NodeHandle nh;
  ros::Publisher pubMkArr;
  int ctr = 0;
  double loopRate = 1.0;

  int markerId = 0;
  visualization_msgs::MarkerArray mkArr;
  // visualization_msgs::Marker mk_a;
  visualization_msgs::Marker mk_a_init;
  visualization_msgs::Marker mk_n;

  Eigen::Vector2d mean;
  std::vector<Eigen::Vector2d> means;
  Eigen::Matrix2d covariance;
  std::vector<Eigen::Matrix2d> covariances;
  const double MULTIPLIER = 20; // For easier visualization and debugging. IS THE REASON WHY probability peak value is > 1.0 in most plots
  int state_dimension;
  double three_std_dev_1;
  double three_std_dev_2;
  double mean_probability;

  bool is_only_real = true; // TODO: Identify if/when a covariance matrix will have complex eigen values and set accordingly
  double threshold_probability;
  Eigen::Matrix2d eigen_vectors;
  Eigen::Vector2d eigen_values;
  Eigen::Matrix2cd eigen_vectors_complex;
  Eigen::Vector2cd eigen_values_complex;

  std::vector<geometry_msgs::Point> distribution_points; //TODO: Reserve size based on quantization value chosen.



 public:
  GaussianDensity() 
  {
    pubMkArr = nh.advertise<visualization_msgs::MarkerArray>("/gaussian_distribution/uniform_samples", 1);
  }

  double probabilityValue(Eigen::Vector2d x) {
    double normalizer = 1 / (std::sqrt( std::pow(2*M_PI, state_dimension)*covariance.determinant() ));
    double power = ( (x-mean).transpose() * covariance.inverse() * (x-mean) );
    power *= -0.5;
    // std::cout << (std::pow(2*M_PI, state_dimension)*covariance.determinant() ) << std::endl;
    // std::cout << "det: " << covariance.determinant() << std::endl;
    // std::cout << "normalizer: " << normalizer << std::endl;
    // std::cout << "power: " << power << std::endl;
    return MULTIPLIER * normalizer * std::exp(power);
  }

  void setDistributions() {

    // One line declaration using .finished() documented at the end of: https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
    means = {
      (Eigen::MatrixXd(2,1) << 0.0, 0.0).finished(),
      (Eigen::MatrixXd(2,1) << 3.0, 3.0).finished()
    };

    // covariances = {
    //   (Eigen::MatrixXd(2,2) << 2.0, 0.0, 0.0, 3.0).finished(),
    //   (Eigen::MatrixXd(2,2) << 2.0, 0.0, 0.0, 3.0).finished()
    // };
    covariances = {
      (Eigen::MatrixXd(2,2) << 5/4.0, -4/4.0, -4/4.0, 12/4.0).finished(),
      (Eigen::MatrixXd(2,2) << 5/4.0, -4/4.0, -4/4.0, 12/4.0).finished()
    };

    for (int i = 0; i < means.size(); ++i) {
      std::cout << ">>> DISTRIBUTION: " << i << std::endl;
      calcDistributionParams(means[i], covariances[i]);
    }

    /*
      // Covariance matrix meaning and examples: https://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/

      // mean << 0.0, 0.0; 
      // covariance << 2.0, 0.0, 
      //               0.0, 3.0;

      // Very tall distribution. Thus, three_std_dev points (which contain 99% of the volume) appear to drastically cut/chop distribution. 
      // Need to increase x_range += 5, y_range += 5 in calculateDistributionPoints() to view unclipped distribution.
      // covariance << 0.25, 0.1, 
      //               -0.15, 1.0;
      
      // covariance << 5, 4, 
      //               4, 6;
      
      // covariance << 5, -4, 
      //               -4, 6;
      
      // covariance << 5/4.0, -4/4.0, 
      //               -4/4.0, 12/4.0;
    */

  }

  void calcDistributionParams(Eigen::Vector2d _mean, Eigen::Matrix2d _covariance) {
    mean = _mean;
    covariance = _covariance;
    std::cout << "mean" << std::endl;
    std::cout << mean << std::endl;
    std::cout << "covariance" << std::endl;
    std::cout << covariance << std::endl;

    state_dimension = covariance.rows();
    std::cout << "state_dimension: " << state_dimension << std::endl;

    // NOTE: Eigen vectors are reported in increasing order of significance: http://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html#aaf4ed4172a517a4b9f0ab222f629e261
    if(is_only_real) {
      // Uses: Eigen::SelfAdjointEigenSolver(). 
      // This is supposed to have eigen vectors in decreasing order of significance AND be faster computation for symmetric real matrices (ie self-adjoint matrices).
      // https://stackoverflow.com/questions/56323727/efficient-way-of-sorting-eigenvalues-and-eigenvectors-obtained-from-eigen
      // https://forum.kde.org/viewtopic.php?f=9&t=110265
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es_sa(covariance);
      eigen_vectors = es_sa.eigenvectors(); // Column order matrix
      eigen_values = es_sa.eigenvalues();
      std::cout << "eigen_vectors" << std::endl;
      std::cout << eigen_vectors << std::endl;
      std::cout << "eigen values" << std::endl;
      std::cout << eigen_values << std::endl;
      
      three_std_dev_1 = eigen_values(0) * 3;
      three_std_dev_2 = eigen_values(1) * 3;    
      mean_probability = probabilityValue(mean);
      std::cout << "three_std_dev_1" << ", " << "three_std_dev_2" << ", " << "mean_probability" << std::endl;
      std::cout << three_std_dev_1 << ", " << three_std_dev_2 << ", " << mean_probability << std::endl;
    }
    else {
      Eigen::EigenSolver<Eigen::Matrix2d> es(covariance);
      eigen_vectors_complex = es.eigenvectors(); // Column order matrix
      eigen_values_complex = es.eigenvalues();
      std::cout << "eigen_vectors_complex" << std::endl;
      std::cout << eigen_vectors_complex << std::endl;
      std::cout << "eigen values" << std::endl;
      std::cout << eigen_values_complex << std::endl;
      
      three_std_dev_1 = eigen_values_complex(0).real() * 3;
      three_std_dev_2 = eigen_values_complex(1).real() * 3;    
      mean_probability = probabilityValue(mean);
      std::cout << "three_std_dev_1" << ", " << "three_std_dev_2" << ", " << "mean_probability" << std::endl;
      std::cout << three_std_dev_1 << ", " << three_std_dev_2 << ", " << mean_probability << std::endl;
    }

    threshold_probability = 0.01; // OR set as the value at some quartile/std dev of the distribution by calling probabilityValue() function
    calcDistributionPoints();
    visualizeAll();
  }

  /**
   * Limits are set based on length of three_std_dev
   * Then, points sampled along the limits are rotated to be oriented along actual eigen vectors
   * Finally, point is shifted to have a center around the mean of the given distribution
  */
  void calcDistributionPoints() {

    /// Limits set based on length of three_std_dev
    double x_limit = three_std_dev_1;
    double y_limit = three_std_dev_2;
    // Hard increment of x and y limits being plotted for very tall distributions which appear to undergo "clipping" at three_std_dev points.
    // x_limit += 10;
    // y_limit += 10;
    double quantization_x = 50;
    double quantization_y = 50;

    geometry_msgs::Point pt;
    for(double x = -x_limit; x <= x_limit; x += x_limit/quantization_x) {
      for(double y = -y_limit; y <= y_limit; y += y_limit/quantization_y) {

        /// Rotate sample point ranges along eigen vector   
        tf::Vector3 vec1(x, y, 0.0);
        tf::Vector3 vec1_rot;
        double yaw;
        if(is_only_real) yaw = std::atan2(eigen_vectors.col(0)(1), eigen_vectors.col(0)(0));
        else yaw = std::atan2(eigen_vectors_complex.col(0)(0).real(), eigen_vectors_complex.col(0)(1).real());
        vec1_rot = vec1.rotate(tf::Vector3(0.0, 0.0, 1.0), yaw); // Rotate about z axis by 'yaw' angle

        /// After rotation, finally add mean to make relative position of point around the mean 
        pt.x = mean(0) + vec1_rot.getX();
        pt.y = mean(1) + vec1_rot.getY();
        pt.z = probabilityValue(Eigen::Vector2d (pt.x, pt.y));

        // Push points only if above probability threshold 
        if(pt.z > threshold_probability) distribution_points.push_back(pt);
      }
    }
  }

  void fuseDistributions() {

  }

  void visualizeAll() {

    // Visualize distribution_points
    for(const geometry_msgs::Point& pt : distribution_points) {
      addSphereMarker(getMarkerId(),
                      pt.x, pt.y, pt.z,
                      0.0, 0.0, 0.0, 1.0,
                      0.05, 0.05, 0.05, 
                      "map", 
                      "distribution", 
                      1.0, 1.0, 1.0, 0.7
                     );
    }

    // Visualize eigen vectors (from least significant to most significant)
    geometry_msgs::Point eig_vec1_pt1;
    eig_vec1_pt1.x = mean(0);
    eig_vec1_pt1.y = mean(1);
    eig_vec1_pt1.z = 0.0;
    geometry_msgs::Point eig_vec1_pt2;
    if(is_only_real) {
      eig_vec1_pt2.x = mean(0) + three_std_dev_1 * eigen_vectors.col(0)(0); // Eigen vector in column zero, 0th element of vector
      eig_vec1_pt2.y = mean(1) + three_std_dev_1 * eigen_vectors.col(0)(1); // Eigen vector in column zero, 1st element of vector
      eig_vec1_pt2.z = 0.0;
    }
    else {
      eig_vec1_pt2.x = mean(0) + three_std_dev_1 * eigen_vectors_complex.col(0)(0).real(); // Eigen vector in column zero, 0th element of vector, its real part
      eig_vec1_pt2.y = mean(1) + three_std_dev_1 * eigen_vectors_complex.col(0)(1).real(); // Eigen vector in column zero, 1st element of vector, its real part
      eig_vec1_pt2.z = 0.0;
    }
    addArrowMarkerTwoPointForm( getMarkerId(), 
                                eig_vec1_pt1, eig_vec1_pt2, 
                                0.1, 0.1, 0.0, 
                                "map", 
                                "distribution", 
                                1.0, 0.0, 0.0, 1.0);

    geometry_msgs::Point eig_vec2_pt1;
    eig_vec2_pt1.x = mean(0);
    eig_vec2_pt1.y = mean(1);
    eig_vec2_pt1.z = 0.0;    
    geometry_msgs::Point eig_vec2_pt2;
    if(is_only_real) {
      eig_vec2_pt2.x = mean(0) + three_std_dev_2 * eigen_vectors.col(1)(0); // Eigen vector in column one, 0th element of vector
      eig_vec2_pt2.y = mean(1) + three_std_dev_2 * eigen_vectors.col(1)(1); // Eigen vector in column one, 1st element of vector
      eig_vec2_pt2.z = 0.0;
    }
    else {
      eig_vec2_pt2.x = mean(0) + three_std_dev_2 * eigen_vectors_complex.col(1)(0).real(); // Eigen vector in column one, 0th element of vector, its real part
      eig_vec2_pt2.y = mean(1) + three_std_dev_2 * eigen_vectors_complex.col(1)(1).real(); // Eigen vector in column one, 1st element of vector, its real part
      eig_vec2_pt2.z = 0.0;
    }
    addArrowMarkerTwoPointForm( getMarkerId(), 
                                eig_vec2_pt1, eig_vec2_pt2, 
                                0.1, 0.1, 0.0, 
                                "map", 
                                "distribution", 
                                0.0, 1.0, 0.0, 1.0);

    // Visualize a single sphere 
    tf::Quaternion quat; // Only the TF package in ROS has functions to convert roll, pitch and yaw angles to corsp quaternion
    double yaw = std::atan2(eig_vec1_pt2.y-eig_vec1_pt1.y, eig_vec1_pt2.x-eig_vec1_pt2.x);
    std::cout << "yaw eig_vec1 (rad), (deg): " << yaw << ", " << rad2deg(yaw) <<std::endl;
    quat.setRPY(0.0, 0.0, yaw);
    std::cout << quat.getX() << "," << quat.getY() << "," << quat.getZ() << "," << quat.getW() << std::endl;
    addSphereMarker(getMarkerId(),
                mean(0), mean(1), 0.0,
                quat.getX(), quat.getY(), quat.getZ(), quat.getW(),
                // three_std_dev_1, three_std_dev_2, mean_probability, // Oblong in x and y based on variance and as tall as distribution mean
                2*three_std_dev_1, 2*three_std_dev_2, 2*mean_probability, // Oblong in x and y based on variance and as tall as distribution mean
                "map", 
                "distribution", 
                1.0, 1.0, 0.0, 0.4
                );
  }

  int getMarkerId()
  {
    markerId += 1;
    return markerId;
  }

  double rad2deg(double x) {
    return (x * 180) / M_PI;
  }

  void addSphereMarker(const int id,  // Have to provide defaults to all args if multiple of the same type need to exist in a function
                      double x, double y, double z,
                      double ox, double oy, double oz, double ow,
                      double sx, double sy, double sz,
                      const std::string frame_id = "map",
                      const std::string ns = "cylinder",
                      double r = 1.0f, double g = 1.0f, double b = 1.0f, double a = 1.0f) {
    visualization_msgs::Marker mk;

    mk.header.frame_id = frame_id;
    mk.header.stamp = ros::Time::now();
    mk.ns = ns;
    mk.id = id;

    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = z;
    mk.pose.orientation.x = ox;
    mk.pose.orientation.y = oy;
    mk.pose.orientation.z = oz;
    mk.pose.orientation.w = ow;
    mk.scale.x = sx;
    mk.scale.y = sy;
    mk.scale.z = sz;
    mk.color.r = r;
    mk.color.g = g;
    mk.color.b = b;
    mk.color.a = a;

    mk.lifetime = ros::Duration();  // initialize Duration() = Duration = 0.0 = forever

    mkArr.markers.push_back(mk);
  }

  void addArrowMarkerTwoPointForm(const int id,  // Have to provide defaults to all args if multiple of the same type need to exist in a function
                                  geometry_msgs::Point pt1,
                                  geometry_msgs::Point pt2,
                                  double shaft_diam = 0.1, double head_diam = 0.1, double head_len = 0.0,  // To get cylinder, set arrow_head_len to 0.0
                                  const std::string frame_id = "map",
                                  const std::string ns = "cylinder",
                                  double r = 1.0f, double g = 1.0f, double b = 1.0f, double a = 1.0f) {
    visualization_msgs::Marker mk;

    mk.header.frame_id = frame_id;
    mk.header.stamp = ros::Time::now();
    mk.ns = ns;
    mk.id = id;

    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;

    // USING TWO POINTS form of arrow for cylinder as well
    mk.scale.x = shaft_diam;
    mk.scale.y = head_diam;
    mk.scale.z = head_len;

    mk.points.push_back(pt1);
    mk.points.push_back(pt2);

    // Two point form of arrow does not make use of the orientation
    // Setting it to standard unit quaternion just to suppress RVIZ warning
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    mk.color.r = r;
    mk.color.g = g;
    mk.color.b = b;
    mk.color.a = a;

    mk.lifetime = ros::Duration();  // initialize Duration() = Duration = 0.0 = forever

    mkArr.markers.push_back(mk);
  }

 public: 

  void loop() {
    // Set max doubleing point precision.
    // std::streamsize ss = std::cout.precision(); // Save original precision value
    std::cout.precision(std::numeric_limits<double>::max_digits10);  // set new precision. OR can set as: std::cout.precision(6); for 6 digit precision
    // std::cout.precision(ss); // Change back to default precision.

    setDistributions();

    fuseDistributions();

    std_msgs::Header hdr;
    hdr.frame_id = "map";

    ros::Rate r(loopRate);
    while (ros::ok()) {
      std::cout << "---" << std::endl;
      pubMkArr.publish(mkArr);
      r.sleep();
    }
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gaussian_distribution");

  GaussianDensity gd;
  gd.loop();

  return 0;
}
