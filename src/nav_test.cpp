#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class NavTest : public rclcpp::Node{
  public:
    NavTest(): Node("nav_test_node"){
      // Get drone_id parameter
      this->declare_parameter("drone_id", 0);
      drone_id = this->get_parameter("drone_id").as_int();
      
      // Subscribers
      estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/estimator/pose", 10, std::bind(&NavTest::estimator_pose_callback, this, std::placeholders::_1));
      //estimator_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vicon/TelloMount" + std::to_string(drone_id+1) + "/TelloMount" + std::to_string(drone_id+1), 10, std::bind(&NavTest::estimator_pose_callback, this, std::placeholders::_1));
      estimator_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/estimator/velocity", 10, std::bind(&NavTest::estimator_velocity_callback, this, std::placeholders::_1));
      reference_pose_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("tello/reference/pose", 10, std::bind(&NavTest::position_reference_callback, this, std::placeholders::_1));
      reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/reference/velocity", 10, std::bind(&NavTest::velocity_reference_callback, this, std::placeholders::_1));

      // Publishers
      uaux_publisher    = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);
      sigma_publisher   = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/sigma", 10);
      error_publisher   = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/error", 10);
      ref_rot_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tello/control/ref_rot", 10);

      k_publisher     = this->create_publisher<geometry_msgs::msg::TwistStamped>("tello/control/k", 10);
      e_dot_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("tello/control/e_dot", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&NavTest::control_callback, this));
  
      // Initialize variables 
      //zetta1 << 1, 1, 1, 4;
      zetta1  << 3.45, 3.45, 4.25, 4;
      //zetta2  << 0.045, 0.045, 0.25, 4;
      zetta2  << 3.45, 3.45, 4.25, 4;
      //lambda1 << 1.1, 1.1, 1.2, 1;
      //lambda1 << 1.1, 1.1, 1.2, 1.6;
      //lambda1 << 3.6, 3.6, 2.2, 6.6;
      lambda1 << 1.1, 1.1, 1.1, 1.1;
      lambda2 << 1.1, 1.1, 1.1, 1.1;
      //lambda1 << 0.6, 0.6, 2.2, 6.6;
      alpha   << 0.001, 0.001, 0.001, 0.001;
      beta    << 0.01, 0.01, 0.01, 0.01;

      this->declare_parameter("gains", std::vector<double>{
	  zetta1(0),  zetta1(1),  zetta1(2),  zetta1(3), 
	  zetta2(0),  zetta2(1),  zetta2(2),  zetta2(3),
	  lambda1(0), lambda1(1), lambda1(2), lambda1(3), 
	  lambda2(0), lambda2(1), lambda2(2), lambda2(3), 
	  alpha(0),  alpha(1),  alpha(2),  alpha(3), 
	  beta(0),   beta(1),   beta(2),   beta(3)});

      lambda1_minus_1 << lambda1(0)-1, lambda1(1)-1, lambda1(2)-1, lambda1(3)-1;

      e          << 0, 0, 0, 0;
      ref_rot    << 0, 0, 0, 0;
      e_dot      << 0, 0, 0, 0;
      q_hat      << 1, 0, 0, 0;
      q_hat_conj << 1, 0, 0, 0;
      q_d        << 1, 0, 0, 0;
      q_e        << 1, 0, 0, 0;
      eta_e      << 0, 0, 0, 0;
      K          << 0, 0, 0, 0;
      K_dot      << 0, 0, 0, 0;
      sigma      << 0, 0, 0, 0;
      uaux       << 0, 0, 0, 0;
      u          << 0, 0, 0, 0;
      u_int      << 0, 0, 0, 0;
      u_prev     << 0, 0, 0, 0;
      u_t        << 0, 0, 0, 0;

      // 7.321 x 10^-5 kg m^2
      Ixx = 0.00007321;
      Iyy = 0.00013604;
      Izz = 0.00007317;

      J << Ixx, 0, 0,
	    0, Iyy, 0,
	    0, 0, Izz;

      reference_pose.pose.position.x = (double)drone_id; 
      reference_pose.pose.position.y = 0;
      reference_pose.pose.position.z = 1;
      reference_pose.pose.orientation.w = 1;
      reference_pose.pose.orientation.x = 0;
      reference_pose.pose.orientation.y = 0;
      reference_pose.pose.orientation.z = 0;

    }

    void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      estimator_pose = *msg;
    }
    void estimator_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      estimator_velocity = *msg;
    }
    void position_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      reference_pose = *msg;
    }
    void velocity_reference_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      reference_velocity = *msg;
    }


    void control_callback(){
      gains = this->get_parameter("gains").as_double_array();
      zetta1  << gains[0],  gains[1],  gains[2],  gains[3];
      zetta2  << gains[4],  gains[5],  gains[6],  gains[7];
      lambda1 << gains[8],  gains[9],  gains[10], gains[11];
      lambda2 << gains[12], gains[13], gains[14], gains[15];
      alpha   << gains[16], gains[17], gains[18], gains[19];
      beta    << gains[20], gains[21], gains[22], gains[23];
      /*lambda1 << gains[4], gains[5], gains[6], gains[7];
      alpha  << gains[8], gains[9], gains[10], gains[11];
      beta   << gains[12], gains[13], gains[14], gains[15];*/

      // Conjugate q_hat
      q_hat << estimator_pose.pose.orientation.w, estimator_pose.pose.orientation.x, estimator_pose.pose.orientation.y, estimator_pose.pose.orientation.z;
      q_hat_conj << q_hat(0), -q_hat(1), -q_hat(2), -q_hat(3);

      q_d << reference_pose.pose.orientation.w, reference_pose.pose.orientation.x, reference_pose.pose.orientation.y, reference_pose.pose.orientation.z;
      q_e = kronecker(q_hat_conj, q_d);

      // Normalize q_e
      double q_e_norm = sqrt(q_e(0)*q_e(0) + q_e(1)*q_e(1) + q_e(2)*q_e(2) + q_e(3)*q_e(3));
      if (q_e_norm != 0){
	      q_e << q_e(0)/q_e_norm, q_e(1)/q_e_norm, q_e(2)/q_e_norm, q_e(3)/q_e_norm;
      }else{
	q_e << 1, 0, 0, 0;
      }

      // Logarithmic mapping of q_e
      eta_e << qlm(q_e);

      // Check for NaN in eta_e
      for (int i = 0; i < 4; i++){
        if (std::isnan(eta_e(i))){
          std::cout << "Eta_e is NaN at element " << i << std::endl;
          eta_e(i) = 0;
        }
      }

      e << reference_pose.pose.position.x - estimator_pose.pose.position.x,
           reference_pose.pose.position.y - estimator_pose.pose.position.y,
           reference_pose.pose.position.z - estimator_pose.pose.position.z,
	   eta_e(3);
      
      e_dot << reference_velocity.twist.linear.x  - estimator_velocity.twist.linear.x,
	       reference_velocity.twist.linear.y  - estimator_velocity.twist.linear.y,
	       reference_velocity.twist.linear.z  - estimator_velocity.twist.linear.z,
	       reference_velocity.twist.angular.z - estimator_velocity.twist.angular.z;
      Eigen::Vector4d xd_dot;
      xd_dot << reference_velocity.twist.linear.x,
                reference_velocity.twist.linear.y,
                reference_velocity.twist.linear.z,
                reference_velocity.twist.angular.z;

      // Sliding surface 
      //sigma << e + ewise(zetta1, sig(e, lambda1)); 
      sigma << e + ewise(zetta1, sig(e, lambda1)) + ewise(zetta2, sig(e_dot, lambda2));

      // Check for NaN in sigma. Old check, not needed anymore, kept for security
      for (int i = 0; i < 4; i++){
	if (std::isnan(sigma(i))){
	  std::cout << "Sigma is NaN at element " << i << std::endl;
	  sigma(i) = 0;
	}
      }

      // K dynamics
      K_dot << ewise(exp4(alpha, 0.5), exp4(sigma.cwiseAbs(), 0.5)) - ewise(exp4(beta, 0.5), exp4(K, 2));

      // Euler integrate K_dot to get K
      K += K_dot * 0.01;


      // Control law 
      //uaux << -2 * ewise(K, sig(sigma, 0.5)) - ewise(exp4(K, 2), sigma) * 0.5; //Original
      uaux << -2 * ewise(K, sig(sigma, 0.5)) - ewise(K, sigma) * 0.5;
      //uaux << -1 * ewise(zetta1, e) - 1 * ewise(lambda1, e_dot);
      //uaux << ewise(e, zetta1); 
              
      // Original feedback linearization
      /*u_rot << 0,
               -uaux(0) + ewise(zetta1, sig(e, lambda1))(0) + xd_dot(0),
               -uaux(1) + ewise(zetta1, sig(e, lambda1))(1) + xd_dot(1),
               -uaux(2) + ewise(zetta1, sig(e, lambda1))(2) + xd_dot(2);*/
      
      // New feedback linearization, lyapunov based
      term1 << 1 + zetta1(0) * lambda1(0) * pow(abs(e(0)), lambda1_minus_1(0)),
	       1 + zetta1(1) * lambda1(1) * pow(abs(e(1)), lambda1_minus_1(1)), 
	       1 + zetta1(2) * lambda1(2) * pow(abs(e(2)), lambda1_minus_1(2)),
	       1 + zetta1(3) * lambda1(3) * pow(abs(e(3)), lambda1_minus_1(3));
      
      Eigen::Vector4d term3;
      term3 << sig(e_dot(0), 2 - lambda2(0)) / (zetta2(0) * lambda2(0)),
	       sig(e_dot(1), 2 - lambda2(1)) / (zetta2(1) * lambda2(1)),
	       sig(e_dot(2), 2 - lambda2(2)) / (zetta2(2) * lambda2(2)),
	       sig(e_dot(3), 2 - lambda2(3)) / (zetta2(3) * lambda2(3)); 

      /*u_rot << 0,
               xd_dot(0) -uaux(0)/term1(0),
	       xd_dot(1) -uaux(1)/term1(1),
	       xd_dot(2) -uaux(2)/term1(2);*/

      // Attitude term
      Eigen::Vector3d term2;
      Eigen::Vector3d OMEGA;
      Eigen::Matrix3d OMEGA_cross;
      Eigen::Matrix3d J_inv;
      J_inv = J.inverse();
      OMEGA << estimator_velocity.twist.angular.x, estimator_velocity.twist.angular.y, estimator_velocity.twist.angular.z;
      OMEGA_cross << 0, -OMEGA(2), OMEGA(1),
	             OMEGA(2), 0, -OMEGA(0),
	             -OMEGA(1), OMEGA(0), 0;
      
      // omega x J omega
      //std::cout << "Sanity check " <<  J * OMEGA << std::endl;
      //std::cout << "Sanity check " <<  OMEGA_cross * (J * OMEGA) << std::endl;
      //std::cout << "OMEGA" << OMEGA << std::endl;
      //std::cout << "Sanity check " <<  J_inv * (OMEGA_cross * (J * OMEGA)) << std::endl;
      term2 = J_inv * (OMEGA_cross * (J * OMEGA)); 

      /*u << -uaux(0), 
	   -uaux(1), 
	   -uaux(2), 
	   -uaux(3) - term2(2);*/
      u << -uaux(0) + term1(0) * term2(0),
	   -uaux(1) + term1(1) * term2(1),
	   -uaux(2) + term1(2) * term2(2),
	   -uaux(3) - term2(2) + term1(3) * term3(2);

	   //-uaux(3);*/

      // Trapezoidal integral of u
      u_int += 0.01 * (u + u_prev) / 2;
      u_prev = u;


      u_rot << 0,
               /*xd_dot(0) - ewise(ewise(zetta1, lambda1), uaux)(0)/term1(0),
	       xd_dot(1) - ewise(ewise(zetta1, lambda1), uaux)(1)/term1(1),
	       xd_dot(2) - ewise(ewise(zetta1, lambda1), uaux)(2)/term1(2);*/
               /*-uaux(0),
	       -uaux(1),
	       -uaux(2);*/
               u_int(0),
	       u_int(1),
	       u_int(2);
      u_rot = kronecker(kronecker(q_hat_conj, u_rot), q_hat);
      //u_rot = kronecker(kronecker(q_hat, u_rot), q_hat_conj);

      //uaux(3) = -uaux(3) + xd_dot(3) + ewise(zetta1, sig(e, lambda1))(3);
      //uaux(3) = xd_dot(3) - uaux(3) / term1(3);
      //uaux(3) = xd_dot(3) - ewise(ewise(zetta1, lambda1), uaux)(3) / term1(3);
      /*u << u_rot(1), 
	   u_rot(2), 
	   u_rot(3), 
	   -uaux(3) + term2(2);

      // Trapezoidal integral of u
      u_int += 0.01 * (u + u_prev) / 2;
      u_prev = u;*/
      
      
      //std::cout << "u: " << u << std::endl;
      //std::cout << "u_int: " << u_int << std::endl;
      /*uaux(0) = std::min(std::max(u_rot(1), -1.6), 1.6);
      uaux(1) = std::min(std::max(u_rot(2), -1.6), 1.6);
      uaux(2) = std::min(std::max(u_rot(3), -1.0), 1.0);
      uaux(3) = std::min(std::max(uaux(3),  -1.0), 1.0);*/

      /*u_t(0) = std::min(std::max(u_int(0), -1.6), 1.6);
      u_t(1) = std::min(std::max(u_int(1), -1.6), 1.6);
      u_t(2) = std::min(std::max(u_int(2), -1.0), 1.0);
      u_t(3) = std::min(std::max(u_int(3), -1.0), 1.0);*/
      u_t(0) = std::min(std::max(u_rot(1), -1.6), 1.6);
      u_t(1) = std::min(std::max(u_rot(2), -1.6), 1.6);
      u_t(2) = std::min(std::max(u_rot(3), -1.0), 1.0);
      u_t(3) = std::min(std::max(u_int(3), -1.0), 1.0);

      // Normalize control output
      
      /*uaux(0) = 100 * (uaux(0) + 1.6)/(3.2) - 50;
      uaux(1) = 100 * (uaux(1) + 1.6)/(3.2) - 50;
      //uaux(2) = 100 * (uaux(2) + 0.9)/(1.9) - 50;
      uaux(2) = 100 * (uaux(2) + 1.0)/(2.0) - 50;
      uaux(3) = 100 * (uaux(3) + 1.0)/(2.0) - 50;*/
      
      u_t(0) = 200 * (u_t(0) + 1.6)/(3.2) - 100;
      u_t(1) = 200 * (u_t(1) + 1.6)/(3.2) - 100;
      u_t(2) = 200 * (u_t(2) + 1.0)/(2.0) - 100;
      u_t(3) = 200 * (u_t(3) + 1.0)/(2.0) - 100;

      /*_uaux.linear.x =  uaux(0);
      _uaux.linear.y =  uaux(1);
      _uaux.linear.z =  uaux(2);
      _uaux.angular.z = uaux(3);*/

      _uaux.linear.x  = u_t(0);  
      _uaux.linear.y  = u_t(1); 
      _uaux.linear.z  = u_t(2); 
      _uaux.angular.z = u_t(3);

      _sigma.linear.x = sigma(0);
      _sigma.linear.y = sigma(1);
      _sigma.linear.z = sigma(2);
      _sigma.angular.z = sigma(3);

      _error.pose.position.x = e(0);
      _error.pose.position.y = e(1);
      _error.pose.position.z = e(2);
      _error.pose.orientation.w = e(3);

      _ref_rot.pose.position.x = u_rot(1);
      _ref_rot.pose.position.y = u_rot(2);
      _ref_rot.pose.position.z = u_rot(3);
      //_ref_rot.pose.position.x = K(0);
      //_ref_rot.pose.position.y = K(1);
      //_ref_rot.pose.position.z = K(2);
      //_ref_rot.pose.orientation.w = K(3);
      _ref_rot.pose.orientation.w = reference_pose.pose.orientation.w;
      _ref_rot.pose.orientation.x = reference_pose.pose.orientation.x;
      _ref_rot.pose.orientation.y = reference_pose.pose.orientation.y;
      _ref_rot.pose.orientation.z = reference_pose.pose.orientation.z;
      _ref_rot.header.frame_id = "world";

      _k.twist.linear.x  = K(0);
      _k.twist.linear.y  = K(1);
      _k.twist.linear.z  = K(2);
      _k.twist.angular.z = K(3);
      _k.header.frame_id = "tello";
      _k.header.stamp = this->now();

      _e_dot.twist.linear.x  = e_dot(0);
      _e_dot.twist.linear.y  = e_dot(1);
      _e_dot.twist.linear.z  = e_dot(2);
      _e_dot.twist.angular.z = e_dot(3);
      _e_dot.header.frame_id = "tello";
      _e_dot.header.stamp = this->now();

      uaux_publisher->publish(_uaux);
      sigma_publisher->publish(_sigma);
      error_publisher->publish(_error);
      ref_rot_publisher->publish(_ref_rot);
      
      k_publisher->publish(_k);
      e_dot_publisher->publish(_e_dot);
    }

  private:

    geometry_msgs::msg::PoseStamped estimator_pose; 
    geometry_msgs::msg::PoseStamped reference_pose;
    geometry_msgs::msg::PoseStamped _error;
    geometry_msgs::msg::PoseStamped _ref_rot;
    geometry_msgs::msg::TwistStamped estimator_velocity;
    geometry_msgs::msg::TwistStamped reference_velocity;
    geometry_msgs::msg::TwistStamped _k;
    geometry_msgs::msg::TwistStamped _e_dot;
    geometry_msgs::msg::Twist _uaux;
    geometry_msgs::msg::Twist _sigma;

    Eigen::Vector4d zetta1;
    Eigen::Vector4d zetta2;
    Eigen::Vector4d lambda1;
    Eigen::Vector4d lambda1_minus_1;
    Eigen::Vector4d lambda2;
    Eigen::Vector4d term1;
    Eigen::Vector4d e;
    Eigen::Vector4d ref_rot;
    Eigen::Vector4d e_dot;
    Eigen::Vector4d q_hat;
    Eigen::Vector4d q_hat_conj;
    Eigen::Vector4d q_d;
    Eigen::Vector4d q_e;
    Eigen::Vector4d eta_e;
    Eigen::Vector4d K;
    Eigen::Vector4d K_dot;
    Eigen::Vector4d alpha;
    Eigen::Vector4d beta;
    Eigen::Vector4d sigma;
    Eigen::Vector4d uaux;
    Eigen::Vector4d u_rot;
    Eigen::Vector4d u;
    Eigen::Vector4d u_int;
    Eigen::Vector4d u_t;
    Eigen::Vector4d u_prev;
    Eigen::Matrix3d J;

    int drone_id;

    float Ixx;
    float Iyy;
    float Izz;

    std::vector<double> gains;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr estimator_velocity_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sigma_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rot_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr k_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr e_dot_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavTest>());
  rclcpp::shutdown();
  return 0;
}
