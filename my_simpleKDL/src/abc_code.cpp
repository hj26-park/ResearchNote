#include <ros/ros.h>
#include <std_msgs/Int32.h>

// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <urdf/model.h>
// for wrenches
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49


int main(int argc, char ** argv)
{
    //Initialize and start the node
    ros::init(argc, argv, "my_simple_KDL");
    ros::NodeHandle nh;    
    hardware_interface::EffortJointInterface *hw;
    
    // ros publisher
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("abc_my_simple_topic", 1000);
    std_msgs::Int32 abc;
    abc.data = 0;
    ros::Rate(200);
    ros::Rate r(1000);

    int control_cnt = 0;
    //============== kdl variable ==========
    double t;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?
    
    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_, qddot_;
    KDL::JntArray e_, e_dot_, e_int_;
    KDL::JntArray torques_;   

    // Wrench
    KDL::Wrench f_ext_;
    KDL::Wrenches f_i;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;    

    //kdl ros publisher    
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;
    //========================


   // pub.publish(abc);
    ROS_INFO("my_simple_topic START!!");

    for(int ii = 0; ii < 10; ii++)
    {
        pub.publish(abc);
        ros::spinOnce();    
        
        //ROS_INFO("%d \n", ii);
        printf("counter: %d \t", ii);
        printf("\n");
    }
    
    // * ============= custom urdf/kdl parser start ============    
    urdf::Model urdf;
    //if (!urdf.initParam("/dsr01/robot_description"))
    if (!urdf.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }
    else
    {
        ROS_INFO("Found robot_description");
    }

    // 4.1 kdl parser
    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    else
    {
        ROS_INFO("Constructed kdl tree");
    }

    // ********* 1. Get joint name / gain from the parameter server *********
    // 1.1 Joint Name
    joint_names_.push_back("joint1");
    joint_names_.push_back("joint2");
    joint_names_.push_back("joint3");
    joint_names_.push_back("joint4");
    joint_names_.push_back("joint5");
    joint_names_.push_back("joint6");
    n_joints_ = joint_names_.size();

    /// for debugging
    std::cout << n_joints_ << std::endl;
    for (int ii = 0; ii < n_joints_; ii++)
    {
        std::cout << joint_names_[ii] << std::endl;
    }
     
    // 4.2 kdl chain
    std::string root_name = "base_0";
    std::string tip_name = "link6";

    if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for (it = segment_map.begin(); it != segment_map.end(); it++)
            ROS_ERROR_STREAM("    " << (*it).first);

        return false;
    }
    else
    {
        ROS_INFO("Got kdl chain");
    }

    // 4.3 inverse dynamics solver 초기화
    gravity_ = KDL::Vector::Zero(); // ?
    gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

    // ********* 5. 각종 변수 초기화 *********

    // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
    tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

    qd_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

    q_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_.data = Eigen::VectorXd::Zero(n_joints_);
    qddot_.data = Eigen::VectorXd::Zero(n_joints_);  
    torques_.data = Eigen::VectorXd::Zero(n_joints_);  
    // M_.data = Eigen::VectorXd::Zero(n_joints_);  
    // C_.data = Eigen::VectorXd::Zero(n_joints_);  
    // G_.data = Eigen::VectorXd::Zero(n_joints_);  
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());
    SetToZero(f_ext_);
    
    e_.data = Eigen::VectorXd::Zero(n_joints_);
    e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
    e_int_.data = Eigen::VectorXd::Zero(n_joints_);

    // * ============= custom urdf/kdl parser end ============* /    
    static int count = 0;
    const ros::Duration period;
    int joint_angle = 90;

    // RNE solver constructor init
    KDL::ChainIdSolver_RNE rne_solver(kdl_chain_, gravity_);
    // rne_solver.CartToJnt(q_, qd_, qddot_, f_ext_, torques_);  f_i
    // rne_solver.CartToJnt(q_, qd_, qddot_, f_i, torques_);

    // std::cout << f_ext_ << std::endl;
    // printf(" =============== f_ext_: %f \n", f_ext_);
    // printf(" =============== \n \n \n ");

    // printf(" =============== torques_: %f \n", torques_(0));
    // printf(" =============== \n \n \n ");

// ============ qd, qdd = 0 =============
    // q_(0) = 0 * D2R;
    // q_(1) = joint_angle * D2R;
    // q_(2) = 0 * D2R;
    // q_(3) = 0 * D2R;
    // q_(4) = 0 * D2R;
    // q_(5) = 0 * D2R;

    // qdot_(0) = 0 * D2R;
    // qdot_(1) = 0 * D2R;
    // qdot_(2) = 0 * D2R;
    // qdot_(3) = 0 * D2R;
    // qdot_(4) = 0 * D2R;
    // qdot_(5) = 0 * D2R;
    
    // qddot_(0) = 0 * D2R;
    // qddot_(1) = joint_angle * D2R;
    // qddot_(2) = 0 * D2R;
    // qddot_(3) = 0 * D2R;
    // qddot_(4) = 0 * D2R;
    // qddot_(5) = 0 * D2R;

// ============ q, qd, qdd = 90 =============
    // q_(0) = 0 * D2R;
    // q_(1) = joint_angle * D2R;
    // q_(2) = 0 * D2R;
    // q_(3) = 0 * D2R;
    // q_(4) = 0 * D2R;
    // q_(5) = 0 * D2R;

    // qdot_(0) = 0 * D2R;
    // qdot_(1) = joint_angle * D2R;
    // qdot_(2) = 0 * D2R;
    // qdot_(3) = 0 * D2R;
    // qdot_(4) = 0 * D2R;
    // qdot_(5) = 0 * D2R;
    
    // qddot_(0) = 0 * D2R;
    // qddot_(1) = joint_angle * D2R;
    // qddot_(2) = 0 * D2R;
    // qddot_(3) = 0 * D2R;
    // qddot_(4) = 0 * D2R;
    // qddot_(5) = 0 * D2R;
        
// ============ all joint q, qd, qdd = 90 =============        
    q_(0) = joint_angle * D2R;
    q_(1) = joint_angle * D2R;
    q_(2) = joint_angle * D2R;
    q_(3) = joint_angle * D2R;
    q_(4) = joint_angle * D2R;
    q_(5) = joint_angle * D2R;

    qdot_(0) = joint_angle * D2R;
    qdot_(1) = joint_angle * D2R;
    qdot_(2) = joint_angle * D2R;
    qdot_(3) = joint_angle * D2R;
    qdot_(4) = joint_angle * D2R;
    qdot_(5) = joint_angle * D2R;
    
    qddot_(0) = joint_angle * D2R;
    qddot_(1) = joint_angle * D2R;
    qddot_(2) = joint_angle * D2R;
    qddot_(3) = joint_angle * D2R;
    qddot_(4) = joint_angle * D2R;
    qddot_(5) = joint_angle * D2R;

    id_solver_->JntToMass(q_, M_); 
    id_solver_->JntToCoriolis(q_, qdot_, C_); 
    id_solver_->JntToGravity(q_, G_); 

    aux_d_.data = M_.data * qddot_.data;
    comp_d_.data = C_.data + G_.data;
    tau_d_.data = aux_d_.data + comp_d_.data;
    // rne_solver.CartToJnt(q_, qd_, qddot_, f_i, torques_);
    // printf("%f \n ", M_(1));
    printf("coli: %f \n ", C_(1));
    printf("grav: %f \n ", G_(1));
    printf("grav: %f %f %f %f %f %f \n ", G_(1), G_(2), G_(3), G_(4), G_(5), G_(6));
    printf("tau_: %f \n ", tau_d_(1));


    printf("*** Actual State in Joint Space (unit: deg) ***\n");
    printf("q_: %f, %f, %f, %f, %f, %f ", 
                    q_(0) * R2D, 
                    q_(1) * R2D, 
                    q_(2) * R2D, 
                    q_(3) * R2D, 
                    q_(4) * R2D, 
                    q_(5) * R2D);
    printf("\n");      

    printf("*** Desired State in Joint Space (unit: deg) ***\n");
    printf("qdot_: %f, %f, %f, %f, %f, %f ", 
                    qdot_(0) * R2D, 
                    qdot_(1) * R2D, 
                    qdot_(2) * R2D, 
                    qdot_(3) * R2D, 
                    qdot_(4) * R2D, 
                    qdot_(5) * R2D);
    printf("\n");


    printf("*** Actual State in Joint Space (unit: deg/s) ***\n");
    printf("qddot_: %f, %f, %f, %f, %f, %f ", 
                    qddot_(0) * R2D, 
                    qddot_(1) * R2D, 
                    qddot_(2) * R2D, 
                    qddot_(3) * R2D, 
                    qddot_(4) * R2D, 
                    qddot_(5) * R2D);
    printf("\n"); 

    printf("*** Caculated Torque (unit: ) ***\n");            
    printf("tau : %f, %f, %f, %f, %f, %f ", 
                    tau_d_(0), 
                    tau_d_(1), 
                    tau_d_(2), 
                    tau_d_(3), 
                    tau_d_(4), 
                    tau_d_(5));
    printf("\n"); 



    return 0;  

    

///////////////////////////////////////////////
    // while (ros::ok)
    // {   
        
    //     // ==================== Dynamics Cal ==================
    //      // ********* 0. Get states from gazebo *********
    //     // 0.1 sampling time
    //     double dt = period.toSec();
    //     t = t + 0.001;


      
    //     rne_solver.CartToJnt(q_, qd_, qddot_, f_i, torques_);
                
    //     count++;
    //     if(count > 300)
    //     {
    //         printf("now t: %lf \n", t);            
    //         // printf("now dt: %lf \n", dt);            
    //         printf("*** Actual State in Joint Space (unit: deg) ***\n");
    //         printf("q_: %f, %f, %f, %f, %f, %f ", 
    //                         q_(0) * R2D, 
    //                         q_(1) * R2D, 
    //                         q_(2) * R2D, 
    //                         q_(3) * R2D, 
    //                         q_(4) * R2D, 
    //                         q_(5) * R2D);
    //         printf("\n");      

    //         printf("*** Desired State in Joint Space (unit: deg) ***\n");
    //         printf("qdot_: %f, %f, %f, %f, %f, %f ", 
    //                         qdot_(0) * R2D, 
    //                         qdot_(1) * R2D, 
    //                         qdot_(2) * R2D, 
    //                         qdot_(3) * R2D, 
    //                         qdot_(4) * R2D, 
    //                         qdot_(5) * R2D);
    //         printf("\n");


    //         printf("*** Actual State in Joint Space (unit: deg/s) ***\n");
    //         printf("qddot_: %f, %f, %f, %f, %f, %f ", 
    //                         qddot_(0) * R2D, 
    //                         qddot_(1) * R2D, 
    //                         qddot_(2) * R2D, 
    //                         qddot_(3) * R2D, 
    //                         qddot_(4) * R2D, 
    //                         qddot_(5) * R2D);
    //         printf("\n"); 

    //         printf("*** Caculated Torque (unit: ) ***\n");            
    //         printf("tau : %f, %f, %f, %f, %f, %f ", 
    //                         torques_(0), 
    //                         torques_(1), 
    //                         torques_(2), 
    //                         torques_(3), 
    //                         torques_(4), 
    //                         torques_(5));
    //         printf("\n"); 
    //         // std::cout << "tau: " << torques_(3) << std::endl;

    //         count = 0;
    //     }
        
    //     r.sleep();
    // }
}