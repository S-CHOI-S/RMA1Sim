//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>


namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {

    /// create world
    world_ = std::make_unique<raisim::World>();
    world_->setGravity(Eigen::Vector3d(0, 0, -9.81));

    /// add objects
    a1_ = world_->addArticulatedSystem(resourceDir_+"/a1/urdf/a1.urdf");
    //a1_ = world_->addArticulatedSystem(resourceDir_+"/anymal/urdf/anymal.urdf");
    a1_->setName("a1");
    a1_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    auto ground = world_->addGround();
    //world_->setMaterialPairProp("ground","a1_",0.8,0.5,0.1);

    /// get robot data
    gcDim_ = a1_->getGeneralizedCoordinateDim();
    gvDim_ = a1_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of anymal
    //a1
    gc_init_ << 0, 0, 0.34, 1.0, 0.0, 0.0, 0.0, 0.05, 0.8, -1.4, -0.05, 0.8, -1.4, 0.05, 0.8, -1.4, -0.05, 0.8, -1.4;
    //anymal
    //gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    //jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
    //jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);
    //a1
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(55.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.6);

    a1_->setPdGains(jointPgain, jointDgain);
    a1_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 38;  // 
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    
    footIndices_.insert(a1_->getBodyIdx("FR_calf"));
    footIndices_.insert(a1_->getBodyIdx("FL_calf"));
    footIndices_.insert(a1_->getBodyIdx("RR_calf"));
    footIndices_.insert(a1_->getBodyIdx("RL_calf"));

    //footIndices_.insert(a1_->getBodyIdx("LF_SHANK"));
    //footIndices_.insert(a1_->getBodyIdx("RF_SHANK"));
    //footIndices_.insert(a1_->getBodyIdx("LH_SHANK"));
    //footIndices_.insert(a1_->getBodyIdx("RH_SHANK"));

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_ -> launchServer();
      server_ -> focusOn(a1_);
    }
  }

  void init() final { }

  void reset() final {
    a1_->setState(gc_init_, gv_init_);
    updateObservation();
  }

  double generateBoundedGaussianNoise(double mean, double stdDev, double lowerBound, double upperBound) {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<double> distribution(mean, stdDev);
    
    double value;
    do {
        value = distribution(generator);
    } while (value < lowerBound || value > upperBound);

    return value;
  }

  inline void printRandomNumber() {
    int control  = 1;
    int rotation = 0;
    int idx=0;
    double radians =0;
    if (control==0){
      if (callCount==0) {
        ac_<< 0, 0, 0;
        std::srand(static_cast<unsigned int>(std::time(0)));
        radians = ((static_cast<double>(std::rand()) / RAND_MAX)-0.5) * M_PI;
        //radians = generateBoundedGaussianNoise(0, 0.2, -0.5, 0.5) * M_PI;
        //radians = 0.0;   // - M_PI/2 ~ M_PI/2 
        fixed1 = 0.5*std::cos(radians);
        fixed2 = 0.5*std::sin(radians);
      }
      if (callCount < (THRESHOLD) ) {
        ac_[0] = fixed1;
        ac_[1] = fixed2;
        ac_[2] = rotation;
        ++callCount;
      } 
      else {
        ac_[0] = 0.0;
        ac_[1] = 0;
        ac_[2] = rotation;
        a1_->setState(gc_init_, gv_init_);
        callCount=0;
      }
    }
    else{
      if (callCount==0) {
        angle_<<0;
        ac_<< 0, 0, 0;
        std::srand(static_cast<unsigned int>(std::time(0)));
        radians = ((static_cast<double>(std::rand()) / RAND_MAX)-0.5) * M_PI;

        radians =  -M_PI/2;  // - M_PI/2 ~ M_PI/2 
        fixed1 = 0.5*std::cos(radians);
        fixed2 = 0.5*std::sin(radians);
      
        std::cout << "direction : " <<  fixed1 <<  fixed2 << std::endl;
        callCount = 1;
      }
      ac_[0] =  fixed1;
      ac_[1] =  fixed2;
      ac_[2] =  rotation;
      std::ofstream file("ai_p_pi2.csv", std::ios::app);  // Append mode
      if (file.is_open()) {
            file << std::fixed << std::setprecision(6);  // Set precision for floating point numbers 
            file << gc_[0] << "," << gc_[1] << "," << fixed1 <<  "," << fixed2 <<  "," << angle_[0]<<"\n";
            file.close();
        }
    }
    // difference between angle
    double inner = (ac_[0] * gv_[0]) + (ac_[1] * gv_[1]);
    double v1 = sqrt(pow(gv_[0], 2) + pow(gv_[1], 2));
    double v2 = sqrt(pow(ac_[0], 2) + pow(ac_[1], 2));
    double cosTheta=0;
    if( v1 == 0 || v2 == 0 )
      angle_[0] = 0;
    else
      cosTheta = inner / (v1*v2) ;
      if ( cosTheta >  1.0) cosTheta =  1.0;
      if ( cosTheta < -1.0) cosTheta = -1.0;
      angle_[0] = acos(cosTheta)* 180.0 / M_PI;
      //std::cout << "cos " << cosTheta << std::endl;

    //std::cout << "angle_[0] " << angle_[0]<< std::endl;
  }

  double compute_forward_reward()
    {
    //ac_[0],ac_[1],ac_[2];
    double r = 0.0;
    double max_speed=0.5;
    double ang_speed=0.0;
    double adaptiveForwardVelRewardCoeff_=35.0;
    double adaptiveAngularVelRewardCoeff_=25.0;
    double angular_r = 0.0;
    double forward_r = 0.0;
    double z_rot = 0.0;
    double pos = -10.0;
    double abs_v = std::sqrt(std::pow(gv_[1], 2.) + std::pow(gv_[0], 2.));

    forward_r = adaptiveForwardVelRewardCoeff_ * (
      std::min(std::abs(ac_[0]),std::abs(gv_[0]))
      +std::min(std::abs(ac_[1]),std::abs(gv_[1])));

    if (abs_v > max_speed)
      r += -adaptiveForwardVelRewardCoeff_ *abs_v;

    if ( gc_[2]< 0.2 )
      r += pos;

    if (gv_[0] > (ac_[0]+0.1) || gv_[0] < (ac_[0]- 0.1))
  	    forward_r -= 20 * std::abs(gv_[0] - ac_[0]);

    if (gv_[1] > (ac_[1]+0.1) || gv_[1] < (ac_[1]- 0.1))
  	    forward_r -= 20 * std::abs(gv_[1] - ac_[1]);

    z_rot=std::atan2(2.0 * (gc_[3]* gc_[6] + gc_[4] * gc_[5]), 1.0 - 2.0 * (gc_[5] * gc_[5] + gc_[6] * gc_[6]));

    angular_r = adaptiveAngularVelRewardCoeff_ * (
      - 5*abs(z_rot)
      - std::abs(angle_[0])/70);

    r += forward_r;
    r += angular_r;
    r += 18.0;

    return r;
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    a1_->setPdTarget(pTarget_, vTarget_);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }
    printRandomNumber();
    updateObservation();

    rewards_.record("torque", a1_->getGeneralizedForce().squaredNorm());
    rewards_.record("forwardVel", compute_forward_reward()/100);

    return rewards_.sum();
  }

  void updateObservation() {
    a1_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    obDouble_ << gc_[2], /// body height
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12), /// joint velocity
        ac_[0],ac_[1],ac_[2],
        angle_[0];
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: a1_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
        return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* a1_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_,ac_,angle_;
  std::set<size_t> footIndices_;
  int callCount=0;
  int THRESHOLD = 300; // 동일한 값을 출력할 횟수
  double fixed1 = 0; // 일정 횟수까지 출력할 고정된 숫자
  double fixed2 = 0;
  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

