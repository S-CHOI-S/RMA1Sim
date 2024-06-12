//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <math.h>
#include <set>
#include "../../RaisimGymEnv.hpp"

namespace raisim {

    class ENVIRONMENT : public RaisimGymEnv {

    public:

        explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
                RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {

            /// create world
            world_ = std::make_unique<raisim::World>();

            /// add objects
            anymal_ = world_->addArticulatedSystem(resourceDir_+"/anymal/urdf/anymal.urdf");
            anymal_->setName("anymal");
            anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
            world_->addGround();

            /// add box
            
            posError_.setZero();
            READ_YAML(double, boxSize_, cfg["box"]["box_size"]);
            READ_YAML(double, finalBoxHeight_, cfg["box"]["final_box_height"]);
            READ_YAML(double, curriculumFactor_, cfg["curriculum"]["initial_factor"]);
            READ_YAML(double, curriculumDecayFactor_, cfg["curriculum"]["decay_factor"]);
            goalPos_ << 1.5, 0.0, 0.0;
            goalVel_ << 0.0, 0.0, sqrt(2 * 10 * (finalBoxHeight_ + 0.5));
            boxHeight_ = finalBoxHeight_;
            // auto box_ = world_->addBox(boxSize_, boxSize_, boxHeight_, 100);
            box_ = static_cast<raisim::Box*>(world_->addBox(boxSize_, boxSize_, boxHeight_, 100));

            box_->setPosition(goalPos_(0), goalPos_(1), goalPos_(2) + boxHeight_ / 2);
            box_->setBodyType(raisim::BodyType::STATIC);

            /// get robot data
            gcDim_ = anymal_->getGeneralizedCoordinateDim();
            gvDim_ = anymal_->getDOF();
            nJoints_ = gvDim_ - 6;

            /// initialize containers
            gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
            gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
            pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

            /// this is nominal configuration of anymal
            gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

            /// set pd gains
            Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
            jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
            jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);
            anymal_->setPdGains(jointPgain, jointDgain);
            anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

            /// MUST BE DONE FOR ALL ENVIRONMENTS
            obDim_ = 36;
            actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
            obDouble_.setZero(obDim_);
            previousAction_.setZero(actionDim_);

            /// action scaling
            actionMean_ = gc_init_.tail(nJoints_);
            double action_std;
            READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
            actionStd_.setConstant(action_std);
            previousAction_ << actionMean_;

            /// Reward coefficients
            READ_YAML(double, positionCoeff_, cfg["reward"]["position"]["coeff"]);
            READ_YAML(double, jumpCoeff_, cfg["reward"]["jump"]["coeff"]);
            READ_YAML(double, torqueCoeff_, cfg["reward"]["torque"]["coeff"]);

            /// indices of links that should not make contact with ground
            footIndices_.insert(anymal_->getBodyIdx("LF_SHANK"));
            footIndices_.insert(anymal_->getBodyIdx("RF_SHANK"));
            footIndices_.insert(anymal_->getBodyIdx("LH_SHANK"));
            footIndices_.insert(anymal_->getBodyIdx("RH_SHANK"));

            /// visualize if it is the first environment
            if (visualizable_) {
                server_ = std::make_unique<raisim::RaisimServer>(world_.get());
                server_->launchServer();
                server_->focusOn(anymal_);
            }
        }

        void init() final { }

        void reset() final {
            anymal_->setState(gc_init_, gv_init_);
            updateObservation();
        }

        float step(const Eigen::Ref<EigenVec>& action) final {
            /// action scaling
            pTarget12_ = action.cast<double>();
            pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
            pTarget12_ += actionMean_;
            pTarget_.tail(nJoints_) = pTarget12_;

            anymal_->setPdTarget(pTarget_, vTarget_);

            positionReward_ = 0.0;
            jumpReward_ = 0.0;
            torqueReward_ = 0.0;

            double lowerError = 5.0;

            auto current_box_height = goalPos_(2) + curriculumFactor_ * boxHeight_;

            box_->setPosition(goalPos_(0), goalPos_(1), current_box_height - boxHeight_ / 2);
            float howManySteps = 0.0;
            for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
                if(server_) server_->lockVisualizationServerMutex();
                world_->integrate();
                if(server_) server_->unlockVisualizationServerMutex();
                updateObservation();

                bool allFeetInAir = true;
                for(auto& contact : anymal_->getContacts()){
                    if(footIndices_.find(contact.getlocalBodyIndex()) != footIndices_.end()){
                        allFeetInAir = false;
                        break;
                    }
                }

                // bool allFeetOnBox = true;
                // for (auto& footIndex : footIndices_) {
                //     raisim::Vec<3> footPos;
                //     anymal_->getPosition(footIndex, footPos);
                //     if (!(footPos[0] >= goalPos_[0] - boxSize_ / 2 && footPos[0] <= goalPos_[0] + boxSize_ / 2 &&
                //         footPos[1] >= goalPos_[1] - boxSize_ / 2 && footPos[1] <= goalPos_[1] + boxSize_ / 2)) {
                //         allFeetOnBox = false;
                //         break;
                //     }
                // }

                posError_ = goalPos_ - gc_.head(3);
                float velError_ = (goalVel_ - gv_.head(3)).tail(2).norm();
                torqueReward_ += torqueCoeff_ * anymal_->getGeneralizedForce().squaredNorm() * getSimulationTimeStep();
                float xyError = posError_.norm();
                
                double bodyDirectionError = ((gv_.head(2)) - (bodyLinearVel_.head(2))).norm() + abs(bodyAngularVel_(0)) + 10 * abs(bodyAngularVel_(2));
                double landingError = (gv_init_.tail(12) - gv_.tail(12)).norm() + (gc_init_.tail(12) - gc_.tail(12)).norm() + abs(current_box_height + 0.5 - gc_(2));
                if(xyError < boxSize_ / 2){
                    jumpReward_ += jumpCoeff_ * exp(-3 * landingError);
                    positionReward_ += positionCoeff_ * exp(-3 * xyError);
                }
                else{
                    jumpReward_ += jumpCoeff_ * exp(-5 * velError_ - 4 * bodyDirectionError);
                    if(allFeetInAir){
                        positionReward_ += positionCoeff_ * exp(-3 * xyError);
                    }
                }
                
                howManySteps_ += 1;
            }

            float reward = float(torqueReward_ + jumpReward_ + positionReward_);
            // std::cout << "t, p, h" << torqueReward_ << positionReward_ << ", " << float(howManySteps_) << std::endl;
            return reward;
        }

        void updateObservation() {
            anymal_->getState(gc_, gv_);
            raisim::Vec<4> quat;
            raisim::Mat<3,3> rot;
            quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
            raisim::quatToRotMat(quat, rot);
            bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
            bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

            obDouble_ << gc_.head(3), /// body height
                    rot.e().row(2).transpose(), /// body orientation
                    gc_.tail(12), /// joint angles
                    bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
                    gv_.tail(12);  /// joint velocity

            if (obDouble_.hasNaN()) {
                std::cerr << "NaN detected in observation: " << obDouble_.transpose() << std::endl;
            }
        }

        void observe(Eigen::Ref<EigenVec> ob) final {
            /// convert it to float
            ob = obDouble_.cast<float>();
        }

        bool isTerminalState(float& terminalReward) final {
            terminalReward = float(terminalRewardCoeff_);

            /// if the contact body is not feet
            for(auto& contact: anymal_->getContacts()){
                // auto contactObject = world_->getObject(contact.getPairObjectIndex());
                if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
                    return true;
            } 

            auto xy_error = posError_.head(2).norm();

            if(xy_error > goalPos_.norm() + 0.2){
                terminalReward = -0.1;
                return true;
            }
            if(abs(gc_(1)) > boxSize_ / 2){
                terminalReward = -0.1;
                return true;
            }
            if(gc_(1) > goalPos_(1) + 0.3){
                terminalReward = -0.1;
                return true;
            }


            // auto joint_velocity = gv_.tail(12);
            // for(int i=0; i < 12; i++){
            //     if (abs(joint_velocity[i]) >= 40 + 40 * (0.98-curriculumFactor_)){
            //         terminalReward = -1.f;
            //         return true;
            //     }
            // }

            terminalReward = 0.f;
            return false;
        }

        void curriculumUpdate() final {
            curriculumFactor_ = std::pow(curriculumFactor_, curriculumDecayFactor_);
            auto current_cylinder_height = goalPos_(2) + (-boxHeight_/2 * (1-curriculumFactor_)) + curriculumFactor_ * boxHeight_/2 + boxHeight_/2;
            RSINFO_IF(visualizable_, "Curriculum factor: "<< curriculumFactor_);
            RSINFO_IF(visualizable_, "Current height: " << current_cylinder_height);
        };

    private:
        int gcDim_, gvDim_, nJoints_;
        int howManySteps_ = 0;
        bool visualizable_ = false;
        raisim::ArticulatedSystem* anymal_;
        raisim::Box* box_;
        Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
        double terminalRewardCoeff_ = -5.;
        Eigen::VectorXd previousAction_;
        Eigen::VectorXd actionMean_, actionStd_, obDouble_;
        Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
        Eigen::Vector3d goalPos_, posError_, goalVel_, velError_;
        // Eigen::Vector4d footPos_;
        std::set<size_t> footIndices_;
        double boxSize_, boxHeight_ , finalBoxHeight_;
        double positionReward_, jumpReward_, torqueReward_;
        double positionCoeff_, jumpCoeff_, torqueCoeff_;
        double curriculumFactor_, curriculumDecayFactor_;

        /// these variables are not in use. They are placed to show you how to create a random number sampler.
        std::normal_distribution<double> normDist_;
        thread_local static std::mt19937 gen_;
    };
    thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

