//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_OBJECT_HPP
#define RAISIM_OBJECT_HPP

#include <memory>
#include <vector>
#include "raisim/math.hpp"
#include "raisim/contact/PerObjectContactList.hpp"
#include "raisim/contact/BisectionContactSolver.hpp"
#include "raisim/Materials.hpp"
#include "raisim/configure.hpp"

#include "ode/ode.h"

namespace raisim {

class World;

namespace contact {
class BisectionContactSolver;
class Single3DContactProblem;
}

class Object {
  friend class raisim::World;
  friend class raisim::contact::BisectionContactSolver;

 public:
  explicit Object();
  virtual ~Object() = default;

 public:
  void clearPerObjectContact();
  void addContactToPerObjectContact(contact::Contact &contact);
  void setIndexInWorld(size_t indexInWorld_);

  size_t getIndexInWorld() const;
  const std::vector<contact::Contact> &getContacts() const;
  std::vector<contact::Contact> &getContacts();

  virtual void updateCollision() = 0;
  virtual void preContactSolverUpdate1(const Vec<3> &gravity, double dt) = 0;
  virtual void preContactSolverUpdate2(const Vec<3> &gravity, double dt) = 0;
  virtual void integrate(double dt, const Vec<3>& gravity) = 0;

  /// apply forces at the Center of Mass
  virtual void setExternalForce(size_t localIdx, const Vec<3>& force) = 0;
  /// apply torque on a body
  virtual void setExternalTorque(size_t localIdx, const Vec<3>& torque) = 0;
  /// apply force (expressed in the world frame) at specific location of the body (expressed in the body frame)
  virtual void setExternalForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) = 0;

  virtual double getMass(size_t localIdx) const = 0;
  virtual ObjectType getObjectType() const = 0;
  virtual void getPosition(size_t localIdx, Vec<3>& pos_w) const = 0;
  virtual void getVelocity(size_t localIdx, Vec<3>& vel_w) const = 0;
  virtual void getOrientation(size_t localIdx, Mat<3,3>& rot) const = 0;
  virtual void getPosition(size_t localIdx, const Vec<3>& pos_b, Vec<3>& pos_w) const = 0;
  virtual BodyType getBodyType(size_t localIdx) const { return bodyType_; };
  virtual BodyType getBodyType() const { return bodyType_; };
  virtual void getContactPointVel(size_t pointId, Vec<3> &vel) = 0;
  void setName(const std::string& name) { name_ = name; }
  const std::string& getName() const { return name_; }

 protected:
  std::vector<std::pair<std::vector <raisim::Mat<3, 3>>, raisim::Vec<3>>> &getDelassusAndTauStar();
  double &getImpactVel(size_t idx);
  contact::PerObjectContactList &getPerObjectContact();
  virtual void destroyCollisionBodies(dSpaceID id) = 0;
  virtual void addContactPointVel(size_t pointId, Vec<3>& vel) = 0;
  virtual void subContactPointVel(size_t pointId, Vec<3>& vel) = 0;
  virtual void updateTimeStep(double dt) = 0;
  virtual void updateTimeStepIfNecessary(double dt) = 0;
  virtual void updateGenVelWithImpulse(size_t pointId, const Vec<3>& imp) = 0;
  virtual void appendJointLimits(std::vector<contact::Single3DContactProblem, AlignedAllocator<contact::Single3DContactProblem, 32>>& problem) {};
  virtual double enforceJointLimits(contact::Single3DContactProblem& problem) { return 0; };

  contact::PerObjectContactList perObjectContact_;
  size_t contactSize_ = 0;

  size_t indexInWorld_;
  BodyType bodyType_ = BodyType::DYNAMIC;
  bool useDel_ = true;
  bool visualizeFramesAndCom_ = true;
  std::string name_;
  std::vector<std::string> localNames_;
};

} // raisim

#endif //RAISIM_COLLISIONOBJECT_HPP
