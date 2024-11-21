#ifndef XPP_VIS_GO1LEG_INVERSE_KINEMATICS_H_
#define XPP_VIS_GO1LEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

namespace xpp {

enum Go1JointID {HAA=0, HFE, KFE, Go1legJointCount};

/**
 * @brief Converts a go1 foot position to joint angles.
 */
class Go1legInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  Go1legInverseKinematics () = default;
  virtual ~Go1legInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, Go1JointID joint) const;

private:
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.0); //distance of HFE to HAA in z direction
  double length_thigh = 0.213; // length of upper leg
  double length_shank = 0.213; // length of lower leg
};

} /* namespace xpp */

#endif /* XPP_VIS_GO1LEG_INVERSE_KINEMATICS_H_ */