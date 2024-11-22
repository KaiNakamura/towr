#ifndef XPP_VIS_INVERSEKINEMATICS_GO1_H_
#define XPP_VIS_INVERSEKINEMATICS_GO1_H_

#include <xpp_vis/inverse_kinematics.h>
#include <towr/models/go1/inverse_kinematics_go1.h>
#include <towr/models/go1/go1leg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the Unitree Go1 robot.
 */
class InverseKinematicsGo1 : public InverseKinematics {
public:
  InverseKinematicsGo1() = default;
  virtual ~InverseKinematicsGo1() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };

private:
  Vector3d base2hip_LF_ = Vector3d(0.1881, 0.04675 + 0.08, 0.0);
  Go1legInverseKinematics leg;
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSEKINEMATICS_GO1_H_ */