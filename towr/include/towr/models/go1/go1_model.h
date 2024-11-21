// Values obtained from: https://github.com/KaiNakamura/legged_control/blob/master/legged_examples/legged_unitree/legged_unitree_description/urdf/go1/const.xacro

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_GO1_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_GO1_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr
{

  /**
   * @brief The Kinematics of the quadruped robot Unitree Go1.
   */
  class Go1KinematicModel : public KinematicModel
  {
  public:
    Go1KinematicModel() : KinematicModel(4)
    {
      const double x_nominal_b = 0.1881;
      const double y_nominal_b = 0.04675 + 0.08;
      const double z_nominal_b = -0.3;

      nominal_stance_.at(LF) << x_nominal_b, y_nominal_b, z_nominal_b;
      nominal_stance_.at(RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
      nominal_stance_.at(LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
      nominal_stance_.at(RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

      max_dev_from_nominal_ << 0.16, 0.12, 0.06;
    }
  };

  /**
   * @brief The Dynamics of the quadruped robot Unitree Go1.
   */
  class Go1DynamicModel : public SingleRigidBodyDynamics
  {
  public:
    Go1DynamicModel() : SingleRigidBodyDynamics(12.84,
                                                0.0168128557, 
                                                0.063009565,
                                                0.0716547275,
                                                -0.0002296769,
                                                -0.0002945293,
                                                -0.0000418731,
                                                4) {}
  };

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_GO1_MODEL_H_ */
