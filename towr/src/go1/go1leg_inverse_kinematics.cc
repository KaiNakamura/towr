#include <towr/models/go1/go1leg_inverse_kinematics.h>

#include <cmath>
#include <map>
#include <algorithm>

#include <xpp_states/cartesian_declarations.h>


namespace xpp {

double clamp(double val, double min_val, double max_val) {
  return std::max(min_val, std::min(max_val, val));
}

Go1legInverseKinematics::Vector3d
Go1legInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, KneeBend bend) const
{
  // Define the joint angles
  // HAA = Hip Abduction/Adduction
  // HFE = Hip Flexion/Extension
  // KFE = Knee Flexion/Extension
  double q_HAA, q_HFE, q_KFE;

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  // Translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  xr = ee_pos_B;

  // Compute the HAA angle
  q_HAA = -atan2(xr[Y], -xr[Z]);

  // Rotate into the HFE coordinate system (rot around X)
  R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA), -sin(q_HAA), 0.0, sin(q_HAA), cos(q_HAA);

  xr = (R * xr).eval();

  // translate into the HFE coordinate system (along Z axis)
  xr += hfe_to_haa_z;  // distance of HFE to HAA in z direction

  // Compute square of length from HFE to foot
  double hfe_to_haa_squared = pow(xr[X], 2) + pow(xr[Z], 2);

  // Create aliases for the leg lengths
  double lu = length_thigh; // length of upper leg
  double ll = length_shank; // length of lower leg

  // Define alpha based on knee bend
  double alpha;
  if (bend == Forward) {
    alpha = atan2(-xr[Z], xr[X]) - 0.5 * M_PI;
  } else {
    alpha = atan2(-xr[Z], -xr[X]) - 0.5 * M_PI;
  }

  // Use law of cosines to find beta
  double beta = (pow(lu, 2) + hfe_to_haa_squared - pow(ll, 2)) / (2. * lu * sqrt(hfe_to_haa_squared));
  beta = clamp(beta, -1, 1); // Clamp between -1 and 1
  beta = acos(beta);

  // Compute Hip FE angle
  q_HFE = alpha + beta;

  // Use law of cosines to find gamma
  double gamma = (pow(ll, 2) + pow(lu, 2) - hfe_to_haa_squared) / (2. * ll * lu);
  gamma = clamp(gamma, -1, 1); // Clamp between -1 and 1
  gamma = acos(gamma);

  // Calculate Knee FE angle
  q_KFE = gamma - M_PI;

  // Enforce limits
  EnforceLimits(q_HAA, HAA);
  EnforceLimits(q_HFE, HFE);
  EnforceLimits(q_KFE, KFE);

  return Vector3d(q_HAA, q_HFE, q_KFE);
}

void
Go1legInverseKinematics::EnforceLimits (double& val, Go1JointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -180;
  const static double haa_max =  90;

  const static double hfe_min = -90;
  const static double hfe_max =  90;

  const static double kfe_min = -180;
  const static double kfe_max =  0;

  // reduced joint angles for optimization
  static const std::map<Go1JointID, double> max_range {
    {HAA, haa_max / 180.0 * M_PI},
    {HFE, hfe_max / 180.0 * M_PI},
    {KFE, kfe_max / 180.0 * M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<Go1JointID, double> min_range {
    {HAA, haa_min / 180.0 * M_PI},
    {HFE, hfe_min / 180.0 * M_PI},
    {KFE, kfe_min / 180.0 * M_PI}
  };

  double max = max_range.at(joint);
  val = val > max ? max : val;

  double min = min_range.at(joint);
  val = val < min ? min : val;
}

} /* namespace xpp */