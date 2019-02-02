#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

bool RayIntersectsAABB(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                       const Eigen::Vector3d &lb, const Eigen::Vector3d &rt);

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
             const Eigen::Vector3d &min, const Eigen::Vector3d &max,
             std::vector<Eigen::Vector3d> *output);

#endif // RAYCAST_H_ 