#include "raycast.h"
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>

int signum(int x) {
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
    return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds < 0) {
        return intbound(-s, -ds);
    } else {
        s = mod(s, 1);
        // problem is now s+t*ds = 1
        return (1 - s) / ds;
    }
}

bool RayIntersectsAABB(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &lb,
                       const Eigen::Vector3d &rt) {
    Eigen::Vector3d dir = (end - start).normalized();
    Eigen::Vector3d dirfrac(1.0f / dir.x(), 1.0f / dir.y(), 1.0f / dir.z());

    // r.dirs_ is unit dirs_ vector of ray
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // start is origin of ray
    double t1 = (lb.x() - start.x()) * dirfrac.x();
    double t2 = (rt.x() - start.x()) * dirfrac.x();
    double t3 = (lb.y() - start.y()) * dirfrac.y();
    double t4 = (rt.y() - start.y()) * dirfrac.y();
    double t5 = (lb.z() - start.z()) * dirfrac.z();
    double t6 = (rt.z() - start.z()) * dirfrac.z();

    double tmin = fmax(fmax(fmin(t1, t2), fmin(t3, t4)), fmin(t5, t6));
    double tmax = fmin(fmin(fmax(t1, t2), fmax(t3, t4)), fmax(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0) {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        return false;
    }

    return true;
}

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
             const Eigen::Vector3d &min, const Eigen::Vector3d &max,
             std::vector<Eigen::Vector3d> *output) {
//    std::cout << start << ' ' << end << std::endl;
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
    // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
    // Extensions to the described algorithm:
    //   • Imposed a distance_ limit.
    //   • The face passed through to reach the current cube is provided to
    //     the callback.

    // The foundation of this algorithm is a parameterized representation of
    // the provided ray,
    //                    origin + t * dirs_,
    // except that t is not actually stored; rather, at any given point_ in the
    // traversal, we keep track of the *greater* t values which we would have
    // if we took a step sufficient to cross a cube boundary along that axis
    // (i.e. change the integer part of the coordinate) in the variables
    // tMaxX, tMaxY, and tMaxZ.

    // Cube containing origin point_.
    int x = (int) std::floor(start.x());
    int y = (int) std::floor(start.y());
    int z = (int) std::floor(start.z());
    int endX = (int) std::floor(end.x());
    int endY = (int) std::floor(end.y());
    int endZ = (int) std::floor(end.z());
    Eigen::Vector3d direction = (end - start);
    double maxDist = direction.squaredNorm();

    // Break out dirs_ vector.
    double dx = endX - x;
    double dy = endY - y;
    double dz = endZ - z;

    // Direction to increment x,y,z when stepping.
    int stepX = (int) signum((int) dx);
    int stepY = (int) signum((int) dy);
    int stepZ = (int) signum((int) dz);

    // See description above. The initial values depend on the fractional
    // part of the origin.
    double tMaxX = intbound(start.x(), dx);
    double tMaxY = intbound(start.y(), dy);
    double tMaxZ = intbound(start.z(), dz);

    // The change in t when taking a step (always positive).
    double tDeltaX = ((double) stepX) / dx;
    double tDeltaY = ((double) stepY) / dy;
    double tDeltaZ = ((double) stepZ) / dz;

    output->clear();

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0)
        return;

    double dist = 0;
    while (true) {

        if (x >= min.x() && x < max.x() &&
            y >= min.y() && y < max.y() &&
            z >= min.z() && z < max.z()) {
            output->push_back(Eigen::Vector3d(x, y, z));

            dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

            if (dist > maxDist) return;

            if (output->size() > 1500) {
                std::cerr << "Error, too many racyast voxels." << std::endl;
                throw std::out_of_range("Too many RaycasMultithread voxels");
            }
        }

        if (x == endX && y == endY && z == endZ) break;

        // tMaxX stores the t-value at which we cross a cube boundary along the
        // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
        // chooses the closest cube boundary. Only the first case of the four
        // has been commented in detail.
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                // Update which cube we are now in.
                x += stepX;
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}