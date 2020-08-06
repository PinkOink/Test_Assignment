#include <key_points.h>

#include <Dense>
#include <cmath>

static const double ERROR = 1e-5;

Eigen::Vector3d GetOrthVector(Eigen::Vector3d &vec) {
    if (std::abs(vec(0)) < ERROR) {
        Eigen::Vector3d orth(1, 0, 0);
        return orth.normalized();
    }
    if (std::abs(vec(1)) < ERROR) {
        Eigen::Vector3d orth(0, 1, 0);
        return orth.normalized();
    }
    if (std::abs(vec(2)) < ERROR) {
        Eigen::Vector3d orth(0, 0, 1);
        return orth.normalized();
    }
    Eigen::Vector3d orth(vec(1), -vec(0), 0);
    return orth.normalized();
}

KeyPoints::KeyPoints(size_t points_number, double points[])
  : points_(points_number, 3)
{
    for (size_t i = 0; i < 3 * points_number; ++i) {
        points_(i / 3, i % 3) = points[i];
    }
}
void KeyPoints::TranslatePointsMedianToOrigin() {
    double x_med = points_.col(0).mean();
    double y_med = points_.col(1).mean();
    double z_med = points_.col(2).mean();

    for (size_t i = 0; i < points_.rows(); ++i) {
        points_(i, 0) -= x_med;
        points_(i, 1) -= y_med;
        points_(i, 2) -= z_med;
    }
}
bool KeyPoints::RotateToVector(Eigen::Vector3d &target, size_t position) {
    Eigen::Vector3d source = points_.row(position);

    if (fabs(source.norm() - target.norm()) >= ERROR) {
        return false;
    }
    if ((source - target).norm() <= ERROR) {
        return true;
    }
    RotateFromVectorToVector(target, source);

    return true;
}
bool KeyPoints::RotateToVectorWithAxis(Eigen::Vector3d &axis, Eigen::Vector3d &target, size_t position) {
    Eigen::Vector3d source = points_.row(position);

    if (fabs(source.norm() - target.norm()) >= ERROR) {
        return false;
    }
    if ((source - target).norm() <= ERROR) {
        return true;
    }

    Eigen::Vector3d target_projection = target - axis * axis.dot(target);
    Eigen::Vector3d source_projection = source - axis * axis.dot(source);

    RotateFromVectorToVector(target_projection, source_projection);

    return true;
}
void KeyPoints::RotateFromVectorToVector(Eigen::Vector3d &target, Eigen::Vector3d &source) {
    Eigen::Vector3d axis;
    double pivot_sin;
    double pivot_cos;
    if ((source + target).norm() <= ERROR) {
        axis = GetOrthVector(target);
        pivot_sin = 0;
        pivot_cos = -1;
    } else {
        axis = source.cross(target);
        pivot_sin = axis.norm() / (source.norm() * target.norm());
        pivot_cos = target.dot(source) / (source.norm() * target.norm());
        axis.normalize();
    }
    Eigen::Matrix3d rotation_mat;
    rotation_mat << pivot_cos + (1 - pivot_cos) * axis.x() * axis.x(), (1 - pivot_cos) * axis.x() * axis.y() - pivot_sin * axis.z(), (1 - pivot_cos) * axis.x() * axis.z() + pivot_sin * axis.y(),
            (1 - pivot_cos) * axis.y() * axis.x() + pivot_sin * axis.z(), pivot_cos + (1 - pivot_cos) * axis.y() * axis.y(), (1 - pivot_cos) * axis.y() * axis.z() - pivot_sin * axis.x(),
            (1 - pivot_cos) * axis.z() * axis.x() - pivot_sin * axis.y(), (1 - pivot_cos) * axis.z() * axis.y() + pivot_sin * axis.x(), pivot_cos + (1 - pivot_cos) * axis.z() * axis.z();
    points_ = (rotation_mat * points_.transpose()).transpose();
}
Eigen::Vector3d KeyPoints::GetPoint(size_t position) {
    return points_.row(position);
}
size_t KeyPoints::GetPointsNumber() {
    return points_.rows();
}
bool KeyPoints::Compare(const KeyPoints& arg) {
    return (points_ - arg.points_).norm() < ERROR;
}