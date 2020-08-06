#ifndef TEST_ASSIGNMENT_KEY_POINTS_H
#define TEST_ASSIGNMENT_KEY_POINTS_H

#include <Dense>


class KeyPoints {
public:
    KeyPoints(size_t points_number, double points[]);

    void TranslatePointsMedianToOrigin();
    bool RotateToVector(Eigen::Vector3d &target, size_t position);
    bool RotateToVectorWithAxis(Eigen::Vector3d &axis, Eigen::Vector3d &target, size_t position);

    Eigen::Vector3d GetPoint(size_t position);
    size_t GetPointsNumber();

    bool Compare(const KeyPoints& arg);

private:
    void RotateFromVectorToVector(Eigen::Vector3d &target, Eigen::Vector3d &source);
    Eigen::MatrixX3d points_;
};


#endif //TEST_ASSIGNMENT_KEY_POINTS_H
