#include "task_solver.h"

#include <fstream>
#include <vector>
#include <string>

static const double ERROR = 1e-5;

TaskSolver::TaskSolver(const char* input_filename) {
    std::ifstream in_file;
    in_file.open(input_filename);

    size_t cars_number;
    size_t points_number;
    double* points_buf;

    in_file >> cars_number;
    in_file >> points_number;
    points_buf = new double[3 * points_number];
    cars_.reserve(cars_number);

    for (size_t i = 0; i < cars_number; ++i) {
        for (size_t j = 0; j < 3 * points_number; ++j) {
            in_file >> points_buf[j];
        }
        cars_.push_back(new KeyPoints(points_number, points_buf));
    }
    for (size_t j = 0; j < 3 * points_number; ++j) {
        in_file >> points_buf[j];
    }
    car_target_ = new KeyPoints(points_number, points_buf);

    delete[] points_buf;
    in_file.close();
}
void TaskSolver::Solve() {
    for (auto car : cars_) {
        car->TranslatePointsMedianToOrigin();
    }
    car_target_->TranslatePointsMedianToOrigin();

    size_t cur_point_pos = 0;
    size_t first_point_pos = 0;
    size_t second_point_pos = 0;
    Eigen::Vector3d axis;
    Eigen::Vector3d first_target;
    Eigen::Vector3d second_target;
    for (; cur_point_pos < car_target_->GetPointsNumber(); ++cur_point_pos) {
        first_target = car_target_->GetPoint(cur_point_pos);
        if (first_target.norm() > ERROR) {
            axis = first_target.normalized();
            first_point_pos = cur_point_pos;
            break;
        }
    }
    for (cur_point_pos++; cur_point_pos < car_target_->GetPointsNumber(); ++cur_point_pos) {
        second_target = car_target_->GetPoint(cur_point_pos);
        if (second_target.norm() > ERROR &&
            std::abs(second_target.norm() - std::abs(second_target.dot(axis))) > ERROR) {
            second_point_pos = cur_point_pos;
            break;
        }
    }

    for (size_t i = 0; i < cars_.size(); ++i) {
        KeyPoints &cur_car = *cars_[i];

        bool result = true;
        result = cur_car.RotateToVector(first_target, first_point_pos);
        if (!result) {
            continue;
        }
        cur_car.RotateToVectorWithAxis(axis, second_target, second_point_pos);
        if (cur_car.Compare(*car_target_)) {
            result_ += std::to_string(i);
        }
    }
    if (result_.empty()) {
        result_ = "-1";
    }
};
void TaskSolver::GetResult(const char* output_filename) {
    std::ofstream out_file;
    out_file.open(output_filename);
    out_file << result_;
}
TaskSolver::~TaskSolver() {
    for (auto car : cars_) {
        delete car;
    }
    delete car_target_;
}