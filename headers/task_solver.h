#ifndef TEST_ASSIGNMENT_TASK_SOLVER_H
#define TEST_ASSIGNMENT_TASK_SOLVER_H

#include <string>

#include <key_points.h>


class TaskSolver {
public:
    explicit TaskSolver(const char* input_filename);

    void Solve();
    void GetResult(const char* output_filename);

    ~TaskSolver();

private:
    std::vector<KeyPoints*> cars_;
    KeyPoints *car_target_;
    std::string result_;
};


#endif //TEST_ASSIGNMENT_TASK_SOLVER_H
