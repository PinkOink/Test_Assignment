#include <task_solver.h>

int main()
{
    const char* input_file = "inputfile.txt";
    const char* output_file = "outputfile.txt";

    TaskSolver ts(input_file);
    ts.Solve();
    ts.GetResult(output_file);

    return 0;
}