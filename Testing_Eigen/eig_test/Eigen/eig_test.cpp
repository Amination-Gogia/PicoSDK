#include "Dense"
#include <stdio.h>
#include "pico/stdlib.h"

class EigenTutorial
{
public:
    Eigen::Matrix3f matA;
    Eigen::Matrix3f matB;
    Eigen::Matrix3f matC;

    EigenTutorial()
    {
        // Initialize the matrices
        matA << 1, 2, 3,
            4, 5, 6,
            7, 8, 9;

        matB << 9, 8, 7,
            6, 5, 4,
            3, 2, 1;

        // Perform matrix addition
        matC = matA + matB;
    }

    void printResult()
    {
        printf("Matrix A + B = \n");
        printMatrix(matC);
    }

private:
    void printMatrix(const Eigen::Matrix3f &matrix)
    {
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                printf("%f ", matrix(i, j));
            }
            printf("\n");
        }
    }
};

int main()
{
    stdio_init_all();
    EigenTutorial tutorial;
    tutorial.printResult();

    return 0;
}
