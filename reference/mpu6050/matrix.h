#include <math.h>

// Only include the correct libraries while using on Arduino
// Replace all couts with printf()
double det(double p[15][15], int N)
{

    int m = N;
    int n = N;
    double t[15];
    int cnt = 0;
    double a[15][15];
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            a[i][j] = p[i][j];
        }
    }
    for (int j = 0; j < n; j++)
    {
        if (cnt == n)
        {
            break;
        }
        int b = 0;
        for (int i = cnt; i < m; i++)
        {
            if (a[i][j] != 0)
            {
                b++;
            }
            else
                continue;
        }
        if (b != 0)
        {
            int k;
            for (k = cnt; k < m; k++)
            {
                if (a[k][j] != 0)
                {
                    break;
                }
                else
                    continue;
            }
            for (int l = 0; l < n; l++)
            {
                t[l] = a[k][l];
            }
            for (int l = 0; l < n; l++)
            {
                a[k][l] = a[cnt][l];
            }
            for (int l = 0; l < n; l++)
            {
                a[cnt][l] = t[l];
            }
            for (int p = cnt + 1; p < m; p++)
            {
                double f2 = a[p][j] / a[cnt][j];
                for (int l = 0; l < j; l++)
                {
                    a[p][l] = a[p][l] - f2 * a[cnt][l];
                }
                a[p][j] = 0;
                for (int l = j + 1; l < n; l++)
                {
                    a[p][l] = a[p][l] - f2 * a[cnt][l];
                }
            }
            cnt++;
        }
        else
            continue;
    }
    double det = 1;
    for (int i = 0; i < n; i++)
        det *= a[i][i];
    return det;
}

double detsubmat(double M[15][15], int N, int m, int n)
{

    double submat[15][15];
    for (int i = 0; i < m - 1; i++)
    {
        for (int j = 0; j < n - 1; j++)
            submat[i][j] = M[i][j];
    }
    for (int i = 0; i < m - 1; i++)
    {
        for (int j = n; j < N; j++)
            submat[i][j - 1] = M[i][j];
    }
    for (int i = m; i < N; i++)
    {
        for (int j = 0; j < n - 1; j++)
            submat[i - 1][j] = M[i][j];
    }
    for (int i = m; i < N; i++)
    {
        for (int j = n; j < N; j++)
            submat[i - 1][j - 1] = M[i][j];
    }
    return det(submat, N - 1);
}

class Matrix
{

public:
    // Constructor

    double matrix[15][15];
    int rows, cols;

    Matrix(int r, int c)
    {
        rows = r;
        cols = c;
        if (r > 15 || c > 15)
        {
            printf("Matrices of upto 15 * 15 order supported.");
        }
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                if (i < rows && j < cols)
                {
                    matrix[i][j] = 0;
                }
                else
                {
                    matrix[i][j] = -404;
                }
            }
        }
    }

    Matrix operator+(Matrix other)
    {
        Matrix temp(rows, cols);
        if ((cols == other.cols) && (rows == other.rows))
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    temp.matrix[i][j] = matrix[i][j] + other.matrix[i][j];
                }
            }
        }
        else
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    temp.matrix[i][j] = -69;
                }
            }
            printf("Adding matrices of wrong order"); // printf("Adding matrices of orders that don't match")
        }
        return temp;
    }

    Matrix operator*(Matrix other)
    {
        Matrix temp(rows, other.cols);
        if (cols == other.rows)
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < other.cols; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < cols; k++)
                    {
                        sum += matrix[i][k] * other.matrix[k][j];
                    }
                    temp.matrix[i][j] = sum;
                }
            }
        }
        else
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    temp.matrix[i][j] = -63333333333;
                }
            }
            printf("Multiplying matrices of unsuitable order"); // printf("Adding matrices of orders that don't match")
        }
        return temp;
    }

    Matrix operator-(Matrix other)
    {
        return *this + ((other) * (-1));
    }

    Matrix operator*(double x)
    {
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result.matrix[i][j] = x * matrix[i][j];
            }
        }
        return result;
    }

    Matrix transpose()
    {
        Matrix temp(cols, rows);
        for (int i = 0; i < cols; i++)
        {
            for (int j = 0; j < rows; j++)
            {
                temp.matrix[i][j] = matrix[j][i];
            }
        }
        return temp;
    }

    double determinant()
    {
        if (rows != cols)
        {
            printf("Cannot compute determinant of a non-square matrix");
            return 404;
        }
        return det(matrix, rows);
    }

    Matrix inverse()
    {
        if (rows != cols)
        {
            printf("Matrix is not square, therefore not invertible");
            return (*this).transpose();
        }
        else
        {
            Matrix inv(rows, rows);
            double d = det(matrix, rows);
            if (d * d < 10e-30)
            {
                printf("Matrix is not invertible, its determinant is zero");
                return (*this);
            }
            double Cof[15][15];
            int N = rows;
            double Mat[15][15];
            for (int m = 0; m < N; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    Mat[m][n] = matrix[m][n];
                }
            }
            for (int m = 0; m < N; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    Cof[m][n] = (2 * ((m + n + 1) % 2) - 1) * detsubmat(Mat, N, m + 1, n + 1);
                }
            }
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                    inv.matrix[i][j] = Cof[j][i] / d;
            }
            return inv;
        }
    }

    void display()
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                printf(matrix[i][j]); // printf(matrix[i][j])
                printf(" ");
            }
            printf(" "); // printf('')
        }
    }
};

Matrix operator*(double t, Matrix m)
{
    return m * t;
}

Matrix identity(int n)
{
    Matrix result(n, n);
    if (n > 15)
    {
        printf("Matrices of upto order 15 * 15 only implemented here, change 15 everywhere in the file to suit your requirements");
        n = 15;
    }
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            result.matrix[i][j] = (i == j) ? 1 : 0;
        }
    }
    return result;
}

Matrix zeros(int row, int columns)
{
    Matrix result(row, columns);
    return result;
}

Matrix cross_pdt_matrix(double v1, double v2, double v3)
{
    Matrix temp(3, 3);
    double res[][3] = {{0, -v3, v2},
                       {v3, 0, -v1},
                       {-v2, v1, 0}};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            temp.matrix[i][j] = res[i][j];
    }
    return temp;
}

class Vector : public Matrix
{
public:
    Vector(double x1 = 0, double x2 = 0, double x3 = 0) : Matrix(3, 1)
    {
        matrix[0][0] = x1;
        matrix[1][0] = x2;
        matrix[2][0] = x3;
    }

    Vector(Matrix m) : Matrix(3, 1)
    {
        matrix[0][0] = m.matrix[0][0];
        matrix[1][0] = m.matrix[1][0];
        matrix[2][0] = m.matrix[2][0];
    }

    void setX(double X)
    {
        matrix[0][0] = X;
    }

    void setY(double Y)
    {
        matrix[1][0] = Y;
    }

    void setZ(double Z)
    {
        matrix[2][0] = Z;
    }

    double getX()
    {
        return matrix[0][0];
    }

    double getY()
    {
        return matrix[1][0];
    }

    double getZ()
    {
        return matrix[2][0];
    }

    double modulus()
    {
        double x = this->getX();
        double y = this->getY();
        double z = this->getZ();
        return sqrt(x * x + y * y + z * z);
    }

    void normalize()
    {
        double mod = (*this).modulus();
        for (int i = 0; i < 3; i++)
        {
            matrix[i][0] /= mod;
        }
    }

    Vector operator+(Vector other)
    {
        return Vector(this->getX() + other.getX(), this->getY() + other.getY(), this->getZ() + other.getZ());
    }

    Vector operator-(Vector other)
    {
        return *this + (-1 * other);
    }

    Vector operator*(double x)
    {
        return Vector(x * this->getX(), x * this->getY(), x * this->getZ());
    }

    Vector operator/(double x)
    {
        return Vector(this->getX() / x, this->getY() / x, this->getZ() / x);
    }

    Matrix skew_from_vec()
    {
        return cross_pdt_matrix(this->getX(), this->getY(), this->getZ());
    }

    Vector operator%(Vector other)
    {
        return this->skew_from_vec() * other;
    }

    Matrix omega_matrix()
    {
        double wx = this->getX();
        double wy = this->getY();
        double wz = this->getZ();
        Matrix result(4, 4);
        double temp[][4] = {{0, wz, -wy, wx}, {-wz, 0, wx, wy}, {wy, -wx, 0, wz}, {-wx, -wy, -wz, 0}};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result.matrix[i][j] = temp[i][j];
            }
        }
        return result;
    }
};

Vector operator*(double x, Vector v)
{
    return v * x;
}

// Vector operator*(Matrix m, Vector v)
// {
//     Matrix temp(3, 1);
//     temp.matrix[0][0] = v.getX();
//     temp.matrix[1][0] = v.getY();
//     temp.matrix[2][0] = v.getZ();
//     return Vector(m * temp);
// }

Matrix matrix_from_vectors(Vector list[], int num_vectors)
{
    // Takes in the three column vectors of the Matrix, and returns a three cross three matrix
    if (num_vectors > 15)
    {
        printf("Matrix implementation upto 15 cross 15 only implemented");
    }
    Matrix result(3, num_vectors);
    for (int i = 0; i < num_vectors; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.matrix[j][i] = list[i].matrix[j][0];
        }
    }
    return result;
}

class Quaternion : public Matrix
{
public:
    Quaternion(double temp_q1 = 0, double temp_q2 = 0, double temp_q3 = 0, double temp_q4 = 1) : Matrix(4, 1)
    {
        // q4 is the scalar part
        matrix[0][0] = temp_q1;
        matrix[1][0] = temp_q2;
        matrix[2][0] = temp_q3;
        matrix[3][0] = temp_q4;
    }

    void setq1(double X)
    {
        matrix[0][0] = X;
    }

    void setq2(double X)
    {
        matrix[1][0] = X;
    }

    void setq3(double X)
    {
        matrix[2][0] = X;
    }

    void setq4(double X)
    {
        matrix[3][0] = X;
    }

    double getq1()
    {
        return matrix[0][0];
    }

    double getq2()
    {
        return matrix[1][0];
    }

    double getq3()
    {
        return matrix[2][0];
    }

    double getq4()
    {
        return matrix[3][0];
    }

    double norm()
    {
        double q1 = matrix[0][0];
        double q2 = matrix[1][0];
        double q3 = matrix[2][0];
        double q4 = matrix[3][0];
        return sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    }

    void normalize()
    {
        double mod = this->norm();
        matrix[0][0] /= mod;
        matrix[1][0] /= mod;
        matrix[2][0] /= mod;
        matrix[3][0] /= mod;
    }

    Matrix epsillon()
    {
        Matrix result(4, 3);
        double q1 = matrix[0][0];
        double q2 = matrix[1][0];
        double q3 = matrix[2][0];
        double q4 = matrix[3][0];
        double f[][3] = {{q4, -q3, q2}, {q3, q4, -q1}, {-q2, q1, q4}, {-q1, -q2, -q3}};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result.matrix[i][j] = f[i][j];
            }
        }
        return result;
    }

    Matrix psi()
    {
        Matrix result(4, 3);
        double q1 = matrix[0][0];
        double q2 = matrix[1][0];
        double q3 = matrix[2][0];
        double q4 = matrix[3][0];
        double f[][3] = {{q4, q3, -q2}, {-q3, q4, q1}, {q2, -q1, q4}, {-q1, -q2, -q3}};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result.matrix[i][j] = f[i][j];
            }
        }
        return result;
    }

    Matrix attitude_matrix()
    {
        return this->epsillon().transpose() * this->psi();
    }

    void display()
    {
        for (int i = 0; i < 4; i++)
        {
            printf(matrix[i][0]);
            printf(' ');
        }
        printf(" ");
    }

    Quaternion operator+(Matrix other)
    {
        if (other.rows != 4 || other.cols != 1)
        {
            printf("Adding quaternion to some illogical thing");
        }
        return Quaternion(matrix[0][0] + other.matrix[0][0], matrix[1][0] + other.matrix[1][0], matrix[2][0] + other.matrix[2][0], matrix[3][0] + other.matrix[3][0]);
    }

    Quaternion operator*(double x)
    {
        return Quaternion(matrix[0][0] * x, matrix[1][0] * x, matrix[2][0] * x, matrix[3][0] * x);
    }

    Quaternion operator/(double x)
    {
        return Quaternion(matrix[0][0] / x, matrix[1][0] / x, matrix[2][0] / x, matrix[3][0] / x);
    }
};

Quaternion operator*(double x, Quaternion q)
{
    return q * x;
}