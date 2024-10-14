#include <cstdio>
#include <matrix.h>
#include "qmc.cpp"
#include <string.h>
#include <pico/time.h>
#include "mpu_c.c"
#include "pico/stdlib.h"
// Define constants and variables

#define QMC5883L_ADDR 0x0D
float ax, ay, az, gx, gy, gz;
float temp;
double RateRoll, RatePitch, RateYaw;
double AccX, AccY, AccZ;

QMC5883L compass(i2c0, QMC5883L_ADDR);
double xo, yo, zo;
float x_fin, y_fin, z_fin;

// MEKF variable declarations
Matrix att_triad(3, 3);
Quaternion qo, q_prop, qcurr;
Vector b_w(0.0, 0.0, 0.0);
Vector b_f(0.0, 0.0, 0.0);
Vector b_m(0.0, 0.0, 0.0);
Vector a;
// double temp[15][15];
//  In gain
Matrix H(6, 6);
Matrix H1(3, 3);
Matrix H2(3, 3);
Matrix K_gain(6, 6);
Matrix P_pre = 0.10 * identity(6);
Matrix R = 0.02 * identity(6);
Matrix attitude(3, 3);
Vector accel_g(0, 0, -1.0);
Vector mag_fixed(1.0, 0, 0);
// In update
Matrix P_update(6, 6);
Matrix delta_x(12, 1);
Quaternion q_est;
// In propagation
Vector w_est;
Vector w_measured;
Matrix phi(12, 12);
Matrix Fk(12, 12);
Matrix Q = zeros(12, 12);
unsigned long curr_time;
unsigned long pre_time = 0;
double dt;
Vector sigma_w(0.1, 0.1, 0.3);
double sigma_bw = 0.1;
double sigma_bf = 0.001;
double sigma_bm = 0.2;

void gyro_signals()
{
    // Read accelerometer and gyroscope data from MPU6050
    read_mpu6050_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);

    // Convert raw values to physical units
    AccX = ay;
    AccY = ax;
    AccZ = -az;
    // printf("Acceleration: %f\n", AccX);
    // printf("Acceleration: %f\n", AccY);
    // printf("Acceleration: %f\n", AccZ);

    RateRoll = gy;
    RatePitch = gx;
    RateYaw = -gz;
}

void read_mag()
{
    float x, y, z;
    x = compass.getX() / 12000;
    y = compass.getY() / 12000;
    z = compass.getZ() / 12000;

    calibrate(x, y, z, x_fin, y_fin, z_fin);
}

void triad()
{
    int x, y, z;

    // Read compass values
    // y = (double)compass.getX();
    // x = -1.0 * (double)(compass.getY());
    // z = (double)compass.getZ();
    read_mag();
    // Read IMU
    gyro_signals();

    Vector r1(0.0f, 0.0f, -1.0f);
    Vector r2(xo, yo, zo);

    r1.normalize();

    // printf("New");
    // r2.display();
    r2.normalize();
    // r2.display();
    Vector b1(AccX, AccY, AccZ);
    Vector b2(x_fin, y_fin, z_fin);
    b1.normalize();
    b2.normalize();
    // TRIAD
    Vector v1(r1.getX(), r1.getY(), r1.getZ());
    Vector v2 = r1 % r2;
    v2.normalize();
    Vector v3 = r1 % v2;
    Vector w1(b1.getX(), b1.getY(), b1.getZ());
    Vector w2 = b1 % b2;
    w2.normalize();
    Vector w3 = b1 % w2;
    Vector v1_v2_v3[] = {v1, v2, v3};
    Matrix v = matrix_from_vectors(v1_v2_v3, 3);
    Vector w1_w2_w3[] = {w1, w2, w3};
    Matrix w = matrix_from_vectors(w1_w2_w3, 3);
    att_triad = w * v.transpose();
}

// Function to compute the sensitivity matrix
Matrix sensitivity_matrix(const Vector &pred_meas)
{
    // Check dimensions of pred_meas

    // Predicted unit vectors along gravity and magnetic field
    Vector g_pred(pred_meas.matrix[0][0], pred_meas.matrix[1][0], pred_meas.matrix[2][0]);
    Vector b_pred(pred_meas.matrix[3][0], pred_meas.matrix[4][0], pred_meas.matrix[5][0]);

    Matrix g_skew = g_pred.skew_from_vec(); // Assuming you have a method to get the skew matrix
    Matrix b_skew = b_pred.skew_from_vec(); // Same as above

    Matrix o3 = zeros(3, 3); // 3x3 zero matrix
    Matrix H(6, 6);          // 6x6 matrix

    // Fill the sensitivity matrix H
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            H.matrix[i][j] = g_skew.matrix[i][j];
            H.matrix[3 + i][j] = b_skew.matrix[i][j];
            H.matrix[3 + i][3 + j] = o3.matrix[i][j];
            H.matrix[i][3 + j] = o3.matrix[i][j];
        }
    }

    return H;
}

// Function to compute the predicted measurement
Vector pred_measurement(Quaternion &q_curr, const Vector &acc_prop_iner, const Vector &mag_prop_iner)
{
    // Check the types of inputs
    // if (acc_prop_iner.shape() != std::make_pair(3, 1))
    // {
    //     std::cout << "Error: acc_prop_iner must be a 3x1 vector." << std::endl;
    //     return Vector(6, 1); // Return a zero vector on error

    // if (mag_prop_iner.shape() != std::make_pair(3, 1))
    // {
    //     std::cout << "Error: mag_prop_iner must be a 3x1 vector." << std::endl;
    //     return Vector(6, 1); // Return a zero vector on error
    // }

    // Attitude Matrix of body frame w.r.t. reference frame
    Matrix A = q_curr.attitude_matrix(); // Assuming this method exists

    // Transform the reference frame vectors to the body frame
    Vector b_rotated = A * mag_prop_iner; // Matrix-vector multiplication
    Vector g_rotated = A * acc_prop_iner; // Same as above

    // Concatenate the predicted measurements into one predicted measurement vector
    Vector hxk(6, 1); // Create a 6x1 vector
    hxk.matrix[0][0] = g_rotated.matrix[0][0];
    hxk.matrix[1][0] = g_rotated.matrix[1][0];
    hxk.matrix[2][0] = g_rotated.matrix[2][0];
    hxk.matrix[3][0] = b_rotated.matrix[0][0];
    hxk.matrix[4][0] = b_rotated.matrix[1][0];
    hxk.matrix[5][0] = b_rotated.matrix[2][0];

    return hxk;
}

// Function for the measurement update
void measurement_update(Vector &acc_measure, Vector &mag_measure,
                        Matrix &inertial_propagation, Matrix &z_reference,
                        Quaternion &q_prop, Quaternion &q_est,
                        Matrix &P_prop, Matrix &R,
                        Matrix &x_ref, Matrix &del_x,
                        bool &first_estimate_done)
{
    Vector g_inertial, b_inertial;

    // Set inertial propagation reference vectors
    if (inertial_propagation.matrix[0][0] == -1 && inertial_propagation.matrix[1][0] == -1 && inertial_propagation.matrix[2][0] == -1)
    {
        g_inertial = Vector(z_reference.matrix[0][0], z_reference.matrix[1][0], z_reference.matrix[2][0]);
        b_inertial = Vector(z_reference.matrix[3][0], z_reference.matrix[4][0], z_reference.matrix[5][0]);
    }
    else
    {
        g_inertial = Vector(inertial_propagation.matrix[0][0], inertial_propagation.matrix[1][0], inertial_propagation.matrix[2][0]);
        b_inertial = Vector(inertial_propagation.matrix[3][0], inertial_propagation.matrix[4][0], inertial_propagation.matrix[5][0]);
    }

    // Normalize measurement vectors
    Vector b_meas = mag_measure.unit_vector(); // Normalized magnetic field measurement in body frame
    Vector g_meas = acc_measure.unit_vector(); // Normalized acceleration measurement in body frame

    if (first_estimate_done)
    {
        // Normal MEKF measurement update
        Matrix Rk = R; // Measurement Noise Covariance

        // Predicted measurement according to the predicted quaternion
        Vector hxk = pred_measurement(q_prop, g_inertial, b_inertial);

        // Sensitivity matrix H(xk(-))
        Matrix H = sensitivity_matrix(hxk);

        // Kalman Gain Calculation
        Matrix P_pro = P_prop; // The propagated state covariance
        Matrix H_T = H.transpose();
        Matrix S = H * P_pro * H_T + Rk;
        Matrix K = (P_pro * H_T) * S.inverse(); // K = P*H' * (H*P*H' + R)^-1

        // Calculation of correction in del_x
        Matrix zk = acc_measure.concat(mag_measure);
        Matrix K_del_z = K * (zk - hxk); // (zk is the combined measurement vector: acc_meas, mag_meas)
        del_x = del_x + K_del_z;         // del_x(+) = del_x(-) + K(zk - h(xk))

        // Correction in q_est
        Vector del_theta = Vector(K_del_z.matrix[0][0], K_del_z.matrix[1][0], K_del_z.matrix[2][0]);
        q_est = q_prop + 0.5 * q_prop.epsillon() * del_theta;
        q_est.normalize();

        Matrix I6 = identity(6);

        // State and covariance update
        Matrix x_prop = x_ref + del_x;
        P_prop = (I6 - K * H) * P_prop;

        // Ensuring the latest quaternion is in propagation
        q_prop = q_est;
    }
    else
    {
        // Applying TRIAD to get an initial estimate of attitude

        // First Triad of vectors in reference frame 'V'
        Vector v1 = g_inertial;
        Vector v2 = b_inertial % v1; // Assuming cross product method is defined
        v2.normalize();
        Vector v3 = v2 % v1; // Cross product again
        Vector ref[3] = {v1, v2, v3};
        Matrix V = matrix_from_vectors(ref, 3);

        // Second Triad of vectors in body frame 'W'
        Vector w1 = g_meas;
        Vector w2 = w1 % b_meas; // Cross product
        w2.normalize();
        Vector w3 = w1 % w2; // Cross product
        Vector meas[3] = {v1, v2, v3};
        Matrix W = matrix_from_vectors(meas, 3);

        // The attitude matrix at the start of filtering action, A_start
        Matrix A_start = W * V.transpose();

        // Setting quaternion corresponding to A_start as the estimate in the filter equations
        Quaternion q_start = attitude_matrix_to_quaternion(A_start);
        q_est = q_start;
        q_prop = q_start;

        first_estimate_done = true; // Since we have a first estimate now
    }
}

void reset(Vector &del_x, Vector &x_ref, Quaternion &q_ref, Quaternion &q_est)
{
    // Resets the del_x vector to zero,
    // Updates the x_ref vector and the reference quaternion q_ref

    // Update the reference quaternion
    Matrix eps = q_est.epsillon(); // Assuming epsilon() returns a Vector
    Vector del_theta(del_x.matrix[0][0], del_x.matrix[1][0], del_x.matrix[2][0]);

    q_ref = q_ref + 0.5 * eps * del_theta; // Assuming you have an overloaded operator for Quaternion
    q_ref.normalize();                     // Assuming normalize() modifies q_ref in place

    // Gyro bias update
    for (int i = 3; i < 6; ++i)
    {
        x_ref.matrix[i][0] += del_x.matrix[i][0]; // Update gyro bias in x_ref
    }

    // Reset del_x to zero
    del_x = zeros(6, 1); // Assuming Vector can be initialized this way
}

void predict_state(Vector &x_prop, Vector &del_x, Vector &omega_meas,
                   Matrix &std_dev_gyro, Matrix &P_prop, double dt)
{

    // State Prediction Step
    // Takes in the angular velocity readings in body frame (with bias): omega_meas to predict the state

    // Subtracting the bias
    Vector bias(x_prop.matrix[0][0], x_prop.matrix[1][0], x_prop.matrix[2][0]);
    Vector w = omega_meas - bias;

    // Cross product matrix
    Matrix w_cross = w.skew_from_vec(); // Assuming this method returns a matrix

    // State Transition Matrix phi = I6 + F * dt
    Matrix F = zeros(6, 6);
    Matrix I3 = identity(3);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            F.matrix[i][j] = -w_cross.matrix[i][j];
            F.matrix[i][j + 3] = -I3.matrix[i][j];
        }
    }

    Matrix I6 = identity(6);
    Matrix phi = I6 + F * dt;

    // Predicting del_x
    x_prop = x_prop + F * del_x * dt; // x_prop(t) = x_prop(t - 1) + F * del_x(t - 1)
    del_x = phi * del_x;              // del_x(t) = phi * del_x(t - 1)

    // Process noise calculation
    double sig_gyr_bias[3] = {std_dev_gyro.matrix[0][0], std_dev_gyro.matrix[1][0], std_dev_gyro.matrix[2][0]};
    Matrix sigma_gyro_bias = diagonal(sig_gyr_bias, 3);
    double sig_gyr_noi[3] = {std_dev_gyro.matrix[3][0], std_dev_gyro.matrix[4][0], std_dev_gyro.matrix[5][0]};
    Matrix sigma_gyro_noise = diagonal(sig_gyr_noi, 3);
    sigma_gyro_bias = sigma_gyro_bias * sigma_gyro_bias;
    sigma_gyro_noise = sigma_gyro_noise * sigma_gyro_noise;

    Matrix Q_k = Matrix(6, 6);
    Matrix Q1 = sigma_gyro_bias * dt + (1.0 / 3.0) * sigma_gyro_noise * (dt * dt * dt);
    Matrix Q2 = -0.5 * sigma_gyro_noise * (dt * dt);
    Matrix Q4 = sigma_gyro_noise * dt;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Q_k.matrix[i][j] = Q1.matrix[i][j];
            Q_k.matrix[3 + i][j] = Q2.matrix[i][j];
            Q_k.matrix[i][3 + j] = Q2.matrix[i][j];
            Q_k.matrix[3 + i][3 + j] = Q4.matrix[i][j];
        }
    }

    // Predicting the state covariance
    Matrix P_pre = phi * P_prop * phi.transpose() + Q_k; // P_predicted = phi * P_prop * phi' + Q_k
    P_prop = P_pre;                                      // keeping latest P in propagation

    // Predicting the quaternion
    Matrix eps = q_prop.epsillon(); // Assuming you have a way to get this
    Quaternion q_est = q_prop + 0.5 * eps * w * dt;
    q_est.normalize();
    q_prop = q_est; // keeping latest quaternion in propagation
}

void setup()
{
    int n = 1000;
    i2c_init(i2c0, 100000);
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);
    mpu6050_init();
    compass.init();
    // compass.setSmoothing(5, true);

    // Initialize MPU6050 (already done in MPU6050 class constructor)

    // Get magnetometer initial values
    for (int i = 0; i < n; i++)
    {
        read_mag();
        xo += x_fin;
        yo += y_fin;
        zo += z_fin;
    }
    xo /= n;
    yo /= n;
    zo /= n;
    triad();
    attitude = att_triad;
    float mod = sqrt(xo * xo + yo * yo + zo * zo);
    mag_fixed.setX(xo / mod);
    mag_fixed.setY(yo / mod);
    mag_fixed.setZ(zo / mod);
    pre_time = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int k = 0; k < 3; k++)
        {
            if (i == k)
                R.matrix[i][k] = 0.01;
            else
                R.matrix[i][k] = 0.001;
        }
        R.matrix[i + 3][i + 3] = 1.0;
    }

    // Initial calibration
    int num = 1000;
    Vector Total_acc(0, 0, 0);
    Vector Total_w(0, 0, 0);
    while (num > 0)
    {
        gyro_signals();
        Total_acc = Total_acc + Vector(-9.8 * AccX, -9.8 * AccY, -9.8 * AccZ);
        Total_w = Total_w + Vector(RateRoll, RatePitch, RateYaw);
        num--;
    }
    Total_acc = Total_acc / 1000;
    Total_w = Total_w / 1000;
    for (int i = 0; i < 3; i++)
    {
        b_w.matrix[i][0] = Total_w.matrix[i][0];
    }
    b_f = 1 / 9.8 * (Total_acc - Vector(0.0, 0.0, -9.8));
}

int main()
{
    stdio_init_all();
    setup();
    while (true)
    {
        triad();
        att_triad.display();
        // printf("\n");
        // q_est = attitude_matrix_to_quaternion(att_triad);
        // att_triad.display();
        // predict_state(x_prop, del_x, omega_meas,
        //             std_dev_gyro, Matrix &P_prop, dt)
        for (int i = 0; i < 3; i++)
        {
            printf("%f, ", q_est.matrix[i][0]);
        }
        printf("%f\n", q_est.matrix[3][0]);
        sleep_ms(100);
    }

    return 0;
}
