#include <matrix.h>
#include <qmc.h>
#include <string.h>
#include <pico/time.h>
#include <mpu_cpp.cpp>

// Define constants and variables
double RateRoll, RatePitch, RateYaw;
double AccX, AccY, AccZ;
double AngleRoll, AnglePitch;
QMC5883L compass;
double xo, yo, zo;
double x_fin, y_fin, z_fin;

// MEKF variable declarations
Matrix att_triad(3, 3);
Quaternion qo, q_prop;
Vector b_w(0.0, 0.0, 0.0);
Vector b_f(0.0, 0.0, 0.0);
Vector b_m(0.0, 0.0, 0.0);
Vector a;
double temp[15][15];
// In gain
Matrix H(6, 12);
Matrix H1(3, 3);
Matrix H2(3, 3);
Matrix K_gain(12, 6);
Matrix P_pre = 0.10 * identity(12);
Matrix R = 0.02 * identity(6);
Matrix attitude(3, 3);
Vector accel_g(0, 0, -1.0);
Vector mag_fixed(1.0, 0, 0);
// In update
Matrix P_update(12, 12);
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
string message;

void mag_calibration()
{
    float x_value;
    float y_value;
    float z_value;

    x_value = compass.getX();
    y_value = compass.getY();
    z_value = compass.getZ();

    double bi1 = -158.227540;
    double bi2 = -485.725478;
    double bi3 = 217.206439;

    double x_ab = x_value - bi1;
    double y_ab = y_value - bi2;
    double z_ab = z_value - bi3;

    double a1 = 0.296698,
           a2 = 0.007968,
           a3 = -0.004701,
           bb1 = 0.007968,
           bb2 = 0.369302,
           bb3 = -0.010403,
           c1 = -0.004701,
           c2 = -0.010403,
           c3 = 0.365830;

    x_fin = a1 * x_ab + a2 * y_ab + a3 * z_ab;
    y_fin = bb1 * x_ab + bb2 * y_ab + bb3 * z_ab;
    z_fin = c1 * x_ab + c2 * y_ab + c3 * z_ab;
}

void triad()
{
    int x, y, z;

    // Read compass values
    y = (double)compass.getX();
    x = -1.0 * (double)(compass.getY());
    z = (double)compass.getZ();

    // Read IMU
    gyro_signals();

    Vector r1(0.0f, 0.0f, -1.0f);
    Vector r2(xo, yo, zo);
    r1.normalize();
    r2.normalize();
    Vector b1(AccX, AccY, AccZ);
    Vector b2(x, y, z);
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
    att_triad = v * w.transpose();
}

void setup()
{
    compass.init();
    compass.setSmoothing(5, true);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    // Initialize I2C
    i2c_init(i2c0, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    uint8_t data = 0x00;
    i2c_write_blocking(i2c0, MPU6050_ADDR, &PWR_MGMT_1, 1, false);
    i2c_write_blocking(i2c0, MPU6050_ADDR, &data, 1, false);

    delay_ms(250);

    // Get magnetometer initial values
    for (int i = 0; i < 10; i++)
    {
        mag_calibration();
        xo += x_fin;
        yo += y_fin;
        zo += z_fin;
    }
    xo /= 10.0;
    yo /= 10.0;
    zo /= 10.0;
    triad();
    attitude = att_triad;
    mag_fixed.setX(1);
    mag_fixed.setY(0);
    mag_fixed.setZ(0);
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
    {
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
        b_f = Total_acc - Vector(0.0, 0.0, -9.8);
    }
}

int main()
{
    setup();
    while (true)
    {
        gyro_signals();
        mag_calibration();

        // Prediction
        curr_time = time_us_32();
        dt = (curr_time - pre_time) / 1000000.0; // Convert microseconds to seconds
        pre_time = curr_time;

        // Gain calculation
        H1 = Vector(attitude * accel_g).skew_from_vec();
        H2 = Vector(attitude * mag_fixed).skew_from_vec();
        for (int i = 0; i < 6; i++)
        {
            for (int k = 0; k < 12; k++)
            {
                H.matrix[i][k] = 0.0;
            }
        }
        H.matrix[0][6] = 1.0;
        H.matrix[1][7] = 1.0;
        H.matrix[2][8] = 1.0;
        H.matrix[3][9] = 1.0;
        H.matrix[4][10] = 1.0;
        H.matrix[5][11] = 1.0;
        for (int i = 0; i < 3; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                H.matrix[i][k] = H1.matrix[i][k];
                H.matrix[i + 3][k] = H2.matrix[i][k];
            }
        }

        K_gain = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
        Matrix I = identity(12);
        P_update = (I - K_gain * H) * P_pre;

        Matrix temp1(6, 1);
        mag_fixed.normalize();
        Vector rotated_g = attitude * accel_g;
        Vector rotated_m = attitude * mag_fixed;
        Vector accel_measured;
        Vector mag_measured;
        double acc_norm = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
        double mag_norm = sqrt(x * x + y * y + z * z);

        accel_measured = Vector(AccX, AccY, AccZ).normalize();
        mag_measured = Vector(x, y, z).normalize();

        Vector error_acc = accel_measured - rotated_g;
        Vector error_mag = mag_measured - rotated_m;

        Vector delta_a = H1 * error_acc;
        Vector delta_m = H2 * error_mag;
        delta_x = delta_a.concat(delta_m);

        // Update state
        q_est = q_est + (K_gain * delta_x);
        q_est.normalize();

        // Update matrices
        P_pre = P_update;

        // Sleep for a short duration to maintain loop frequency
        sleep_ms(10); // Sleep for 10 milliseconds
    }

    return 0;
}
