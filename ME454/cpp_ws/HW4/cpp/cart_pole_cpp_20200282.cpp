#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>


class Logger {
public:
    std::ofstream file;
    Logger(const std::string& filename) {
        file.open(filename, std::ios::out);
        file << "Time,Position,theta,Force\n";
    }
    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }
    void log(double time, double position, double theta, double Force) {
        file << time << "," << position << "," << theta << "," << Force << "\n";
    }
};

class CartPole {
public:
    double x1, d_x1, theta2, d_theta2;
    double F;
    double m_1 = 1.0, m_2 = 8.0, I_1 = 0.024167, I_2 = 1.01, length = 0.6, g = 9.81; // Model parameters
    int count = 0;
    double err_theta_integ = 0.0, err_x_integ = 0.0, err_theta_old = 0.0, err_x_old = 0.0;
    Logger logger;

    CartPole() : x1(0.0), d_x1(0.0), theta2(0.523599), d_theta2(0.0), F(0.0), logger("results_20200282.csv") {} // Initial condition

    void semi_implicit_integration() {
        double h = 0.001;

        pid_controller();

        ////////////////// TODO //////////////////////
        /// Implement the semi-implicit Euler method and calculate x1, d_x1, theta2, d_theta2 (Part2. 2-2)

        Eigen::VectorXd ddot(10);
        ddot = ((Calculate_Left_Matrix()).inverse()) * (Calculate_Right_Vector());

        d_x1 = d_x1 + ddot(0) * h;
        d_theta2 = d_theta2 + ddot(5) * h;

        x1 = x1 + d_x1 * h;
        theta2 = theta2 + d_theta2 * h;

        ////////////////// TODO END //////////////////

        // loogging
        logger.log(h * count, x1, theta2, F);
        count++;
    }

    Eigen::MatrixXd Calculate_Left_Matrix() {
        Eigen::MatrixXd matrix(10, 10);
        matrix.setZero();

        ////////////////// TODO //////////////////////
        /// Calculate the matrix on the left of the motion equation (Part2. 2-1)
        matrix (0, 0) = m_1;
        matrix (1, 1) = m_1;
        matrix (2, 2) = I_1;
        matrix (3, 3) = m_2;
        matrix (4, 4) = m_2;
        matrix (5, 5) = I_2;
        
        matrix (0, 8) = -1;
        matrix (1, 6) = 1;
        matrix (1, 9) = -1;
        matrix (2, 7) = 1;
        matrix (3, 8) = 1;
        matrix (4, 9) = 1;
        matrix (5, 8) = -length * cos(theta2);
        matrix (5, 9) = length * sin(theta2);

        matrix (6, 1) = 1;
        matrix (7, 2) = 1;
        matrix (8, 0) = -1;
        matrix (8, 3) = 1;
        matrix (8, 5) = -length * cos(theta2);
        matrix (9, 1) = -1;
        matrix (9, 4) = 1;
        matrix (9, 5) = length * sin(theta2);

        ////////////////// TODO END //////////////////

        return matrix;
    }

    Eigen::VectorXd Calculate_Right_Vector() {
        Eigen::VectorXd vec(10);

        ////////////////// TODO //////////////////////
        /// Calculate the vector on the right of the motion equation (Part2. 2-1)

        vec.setZero();
        vec(0) = F;
        vec(1) = -m_1 * g;
        vec(4) = -m_2 * g;

        vec(8) = -length * d_theta2 * d_theta2 * sin(theta2);
        vec(9) = -length * d_theta2 * d_theta2 * cos(theta2);

        ////////////////// TODO END //////////////////

        return vec;
    }

    void pid_controller()
    {
        ////////////////// TODO //////////////////////
        /// Calculate the force to keep the pole vertical (Part3. 3-1)

        double h = 0.001;

        double P_gain_theta = -316.23;
        double I_gain_theta = 0.2;
        double D_gain_theta = -68.454;

        double P_gain_x = -51.675;
        double I_gain_x = 0.1;
        double D_gain_x = -39.677;

        double target_angle = 0.0;
        double target_x = 0.0;

        double err_theta = (target_angle - theta2);
        double err_x = (target_x - x1);
        err_theta_integ += err_theta * h;
        err_x_integ += err_x * h;
        
        double err_theta_diff = (err_theta - err_theta_old) / h;
        double err_x_diff = (err_x - err_x_old) / h;
        
        F = (P_gain_theta * err_theta) + (I_gain_theta * err_theta_integ) + (D_gain_theta * err_theta_diff);
        F += (P_gain_x * err_x) + (I_gain_x * err_x_integ) + (D_gain_x * err_x_diff);
        err_theta_old = err_theta;
        err_x_old = err_x;

        ////////////////// TODO END //////////////////
    }
};

int main() {
    CartPole cartpole;
    for (int i = 0; i <= 30000; ++i) {
        cartpole.semi_implicit_integration();
    }
    return 0;
}
