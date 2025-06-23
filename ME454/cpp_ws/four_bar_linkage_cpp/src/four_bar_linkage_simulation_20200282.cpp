#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

double L1_, L2_, L3_, L0_;
double m1_, m2_, m3_;
double ixx1_, iyy1_, izz1_;
double ixx2_, iyy2_, izz2_;
double ixx3_, iyy3_, izz3_;
double y1_init_, y2_init_, y3_init_, z1_init_,z2_init_,z3_init_;
double q1_init_, q2_init_, q3_init_;
double q1dot_init_, q2dot_init_, q3dot_init_;

Eigen::MatrixXd M_(9,9), M_inv_(9,9);

double g = 9.81;

void initialize(){

    ////////////////// TODO //////////////////////
    // Edit the value of the properties

    L0_ = 3.0;

    m1_ = 1.0;
    L1_ = 1.0;
    ixx1_ = m1_ * L1_ * L1_ / 3.0;
    iyy1_ = 0.0;
    izz1_ = 0.0;
    q1_init_ = 1.578;
    y1_init_ = L1_ * cos(q1_init_) / 2.0;
    z1_init_ = L1_ * sin(q1_init_) / 2.0;

    m2_ = 4.0;
    L2_ = 4.0;
    ixx2_ = m2_ * L2_ * L2_ / 3.0;
    iyy2_ = 0.0;
    izz2_ = 0.0;
    q2_init_ = 0.3533;
    y2_init_ = L1_ * cos(q1_init_) + L2_ * cos(q2_init_) / 2.0;
    z2_init_ = L1_ * sin(q1_init_) + L2_ * sin(q2_init_) / 2.0;

    m3_ = 2.5;
    L3_ = 2.5;
    ixx3_ = m3_ * L3_ * L3_ / 3.0;
    iyy3_ = 0.0;
    izz3_ = 0.0;
    q3_init_ = 1.2649;
    y3_init_ = L1_ * cos(q1_init_) + L2_ * cos(q2_init_) - L3_ * cos(q3_init_) / 2.0;
    z3_init_ = L1_ * sin(q1_init_) + L2_ * sin(q2_init_) - L3_ * sin(q3_init_) / 2.0;

    // ToDo: set the mass matrix and the inversion of the mass matrix
    M_.setZero();
    M_(0, 0) = m1_;
    M_(1, 1) = m1_;
    M_(2, 2) = ixx1_;
    M_(3, 3) = m2_;
    M_(4, 4) = m2_;
    M_(5, 5) = ixx2_;
    M_(6, 6) = m3_;
    M_(7, 7) = m3_;
    M_(8, 8) = ixx3_;
    M_inv_.setZero();
    M_inv_ = M_.inverse();

    ////////////////// TODO END //////////////////////

}

Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd q){
    Eigen::MatrixXd J;
    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian using the input state vector q
    J.resize(8, 9);

    J <<    1.0, 0.0, L1_ / 2.0 * sin(q(2)), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, - L1_ / 2.0 * cos(q(2)), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ * sin(q(2)), 1.0, 0.0, L2_ / 2.0 * sin(q(5)), 0.0, 0.0, 0.0,
            0.0, 0.0, - L1_ * cos(q(2)), 0.0, 1.0, - L2_ / 2.0 * cos(q(5)), 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ * sin(q(2)), 0.0, 0.0, L2_ * sin(q(5)), 1.0, 0.0, - L3_ / 2.0 * sin(q(8)),
            0.0, 0.0, - L1_ * cos(q(2)), 0.0, 0.0, - L2_ * cos(q(5)), 0.0, 1.0, L3_ / 2.0 * cos(q(8)),
            0.0, 0.0, L1_ * sin(q(2)), 0.0, 0.0, L2_ * sin(q(5)), 0.0, 0.0, - L3_ * sin(q(8)),
            0.0, 0.0, - L1_ * cos(q(2)), 0.0, 0.0, - L2_ * cos(q(5)), 0.0, 0.0, L3_ * cos(q(8));

    ////////////////// TODO END //////////////////////
    return J;
}

Eigen::MatrixXd calculate_jacobian_dot(Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::MatrixXd Jdot;

    ////////////////// TODO //////////////////////
    /// Calculate the time derivative of Jacobian using the input state vectors q and qdot
    Jdot.resize(8, 9);
    
    Jdot << 0.0, 0.0, L1_ / 2.0 * cos(q(2)) * qdot(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ / 2.0 * sin(q(2)) * qdot(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ * cos(q(2)) * qdot(2), 0.0, 0.0, L2_ / 2.0 * cos(q(5)) * qdot(5), 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ * sin(q(2)) * qdot(2), 0.0, 0.0, L2_ * cos(q(5)) * qdot(5), 0.0, 0.0, 0.0,
            0.0, 0.0, L1_ * cos(q(2)) * qdot(2), 0.0, 0.0, L2_ * sin(q(5)) * qdot(5), 0.0, 0.0, - L3_ / 2.0 * cos(q(8)) * qdot(8),
            0.0, 0.0, L1_ * sin(q(2)) * qdot(2), 0.0, 0.0, L2_ * sin(q(5)) * qdot(5), 0.0, 0.0, - L3_ / 2.0 * sin(q(8)) * qdot(8),
            0.0, 0.0, L1_ * cos(q(2)) * qdot(2), 0.0, 0.0, L2_ * cos(q(5)) * qdot(5), 0.0, 0.0, - L3_ * cos(q(8)) * qdot(8),
            0.0, 0.0, L1_ * sin(q(2)) * qdot(2), 0.0, 0.0, L2_ * sin(q(5)) * qdot(5), 0.0, 0.0, - L3_ * sin(q(8)) * qdot(8);

    ////////////////// TODO END //////////////////////
    return Jdot;
}

Eigen::VectorXd calculate_Fext(Eigen::VectorXd q){
    Eigen::VectorXd Fext;
    ////////////////// TODO //////////////////////
    /// Calculate the external force using the input state vector q
    Fext.resize(9);
    Fext << 0.0, m1_ * g, 0.0, 0.0, m2_ * g, 0.0, 0.0, m3_ * g, 0.0;
    ////////////////// TODO END //////////////////////
    return Fext;
}

Eigen::VectorXd calculate_lambda(Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd Fext, Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::VectorXd lambda;
    ////////////////// TODO //////////////////////
    /// Calculate lambda which connects the constraint force and the jacobian
    lambda.resize(8);
    Eigen::MatrixXd JT = J.transpose();
    lambda = (J * M_inv_ * JT).inverse() * ((- (Jdot * qdot) - (J * M_inv_ * Fext)));

    ////////////////// TODO END //////////////////////
    return lambda;
}

Eigen::VectorXd calculate_Fc(Eigen::MatrixXd J, Eigen::VectorXd lambda){
    Eigen::VectorXd Fc;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint force using the Jacobian and lambda
    Fc.resize(9);
    Eigen::MatrixXd JT = J.transpose();
    Fc = JT * lambda;

    ////////////////// TODO END //////////////////////
    return Fc;
}

Eigen::VectorXd calculate_constraint_error(Eigen::VectorXd q){
    Eigen::VectorXd ConstraintErr;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint errors under given state vector q
    ConstraintErr.resize(8);
    ConstraintErr(0) = q(0) - L1_ * cos(q(2)) / 2;
    ConstraintErr(1) = q(1) - L1_ * sin(q(2)) / 2;
    ConstraintErr(2) = q(3) - L1_ * cos(q(2)) - L2_ * cos(q(5)) / 2;
    ConstraintErr(3) = q(4) - L1_ * sin(q(2)) - L2_ * sin(q(5)) / 2;
    ConstraintErr(4) = q(6) - L1_ * cos(q(2)) - L2_ * cos(q(5)) + L3_ * cos(q(8)) / 2;
    ConstraintErr(5) = q(7) - L1_ * sin(q(2)) - L2_ * sin(q(5)) + L3_ * sin(q(8)) / 2;
    ConstraintErr(6) = L0_ - L1_ * cos(q(2)) - L2_ * cos(q(5)) + L3_ * sin(q(8));
    ConstraintErr(7) = - L1_ * sin(q(2)) - L2_ * sin(q(5)) + L3_ * sin(q(8));
    
    ////////////////// TODO END //////////////////
    return ConstraintErr;
}

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream qdot_results("qdot_results_cpp.csv");
    std::ofstream constraint_error_results("constraint_error_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 50000; // number of simulation steps (1000 s)
    int rec_steps = 10; // record the result every n steps (every 0.1 s)
    initialize();

    Eigen::VectorXd q(9);
    Eigen::VectorXd qdot(9);

    std::cout << "Simulation start" << std::endl;
    
    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.
    q(0) = y1_init_;
    q(1) = z1_init_;
    q(2) = q1_init_;
    q(3) = y2_init_;
    q(4) = z2_init_;
    q(5) = q2_init_;
    q(6) = y3_init_;
    q(7) = z3_init_;
    q(8) = q3_init_;
    Eigen::MatrixXd J_(8, 9);
    Eigen::MatrixXd Jdot_(8, 9);
    Eigen::VectorXd Fext_(9);
    Eigen::VectorXd lambda_(8);
    Eigen::VectorXd Fc_(9);
    Eigen::VectorXd Error_(8);
    double t = 0;

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        
        
        // TODO: implement semi-implicit Euler method to simulate the four bar linkage motion
        J_ = calculate_jacobian(q);
        Jdot_ = calculate_jacobian_dot(q, qdot);
        Fext_ = calculate_Fext(q);
        lambda_ = calculate_lambda(J_, Jdot_, Fext_, q, qdot);
        Fc_ = calculate_Fc(J_, lambda_);
        qdot = qdot + M_inv_ * (Fext_ + Fc_) * h;
        q = q + qdot * h;
        Error_ = calculate_constraint_error(q);
        

        if (i_sim % rec_steps == 0)
        {
            q_results << t << ",";
            qdot_results << t << ",";
            // TODO: record q_results_cpp.csv contains the value of the angle q1, q2, and q3 in radian (q1,q2,q3)
            q_results << q(2) << ", "; 
            q_results << q(5) << ", "; 
            q_results << q(8) << ", "; 

            q_results <<"\n";

            // TODO: record qdot_results_cpp.csv contains the value of the angular velocity qdot1, qdot2, and qdot3 in radian (qdot1,qdot2,qdot3)
            qdot_results << qdot(2) << ", "; 
            qdot_results << qdot(5) << ", "; 
            qdot_results << qdot(8) << ", "; 

            qdot_results <<"\n";

            // TODO: record constraint_error_cpp.csv contains the constraint error at this time step (error1,error2,...)
            constraint_error_results << t << ",";
            for(int i = 0; i < 8; i++) constraint_error_results << Error_(i) << ", ";

            constraint_error_results << "\n";
        }
        //////////////// TODO end ////////////////
        t = t + h;
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    qdot_results.close();
    constraint_error_results.close();

    return 0;
}


