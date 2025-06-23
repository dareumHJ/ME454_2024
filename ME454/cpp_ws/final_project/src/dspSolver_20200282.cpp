#include "dspSolver.h"
#include <iostream>

// Implement this function
dspSolver::dspSolver(dspLinkConfig& linkConfig) : LinkConfig(linkConfig) {
    /// Solver variable initialization
    /// The link with index 0 is the ground link
    /// There are only two joint types : revolute and prismatic
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    /// You have to initialize following variables
    q.setZero(3 * link_num);
    qdot.setZero(3 * link_num);
    qddot.setZero(3 * link_num);
    M.setZero(3 * link_num, 3 * link_num);
    M_inv.setZero(3 * link_num, 3 * link_num);
    J.setZero(2 * joint_num, 3 * link_num);
    J_dot.setZero(2 * joint_num, 3 * link_num);
    F_ext.setZero(3 * link_num, 1);
    lambda.setZero(2 * joint_num, 1);
}

// Implement this function
bool dspSolver::CalculateConstraintError(Eigen::VectorXd& error_c) {
    /// calculate error what you define as constraint

    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    for(int i = 0; i < joint_num; i++){
        // Current joint
        dspJoint* cur_joint = LinkConfig.GetJoint(i);

        if ((cur_joint -> GetType()) == 0) { // Revolute Joint
            Eigen::Vector2d v1_lcs;
            Eigen::Vector2d v2_lcs;

            cur_joint->GetP1_Link1_LCS(v1_lcs);
            cur_joint->GetP2_Link2_LCS(v2_lcs);

            int link1_idx = cur_joint->GetLink1ID();
            int link2_idx = cur_joint->GetLink2ID();

            double x1, y1, t1, x2, y2, t2;
            LinkConfig.GetLink(link1_idx)->GetQ(x1, y1, t1);
            LinkConfig.GetLink(link2_idx)->GetQ(x2, y2, t2);

            double err_x = x2 - x1 - (v1_lcs(0) * cos(t1) - v1_lcs(1) * sin(t1)) - (- v2_lcs(0) * cos(t2) + v2_lcs(1) * sin(t2));
            double err_y = y2 - y1 - (v1_lcs(0) * sin(t1) + v1_lcs(1) * cos(t1)) - (- v2_lcs(0) * sin(t2) - v2_lcs(1) * cos(t2));
            
            // Update error vector
            error_c(2 * i) = err_x;
            error_c(2 * i + 1) = err_y;
        }

        else { // Prismatic Joint
            int link1_idx = LinkConfig.GetJoint(i)->GetLink1ID();
            int link2_idx = LinkConfig.GetJoint(i)->GetLink2ID();
            
            Eigen::Vector2d p_v1_lcs;
            Eigen::Vector2d p_v2_lcs;
            Eigen::Vector2d q_v1_lcs;
            Eigen::Vector2d q_v2_lcs;

            LinkConfig.GetJoint(i)->GetP1_Link1_LCS(p_v1_lcs);
            LinkConfig.GetJoint(i)->GetP2_Link2_LCS(p_v2_lcs);
            LinkConfig.GetJoint(i)->GetQ1_Link1_LCS(q_v1_lcs);
            LinkConfig.GetJoint(i)->GetQ2_Link2_LCS(q_v2_lcs);

            double p1_x = p_v1_lcs(0); // x1*
            double p1_y = p_v1_lcs(1); // y1*
            double p2_x = p_v2_lcs(0); // x2*
            double p2_y = p_v2_lcs(1); // y2*

            double q1_x = q_v1_lcs(0); // x1**
            double q1_y = q_v1_lcs(1); // y1**
            double q2_x = q_v2_lcs(0); // x2**
            double q2_y = q_v2_lcs(1); // y2**

            double a = q1_x - p1_x;
            double b = q1_y - p1_y;
            
            double x1, y1, t1, x2, y2, t2;
            LinkConfig.GetLink(link1_idx) -> GetQ(x1, y1, t1);
            LinkConfig.GetLink(link2_idx) -> GetQ(x2, y2, t2);

            Eigen::Vector2d v1_perp(2);
            v1_perp(0) = a * sin(t1) - b * cos(t1);
            v1_perp(1) = a * cos(t1) + b * sin(t1);

            Eigen::Vector2d Pvec_diff(2);
            Pvec_diff(0) = x2 - x1 + p2_x*cos(t2) - p1_x*cos(t1) + p2_y*sin(t2) - p1_y*sin(t1);
            Pvec_diff(0) = y2 - y1 - p2_x*sin(t2) + p1_x*sin(t1) + p2_y*cos(t2) - p1_y*cos(t1);

            double err_x = v1_perp.dot(Pvec_diff);
            double err_y = t1 - t2;
            
            // Update error vector
            error_c(2 * i) = err_x;
            error_c(2 * i + 1) = err_y;
        }
        
    }

    return true;
}

// Implement this function
bool dspSolver::Make_M(void) { 
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    /// make mass matrix : M
    for(int i = 1; i <= link_num; i++){
        dspLink* cur_link = LinkConfig.GetLink(i);
        M(3 * (i-1), 3 * (i-1)) = cur_link->GetMass();
        M(3 * (i-1) + 1, 3 * (i-1) + 1) = cur_link->GetMass();
        M(3 * (i-1) + 2, 3 * (i-1) + 2) = cur_link->GetInertia();
    }
    return true;
}

// Implement this function
bool dspSolver::Make_J(void) {
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();
    /// make Jacobian matrix which has first-order partial derivatives of vector q's components : J
    for(int i = 0; i < joint_num; i++){
        // Current joint
        dspJoint* cur_joint = LinkConfig.GetJoint(i);
        
        if ((cur_joint -> GetType()) == 0) { // Revolue Joint

            Eigen::Vector2d v1_lcs;
            Eigen::Vector2d v2_lcs;

            cur_joint->GetP1_Link1_LCS(v1_lcs);
            cur_joint->GetP2_Link2_LCS(v2_lcs);

            int link1_idx = cur_joint->GetLink1ID();
            int link2_idx = cur_joint->GetLink2ID();

            double t1 = LinkConfig.GetLink(link1_idx)->GetTheta();
            double t2 = LinkConfig.GetLink(link2_idx)->GetTheta();

            if (link1_idx != 0){
                J(2 * i, 3 * (link1_idx - 1) + 2) += (sin(t1) * v1_lcs(0) + cos(t1) * v1_lcs(1));
                J(2 * i + 1, 3 * (link1_idx - 1) + 2) += (-cos(t1) * v1_lcs(0) + sin(t1) * v1_lcs(1));

                J(2 * i, 3 * (link1_idx - 1)) = -1.0;
                J(2 * i + 1, 3 * (link1_idx - 1) + 1) = -1.0;
            }

            if (link2_idx != 0){
                J(2 * i, 3 * (link2_idx - 1) + 2) += -(sin(t2) * v2_lcs(0) + cos(t2) * v2_lcs(1));
                J(2 * i + 1, 3 * (link2_idx - 1) + 2) += -(-cos(t2) * v2_lcs(0) + sin(t2) * v2_lcs(1));

                J(2 * i, 3 * (link2_idx - 1)) = 1.0;
                J(2 * i + 1, 3 * (link2_idx - 1) + 1) = 1.0;
            }

        }
        else { // Prismatic Joint
            int link1_idx = LinkConfig.GetJoint(i)->GetLink1ID();
            int link2_idx = LinkConfig.GetJoint(i)->GetLink2ID();

            Eigen::Vector2d p_v1_lcs;
            Eigen::Vector2d p_v2_lcs;
            Eigen::Vector2d q_v1_lcs;
            Eigen::Vector2d q_v2_lcs;

            LinkConfig.GetJoint(i)->GetP1_Link1_LCS(p_v1_lcs);
            LinkConfig.GetJoint(i)->GetP2_Link2_LCS(p_v2_lcs);
            LinkConfig.GetJoint(i)->GetQ1_Link1_LCS(q_v1_lcs);
            LinkConfig.GetJoint(i)->GetQ2_Link2_LCS(q_v2_lcs);

            double p1_x = p_v1_lcs(0); // x1*
            double p1_y = p_v1_lcs(1); // y1*
            double p2_x = p_v2_lcs(0); // x2*
            double p2_y = p_v2_lcs(1); // y2*

            double q1_x = q_v1_lcs(0); // x1**
            double q1_y = q_v1_lcs(1); // y1**
            double q2_x = q_v2_lcs(0); // x2**
            double q2_y = q_v2_lcs(1); // y2**

            double a = q1_x - p1_x;
            double b = q1_y - p1_y;
            
            double x1, y1, t1, x2, y2, t2;
            LinkConfig.GetLink(link1_idx) -> GetQ(x1, y1, t1);
            LinkConfig.GetLink(link2_idx) -> GetQ(x2, y2, t2);

            if(link1_idx != 0){
                J(2 * i, 3 * (link1_idx - 1)) = -a * sin(t1) + b * cos(t1);
                J(2 * i, 3 * (link1_idx - 1) + 1) = -a * cos(t1) -b * sin(t1);
                J(2 * i, 3 * (link1_idx - 1) + 2) = (a*(y1-y2) + b*(x2-x1))*sin(t1) + (a*p2_x+b*p2_y)*cos(t1 - t2) + (a*p2_y-b*p2_x)*sin(t2 - t1);
                J(2 * i + 1, 3 * (link1_idx - 1) + 2) = 1;
            }
            if(link2_idx != 0){
                J(2 * i, 3 * (link2_idx - 1)) = a * sin(t1) - b * cos(t1);
                J(2 * i, 3 * (link2_idx - 1) + 1) = a * cos(t1) + b * sin(t1);
                J(2 * i, 3 * (link2_idx - 1) + 2) = -(a*p2_x+b*p2_y)*cos(t1-t2) + (a*p2_y-b*p2_x)*sin(t1-t2);
                J(2 * i + 1, 3 * (link2_idx - 1) + 2) = -1;
            }
        }
        
    }

    return true;
}

// Implement this function
bool dspSolver::Make_J_dot(void) {
    /// make time derivative of Jacobian matrix  : J_dot
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    for(int i = 0; i < joint_num; i++){
        dspJoint* cur_joint = LinkConfig.GetJoint(i);

        if ((cur_joint -> GetType()) == 0) { // Revolue Joint

            Eigen::Vector2d v1_lcs;
            Eigen::Vector2d v2_lcs;

            cur_joint->GetP1_Link1_LCS(v1_lcs);
            cur_joint->GetP2_Link2_LCS(v2_lcs);

            int link1_idx = cur_joint->GetLink1ID();
            int link2_idx = cur_joint->GetLink2ID();

            double t1 = LinkConfig.GetLink(link1_idx)->GetTheta();
            double t2 = LinkConfig.GetLink(link2_idx)->GetTheta();

            double dt1 = LinkConfig.GetLink(link1_idx)->GetOmega();
            double dt2 = LinkConfig.GetLink(link2_idx)->GetOmega();

            if (link1_idx != 0){
                J_dot(2 * i, 3 * (link1_idx - 1) + 2) += dt1 * (cos(t1) * v1_lcs(0) - sin(t1) * v1_lcs(1));
                J_dot(2 * i + 1, 3 * (link1_idx - 1) + 2) += dt1 * (sin(t1) * v1_lcs(0) + cos(t1) * v1_lcs(1));
            }

            if (link2_idx != 0){
                J_dot(2 * i, 3 * (link2_idx - 1) + 2) += -dt2 * (cos(t2) * v2_lcs(0) - sin(t2) * v2_lcs(1));;
                J_dot(2 * i + 1, 3 * (link2_idx - 1) + 2) += -dt2 * (sin(t2) * v2_lcs(0) + cos(t2) * v2_lcs(1));
            }

        }
        else { // Prismatic Joint
            int link1_idx = LinkConfig.GetJoint(i)->GetLink1ID();
            int link2_idx = LinkConfig.GetJoint(i)->GetLink2ID();

            Eigen::Vector2d p_v1_lcs;
            Eigen::Vector2d p_v2_lcs;
            Eigen::Vector2d q_v1_lcs;
            Eigen::Vector2d q_v2_lcs;

            LinkConfig.GetJoint(i)->GetP1_Link1_LCS(p_v1_lcs);
            LinkConfig.GetJoint(i)->GetP2_Link2_LCS(p_v2_lcs);
            LinkConfig.GetJoint(i)->GetQ1_Link1_LCS(q_v1_lcs);
            LinkConfig.GetJoint(i)->GetQ2_Link2_LCS(q_v2_lcs);

            double p1_x = p_v1_lcs(0); // x1*
            double p1_y = p_v1_lcs(1); // y1*
            double p2_x = p_v2_lcs(0); // x2*
            double p2_y = p_v2_lcs(1); // y2*

            double q1_x = q_v1_lcs(0); // x1**
            double q1_y = q_v1_lcs(1); // y1**
            double q2_x = q_v2_lcs(0); // x2**
            double q2_y = q_v2_lcs(1); // y2**

            double a = q1_x - p1_x;
            double b = q1_y - p1_y;
            
            double x1, y1, t1, x2, y2, t2;
            double dx1, dy1, dt1, dx2, dy2, dt2;
            LinkConfig.GetLink(link1_idx) -> GetQ(x1, y1, t1);
            LinkConfig.GetLink(link2_idx) -> GetQ(x2, y2, t2);
            LinkConfig.GetLink(link1_idx) -> GetQDot(dx1, dy1, dt1);
            LinkConfig.GetLink(link2_idx) -> GetQDot(dx2, y2, t2);

            if(link1_idx != 0){
                J_dot(2 * i, 3 * (link1_idx - 1)) = dt1 * (-a * cos(t1) - b * sin(t1));  // (-a * sin(t1) + b * cos(t1))
                J_dot(2 * i, 3 * (link1_idx - 1) + 1) = dt1 * (a * sin(t1) -b * cos(t1)); // -a * cos(t1) -b * sin(t1)
                // (a*(y1-y2) + b*(x2-x1))*sin(t1) + (a*p2_x+b*p2_y)*cos(t1 - t2) + (a*p2_y-b*p2_x)*sin(t2 - t1)
                J_dot(2 * i, 3 * (link1_idx - 1) + 2) = dt1 * (a*(y1-y2) + b*(x2-x1))*cos(t1) + (a*(dy1-dy2) + b*(dx2-dx1))*sin(t1) - (dt1 - dt2) * (a*p2_x+b*p2_y)*sin(t1 - t2) + (dt2 - dt1) * (a*p2_y-b*p2_x)*cos(t2 - t1);
            }
            if(link2_idx != 0){
                J_dot(2 * i, 3 * (link2_idx - 1)) = dt1 * (a * cos(t1) + b * sin(t1));  // a * sin(t1) - b * cos(t1)
                J_dot(2 * i, 3 * (link2_idx - 1) + 1) = dt1 * (-a * sin(t1) + b * cos(t1));  // a * cos(t1) + b * sin(t1)
                // -(a*p2_x+b*p2_y)*cos(t1-t2) + (a*p2_y-b*p2_x)*sin(t1-t2)
                J_dot(2 * i, 3 * (link2_idx - 1) + 2) = (dt1 - dt2) * (a*p2_x+b*p2_y)*sin(t1-t2) + (dt1 - dt2) * (a*p2_y-b*p2_x)*cos(t1-t2);
            }
        }
    }
    return true;
}

// Implement this function
bool dspSolver::Make_F_ext(void) {
    /// make a n-dimensional vector for external force/torque : F_ext
    /// it contains x direction of force, y direction of force, and torque
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();
    
    double g = 9.81;

    // Force
    for(int i = 1; i <= link_num; i++){
        double m_ = LinkConfig.GetLink(i)->GetMass();
        F_ext(3 * (i - 1) + 1) = -m_ * g;
    }
    
    return true;
}

// Implement this function
bool dspSolver::CalcLinAlg(void) {
    /// calculate second derivative of vector q : qddot
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    M_inv = M.inverse();
    
    Eigen::MatrixXd A = J * M_inv * J.transpose();
    Eigen::VectorXd B = -J_dot * qdot;
    Eigen::VectorXd C = -J * M_inv * F_ext;

    lambda = A.inverse() * (B+C);
    Eigen::MatrixXd Fc = (J.transpose() * lambda);
    
    qddot = M_inv * (F_ext + Fc);
    
    return true;
}

// Implement this function
bool dspSolver::UpdateCurrentInfo(void) {
    /// load and update information what you need in class variables
    /// current vector q and its derivative should be loaded : q, q_dot
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    double x, y, t, dx, dy, dt;
    for (int i = 1; i <= link_num; i++){
        LinkConfig.GetLink(i)->GetQ(x, y, t);
        LinkConfig.GetLink(i)->GetQDot(dx, dy, dt);

        q(3 * (i-1)) = x;
        q(3 * (i-1) + 1) = y;
        q(3 * (i-1) + 2) = t;
        qdot(3 * (i-1)) = dx;
        qdot(3 * (i-1) + 1) = dy;
        qdot(3 * (i-1) + 2) = dt;
    }
    
    M.setZero();
    J.setZero();
    J_dot.setZero();
    F_ext.setZero();

    Make_M();
    Make_J();
    Make_J_dot();
    Make_F_ext();

    return true;
}

// Implement this function
bool dspSolver::UpdateNextInfo(double timestep) {
    /// update vector q and its derivative : q, q_dot
    /// also save them in class variables for nex step

    qdot = qdot + timestep * qddot;
    q = q + timestep * qdot;

    SetQ();
    SetQDot();

    return true;
}

// Implement this function
bool dspSolver::SetQDot(void) {
    /// update new q_dot information in each link structure
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    double dx, dy, dt;
    for(int i = 1; i <= link_num; i++){
        dx = qdot(3 * (i-1));
        dy = qdot(3 * (i-1) + 1);
        dt = qdot(3 * (i-1) + 2);
        LinkConfig.GetLink(i) -> SetQDot(dx, dy, dt);
    }
    return true;
}

// Implement this function
bool dspSolver::SetQ(void) {
    /// update new q information in each link structure
    int link_num = (LinkConfig.GetNumLink() - 1);
    int joint_num = LinkConfig.GetNumJoint();

    double x, y, t;
    for(int i = 1; i <= link_num; i++){
        x = q(3 * (i-1));
        y = q(3 * (i-1) + 1);
        t = q(3 * (i-1) + 2);
        LinkConfig.GetLink(i) -> SetQ(x, y, t);
    }
    return true;
}

// Do not change this function
bool dspSolver::GetQ(Eigen::VectorXd& q_record) {
    /// get current q information
    q_record = q;
    return true;
}

// Do not change this function
bool dspSolver::GetQDot(Eigen::VectorXd& qdot_record) {
    /// get current q_dot information 
    qdot_record = qdot;
    return true;
}
