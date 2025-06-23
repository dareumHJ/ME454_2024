#include "../include/myQuaternion.hpp"
#include <limits> // For std::numeric_limits

//////////////////// TO DO  ////////////////////
// TODO: refer "myQuaternion.hpp" and complete below functions.
Quaternion::Quaternion() {
}
Quaternion::Quaternion(double w, double x, double y, double z) {
    set_elem(0, 0, w);
    set_elem(1, 0, x);
    set_elem(2, 0, y);
    set_elem(3, 0, z);
}
double Quaternion::w() {
    return get_elem(0, 0);
}
double Quaternion::x() {
    return get_elem(1, 0);
}
double Quaternion::y() {
    return get_elem(2, 0);
}
double Quaternion::z() {
    return get_elem(3, 0);
}
void Quaternion::inversed(){
    double snorm = squarednorm();
    set_elem(0, 0, get_elem(0, 0) / snorm);
    set_elem(1, 0, -get_elem(1, 0) / snorm);
    set_elem(2, 0, -get_elem(2, 0) / snorm);
    set_elem(3, 0, -get_elem(3, 0) / snorm);
}
void Quaternion::normalized(){
    double norm = sqrt(squarednorm());
    set_elem(0, 0, get_elem(0, 0) / norm);
    set_elem(1, 0, get_elem(1, 0) / norm);
    set_elem(2, 0, get_elem(2, 0) / norm);
    set_elem(3, 0, get_elem(3, 0) / norm);
}
void Quaternion::display()
{
    printf("{w : %f, x : %f, y : %f, z : %f}\n", get_elem(0, 0), get_elem(1, 0), get_elem(2, 0), get_elem(3, 0));
}
Quaternion getQuaternionBetweenTimeStep(Vec3 w, double delta_t){
    Vec3 a = w * (delta_t / 2);
    double norm = sqrt(a.squarednorm());
    Quaternion q;
    if (norm == 0) q.set_elem(0, 0, 1);
    else {
        q.set_elem(0, 0, cos(norm));
        q.set_elem(1, 0, sin(norm) / norm * a.get_elem(0, 0));
        q.set_elem(2, 0, sin(norm) / norm * a.get_elem(1, 0));
        q.set_elem(3, 0, sin(norm) / norm * a.get_elem(2, 0));
    }
    return q;
}
Quaternion quatmulquat(Quaternion q1, Quaternion q2){
    Quaternion q;
    q.set_elem(0, 0, q1.get_elem(0, 0) * q2.get_elem(0, 0) - q1.get_elem(1, 0) * q2.get_elem(1, 0) - q1.get_elem(2, 0) * q2.get_elem(2, 0) - q1.get_elem(3, 0) * q2.get_elem(3, 0));
    q.set_elem(1, 0, q1.get_elem(0, 0) * q2.get_elem(1, 0) + q1.get_elem(1, 0) * q2.get_elem(0, 0) + q1.get_elem(2, 0) * q2.get_elem(3, 0) - q1.get_elem(3, 0) * q2.get_elem(2, 0));
    q.set_elem(2, 0, q1.get_elem(0, 0) * q2.get_elem(2, 0) - q1.get_elem(1, 0) * q2.get_elem(3, 0) + q1.get_elem(2, 0) * q2.get_elem(0, 0) + q1.get_elem(3, 0) * q2.get_elem(1, 0));
    q.set_elem(3, 0, q1.get_elem(0, 0) * q2.get_elem(3, 0) + q1.get_elem(1, 0) * q2.get_elem(2, 0) - q1.get_elem(2, 0) * q2.get_elem(1, 0) + q1.get_elem(3, 0) * q2.get_elem(0, 0));
    return q;
}
Vec3 vecRotatedByQuat(Vec3 v, Quaternion q){
    Quaternion vv(0, v.get_elem(0, 0), v.get_elem(1, 0), v.get_elem(2, 0));
    Quaternion q1 = quatmulquat(q, vv);
    q.inversed();
    Quaternion q2 = quatmulquat(q1, q);
    Vec3 ret(q2.get_elem(1, 0), q2.get_elem(2, 0), q2.get_elem(3, 0));
    return ret;
}
Quaternion mat2quat(const Mat33& mat){
    double e11 = mat.get_elem(0, 0);
    double e12 = mat.get_elem(0, 1);
    double e13 = mat.get_elem(0, 2);
    double e21 = mat.get_elem(1, 0);
    double e22 = mat.get_elem(1, 1);
    double e23 = mat.get_elem(1, 2);
    double e31 = mat.get_elem(2, 0);
    double e32 = mat.get_elem(2, 1);
    double e33 = mat.get_elem(2, 2);
    double tr = (e11 + e22 + e33);
    double dn;
    double w;
    double x;
    double y;
    double z;
    if (tr > 0){
        dn = 2 * sqrt(1 + tr);
        w = dn / 4;
        x = (e32 - e23) / dn;
        y = (e12 - e21) / dn;
        z = (e21 - e12) / dn;
    }
    else if ((e11 > e22) && (e11 > e33)) {
        dn = 2 * sqrt(1 + e11 - e22 - e33);
        w = (e32 - e23) / dn;
        x = dn / 4;
        y = (e12 + e21) / dn;
        z = (e12 + e21) / dn;
    }
    else if (e22 > e33){
        dn = 2 * sqrt(1 + e22 - e11 - e33);
        w = (e13 - e31) / dn;
        x = (e12 + e21) / dn;
        y = dn / 4;
        z = (e23 + e32) / dn;
    }
    else{
        dn = 2 * sqrt(1 + e33 - e11 - e22);
        w = (e21 - e12) / dn;
        x = (e13 + e31) / dn;
        y = (e23 + e32) / dn;
        z = dn / 4;
    }
    Quaternion quat(w, x, y, z);
    return quat;
}
Mat33 quat2mat(Quaternion& q) {
    double w = q.get_elem(0, 0);
    double x = q.get_elem(1, 0);
    double y = q.get_elem(2, 0);
    double z = q.get_elem(3, 0);
    double e11 = 1 - (2 * y * y + 2 * z * z);
    double e12 = 2*x*y - 2*z*w;
    double e13 = 2*x*z + 2*y*w;
    double e21 = 2*x*y + 2*z*w;
    double e22 = 1 - (2 * x * x + 2 * z * z);
    double e23 = 2*y*z - 2*x*w;
    double e31 = 2*x*z - 2*y*w;
    double e32 = 2*y*z + 2*x*w;
    double e33 = 1 - (2 * x * x + 2 * y * y);
    Mat33 mat(e11, e12, e13, e21, e22, e23, e31, e32, e33);
    return mat;
}
//////////////////// TO DO end ////////////////////
