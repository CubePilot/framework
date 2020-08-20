#pragma once
#include <Eigen/Dense>

#include <math.h>

template <typename Derived>
static Eigen::Vector3d ecef2lla(const Eigen::DenseBase<Derived>& pos_ecef) {
    Eigen::Vector3d ret;

    auto subx0 = 1/(pow(0.99330562000985867*((pos_ecef(0))*(pos_ecef(0))) + 0.99330562000985867*((pos_ecef(1))*(pos_ecef(1))) + ((pos_ecef(2))*(pos_ecef(2))), 3.0/2.0));
    auto subx1 = ((pos_ecef(2))*(pos_ecef(2))*(pos_ecef(2)))*subx0;
    auto subx2 = sqrt(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))));
    auto subx3 = subx0*pow(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))), 3.0/2.0);
    auto subx4 = ((2.3341955805653505e-5*pos_ecef(2) + subx1)*(2.3341955805653505e-5*pos_ecef(2) + subx1)) + 0.97349017036138108*((2.365764208656198e-5*subx2 - subx3)*(2.365764208656198e-5*subx2 - subx3));
    ret(0) = atan2(pos_ecef(2) + 42841.311513313565*subx1, subx2 - 42269.639397749626*subx3);
    ret(1) = atan2(pos_ecef(1), pos_ecef(0));
    ret(2) = ((fabs(pos_ecef(0)) > 1 && fabs(pos_ecef(1)) > 1) ? ( 42841.311513313565*subx2*sqrt(subx4)/(subx2 - 42269.639397749626*subx3) - 6378137/sqrt(1 - 0.0066943799901413989*((2.3341955805653505e-5*pos_ecef(2) + subx1)*(2.3341955805653505e-5*pos_ecef(2) + subx1))/subx4) ) : ( fabs(pos_ecef(2)) - 6356752.3142451793 ));

    return ret;
}

template <typename Derived>
static Eigen::Vector3d lla2ecef(const Eigen::DenseBase<Derived>& pos_lla) {
    Eigen::Vector3d ret;

    const auto& lat = pos_lla(0);
    const auto& lon = pos_lla(1);
    const auto& alt = pos_lla(2);

    auto subx0 = 1/(sqrt(1 - 0.0066943799901400007*((sin(lat))*(sin(lat)))));
    auto subx1 = (alt + 6378137*subx0)*cos(lat);
    ret(0) = subx1*cos(lon);
    ret(1) = subx1*sin(lon);
    ret(2) = (alt + 6335439.3272928288*subx0)*sin(lat);

    return ret;
}

template <typename Derived1, typename Derived2>
static Eigen::Vector3f gravity_ecef(const Eigen::DenseBase<Derived1>& pos_ecef, const Eigen::DenseBase<Derived2>& vel_ecef) {
    Eigen::Vector3f ret;

    auto subx0 = ((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))) + ((pos_ecef(2))*(pos_ecef(2)));
    auto subx1 = 398600441800000.0*(1 - 66063108268.671356*(5*((pos_ecef(2))*(pos_ecef(2)))/subx0 - 1)/subx0)/pow(subx0, 3.0/2.0);
    ret(0) = -pos_ecef(0)*subx1 + 5.3174954299032819e-9*pos_ecef(0) + 0.00014584231800000001*vel_ecef(1);
    ret(1) = -pos_ecef(1)*subx1 + 5.3174954299032819e-9*pos_ecef(1) - 0.00014584231800000001*vel_ecef(0);
    ret(2) = -398600441800000.0*pos_ecef(2)*(1 - 66063108268.671356*(5*((pos_ecef(2))*(pos_ecef(2)))/subx0 - 3)/subx0)/pow(subx0, 3.0/2.0);

    return ret;
}

template <typename Derived>
static Eigen::Matrix3f rotation_ecef_to_ned_from_ecef(const Eigen::DenseBase<Derived>& pos_ecef) {
    Eigen::Matrix3f ret;

    auto subx0 = 1/(sqrt(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1)))));
    auto subx1 = 1/(pow(40408299984661.445*((pos_ecef(0))*(pos_ecef(0))) + 40408299984661.445*((pos_ecef(1))*(pos_ecef(1))) + 40680631590769*((pos_ecef(2))*(pos_ecef(2))), 3.0/2.0));
    auto subx2 = 1.1115891217205065e+25*((pos_ecef(2))*(pos_ecef(2))*(pos_ecef(2)))*subx1 + pos_ecef(2);
    auto subx3 = -1.0967561373321939e+25*subx1*pow(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))), 3.0/2.0) + sqrt(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))));
    auto subx4 = 1/(sqrt(((subx2)*(subx2)) + ((subx3)*(subx3))));
    ret(0, 0) = -pos_ecef(0)*subx0*subx2*subx4;
    ret(0, 1) = -pos_ecef(1)*subx0;
    ret(0, 2) = -pos_ecef(0)*subx0*subx3*subx4;
    ret(1, 0) = -pos_ecef(1)*subx0*subx2*subx4;
    ret(1, 1) = pos_ecef(0)*subx0;
    ret(1, 2) = -pos_ecef(1)*subx0*subx3*subx4;
    ret(2, 0) = subx3*subx4;
    ret(2, 1) = 0;
    ret(2, 2) = -subx2*subx4;

    return ret;
}


static Eigen::Matrix3f rotation_ecef_to_ned_from_lat_lon(float lat, float lon) {
    Eigen::Matrix3f ret;

    ret(0, 0) = -sin(lat)*cos(lon);
    ret(0, 1) = -sin(lon);
    ret(0, 2) = -cos(lat)*cos(lon);
    ret(1, 0) = -sin(lat)*sin(lon);
    ret(1, 1) = cos(lon);
    ret(1, 2) = -sin(lon)*cos(lat);
    ret(2, 0) = cos(lat);
    ret(2, 1) = 0;
    ret(2, 2) = -sin(lat);

    return ret;
}

template <typename Derived>
static Eigen::Quaternionf quat_ecef_to_ned_from_ecef(const Eigen::DenseBase<Derived>& pos_ecef) {
    Eigen::Quaternionf ret;

    auto subx0 = 1/(pow(0.99330562000985867*((pos_ecef(0))*(pos_ecef(0))) + 0.99330562000985867*((pos_ecef(1))*(pos_ecef(1))) + ((pos_ecef(2))*(pos_ecef(2))), 3.0/2.0));
    auto subx1 = 0.5*atan2(42841.311513313565*((pos_ecef(2))*(pos_ecef(2))*(pos_ecef(2)))*subx0 + pos_ecef(2), -42269.639397749626*subx0*pow(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1))), 3.0/2.0) + sqrt(((pos_ecef(0))*(pos_ecef(0))) + ((pos_ecef(1))*(pos_ecef(1)))));
    auto subx2 = (1.0/2.0)*sqrt(2)*cos(0.5*atan2(pos_ecef(1), pos_ecef(0)));
    auto subx3 = (1.0/2.0)*sqrt(2)*sin(0.5*atan2(pos_ecef(1), pos_ecef(0)));
    ret.w() = -subx2*sin(subx1) + subx2*cos(subx1);
    ret.x() = subx3*sin(subx1) + subx3*cos(subx1);
    ret.y() = -subx2*sin(subx1) - subx2*cos(subx1);
    ret.z() = -subx3*sin(subx1) + subx3*cos(subx1);

    return ret;
}

static Eigen::Quaternionf quat_ecef_to_ned_from_lat_lon(float lat, float lon) {
    Eigen::Quaternionf ret;

    auto subx0 = 0.5*lat;
    auto subx1 = (1.0/2.0)*sqrt(2)*cos(0.5*lon);
    auto subx2 = (1.0/2.0)*sqrt(2)*sin(0.5*lon);
    ret.w() = -subx1*sin(subx0) + subx1*cos(subx0);
    ret.x() = subx2*sin(subx0) + subx2*cos(subx0);
    ret.y() = -subx1*sin(subx0) - subx1*cos(subx0);
    ret.z() = -subx2*sin(subx0) + subx2*cos(subx0);

    return ret;
}
