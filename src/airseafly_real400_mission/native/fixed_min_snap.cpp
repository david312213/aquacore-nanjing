// C ABI around the vendored GCOPTER MINCO_S4NU solver. No ROS or flight I/O.
#include "gcopter/minco.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <vector>

namespace {
using Coeff = Eigen::Matrix<double, 3, 8>;

// All coefficients here are ascending powers of unit segment time u in [0, 1].
Eigen::Vector3d derivative(const Coeff &c, double u, int order) {
    Eigen::Vector3d value = Eigen::Vector3d::Zero();
    for (int k = 7; k >= order; --k) {
        double multiplier = 1.0;
        for (int j = 0; j < order; ++j) multiplier *= k-j;
        value = value*u + multiplier*c.col(k);
    }
    return value;
}

// Critical points of ||d^order p / du^order||^2, using GCOPTER's root isolator.
// Scaling the polynomial avoids the upstream absolute-small-coefficient shortcut
// losing extrema of very small spatial segments. Endpoints are always included.
double maximumDerivative(const Coeff &c, int order) {
    const int degree = 7-order;
    Eigen::MatrixXd d(3, degree+1);
    for (int k = order; k <= 7; ++k) {
        double multiplier = 1.0;
        for (int j = 0; j < order; ++j) multiplier *= k-j;
        d.col(degree-(k-order)) = multiplier*c.col(k);
    }
    const double amplitude = d.cwiseAbs().maxCoeff();
    if (amplitude == 0.0) return 0.0;
    d /= amplitude;
    const Eigen::VectorXd squared = RootFinder::polySqr(d.row(0)) +
        RootFinder::polySqr(d.row(1)) + RootFinder::polySqr(d.row(2));
    Eigen::VectorXd critical(squared.size()-1);
    for (int k=0; k<critical.size(); ++k)
        critical[k] = squared[k]*(squared.size()-1-k);
    double result = std::max(derivative(c, 0., order).norm(),
                             derivative(c, 1., order).norm());
    const double scale = critical.cwiseAbs().maxCoeff();
    if (scale > 0.0) {
        critical /= scale;
        // Solve beyond the segment so an endpoint root cannot violate the root
        // isolator's open-boundary contract. Adjust only the numerical bracket.
        double lo=-0.125, hi=1.125;
        while (std::abs(RootFinder::polyVal(critical, lo)) < 1e-14) lo*=2.;
        while (std::abs(RootFinder::polyVal(critical, hi)) < 1e-14) hi=1.+2.*(hi-1.);
        for (double root: RootFinder::solvePolynomial(critical, lo, hi, 1e-11))
            if (root>=0. && root<=1.) result=std::max(result,derivative(c,root,order).norm());
    }
    return result;
}
}

extern "C" int airseafly_min_snap_abi_version() { return 1; }

// xyz: (segments+1)*3; durations: segments; output: segments*3*8, ascending
// normalized coefficients. durations are updated by ONE global time dilation.
// maxima: achieved speed, achieved acceleration, global time scale.
extern "C" int airseafly_min_snap_solve(int segments, const double *xyz,
    double *durations, double max_velocity, double max_acceleration,
    double *output, double *maxima, char *error, int error_capacity) {
    try {
        if (segments<1 || !xyz || !durations || !output || !maxima ||
            !(max_velocity>0.) || !(max_acceleration>0.) ||
            !std::isfinite(max_velocity) || !std::isfinite(max_acceleration))
            throw std::invalid_argument("invalid MINCO input buffers or limits");
        double time_unit=0.;
        for (int i=0; i<segments; ++i) {
            if (!(durations[i]>0.) || !std::isfinite(durations[i]))
                throw std::invalid_argument("MINCO segment duration must be finite and positive");
            time_unit=std::max(time_unit,durations[i]);
        }
        Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic>> nodes(xyz,3,segments+1);
        if (!nodes.allFinite()) throw std::invalid_argument("nonfinite MINCO coordinates");
        const Eigen::Vector3d origin=nodes.col(0);
        Eigen::Matrix3Xd positions=nodes.colwise()-origin;
        double space_unit=positions.cwiseAbs().maxCoeff();
        if (space_unit==0.) space_unit=1.;
        positions/=space_unit;
        Eigen::Matrix<double,3,4> head=Eigen::Matrix<double,3,4>::Zero(),tail=head;
        head.col(0)=positions.col(0); tail.col(0)=positions.col(segments);
        Eigen::VectorXd times(segments);
        for (int i=0; i<segments; ++i) times[i]=durations[i]/time_unit;
        minco::MINCO_S4NU solver;
        solver.setConditions(head,tail,segments);
        solver.setParameters(positions.middleCols(1,segments-1),times);
        Trajectory<7> trajectory;
        solver.getTrajectory(trajectory);
        std::vector<Coeff> coefficients;
        double speed=0.,acceleration=0.;
        for (int i=0; i<segments; ++i) {
            Coeff c=trajectory[i].normalizePosCoeffMat().rowwise().reverse()*space_unit;
            c.col(0)+=origin;
            // Height transitions are independently monotone, with C3 joins.
            const double dz=nodes(2,i+1)-nodes(2,i);
            c.row(2).setZero(); c(2,0)=nodes(2,i);
            c(2,4)=35.*dz; c(2,5)=-84.*dz; c(2,6)=70.*dz; c(2,7)=-20.*dz;
            if (!c.allFinite()) throw std::runtime_error("MINCO returned nonfinite coefficients");
            coefficients.push_back(c);
            speed=std::max(speed,maximumDerivative(c,1)/durations[i]);
            acceleration=std::max(acceleration,maximumDerivative(c,2)/(durations[i]*durations[i]));
        }
        // Numerical rounding margin only; no per-sample clipping or stop points.
        double factor=std::max(1.,std::max(speed/max_velocity,std::sqrt(acceleration/max_acceleration)));
        factor*=1.+1e-10;
        if (!std::isfinite(factor)) throw std::runtime_error("MINCO time scale is nonfinite");
        for (int i=0; i<segments; ++i) {
            durations[i]*=factor;
            for (int axis=0; axis<3; ++axis)
                for (int k=0; k<8; ++k) output[(i*3+axis)*8+k]=coefficients[i](axis,k);
        }
        maxima[0]=speed/factor; maxima[1]=acceleration/(factor*factor); maxima[2]=factor;
        if (error && error_capacity>0) error[0]='\0';
        return 0;
    } catch (const std::exception &ex) {
        if (error && error_capacity>0) std::snprintf(error,error_capacity,"%s",ex.what());
        return 1;
    }
}
