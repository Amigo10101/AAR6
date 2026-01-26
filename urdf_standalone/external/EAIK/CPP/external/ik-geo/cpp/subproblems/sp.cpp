#include "sp.h"
#include <math.h>
#include <vector>
#include <limits>
#include <complex>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>

namespace IKS
{
	std::pair<Eigen::Vector2d, Eigen::Vector3d> cone_polynomials(const Eigen::Vector3d &p0_i, const Eigen::Vector3d &k_i, const Eigen::Vector3d &p_i, const Eigen::Vector3d &p_i_s, const Eigen::Vector3d &k2)
	{
		Eigen::Vector2d P;
		Eigen::Vector3d R;

		Eigen::Matrix<double, 3, 1> kiXk2 = k_i.cross(k2);
		Eigen::Matrix<double, 3, 1> kiXkiXk2 = k_i.cross(kiXk2);
		double norm_kiXk2_sq = kiXk2.dot(kiXk2);

		Eigen::Matrix<double, 3, 1> kiXpi = k_i.cross(p_i);
		double norm_kiXpi_sq = kiXpi.dot(kiXpi);

		double delta = k2.dot(p_i_s);
		double alpha = (p0_i.transpose() * kiXkiXk2 / norm_kiXk2_sq)(0, 0);
		double beta = (p0_i.transpose() * kiXk2 / norm_kiXk2_sq)(0, 0);

		double P_const = norm_kiXpi_sq + p_i_s.dot(p_i_s) + 2 * alpha * delta;
		P << -2 * alpha, P_const;

		R << -1, 2 * delta, -pow(delta, 2)+norm_kiXpi_sq * norm_kiXk2_sq;
		R = pow(2 * beta, 2) * R;

		return {P, R};
	}

	Eigen::Matrix<double, 1, 3> convolution_2(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2)
	{
		Eigen::Matrix<double, 1, 3> res;
		res << v1.x() * v2.x(), v1.x() * v2.y() + v1.y() * v2.x(), v1.y() * v2.y();
		return res;
	}

	Eigen::Matrix<double, 1, 5> convolution_3(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
	{
		Eigen::Matrix<double, 1, 5> res;
		res << v1.x() * v2.x(), v1.y() * v2.x() + v1.x() * v2.y(), v1.x() * v2.z() + v1.y() * v2.y() + v1.z() * v2.x(),
			v1.y() * v2.z() + v1.z() * v2.y(), v1.z() * v2.z();
		return res;
	}

	std::vector<std::complex<double>> quartic_roots(const Eigen::Matrix<double, 1, 5> &poly)
	{
		Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
		solver.compute(poly);
		const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &roots = solver.roots();

		std::vector<std::complex<double>> result(roots.size());
		for (int i = 0; i < roots.size(); ++i) {
			result[i] = roots[i];
		}

		return result;
	}

	std::vector<std::pair<double, double>> solve_2_ellipse_numeric(const Eigen::Vector2d &xm1, const Eigen::Matrix<double, 2, 2> &xn1,
														const Eigen::Vector2d &xm2, const Eigen::Matrix<double, 2, 2> &xn2)
	{
		/* solve for intersection of 2 ellipses defined by

		xm1'*xm1 + xi'*xn1'*xn1*xi  + xm1'*xn1*xi == 1
		xm2'*xm2 + xi'*xn2'*xn2*xi  + xm2'*xn2*xi == 1
		Where xi = [xi_1; xi_2] */
		const double EPSILON = 1e-5;
		Eigen::Matrix<double, 2, 2> A_1 = xn1.transpose() * xn1;
		double a = A_1.coeffRef(0, 0);
		double b = 2 * A_1.coeffRef(1, 0);
		double c = A_1.coeffRef(1, 1);
		Eigen::Matrix<double, 1, 2> B_1 = 2 * xm1.transpose() * xn1;
		double d = B_1.coeffRef(0, 0);
		double e = B_1.coeffRef(0, 1);
		double f = xm1.transpose() * xm1 - 1;

		Eigen::Matrix<double, 2, 2> A_2 = xn2.transpose() * xn2;
		double a1 = A_2.coeffRef(0, 0);
		double b1 = 2 * A_2.coeffRef(1, 0);
		double c1 = A_2.coeffRef(1, 1);
		Eigen::Matrix<double, 1, 2> B_2 = 2 * xm2.transpose() * xn2;
		double d1 = B_2.coeffRef(0, 0);
		double e1 = B_2.coeffRef(0, 1);
		double fq = xm2.transpose() * xm2 - 1;

    double z0 = f*a*d1*d1 + a*a*fq*fq - d*a*d1*fq + a1*a1*f*f - 2.0*a*fq*a1*f - d*d1*a1*f +
        a1*d*d*fq;

    double z1 = e1*d*d*a1 - fq*d1*a*b - 2.0*a*fq*a1*e - f*a1*b1*d + 2.0*d1*b1*a*f +
        2.0*e1*fq*a*a + d1*d1*a*e - e1*d1*a*d - 2.0*a*e1*a1*f - f*a1*d1*b + 2.0*f*e*a1*a1 -
        fq*b1*a*d - e*a1*d1*d + 2.0*fq*b*a1*d;

    double z2 = e1*e1*a*a + 2.0*c1*fq*a*a - e*a1*d1*b + fq*a1*b*b - e*a1*b1*d - fq*b1*a*b -
        2.0*a*e1*a1*e + 2.0*d1*b1*a*e - c1*d1*a*d - 2.0*a*c1*a1*f + b1*b1*a*f + 2.0*e1*b*a1*d +
        e*e*a1*a1 - c*a1*d1*d - e1*b1*a*d + 2.0*f*c*a1*a1 - f*a1*b1*b + c1*d*d*a1 + d1*d1*a*c -
        e1*d1*a*b - 2.0*a*fq*a1*c;

    double z3 = -2.0*a*a1*c*e1 + e1*a1*b*b + 2.0*c1*b*a1*d - c*a1*b1*d + b1*b1*a*e - e1*b1*a*b -
        2.0*a*c1*a1*e - e*a1*b1*b - c1*b1*a*d + 2.0*e1*c1*a*a + 2.0*e*c*a1*a1 - c*a1*d1*b +
        2.0*d1*b1*a*c - c1*d1*a*b;

    double z4 = a*a*c1*c1 - 2.0*a*c1*a1*c + a1*a1*c*c - b*a*b1*c1 - b*b1*a1*c + b*b*a1*c1 +
        c*a*b1*b1;
		Eigen::Matrix<double, 1, 5> z;
		z << z0,z1, z2, z3, z4;
		std::vector<std::complex<double>> roots = quartic_roots(z);

		std::vector<std::pair<double, double>> xi;
		for (const auto& root : roots)
		{
			if(std::fabs(root.imag()) < EPSILON)
			{
				double y_r = root.real();
				double y_sq = y_r * y_r;
				double x = -(a * fq + a * c1 * y_sq - a1 * c * y_sq + a * e1 * y_r - a1 * e * y_r - a1 * f) / (a * b1 * y_r + a * d1 - a1 * b * y_r - a1 * d);
				xi.push_back({x, y_r});
			}
		}


		return xi;
	}

	// ===== SUBPROBLEMS ===== //

	void sp2E_run(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
				  const Eigen::Vector3d &k1, const Eigen::Vector3d &k2,
				  double &theta1, double &theta2)
	{

		Eigen::Matrix<double, 3, 1> KxP1 = k1.cross(p1);
		Eigen::Matrix<double, 3, 1> KxP2 = k2.cross(p2);

		Eigen::Matrix<double, 3, 2> A_1, A_2;
		A_1 << KxP1, -k1.cross(KxP1);
		A_2 << KxP2, -k2.cross(KxP2);

		Eigen::Matrix<double, 3, 4> A;
		A << A_1, -A_2;

		Eigen::Vector3d p = -k1 * k1.dot(p1) + k2 * k2.dot(p2) - p0;

		double radius_1_sp = KxP1.dot(KxP1);
		double radius_2_sp = KxP2.dot(KxP2);

		double alpha = radius_1_sp / (radius_1_sp + radius_2_sp);
		double beta = radius_2_sp / (radius_1_sp + radius_2_sp);
		Eigen::Matrix<double, 3, 3> M_inv, AAT_inv;
		M_inv = Eigen::Matrix3d::Identity(3, 3) + k1 * k1.transpose() * (alpha / (1 - alpha));
		AAT_inv = 1 / (radius_1_sp + radius_2_sp) * (M_inv + M_inv * k2 * k2.transpose() * M_inv * beta / (1.0 - (k2.transpose() * M_inv * k2 * beta)(0, 0)));
		Eigen::Matrix<double, 4, 1> x_ls = A.transpose() * AAT_inv * p;

		Eigen::Matrix<double, 3, 1> n_sym = k1.cross(k2);
		Eigen::Matrix<double, 2, 3> pinv_A1, pinv_A2;
		pinv_A1 = A_1.transpose() / radius_1_sp;
		pinv_A2 = A_2.transpose() / radius_2_sp;
		Eigen::Matrix<double, 4, 1> A_perp_tilde;
		Eigen::Matrix<double, 4, 3> temp;
		temp << pinv_A1,
			pinv_A2;
		A_perp_tilde = temp * n_sym;

		double num = (pow(x_ls.block<2, 1>(2, 0).norm(), 2) - 1) * pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2) - (pow(x_ls.block<2, 1>(0, 0).norm(), 2) - 1) * pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2);
		double den = 2 * (x_ls.block<2, 1>(0, 0).transpose() * A_perp_tilde.block<2, 1>(0, 0) * pow(A_perp_tilde.block<2, 1>(2, 0).norm(), 2) - x_ls.block<2, 1>(2, 0).transpose() * A_perp_tilde.block<2, 1>(2, 0) * pow(A_perp_tilde.block<2, 1>(0, 0).norm(), 2))(0, 0);

		double xi = num / den;

		Eigen::Matrix<double, 4, 1> sc = x_ls + xi * A_perp_tilde;

		theta1 = atan2(sc(0, 0), sc(1, 0));
		theta2 = atan2(sc(2, 0), sc(3, 0));
	}

	///
	//
	// Subproblem 1: [Circle and Point] 'rot(k, theta) * p1 = p2'
	//
	///

	SP1::SP1(const Eigen::Vector3d &p1,
			 const Eigen::Vector3d &p2,
			 const Eigen::Vector3d &k) : p1(p1), p2(p2), k(k)
	{
	}

	void SP1::solve()
	{
		const Eigen::Vector3d kxp = k.cross(p1);
		Eigen::Matrix<double, 2, 3> a;
		a.row(0) = kxp;
		a.row(1) = -k.cross(kxp);

		const Eigen::Vector2d x = a * p2;

		if(x.norm() > ZERO_THRESH)
		{
			theta = std::atan2(x.x(), x.y());
			_solution_is_ls = std::fabs(p1.norm() - p2.norm()) > ZERO_THRESH ||
			std::fabs(k.dot(p1) - k.dot(p2)) > ZERO_THRESH;
		}
		else
		{
			theta = 0;
			_solution_is_ls = true;
		}


		is_calculated = true;
	}

	double SP1::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("Error() function of SP1 was called before it was solved!\n");
		}
		return error(this->theta);
	}

	double SP1::error(const double& theta) const
	{
		Eigen::Matrix3d rot = Eigen::AngleAxisd(theta, k.normalized()).toRotationMatrix();
		return ((rot * p1) - p2).norm();
	}

	double SP1::get_theta() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta() function of SP1 was called before it was solved!\n");
		}
		return this->theta;
	}

	///
	//
	// Subproblem 2: [Two circles] 'rot(k1, theta1) * p1 = rot(k2, theta2) * p2'
	//
	///

	SP2::SP2(const Eigen::Vector3d &p1,
			 const Eigen::Vector3d &p2,
			 const Eigen::Vector3d &k1,
			 const Eigen::Vector3d &k2) : p1(p1), p2(p2), k1(k1), k2(k2)
	{
	}

	void SP2::solve()
	{
		Eigen::Vector3d norm_p1 = p1.normalized();
		Eigen::Vector3d norm_p2 = p2.normalized();

		SP4 sp4_theta_1(k2, norm_p1, k1, k2.dot(norm_p2));
		SP4 sp4_theta_2(k1, norm_p2, k2, k1.dot(norm_p1));

		sp4_theta_1.solve();
		sp4_theta_2.solve();

		_solution_is_ls = std::fabs(p1.norm() - p2.norm()) > ZERO_THRESH ||
						  sp4_theta_1.solution_is_ls() ||
						  sp4_theta_2.solution_is_ls();

		const std::vector<double> &sp_theta_1 = sp4_theta_1.get_theta();
		const std::vector<double> &sp_theta_2 = sp4_theta_2.get_theta();

		// Reverse theta2 and duplicate any angle with less solutions
		if (sp_theta_1.size() > 1 || sp_theta_2.size() > 1)
		{
			if (sp_theta_2.size() < 2) // Implies that sp_theta_1.size() >= 2
			{
				// Don't change theta 1
				theta_1 = std::vector<double>(sp_theta_1);

				// Copy angle
				theta_2.push_back(sp_theta_2.at(0));
				theta_2.push_back(sp_theta_2.at(0));
			}
			else if (sp_theta_1.size() < 2) // Implies that sp_theta_2.size() >= 2
			{
				// Copy angle
				theta_1.push_back(sp_theta_1.at(0));
				theta_1.push_back(sp_theta_1.at(0));

				// Reverse theta 2
				theta_2.push_back(sp_theta_2.at(1));
				theta_2.push_back(sp_theta_2.at(0));
			}
			else // Implies that sp_theta_1.size() >= 2 && sp_theta_2.size() >= 2
			{
				// Don't change theta 1
				theta_1 = std::vector<double>(sp_theta_1);

				// Reverse theta 2
				theta_2.push_back(sp_theta_2.at(1));
				theta_2.push_back(sp_theta_2.at(0));
			}
		}
		else
		{
			theta_1 = std::vector<double>(sp_theta_1);
			theta_2 = std::vector<double>(sp_theta_2);
		}

		is_calculated = true;
	}

	double SP2::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("Error() function of SP2 was called before it was solved!\n");
		}

		double sum = 0;
		for (unsigned i = 0; i < theta_1.size(); ++i)
		{
			sum += error(this->theta_1.at(i), this->theta_2.at(i));
		}

		return sum / theta_1.size();
	}

	double SP2::error(const double& theta_1, const double& theta_2) const
	{
		Eigen::Matrix3d rot_1 = Eigen::AngleAxisd(theta_1, k1.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_2 = Eigen::AngleAxisd(theta_2, k2.normalized()).toRotationMatrix();
		return (rot_1 * p1 - rot_2 * p2).norm();
	}

	const std::vector<double> &SP2::get_theta_1() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_1() function of SP2 was called before it was solved!\n");
		}

		return this->theta_1;
	}

	const std::vector<double> &SP2::get_theta_2() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_2() function of SP2 was called before it was solved!\n");
		}

		return this->theta_2;
	}

	///
	//
	// Subproblem 3: [Circle + Sphere] '|| rot(k, theta) * p1 - p2 || = d'
	//
	///

	SP3::SP3(const Eigen::Vector3d &p1,
			 const Eigen::Vector3d &p2,
			 const Eigen::Vector3d &k,
			 const double &d) : p1(p1), p2(p2), k(k), d(d)
	{
	}

	void SP3::solve()
	{
		const Eigen::Vector3d kxp = k.cross(p1);
		Eigen::Matrix<double, 3, 2> a_1;
		if(kxp.isZero(ZERO_THRESH) || k.cross(p2).isZero(ZERO_THRESH))
		{
			// Minimization is independent of theta -> probably redundant!
			//std::cout<<"Warning! - minimization of subproblem 3 is indipendent of theta. This may indicate redundancy"<<std::endl;
			is_calculated = true;
			theta.push_back(0); // Default angle
			_solution_is_ls = true;
			return;
		}

		a_1.col(0) = kxp;
		a_1.col(1) = -k.cross(kxp);
		const Eigen::Vector2d a = -2.0 * p2.transpose() * a_1;
		const double norm_a_sq = a.squaredNorm();
		const double norm_a = a.norm();

		const double b = d * d - (p2 - k * k.transpose() * p1).squaredNorm() - kxp.squaredNorm();

		const Eigen::Vector2d x_ls = a_1.transpose() * (-2.0 * p2 * b / norm_a_sq);

		if (x_ls.squaredNorm() > 1.0 - ZERO_THRESH || 1.0 - b * b / norm_a_sq < 0)
		{
			theta.push_back(std::atan2(x_ls.x(), x_ls.y()));
			_solution_is_ls = true;
		}
		else
		{
			const double xi = std::sqrt((1.0 - b * b / norm_a_sq));
			const Eigen::Vector2d a_perp_tilde(a.y(), -a.x());
			const Eigen::Vector2d a_perp = a_perp_tilde / norm_a;

			const Eigen::Vector2d sc_1 = x_ls + xi * a_perp;
			const Eigen::Vector2d sc_2 = x_ls - xi * a_perp;

			theta.push_back(std::atan2(sc_1.x(), sc_1.y()));
			theta.push_back(std::atan2(sc_2.x(), sc_2.y()));

			_solution_is_ls = false;
		}

		is_calculated = true;
	}

	double SP3::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("error() function of SP3 was called before it was solved!\n");
		}

		double sum = 0;
		for (const auto &t : theta)
		{
			sum += error(t);
		}
		return sum / theta.size();
	}

	double SP3::error(const double& theta) const
	{
		Eigen::Matrix3d rot = Eigen::AngleAxisd(theta, k.normalized()).toRotationMatrix();
		return std::fabs((rot * p1 - p2).norm() - d);
	}

	const std::vector<double> &SP3::get_theta() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta() function of SP3 was called before it was solved!\n");
		}
		return this->theta;
	}

	///
	//
	// Subproblem 4: [Circle + Plane] ' h' * rot(k, theta) * p = d'
	//
	///

	SP4::SP4(const Eigen::Vector3d &h,
			 const Eigen::Vector3d &p,
			 const Eigen::Vector3d &k,
			 const double &d) : h(h), p(p), k(k), d(d)
	{
	}

	void SP4::solve()
	{
		const Eigen::Vector3d a_11 = k.cross(p);
		Eigen::Matrix<double, 3, 2> a_1;
		a_1.col(0) = a_11;
		a_1.col(1) = -k.cross(a_11);
		const Eigen::Vector2d a = h.transpose() * a_1;

		const double b = d - (h.transpose() * k * k.transpose() * p).x();
		const double norm_a_sq = a.squaredNorm();
		const Eigen::Vector2d x_ls = a_1.transpose() * h * b;

		if (norm_a_sq - b * b > 0)
		{
			const double xi = std::sqrt(norm_a_sq - b * b );
			const Eigen::Vector2d a_perp_tilde(a.y(), -a.x());

			const Eigen::Vector2d sc_1 = x_ls + xi * a_perp_tilde;
			const Eigen::Vector2d sc_2 = x_ls - xi * a_perp_tilde;

			theta.push_back(std::atan2(sc_1.x(), sc_1.y()));
			theta.push_back(std::atan2(sc_2.x(), sc_2.y()));			
			_solution_is_ls = false;
		}
		else
		{
			if (x_ls.norm() < ZERO_THRESH)
			{
				theta.push_back(0);
			}
			else
			{
				theta.push_back(std::atan2(x_ls.x(), x_ls.y()));
			}

			_solution_is_ls = true;
		}

		is_calculated = true;
	}

	double SP4::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("error() function of SP4 was called before it was solved!\n");
		}

		double sum = 0;
		for (const auto &t : theta)
		{
			sum += error(t);
		}
		return sum / theta.size();
	}

	double SP4::error(const double& theta) const
	{
		Eigen::Matrix3d rot = Eigen::AngleAxisd(theta, k.normalized()).toRotationMatrix();
		return std::fabs(h.transpose() * rot * p - d);
	}

	const std::vector<double> &SP4::get_theta() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta() function of SP4 was called before it was solved!\n");
		}
		return this->theta;
	}

	///
	//
	// Subproblem 5: [Three Circles] 'p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * (p2 + rot(k3, theta3) * p3)'
	//
	///

	SP5::SP5(const Eigen::Vector3d &p0,
			 const Eigen::Vector3d &p1,
			 const Eigen::Vector3d &p2,
			 const Eigen::Vector3d &p3,
			 const Eigen::Vector3d &k1,
			 const Eigen::Vector3d &k2,
			 const Eigen::Vector3d &k3) : p0(p0), p1(p1), p2(p2), p3(p3), k1(k1), k2(k2), k3(k3)
	{
	}

	void SP5::solve()
	{
		const double EPSILON = 1e-6; // Should this be different?
		std::vector<double> theta;
		theta.reserve(8);

		const Eigen::Vector3d p1_s = p0 + k1 * k1.transpose() * p1;
		const Eigen::Vector3d p3_s = p2 + k3 * k3.transpose() * p3;

		const double delta_1 = k2.dot(p1_s);
		const double delta_3 = k2.dot(p3_s);

		const auto &[p_1, r_1] = cone_polynomials(p0, k1, p1, p1_s, k2);
		const auto &[p_3, r_3] = cone_polynomials(p2, k3, p3, p3_s, k2);

		const Eigen::Vector2d p_13 = p_1 - p_3;
		const Eigen::Vector3d p_13_sq = convolution_2(p_13, p_13);

		const Eigen::Vector3d rhs = r_3 - r_1 - p_13_sq;

		const Eigen::Matrix<double, 1, 5> eqn_real = convolution_3(rhs, rhs) - 4.0 * convolution_3(p_13_sq, r_1);

		std::vector<std::complex<double>> all_roots = quartic_roots(eqn_real.row(0).reverse());

		std::vector<double> h_vec;
		for (const auto &root : all_roots)
		{
			if (std::fabs(root.imag()) < EPSILON)
			{
				h_vec.push_back(root.real());
			}
		}

		// No analytical solution found - use all roots and decide later for valid solutions
		if (h_vec.size() == 0)
		{
			for (const auto &root : all_roots)
			{
				h_vec.push_back(root.real());
			}
			_solution_is_ls = true;
		}

		const Eigen::Vector3d kxp1 = k1.cross(p1);
		const Eigen::Vector3d kxp3 = k3.cross(p3);

		Eigen::Matrix<double, 3, 2> a_1;
		a_1.col(0) = kxp1;
		a_1.col(1) = -k1.cross(kxp1);

		Eigen::Matrix<double, 3, 2> a_3;
		a_3.col(0) = kxp3;
		a_3.col(1) = -k3.cross(kxp3);

		std::vector<std::vector<double>> signs(2);
		signs[0] = {1, 1, -1, -1};
		signs[1] = {1, -1, 1, -1};
		Eigen::Matrix2d j;
		j << 0, 1,
			-1, 0;

		// We save angle combinations w.r.t. their error in std::fabs((v1 - h * k2).norm() - (v3 - h * k2).norm())
		// If no combination reaches our threshold we can then still output the best approximate solution
		std::vector<std::tuple<double, double, double, double>> norm_errors_angles;

		for (const auto &h : h_vec)
		{
			Eigen::Vector2d const_1 = a_1.transpose() * k2 * (h - delta_1);
			Eigen::Vector2d const_3 = a_3.transpose() * k2 * (h - delta_3);

			const double hd_1 = h - delta_1;
			const double hd_3 = h - delta_3;
	
			double sq_1 = (a_1.transpose() * k2).squaredNorm() - hd_1 * hd_1;
			double sq_3 = (a_3.transpose() * k2).squaredNorm() - hd_3 * hd_3;

			const Eigen::Vector2d pm_1 = j * a_1.transpose() * k2 * std::sqrt(std::complex<double>(sq_1, 0.0)).real();
			const Eigen::Vector2d pm_3 = j * a_3.transpose() * k2 * std::sqrt(std::complex<double>(sq_3, 0.0)).real();

			for (int i_sign = 0; i_sign < signs[0].size(); ++i_sign)
			{
				const double &sign_1 = signs[0][i_sign];
				const double &sign_3 = signs[1][i_sign];

				Eigen::Vector2d sc_1 = const_1 + sign_1 * pm_1;
				sc_1 = sc_1 / (a_1.transpose() * k2).squaredNorm();

				Eigen::Vector2d sc_3 = const_3 + sign_3 * pm_3;
				sc_3 = sc_3 / (a_3.transpose() * k2).squaredNorm();

				Eigen::Vector3d v1 = a_1 * sc_1 + p1_s;
				Eigen::Vector3d v3 = a_3 * sc_3 + p3_s;

				const double norm_equation_error = std::fabs(v1.norm() - v3.norm());

				SP1 sp(v3, v1, k2);
				sp.solve();
				_solution_is_ls |= sp.solution_is_ls() | norm_equation_error > 1e-4;

				if (sc_1.norm() > ZERO_THRESH)
				{
					theta_1.push_back(std::atan2(sc_1.x(), sc_1.y()));
				}
				else
				{
					theta_1.push_back(0);
					_solution_is_ls = true;
				}

				if (sc_3.norm() > ZERO_THRESH)
				{
					theta_3.push_back(std::atan2(sc_3.x(), sc_3.y()));
				}
				else
				{
					theta_3.push_back(0);
					_solution_is_ls = true;
				}

				theta_2.push_back(sp.get_theta());
			}
		}

		//reduce_solutionset();
		is_calculated = true;
	}

	void SP5::reduce_solutionset()
	{
		// Given n >= 4 solutions, return the top 4 most unique
		if (theta_1.size() <= 4)
		{
			return;
		}

		std::vector<std::tuple<double, double, double>> common_theta;
		common_theta.reserve(theta_1.size());

		for (unsigned i = 0; i < theta_1.size(); ++i)
		{
			common_theta.push_back({theta_1.at(i), theta_2.at(i), theta_3.at(i)});
		}

		// Sort ascending first by theta 1, then 2, then 3
		const auto tuple_sort = [](const auto &a, const auto &b)
		{
			const auto &[a1, a2, a3] = a;
			const auto &[b1, b2, b3] = b;
			if (a1 < b1)
			{
				return true;
			}
			if (a1 > b1)
			{
				return false;
			}
			if (a2 < b2)
			{
				return true;
			}
			if (a2 > b2)
			{
				return false;
			}
			if (a3 < b3)
			{
				return true;
			}
			return false;
		};
		std::sort(common_theta.begin(), common_theta.end(), tuple_sort);

		double last = std::numeric_limits<double>::infinity();
		std::vector<std::pair<double, std::tuple<double, double, double>>> ordered_common_theta;
		ordered_common_theta.reserve(common_theta.size());

		for (const auto &[t1, t2, t3] : common_theta)
		{
			double delta = t1 - last;
			double ordering = 1.0 / (delta * delta);

			ordered_common_theta.push_back({ordering, {t1, t2, t3}});
			last = t1;
		}

		// Sort ascending first by theta 1, then 2, then 3
		const auto ordered_tuple_sort = [&tuple_sort](const auto &a, const auto &b)
		{
			const auto &[o1, t1] = a;
			const auto &[o2, t2] = b;
			if (o1 < o2)
			{
				return true;
			}
			if (o1 > o2)
			{
				return false;
			}
			return tuple_sort(t1, t2);
		};

		std::sort(ordered_common_theta.begin(), ordered_common_theta.end(), ordered_tuple_sort);

		theta_1.clear();
		theta_2.clear();
		theta_3.clear();

		for (unsigned i = 0; i < 4; ++i)
		{
			const auto &[_, thetas] = ordered_common_theta.at(i);
			theta_1.push_back(std::get<0>(thetas));
			theta_2.push_back(std::get<1>(thetas));
			theta_3.push_back(std::get<2>(thetas));
		}
	}

	double SP5::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("error() function of SP5 was called before it was solved!\n");
		}

		double sum = 0;

		for (unsigned i = 0; i < theta_1.size(); i++)
		{
			const double &t1 = theta_1.at(i);
			const double &t2 = theta_2.at(i);
			const double &t3 = theta_3.at(i);
			sum += error(t1, t2, t3);
		}

		return sum / theta_1.size();
	}

	double SP5::error(const double& theta_1, const double& theta_2, const double& theta_3) const
	{
		Eigen::Matrix3d rot_1 = Eigen::AngleAxisd(theta_1, k1.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_2 = Eigen::AngleAxisd(theta_2, k2.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_3 = Eigen::AngleAxisd(theta_3, k3.normalized()).toRotationMatrix();

		return (p0 + rot_1 * p1 - rot_2 * (p2 + rot_3 * p3)).norm();
	}

	const std::vector<double> &SP5::get_theta_1() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_1() function of SP5 was called before it was solved!\n");
		}
		return this->theta_1;
	}

	const std::vector<double> &SP5::get_theta_2() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_2() function of SP5 was called before it was solved!\n");
		}
		return this->theta_2;
	}

	const std::vector<double> &SP5::get_theta_3() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_3() function of SP5 was called before it was solved!\n");
		}
		return this->theta_3;
	}

	///
	//
	// Subproblem 6: [Four Circles] ' h1' * rot(k1, theta1) + h2' * rot(k2, theta2) = d1` and `h3' * rot(k3, theta1) + h4' * rot(k4, theta2) = d2 '
	//
	///

	SP6::SP6(const Eigen::Vector3d &h1,
			 const Eigen::Vector3d &h2,
			 const Eigen::Vector3d &h3,
			 const Eigen::Vector3d &h4,
			 const Eigen::Vector3d &k1,
			 const Eigen::Vector3d &k2,
			 const Eigen::Vector3d &k3,
			 const Eigen::Vector3d &k4,
			 const Eigen::Vector3d &p1,
			 const Eigen::Vector3d &p2,
			 const Eigen::Vector3d &p3,
			 const Eigen::Vector3d &p4,
			 const double &d1,
			 const double &d2) : h1(h1), h2(h2), h3(h3), h4(h4), k1(k1), k2(k2), k3(k3), k4(k4), p1(p1), p2(p2), p3(p3), p4(p4), d1(d1), d2(d2)
	{
	}

	void SP6::solve()
	{
		const Eigen::Vector3d k1xp1 = k1.cross(p1);
		const Eigen::Vector3d k2xp2 = k2.cross(p2);
		const Eigen::Vector3d k3xp3 = k3.cross(p3);
		const Eigen::Vector3d k4xp4 = k4.cross(p4);

		Eigen::Matrix<double, 3, 2> a1;
		a1.col(0) = k1xp1;
		a1.col(1) = -k1.cross(k1xp1);
		Eigen::Matrix<double, 3, 2> a2;
		a2.col(0) = k2xp2;
		a2.col(1) = -k2.cross(k2xp2);
		Eigen::Matrix<double, 3, 2> a3;
		a3.col(0) = k3xp3;
		a3.col(1) = -k3.cross(k3xp3);
		Eigen::Matrix<double, 3, 2> a4;
		a4.col(0) = k4xp4;
		a4.col(1) = -k4.cross(k4xp4);

		Eigen::Matrix<double, 2, 4> A;
		A << (h1.transpose() * a1), (h2.transpose() * a2),
			(h3.transpose() * a3), (h4.transpose() * a4);

		Eigen::Matrix<double, 4, 1> x_min;
		Eigen::Matrix<double, 2, 1> b;
		b << (d1 - h1.transpose() * k1 * k1.transpose() * p1 - h2.transpose() * k2 * k2.transpose() * p2),
			(d2 - h3.transpose() * k3 * k3.transpose() * p3 - h4.transpose() * k4 * k4.transpose() * p4);

		x_min = A.colPivHouseholderQr().solve(b);
		Eigen::CompleteOrthogonalDecomposition<
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
			cod;
		cod.compute(A);
		unsigned rk = cod.rank();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P =
			cod.colsPermutation();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V =
			cod.matrixZ().transpose();
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x_null =
			P * V.block(0, rk, V.rows(), V.cols() - rk);

		Eigen::Matrix<double, 4, 1> x_null_1 = x_null.col(0);
		Eigen::Matrix<double, 4, 1> x_null_2 = x_null.col(1);

		Eigen::Matrix<double, 2, 1> x_min_1 = x_min.block<2, 1>(0, 0);
		Eigen::Matrix<double, 2, 1> x_min_2 = x_min.block<2, 1>(2, 0);

		Eigen::Matrix<double, 2, 2> x_n_1 = x_null.block<2, 2>(0, 0);
		Eigen::Matrix<double, 2, 2> x_n_2 = x_null.block<2, 2>(2, 0);

		std::vector<std::pair<double, double>>  xi = solve_2_ellipse_numeric(x_min_1, x_n_1, x_min_2, x_n_2);

		for (const auto[xi_1, xi_2] : xi)
		{
			Eigen::Matrix<double, 4, 1> x = x_min + x_null_1 * xi_1 + x_null_2 * xi_2;

			if(x.norm() < ZERO_THRESH)
			{
				theta_1.push_back(0);
				theta_2.push_back(0);
			}
			else
			{
				theta_1.push_back(atan2(x(0, 0), x(1, 0)));
				theta_2.push_back(atan2(x(2, 0), x(3, 0)));
			}
		}

		is_calculated = true;
	}

	double SP6::error() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("Error() function of SP6 was called before it was solved!\n");
		}

		double sum = 0;
		for (unsigned i = 0; i < theta_1.size(); i++)
		{
			const double &t1 = theta_1.at(i);
			const double &t2 = theta_2.at(i);

			sum += error(t1, t2);
		}
		return sum/theta_1.size();
	}

	double SP6::error(const double& theta_1, const double& theta_2) const
	{
		Eigen::Matrix3d rot_k1_t1 = Eigen::AngleAxisd(theta_1, k1.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_k2_t2 = Eigen::AngleAxisd(theta_2, k2.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_k3_t1 = Eigen::AngleAxisd(theta_1, k3.normalized()).toRotationMatrix();
		Eigen::Matrix3d rot_k4_t2 = Eigen::AngleAxisd(theta_2, k4.normalized()).toRotationMatrix();

		const Eigen::Vector2d err_vec((h1.transpose() * rot_k1_t1 * p1 + h2.transpose() * rot_k2_t2 * p2).x() - d1,
										  (h3.transpose() * rot_k3_t1 * p3 + h4.transpose() * rot_k4_t2 * p4).x() - d2);
		return err_vec.norm();
	}

	const std::vector<double> &SP6::get_theta_1() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_1() function of SP6 was called before it was solved!\n");
		}
		return this->theta_1;
	}

	const std::vector<double> &SP6::get_theta_2() const
	{
		if (!is_calculated)
		{
			throw std::runtime_error("get_theta_2() function of SP6 was called before it was solved!\n");
		}
		return this->theta_2;
	}
}