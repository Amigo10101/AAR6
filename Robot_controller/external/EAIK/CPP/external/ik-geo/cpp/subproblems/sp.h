#ifndef _SP_H_
#define _SP_H_

#include <vector>
#include <eigen3/Eigen/Dense>

namespace IKS
{

#ifndef ZERO_THRESH
#define ZERO_THRESH 1e-12
#endif

  std::pair<Eigen::Vector2d, Eigen::Vector3d> cone_polynomials(const Eigen::Vector3d &p0_i, const Eigen::Vector3d &k_i, const Eigen::Vector3d &p_i, const Eigen::Vector3d &p_i_s, const Eigen::Vector3d &k2);
  Eigen::Matrix<double, 1, 3> convolution_2(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2);
  Eigen::Matrix<double, 1, 5> convolution_3(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
  std::vector<std::pair<double, double>> solve_2_ellipse_numeric(const Eigen::Vector2d &xm1, const Eigen::Matrix<double, 2, 2> &xn1,
                                                      const Eigen::Vector2d &xm2, const Eigen::Matrix<double, 2, 2> &xn2);
  class Subproblem
  {
  public:
    // Solves the corresponding subproblem
    virtual void solve() = 0;

    // Returns whether or not the solution is an least-squares approximation
    virtual bool solution_is_ls() const final
    {
      return this->_solution_is_ls;
    }

    // Returns the absolte least-squares error (0 if an analytical solution exists)
    virtual double error() const = 0;

  protected:
    Subproblem() = default;

    bool _solution_is_ls{false};
    bool is_calculated{false};
  };

  /// Solves for `theta` where `rot(k, theta) * p1 = p2` if possible.
  /// If not, minimizes `|| rot(k, theta) * p1 - p2 ||`.
  class SP1 final : public Subproblem
  {
  public:
    SP1(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &k);
    void solve() override;

    double error() const override;
    double error(const double& theta) const;

    double get_theta() const;

  private:
    // Inpute members
    const Eigen::Vector3d p1;
    const Eigen::Vector3d p2;
    const Eigen::Vector3d k;

    // Output members
    double theta;
  };

  // Solves for `theta1` and `theta2` where `rot(k1, theta1) * p1 = rot(k2, theta2) * p2` if possible.
  // If not, minimizes `|| rot(k1, theta1) * p1 - rot(k2, theta2) * p2 ||`.
  // There may be 1 or 2 solutions for `theta1` and `theta2`.
  class SP2 final : public Subproblem
  {
  public:
    SP2(const Eigen::Vector3d &p1,
        const Eigen::Vector3d &p2,
        const Eigen::Vector3d &k1,
        const Eigen::Vector3d &k2);
    void solve() override;

    double error() const override;
    double error(const double& theta_1, const double& theta_2) const;  

    const std::vector<double> &get_theta_1() const;
    const std::vector<double> &get_theta_2() const;

  private:
    // Input members
    const Eigen::Vector3d p1;
    const Eigen::Vector3d p2;
    const Eigen::Vector3d k1;
    const Eigen::Vector3d k2;

    // Output members
    std::vector<double> theta_1;
    std::vector<double> theta_2;
  };

  // Solves for `theta` where `|| rot(k, theta) * p1 - p2 || = d` if possible.
  // If not, minimizes `| || rot(k, theta)*p1 - p2 || - d |`.
  class SP3 final : public Subproblem
  {
  public:
    SP3(const Eigen::Vector3d &p1,
        const Eigen::Vector3d &p2,
        const Eigen::Vector3d &k,
        const double &d);
    void solve() override;

    double error() const override;
    double error(const double& theta) const;

    const std::vector<double> &get_theta() const;

  private:
    // Inpute members
    const Eigen::Vector3d p1;
    const Eigen::Vector3d p2;
    const Eigen::Vector3d k;
    const double d;

    // Output members
    std::vector<double> theta;
  };

  /// Solves for `theta` where `h' * rot(k, theta) * p = d` if possible.
  /// If not minimizes `| h' * rot(k, theta) * p - d |`.
  class SP4 final : public Subproblem
  {
  public:
    SP4(const Eigen::Vector3d &h,
        const Eigen::Vector3d &p,
        const Eigen::Vector3d &k,
        const double &d);
    void solve() override;

    double error() const override;
    double error(const double& theta) const;

    const std::vector<double> &get_theta() const;

  private:
    // Inpute members
    const Eigen::Vector3d h;
    const Eigen::Vector3d p;
    const Eigen::Vector3d k;
    const double d;

    // Output members
    std::vector<double> theta;
  };

  /// Solves for `theta1`, `theta2`, and `theta3`
  /// where `p0 + rot(k1, theta1) * p1 = rot(k2, theta2) * (p2 + rot(k3, theta3) * p3)` if possible.
  /// There can be up to 4 solutions.
  class SP5 final : public Subproblem
  {
  public:
    SP5(const Eigen::Vector3d &p0,
        const Eigen::Vector3d &p1,
        const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3,
        const Eigen::Vector3d &k1,
        const Eigen::Vector3d &k2,
        const Eigen::Vector3d &k3);
    void solve() override;

    double error() const override;
    double error(const double& theta_1, const double& theta_2, const double& theta_3) const;

    const std::vector<double> &get_theta_1() const;
    const std::vector<double> &get_theta_2() const;
    const std::vector<double> &get_theta_3() const;

  private:
    void reduce_solutionset();
    // Input members
    const Eigen::Vector3d p0;
    const Eigen::Vector3d p1;
    const Eigen::Vector3d p2;
    const Eigen::Vector3d p3;
    const Eigen::Vector3d k1;
    const Eigen::Vector3d k2;
    const Eigen::Vector3d k3;

    // Output members
    std::vector<double> theta_1;
    std::vector<double> theta_2;
    std::vector<double> theta_3;
  };

  /// Solves for `theta1` and `theta2` where `h1' * rot(k1, theta1) + h2' * rot(k2, theta2) = d1` and `h3' * rot(k3, theta1) + h4' * rot(k4, theta2) = d2`
  /// There can be up to 4 solutions
  class SP6 final : public Subproblem
  {
  public:
    SP6(const Eigen::Vector3d &h1,
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
        const double &d2);
    void solve() override;

    double error() const override;
    double error(const double& theta_1, const double& theta_2) const;

    const std::vector<double> &get_theta_1() const;
    const std::vector<double> &get_theta_2() const;

  private:
    // Input members
    const Eigen::Vector3d h1;
    const Eigen::Vector3d h2;
    const Eigen::Vector3d h3;
    const Eigen::Vector3d h4;
    const Eigen::Vector3d k1;
    const Eigen::Vector3d k2;
    const Eigen::Vector3d k3;
    const Eigen::Vector3d k4;
    const Eigen::Vector3d p1;
    const Eigen::Vector3d p2;
    const Eigen::Vector3d p3;
    const Eigen::Vector3d p4;
    const double d1;
    const double d2;

    // Output members
    std::vector<double> theta_1;
    std::vector<double> theta_2;
  };
}

#endif