#include "aeirobot_math/spline.hpp"
#include <vector>

namespace aeirobot
{
	// Cox–de Boor recursive basis function implementation
	double Spline::BSplineBasisRecursive(int i,
										 int degree,
										 const std::vector<double> &knots,
										 double t)
	{
		if (degree == 0)
		{
			return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
		}
		double denom1 = knots[i + degree] - knots[i];
		double term1 = 0.0;
		if (denom1 > 1e-8)
		{
			term1 = (t - knots[i]) / denom1 *
					BSplineBasisRecursive(i, degree - 1, knots, t);
		}
		double denom2 = knots[i + degree + 1] - knots[i + 1];
		double term2 = 0.0;
		if (denom2 > 1e-8)
		{
			term2 = (knots[i + degree + 1] - t) / denom2 *
					BSplineBasisRecursive(i + 1, degree - 1, knots, t);
		}
		return term1 + term2;
	}

	// Evaluate clamped cubic B-spline by summing basis * control
	double Spline::EvaluateClampedCubicBSpline(const std::vector<double> &control_points,
											   double t,
											   double T)
	{
		const int degree = 3;
		int n_ctrl = static_cast<int>(control_points.size());
		int n_knots = n_ctrl + degree + 1;

		std::vector<double> knots(n_knots);
		// clamp start/end
		for (int j = 0; j <= degree; ++j)
		{
			knots[j] = 0.0;
			knots[n_knots - 1 - j] = T;
		}
		// interior uniformly spaced
		for (int j = degree + 1; j < n_ctrl; ++j)
		{
			knots[j] = T * (static_cast<double>(j - degree) / (n_ctrl - degree));
		}

		double y = 0.0;
		for (int i = 0; i < n_ctrl; ++i)
		{
			double Ni = BSplineBasisRecursive(i, degree, knots, t);
			y += Ni * control_points[i];
		}
		return y;
	}

} // namespace aeirobot
