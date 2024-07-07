/**
 * @file tm_kinematics.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the kinematics of TM5 robotic arm.
 *      Specifically, the program provides the forward and inverse
 *      kinematics of the TM5-700, TM5-900, TM5-1100, and TM5-1300
 *      robotic arms.
 * @note `tm_kinematics.cpp` is part of `marslite_control`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 *  with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>.
 */

#include "tm_kinematics/tm_kinematics.h"

namespace marslite {

namespace control {

TMKinematics::TMKinematics(const RobotType& robotType) : robotType_(robotType)
{
	switch (robotType_) {
	case TM5_700:
		d1_ =  0.1451;
		a2_ =  0.3290;
		a3_ =  0.3115;
		d4_ = -0.1222;
		d5_ =  0.1060;
		d6_ =  0.1144;
		break;
	case TM5_900:
		d1_ =  0.1451;
		a2_ =  0.4290;
		a3_ =  0.4115;
		d4_ = -0.1222;
		d5_ =  0.1060;
		d6_ =  0.1144;
		break;
	case TM5_1100:
		d1_ =  0.1652;
		a2_ =  0.5361;
		a3_ =  0.4579;
		d4_ = -0.1563;
		d5_ =  0.1060;
		d6_ =  0.11315;
		break;
	case TM5_1300:
		d1_ =  0.1652;
		a2_ =  0.6361;
		a3_ =  0.5579;
		d4_ = -0.1563;
		d5_ =  0.1060;
		d6_ =  0.11315;
	default:
		ROS_ERROR_STREAM("Invalid robot type: " << robotType_ << ". Aborting...");
		throw ConstructorInitializationFailedException();
		break;
	}
}

Eigen::MatrixXd TMKinematics::inverseKinematics(const TransformationMatrix& T) const
{
	uint8_t numSolution = 0;
	double solutions[8][6];

	// Solve inverse kinematics
	Eigen::MatrixXd Q1, Q56P, Q234;
	Q1 = solveQ1(T);
	for (uint8_t i = 0; i < Q1.rows(); ++i) {
		Q56P = solveQ56P(T, Q1(i));
		for (uint8_t j = 0; j < Q56P.rows(); ++j) {
			Q234 = solveQ234(T, Q1(i), Q56P(j, 2));
			for (uint8_t k = 0; k < Q234.rows(); ++k) {
				solutions[numSolution][0] = Q1(i);
				solutions[numSolution][1] = Q234(k, 0);
				solutions[numSolution][2] = Q234(k, 1);
				solutions[numSolution][3] = Q234(k, 2);
				solutions[numSolution][4] = Q56P(j, 0);
				solutions[numSolution][5] = Q56P(j, 1);
				++numSolution;
			}
		}
	}

	// Convert solutions to Eigen::MatrixXd
	Eigen::MatrixXd Q(numSolution, 6);
	for (uint8_t i = 0; i < numSolution; ++i) {
		for (uint8_t j = 0; j < 6; ++j) {
			Q(i, j) = solutions[i][j];
		}
	}

	return Q;
}

Eigen::MatrixXd TMKinematics::solveQ1(const TransformationMatrix& T) const
{
	Eigen::MatrixXd Q1;

	const double ax = T(0, 2), ay = T(1, 2);
	const double px = T(0, 3), py = T(1, 3);
	const double px_d6ax = px - d6_ * ax;
	const double py_d6ay = py - d6_ * ay;
	const double r = sqrt(px_d6ax * px_d6ax + py_d6ay * py_d6ay);

	if (r >= -d4_) {
		// [Two solutions]
		Q1.resize(2,1);
		Q1(0) = atan2( py_d6ay,  px_d6ax) + atan2(-d4_, sqrt(r * r - d4_ * d4_));
		Q1(1) = atan2(-py_d6ay, -px_d6ax) - atan2(-d4_, sqrt(r * r - d4_ * d4_));

		// Restrict Q1 to [-pi, pi]
		Q1(0) = restrictAngleWithinPI(Q1(0));
		Q1(1) = restrictAngleWithinPI(Q1(1));
	}

	return Q1;
}

Eigen::MatrixXd TMKinematics::solveQ56P(const TransformationMatrix& T,
										const double& Q1,
										const double& Q6ref) const
{
	Eigen::MatrixXd Q56P(1, 3);

	// T26 = A2 * A3 * A4 * A5 * A6 = inv(A1) * T
	const TransformationMatrix T26 = getTransformationMatrixInverse(A1(Q1)) * T;

	// Solve Q5 based on the value of sin(Q5)
	if (this->distanceCloseToTarget(-T26(2, 2), -1)) {
		// [One solution]
		// Q5 = acos(-1) = pi, S5 = 0
		Q56P(0, 0) = M_PI;
	} else if (this->distanceCloseToTarget(-T26(2, 2), 1)) {
		// [One solution]
		// Q5 = acos(1) = 0, S5 = 0
		Q56P(0, 0) = 0;
	} else {
		// [Two solutions]
		Q56P.resize(2, 3);
		Q56P(0, 0) =  acos(-T26(2, 2));
		Q56P(1, 0) = -acos(-T26(2, 2));
	}

	// Solve Q6 and QP based on the number of valid solutions
	if (Q56P.rows() == 2) {
		// Solve Q6
		Q56P(0, 1) = atan2(-T26(2, 1),  T26(2, 0));
		Q56P(1, 1) = atan2( T26(2, 1), -T26(2, 0));

		// Solve QP
		Q56P(0, 2) = atan2( T26(1, 2),  T26(0, 2));
		Q56P(1, 2) = atan2(-T26(1, 2), -T26(0, 2));
	} else if (Q56P.rows() == 1) {
		// Choose Q6 as Q6ref
		Q56P(0, 1) = Q6ref;

		// Solve QP
		Q56P(0, 2) = Q56P(0, 1) + atan2(T26(1, 0), T26(0, 0));
	}

	return Q56P;
}

Eigen::MatrixXd TMKinematics::solveQ234(const TransformationMatrix& T,
                              			const double& Q1,
                              			const double& QP) const
{
	const double ax = T(0, 2), ay = T(1, 2), az = T(2, 2);
	const double px = T(0, 3), py = T(1, 3), pz = T(2, 3);

	const double A = cos(Q1) * (px - d6_ * ax) + sin(Q1) * (py - d6_ * ay) - d5_ * sin(QP);
	const double B = -pz + d1_ + d6_ * az + d5_ * cos(QP);

	const double r = sqrt(A * A + B * B);
	const double s = a2_ + a3_ - r;
	const double t = r - std::abs(a2_ - a3_);

	Eigen::MatrixXd Q234;
	if (s > kDistanceTolerance_ && t > kDistanceTolerance_) {
		// [Two solutions]
		Q234.resize(2, 3);
		
		// Solve Q3
		Q234(0, 1) = acos((r * r - a2_ * a2_ - a3_ * a3_) / (2 * a2_ * a3_));
		Q234(1, 1) = -Q234(0, 1);

		// Solve Q2
		Q234(0, 0) = atan2(B, A) - acos((r * r + a2_ * a2_ - a3_ * a3_) / (2 * r * a2_));
		Q234(1, 0) = atan2(B, A) + acos((r * r + a2_ * a2_ - a3_ * a3_) / (2 * r * a2_));
	} else if (distanceCloseToTarget(s, 0) || distanceCloseToTarget(t, 0)) {
		// [One solution]
		// s ~= 0, C3 =  1, Q3 = 0
		// t ~= 0, C3 = -1, Q3 = pi
		Q234.resize(1, 3);

		// Solve Q2
		Q234(0, 0) = atan2(B, A);

		// Solve Q3
		if (distanceCloseToTarget(s, 0))
			Q234(0, 1) = 0;
		else if (distanceCloseToTarget(t, 0))
			Q234(0, 1) = M_PI;

	}

	for (int i = 0; i < Q234.rows(); ++i) {
		// Add offset to Q2, and restrict Q2 to [-pi, pi]
		Q234(i, 0) = restrictAngleWithinPI(Q234(i, 0) + M_PI/2);

		// Solve Q4, and restrict Q4 to [-pi, pi]
		Q234(i, 2) = QP - Q234(i, 0) - Q234(i, 1);
		Q234(i, 2) = restrictAngleWithinPI(Q234(i, 2));
	}

	return Q234;
}

} // namespace control

} // namespace marslite