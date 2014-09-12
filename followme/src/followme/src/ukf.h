#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

class UKF {
/*
Wan, E. A. and van der Merwe, R. (2002) The Unscented Kalman Filter, in Kalman Filtering and Neural Networks (ed S. Haykin), John Wiley & Sons, Inc., New York, USA. doi: 10.1002/0471221546.ch7

Assuming additive noise, use unaugmented version.

Model:
state t-1 = [x, y, theta, v]
state t = [x+v*cos(theta), y+v*sin(theta), theta, abs(v)] + noise_w
measurement t = [x+v*cos(theta), y+v*sin(theta)] + noise_v
*/
public:
	static const int L = 4;//dimension of state
	static const int M = 2;//dimension of measurement
	typedef Eigen::Matrix4d MatrixLd;
	typedef Eigen::Matrix2d MatrixMd;
	typedef Eigen::Array4d ArrayLd;
	typedef Eigen::Array2d ArrayMd;
	typedef Eigen::Vector4d VectorLd;
	typedef Eigen::Matrix<double, 2*L+1, 1> Vector2Ld;
private:
	double alpha;//determines the spread of the sigma points around, usually small
	double kappa;//secondary scaling parameter, usually 0
	double beta;//prior knowledge about distribution of x; 2 is optimal for gaussians
	double c;
	double lambda;//scaling parameter

	//process noise covariance
	double q;
	MatrixLd Q;

	//observation noise covariance - process noise covariance
	double r;
	MatrixMd R;

	//weights of mean and covariance
	VectorLd _w;
	Vector2Ld Wm;
	Vector2Ld Wc;

	ArrayLd x;//state
	MatrixLd P;//covariance of state

	Eigen::Array<double, L, 2*L+1> X;//sigma points
	Eigen::LLT<MatrixLd> llt;
	Eigen::SelfAdjointEigenSolver<MatrixMd> es;
public:
	UKF() {
	alpha = 1e-3;//determines the spread of the sigma points around, usually small
	kappa = 0;//secondary scaling parameter, usually 0
	beta = 2;//prior knowledge about distribution of x; 2 is optimal for gaussians
	c = alpha*alpha*(L + kappa);
	lambda = c - L;//scaling parameter

	//process noise covariance
	q = 0.1;
	Q = MatrixLd::Identity() * q * q;

	//observation noise covariance - process noise covariance
	r = 0.1;
	R = MatrixMd::Identity() * r * r - MatrixMd::Identity() * q * q;

	//weights of mean and covariance
	_w = VectorLd::Constant(0.5/c);
	Wm = (Vector2Ld() << _w, lambda/c, _w).finished();
	Wc = (Vector2Ld() << _w, lambda/c+1-alpha*alpha+beta, _w).finished();

	x = ArrayLd::Zero();//state
	P = MatrixLd::Identity();//covariance of state
	}
	void init(const ArrayLd &x_, const MatrixLd &P_) {
		x = x_;
		P = P_;
	}
	double logp(const ArrayMd &z) {
		const auto &mean = x.topRows<M>();
		const auto &covar = P.topLeftCorner<M,M>();
		const auto &zd = (z - mean).matrix();
		return -0.5*(M*log(2*M_PI)+log(covar.determinant())+zd.transpose()*covar.inverse()*zd);
	}
	double mahalanobis(const ArrayMd &z) {
		const auto &mean = x.topRows<M>();
		const auto &covar = P.topLeftCorner<M,M>();
		const auto &zd = (z - mean).matrix();
		return zd.transpose()*covar.inverse()*zd;
	}
	/* chi2inv(?, 2): 
	0.6 1.832581463748310
	0.8 3.218875824868202
	0.9 4.605170185988092
	0.95 5.991464547107981
	0.99 9.210340371976180
	0.999 13.815510557964272
	0.9999 18.420680743952587
	*/
	static const int SAMPLES = 20;
	typedef Eigen::Matrix<double, M, SAMPLES> MatrixMSd;
	MatrixMSd confidence() {
		const auto &mu = x.topRows<M>();
		const auto &sigma = P.topLeftCorner<M,M>() * 9.210340371976180;
		es.compute(sigma);
		const auto &D = es.eigenvalues();
		const auto &V = es.eigenvectors();
		Eigen::Array<double, 1, SAMPLES> t = Eigen::VectorXd::LinSpaced(SAMPLES, 0, 2*M_PI).transpose();
		Eigen::Matrix<double, M, SAMPLES> u;
		u << t.cos().matrix(), t.sin().matrix();
		const auto &w = (V * D.cwiseSqrt().asDiagonal()) * u;
		MatrixMSd z;
		z.colwise() = mu;
		z += w;
		return z;
	}
	const ArrayLd &state(void) {
		return x;
	}
	const MatrixLd &covariance(void) {
		return P;
	}
	void update(const ArrayMd &z) {
		//Cholesky decomposition of P
		MatrixLd A(llt.compute(P).matrixL());
		A *= sqrt(c);

		//make L+1+L sigma points around N(x,P)
		X.colwise() = x;
		X.leftCols<L>() += A.array();
		X.rightCols<L>() -= A.array();

		//transform sigma points
		if (isnan(z[0]))
			X.row(3) *= 0;
		X.row(0) += X.row(3)*X.row(2).cos();
		X.row(1) += X.row(3)*X.row(2).sin();
		X.row(3) = X.row(3).abs();

		//new mean and covariance
		x = X.matrix() * Wm;
		const auto &Xd = (X.colwise() -= x).matrix();
		P.noalias() = Xd*Wc.asDiagonal()*Xd.adjoint() + Q;

		//missing measurement data, skip update
		if (isnan(z[0]))
			return;

		//unscented transformation of measurements
		const auto &Zd = Xd.topRows<M>();
		const auto &S = P.topLeftCorner<M,M>() + R;
		const auto &zhat = x.head<M>();
		
		//cross-covariance
		const auto &C = Xd*Wc.asDiagonal()*Zd.adjoint();
		//Kalman gain
		const auto &K = C*S.inverse();

		//update
		x += (K*(z - zhat).matrix()).array();
		P -= C*K.adjoint();//K S K^T = (C S^-1) S K^T = C K^T
	}
};
