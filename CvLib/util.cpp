#include "util.h"
#include "cameraParam.h"

 std::vector<std::string> Util::loadDirFiles(const std::string dir_path)
{
	 std::vector<std::string> file_names;
	 for (const auto& file : std::filesystem::directory_iterator(dir_path))
	 {
	 	std::string file_name = file.path().string();
	 	std::cout << "file_name : " << file_name << " is loaded. " << std::endl;
		if (file.exists()) 
		{
			file_names.push_back(file_name);
		}
	 }

	 return file_names;
}

 //calibrationボードのdotの三次元位置を算出する関数
 std::vector<cv::Point3f> Util::create_board3dPts(int width, int height, double margin)
 {
	 std::vector<cv::Point3f> calib3dPts;
	 for (int y = 0; y < height; y++)
	 {
		 for (int x = 0; x < width; x++)
		 {
			 cv::Point3f pt = cv::Point3f(x * margin, y * margin, 0);
			 calib3dPts.push_back(pt);
		 }
	 }
	 return calib3dPts;
 }

 void Util::solvePnP(const std::vector<Eigen::Vector3d>& obj_pts, const std::vector<Eigen::Vector2d>& img_pts, const CameraParam& cparam, Eigen::Isometry3d& pose)
 {
	 const int pt_num = obj_pts.size();
	 Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * pt_num, 12);
	 for (int i = 0; i < pt_num; i++)
	 {
		 Eigen::Vector2d img_pt = img_pts[i];
		 img_pt = cparam.normalize(img_pt);

		 const double X = obj_pts[i].x();
		 const double Y = obj_pts[i].y();
		 const double Z = obj_pts[i].z();
		 const double x = img_pt.x();
		 const double y = img_pt.y();

		 A.block<2, 12>(2 * i, 0) << X, Y, Z, 1, 0, 0, 0, 0, -x * X, -x * Y, -x * Z, -x,
			 0, 0, 0, 0, X, Y, Z, 1, -y * X, -y * Y, -y * Z, -y;
	 }

	 //固有値分解の場合、固有値＞０で最小固有値に対応する固有ベクトルを選ばなければならない？
	 //Eigen::Matrix<double, 12, 12> AtA = A.transpose() * A;
	 //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(AtA);
	 //std::cout << es.eigenvalues() << std::endl;
	 //Eigen::VectorXd minEigenVec = es.eigenvectors().col(3);

	 Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	 int min_sigularIndex = 0;	//最小特異値のインデックス
	 /*最小特異ベクトルの抽出*/
	 Eigen::VectorXd sigular_valueVec = svd.singularValues();
	 for (int i = 0; i < 12; i++)
	 {
		 if (sigular_valueVec(i) == 0) {
			 min_sigularIndex = i - 1;
			 break;
		 }
		 min_sigularIndex = i;
	 }
	 Eigen::Matrix<double, 12, 1> minEigenVec = svd.matrixV().col(min_sigularIndex);	//最小特異ベクトル
	 std::cout << "minEigenVec : " << minEigenVec << std::endl;
	 Eigen::Matrix<double, 3, 4> P;
	 P.setIdentity();
	 P << minEigenVec(0), minEigenVec(1), minEigenVec(2), minEigenVec(3),
		 minEigenVec(4), minEigenVec(5), minEigenVec(6), minEigenVec(7),
		 minEigenVec(8), minEigenVec(9), minEigenVec(10), minEigenVec(11);
	 std::cout << "PP : " << P << std::endl;
	 double sq_row0P = std::sqrt(std::pow(P(0, 0), 2) + std::pow(P(0, 1), 2) + std::pow(P(0, 2), 2));
	 double sq_row1P = std::sqrt(std::pow(P(1, 0), 2) + std::pow(P(1, 1), 2) + std::pow(P(1, 2), 2));
	 double sq_row2P = std::sqrt(std::pow(P(2, 0), 2) + std::pow(P(2, 1), 2) + std::pow(P(2, 2), 2));
	 double s = std::pow(sq_row0P * sq_row1P * sq_row2P, 1 / 3.0);

	 P = P / s;

	 Eigen::Vector3d projected_pt = P * Eigen::Vector4d(obj_pts[0].x(), obj_pts[0].y(), obj_pts[0].z(), 1);

	 if (projected_pt.z() < 0)
	 {
		 P = -P;
	 }

	 Eigen::Matrix<double, 3, 3>  R = P.leftCols<3>();
	 //std::cout << "P : " << P << std::endl;
	 //std::cout << "R : " << R << std::endl;
	 Eigen::JacobiSVD<Eigen::MatrixXd> svdR(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	 Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
	 H(2, 2) = (svdR.matrixU() * svdR.matrixV().transpose()).determinant();
	 R = svdR.matrixU() * H * svdR.matrixV().transpose();

	 Eigen::Vector3d t = P.rightCols<1>();
	 Eigen::AngleAxisd rot(R);
	 pose.setIdentity();
	 pose.prerotate(R);
	 pose.pretranslate(t);
	 
	 optimizePose(obj_pts, img_pts, cparam, pose, 100, 0.0001);
 }

bool Util::optimizePose(const std::vector<Eigen::Vector3d>& obj_pts, const std::vector<Eigen::Vector2d>& img_pts, const CameraParam& cparam, Eigen::Isometry3d& pose, const int iteration_num, const int error_thresh)
 {
	 Eigen::Matrix<double, 6, 1> Q;
	 Q.setZero();
	 Eigen::AngleAxisd rot(pose.rotation());
	 Q << rot.angle() * rot.axis(), pose.translation().matrix();

	 Eigen::Matrix<double, 6, 6> JtJ;
	 Eigen::Matrix<double, 6, 1> JtE;


	 const int points_num = obj_pts.size();
	 int cnt = 0;
	 while (1)
	 {
		 JtJ.setZero();
		 JtE.setZero();
		 if (cnt++ > iteration_num)
		 {
			 break;
		 }

		 for (int i = 0; i < points_num; i++)
		 {
			 Eigen::Vector2d img_pt = cparam.normalize(img_pts[i]);

			 Eigen::Vector3d rotVec = Eigen::Vector3d(Q(0), Q(1), Q(2));
			 Eigen::AngleAxisd rot(rotVec.norm(), rotVec.normalized());
			 Eigen::Matrix3d R_mat = rot.toRotationMatrix();
			 Eigen::Vector3d transVec = Eigen::Vector3d(Q(3), Q(4), Q(5));
			 Eigen::Vector3d camPoint = R_mat * obj_pts[i] + transVec;
			 const double X = camPoint.x();
			 const double Y = camPoint.y();
			 const double Z = camPoint.z();
			 const double x = img_pt.x();
			 const double y = img_pt.y();

			 Eigen::Vector3d Pr = camPoint - transVec;
			 const double Xr = Pr.x();
			 const double Yr = Pr.y();
			 const double Zr = Pr.z();

			 Eigen::Matrix<double, 2, 3> Ja;
			 Ja << 1 / Z, 0, -X / (Z * Z),
				 0, 1 / Z, -Y / (Z * Z);
			 Eigen::Matrix<double, 3, 6> Jb;
			 Jb << 0, Zr, -Yr, 1, 0, 0,
				 -Zr, 0, Xr, 0, 1, 0,
				 Yr, -Xr, 0, 0, 0, 1;

			 Eigen::Matrix<double, 2, 6> J;
			 J = Ja * Jb;
			 JtJ += J.transpose() * J;
			 Eigen::Vector2d projected_pt(X / Z, Y / Z);
			 Eigen::Vector2d E = -(img_pt - projected_pt);
			 JtE += J.transpose() * E;
		 }

		 Eigen::FullPivLU< Eigen::Matrix<double, 6, 6>> lu(JtJ);
		 Eigen::Matrix<double, 6, 1> delta_Q;
		 delta_Q = lu.solve(JtE);
		 if (delta_Q.norm() < error_thresh) break;
		 Q -= delta_Q;
	 }

	 Eigen::Vector3d rotVec(Q(0), Q(1), Q(2));
	 Eigen::AngleAxisd _rot(rotVec.norm(), rotVec.normalized());
	 Eigen::Vector3d transVec(Q(3), Q(4), Q(5));
	 pose.setIdentity();
	 pose.prerotate(_rot);
	 pose.pretranslate(transVec);

	 return true;
 }

bool calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, CameraParam& cparam, Eigen::Isometry3d& current_pose)
{
	//3次元点と２次元点の数が異なるときを省く
	if (objPoints.size() != imgPoints.size())
	{
		return false;
	}

	const int points_num = imgPoints.size();
	Eigen::MatrixXd A(2 * points_num, 12);

	for (int i = 0; i < points_num; i++)
	{
		Eigen::Vector2d img_pt = cparam.normalize(imgPoints[i]);
		const double X = objPoints[i].x();
		const double Y = objPoints[i].y();
		const double Z = objPoints[i].z();
		const double x = img_pt.x();
		const double y = img_pt.y();

		Eigen::Matrix<double, 2, 12> a;
		a << X, Y, Z, 1, 0, 0, 0, 0, -x * X, -x * Y, -x * Z, -x,
			0, 0, 0, 0, X, Y, Z, 1, -y * X, -y * Y, -y * Z, -y;

		A.row(2 * i) = a.row(0);
		A.row(2 * i + 1) = a.row(1);
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	int min_sigularIndex = 0;	//最小特異値のインデックス
	/*最小特異ベクトルの抽出*/
	Eigen::VectorXd sigular_valueVec = svd.singularValues();
	for (int i = 0; i < 12; i++)
	{
		if (sigular_valueVec(i) == 0) {
			min_sigularIndex = i - 1;
			break;
		}
		min_sigularIndex = i;
	}
	Eigen::Matrix<double, 12, 1> min_eigenVec = svd.matrixV().col(min_sigularIndex);	//最小特異ベクトル

	Eigen::Matrix<double, 3, 4> P;	//透視投影行列
	P.setIdentity();
	P << min_eigenVec(0), min_eigenVec(1), min_eigenVec(2), min_eigenVec(3),
		min_eigenVec(4), min_eigenVec(5), min_eigenVec(6), min_eigenVec(7),
		min_eigenVec(8), min_eigenVec(9), min_eigenVec(10), min_eigenVec(11);

	/*回転行列R部分を正規化*/
	double scale = std::sqrt(P(0, 0) * P(0, 0) + P(0, 1) * P(0, 1) + P(0, 2) * P(0, 2)) *
		std::sqrt(P(1, 0) * P(1, 0) + P(1, 1) * P(1, 1) + P(1, 2) * P(1, 2)) *
		std::sqrt(P(2, 0) * P(2, 0) + P(2, 1) * P(2, 1) + P(2, 2) * P(2, 2));

	scale = std::pow(scale, 1 / 3.0);

	P = P / scale;
	Eigen::Vector4d world_point(objPoints[0].x(), objPoints[0].y(), objPoints[0].z(), 1);
	Eigen::Vector3d projected_point = P * world_point;
	/*カメラ座標のZが負の場合は符号を逆転*/
	if (projected_point.z() < 0)
	{
		P = -P;
	}

	/*回転行列を求める*/
	Eigen::Matrix<double, 3, 3> R;
	R.setIdentity();
	R = P.leftCols<3>();

	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd_R(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 3, 3> sigma;
	sigma.setIdentity();
	sigma(2, 2) = (svd_R.matrixU() * svd_R.matrixV().transpose()).determinant();
	R = svd_R.matrixU() * sigma * svd_R.matrixV().transpose();

	/*並進ベクトルを求める*/
	Eigen::Matrix<double, 3, 1> t;
	t.setZero();
	t = P.rightCols<1>();

	/*位置姿勢行列*/
	Eigen::Isometry3d pose;
	pose.setIdentity();
	pose.prerotate(R);
	pose.pretranslate(t);
	current_pose = pose;

	if (optimizePose(objPoints, imgPoints, cparam, pose, 10, 0.001))
	{
		current_pose = pose;
	}

	return true;
}

bool optimizePose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, CameraParam& cparam, Eigen::Isometry3d& pose, const int iteration_num, const double error_thresh)
{
	if (objPoints.size() != imgPoints.size())
	{
		return false;
	}

	/*非線形最適化*/
	Eigen::Matrix<double, 6, 6> JtJ;
	Eigen::Matrix<double, 6, 1> JtE;


	//6DoF姿勢パラメータ[ωx, ωy, ωz, tx, ty, tz]
	Eigen::Matrix<double, 6, 1> Q;
	Q.setZero();
	Eigen::AngleAxisd rot(pose.rotation().matrix());
	Q << Eigen::Vector3d(rot.axis() * rot.angle()), pose.translation().matrix();

	const size_t points_num = objPoints.size();
	int cnt = 0;
	while (1) {
		JtJ.setZero();
		JtE.setZero();
		if (cnt++ > iteration_num)
		{
			break;
		}

		for (int i = 0; i < points_num; i++)
		{
			Eigen::Vector3d rotVec(Q(0), Q(1), Q(2));
			Eigen::AngleAxisd rot(rotVec.norm(), rotVec.normalized());
			const Eigen::Matrix<double, 3, 3> R_Mat = rot.toRotationMatrix();
			const Eigen::Vector3d transVec(Q(3), Q(4), Q(5));

			Eigen::Matrix<double, 2, 3> Ja;
			Eigen::Matrix<double, 3, 6> Jb;
			Ja.setZero();
			Jb.setZero();
			const Eigen::Vector3d objPoint = objPoints[i];
			Eigen::Vector3d cam_Point = R_Mat * objPoint + transVec;
			const double X = cam_Point.x();
			const double Y = cam_Point.y();
			const double Z = cam_Point.z();

			Eigen::Vector3d R_objPoint;
			R_objPoint.setZero();
			R_objPoint = R_Mat * objPoint;
			const double& Xr = R_objPoint(0);
			const double& Yr = R_objPoint(1);
			const double& Zr = R_objPoint(2);

			//ヤコビアンの計算
			Ja(0, 0) = 1 / Z;
			Ja(0, 2) = -X / (Z * Z);
			Ja(1, 1) = 1 / Z;
			Ja(1, 2) = -Y / (Z * Z);

			Jb(0, 0) = 0;
			Jb(0, 1) = Zr;
			Jb(0, 2) = -Yr;
			Jb(0, 3) = 1;
			Jb(1, 0) = -Zr;
			Jb(1, 2) = Xr;
			Jb(1, 4) = 1;
			Jb(2, 0) = Yr;
			Jb(2, 1) = -Xr;
			Jb(2, 5) = 1;


			Eigen::Matrix<double, 2, 6> J;
			J = Ja * Jb;
			JtJ += J.transpose() * J;

			Eigen::Vector2d projected_point;
			projected_point.setZero();
			projected_point(0) = X / Z;
			projected_point(1) = Y / Z;

			Eigen::Vector2d E;
			E.setZero();
			//再投影誤差の計算
			Eigen::Vector2d img_pt = cparam.normalize(imgPoints[i]);
			E = img_pt - projected_point;
			E = -E;

			JtE += J.transpose() * E;
		}

		Eigen::FullPivLU< Eigen::Matrix<double, 6, 6>> lu(JtJ);
		Eigen::Matrix<double, 6, 1> delta_Q;
		delta_Q = lu.solve(JtE);
		if (delta_Q.norm() < error_thresh) break;
		Q -= delta_Q;
	}

	Eigen::Vector3d rotVec(Q(0), Q(1), Q(2));

	pose.setIdentity();
	pose.prerotate(Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()));
	pose.pretranslate(Eigen::Vector3d(Q(3), Q(4), Q(5)));

	return true;
}