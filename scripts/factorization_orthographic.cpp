#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

inline double tensor_elem( Mat1d Ut, int i, int j, int k, int l) {

  double sum = 0.0;
  int M = Ut.cols/2;

  for( int n = 0; n < M; ++n ) {
    int k1 = 2*n, k2 = 2*n+1;
    sum += Ut.at<double>(i,k1) * Ut.at<double>(j,k1) * Ut.at<double>(k,k1) * Ut.at<double>(l,k1);
    sum += Ut.at<double>(i,k2) * Ut.at<double>(j,k2) * Ut.at<double>(k,k2) * Ut.at<double>(l,k2);
    sum += ( Ut.at<double>(i,k1) * Ut.at<double>(j,k2) + Ut.at<double>(j,k1) * Ut.at<double>(i,k2) )
      * (Ut.at<double>(k,k1) * Ut.at<double>(l,k2) + Ut.at<double>(l,k1) * Ut.at<double>(k,k2) )/ 4.0;
  }

  return sum;
}
inline double t_e( Mat1d Ut, int i, int j, int k, int l) { return tensor_elem(Ut,i,j,k,l); } //短い版

Mat1d affine_3d_basis( Mat1d data, Mat1d pc ) {
  int M = data.rows, N = data.cols;

  // アフィン空間の固有ベクトルを求める
  Mat1d U(2*M, 3);

  // モーメント行列の計算
  Mat1d C = Mat::zeros(2*M, 2*M, CV_64FC1);

  for( int i = 0; i < N; ++i ) {
    Mat1d v = data.col(i) - pc.col(0);
    Mat1d vvT = v * v.t();
    C += vvT;
  }
  cout << "C = \n" << C << "\n" << endl;
  Mat1d e_value, e_vec, e_vec_t;
  cv::eigen(C, e_value, e_vec);

  e_vec_t = e_vec.t();

  for(int i = 0; i< 3; ++i ) 
    e_vec_t.col(i).copyTo(U.col(i));

  cout << "eigen vector" << endl;
  cout << U << endl;
  cout << endl;

  return U;
}

Mat1d compute_metrix_matrix( Mat1d data, Mat1d affine_3d_basis) {
  int M = data.rows, N = data.cols;

  // 計量行列の計算
  Mat1d T;
  Mat1d Ut = affine_3d_basis.t();
  cout << "Ut\n" << Ut << endl;

  double sq2 = sqrt(2.0);
  Mat1d B = (cv::Mat_<double>(6,6) << 
	     t_e(Ut,0,0,0,0), t_e(Ut,0,0,1,1), t_e(Ut,0,0,2,2), sq2 * t_e(Ut,0,0,1,2), sq2 * t_e(Ut,0,0,2,0), sq2 * t_e(Ut,0,0,0,1),
	     t_e(Ut,1,1,0,0), t_e(Ut,1,1,1,1), t_e(Ut,1,1,2,2), sq2 * t_e(Ut,1,1,1,2), sq2 * t_e(Ut,1,1,2,0), sq2 * t_e(Ut,1,1,0,1),
	     t_e(Ut,2,2,0,0), t_e(Ut,2,2,1,1), t_e(Ut,2,2,2,2), sq2 * t_e(Ut,2,2,1,2), sq2 * t_e(Ut,2,2,2,0), sq2 * t_e(Ut,2,2,0,1),
	     sq2 * t_e(Ut,1,2,0,0), sq2 * t_e(Ut,1,2,1,1), sq2 * t_e(Ut,1,2,2,2), 2 * t_e(Ut,1,2,1,2), 2 * t_e(Ut,1,2,2,0), 2 * t_e(Ut,1,2,0,1),
	     sq2 * t_e(Ut,2,0,0,0), sq2 * t_e(Ut,2,0,1,1), sq2 * t_e(Ut,2,0,2,2), 2 * t_e(Ut,2,0,1,2), 2 * t_e(Ut,2,0,2,0), 2 * t_e(Ut,2,0,0,1),
	     sq2 * t_e(Ut,0,1,0,0), sq2 * t_e(Ut,0,1,1,1), sq2 * t_e(Ut,0,1,2,2), 2 * t_e(Ut,0,1,1,2), 2 * t_e(Ut,0,1,2,0), 2 * t_e(Ut,0,1,0,1)
	     );
  cout << "B \n" << B  << "\n|B| = " << determinant(B) << "\n" << endl;
  Mat1d c = (Mat1d(6,1) << 1,1,1,0,0,0);
  Mat1d t(6,1);
  cv::solve(B, c, t);

  return (Mat1d(3,3) <<
	  t.at<double>(0,0)    , t.at<double>(5,0)/sq2, t.at<double>(4,0)/sq2,
	  t.at<double>(5,0)/sq2, t.at<double>(1,0)    , t.at<double>(3,0)    ,
	  t.at<double>(4,0)/sq2, t.at<double>(3,0)/sq2, t.at<double>(2,0)
	  );
}

Mat1d compute_rotation_matrix(Mat1d mextrix, int i) {
  int ix = 2*i, iy = 2*i+1;
  Mat1d M = Mat::zeros(3,3, CV_64FC1);
  mextrix.col(ix).copyTo(M.col(0));
  mextrix.col(iy).copyTo(M.col(1));

  cv::SVD svd(M);
  Mat1d d = Mat::eye(3,3, CV_64FC1);
  d.at<double>(2,2) = determinant( svd.u * svd.vt );
  return svd.vt.t() * d * svd.u.t();
}

/*
  平行投影のカメラモデルにおける因子分解法
  0番目のカメラの位置姿勢を原点とする
  入力 :src => 対応点のペア。 src[カメラ][対応点] で要素を作る
  出力 :dst_R => カメラの回転行列 3x3
  出力 :dst_t => カメラの位置
  出力 :dst_pt => 各対応点の座標
  パラメータ :Zc => 平均奥行き
 */
void factorization_orthographic( vector< vector<Point2f> > src, 
				 vector<Mat1d> dst_R, vector<Point3f> dst_t, vector<Point3f> dst_pt,
				 double Zc = 1000.0)
{
    // 初期化
    int M = src.size(); // シーンの数
    int N = src[0].size(); // 特徴点の数
    Mat1d data(2*M,N);
    for ( int m = 0; m < M; ++m ) {
      for ( int n = 0; n < N; ++n ) {
	data.at<double>(2*m+0, n) = src[m][n].x;
	data.at<double>(2*m+1, n) = src[m][n].y;
      }
    }

    cout << " N = " << N << endl;
    cout << " M = " << M << endl;
    cout << "data.size() = " << data.size() << endl;

    // 軌跡ベクトルの重心を取得
    Mat1d pc = Mat::zeros(2*M, 1, CV_64FC1); //重心
    for( int n = 0; n < N; ++n ) pc += data.col(n) / (double)N;

    cout << "pc = " << pc << "\n" << endl;
    cout << "pc.col(0) = " << pc.col(0) << "\n" << endl;
    cout << "data.col(0) = " << data.col(0) << "\n" << endl;

    // tx, tyの計算
    // 消してもいいかも
    Mat1d tx(M, 1), ty(M, 1);
    for( int m = 0; m < M; ++m ){
      tx.at<double>(m,0) = pc.at<double>(2*m+0,0);
      ty.at<double>(m,0) = pc.at<double>(2*m+1,0);
    }

    Mat1d U = affine_3d_basis(data, pc);
    Mat1d T = compute_metrix_matrix(data, U);
    cout << "T = \n" << T << "\n" << endl;

    // 並進ベクトルの計算
    Mat1d trans(3,1);
    trans.at<double>(0,0) = pc.at<double>(0,0);
    trans.at<double>(1,0) = pc.at<double>(1,0);
    trans.at<double>(2,0) = 1000; // 適当な初期値

    // 回転行列を求める
    std::vector<Mat1d> R_vec;
    Mat1d Mt(3, 2*M);
    {
      // 計量行列の特異値分解を行い固有値と固有ベクトルを求める
      Mat1d v_e_value, v;
      cv::eigen(T, v_e_value, v);
      cout << "eigen values = " << v_e_value << endl;
      cout << "eigen vector = \n" << v << endl;

      // 行列M_の算出
      Mat1d M_(2*M,3);
      for( int i = 0; i < 3; ++i ) {
	for( int j = 0; j < 2*M; ++j ) {
	  Mat1d v_ = v.col(i), u_ = U.row(j);

	  printf("i, j = %d, %d :", i, j);
	  cout << u_ << " x " << v_ << endl;

	  M_.at<double>(j,i) = sqrt(v_e_value(i,0)) * v_.dot(u_.t());
	}
      }

      Mt = M_.t();
      cout << "Mt =" << Mt << endl;
    }

    for( int j = 0; j < M; ++j ) {
      Mat1d R = compute_rotation_matrix(Mt, j);
      dst_R.push_back(R);
    }

    for( int i = 0; i < M; ++i ) {
      Point3f pt( pc.at<double>(i*2+0,0), pc.at<double>(i*2+1,0) ,Zc);
      dst_t.push_back(pt);
    }

    // 形状の更新用の行列を作る
    Mat1d Metrix_updater = Mat::zeros(2*M, 3, CV_64FC1);
    for (int i = 0; i < M; ++i ) {
      Mat1d PAI = Mat::zeros(3, 2*M, CV_64FC1);
      PAI.at<double>( 0, 2*i + 0) = 1.0;
      PAI.at<double>( 1, 2*i + 1) = 1.0;
      Metrix_updater += PAI.t() * (R_vec[i]);
    }

    cout << "Metrix_updater" << endl << Metrix_updater << endl;

    // 全体を通して利用する回転行列
    Mat1d R = compute_rotation_matrix( Metrix_updater, 0);
    
    // 3次元ベクトルを計算
    for( int i = 0; i < N; ++i ) {
      Mat1d s = (Metrix_updater.t() * Metrix_updater.inv() )* Metrix_updater.t() * data.col(i);
      s = R * s - trans;
      Point3f pt(s.at<double>(0,0), s.at<double>(0,1), s.at<double>(0,2));
      dst_pt.push_back(pt);
    }

}
				 

int main(int argc, char* argv[]) {

  static int MAX_CORNERS = 200;
  cv::Size sub_pix_win_size(10, 10);
  cv::TermCriteria term_crit( cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03 );
  cv::Size img_size;
  vector<Mat> input_images;
  vector< vector<Point2f> > track_points;

  // read images
  for (int i = 1; i < argc-1; ++i )
    input_images.push_back( imread(argv[i], CV_8UC1) );
  
  img_size = input_images[0].size();  

  // initialize trackers
  {
    vector<Point2f> points;
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.001, 10, noArray(), 5, true, 0.04);
    cv::cornerSubPix(input_images[0], points, sub_pix_win_size, cv::Size(-1,-1), term_crit );
    track_points.push_back(points);
  }

  std::vector<uchar> total_status(track_points[0].size(), 1);

  // tracking sequence
  for( int i = 1; i < input_images.size(); ++i ) {
    std::vector<uchar> status;
    std::vector<float> error;
    
    std::vector<Point2f> curr_points, prev_points = track_points[i-1];
    Mat curr_image = input_images[i], prev_image = input_images[i-1];

    cv::calcOpticalFlowPyrLK( prev_image, curr_image, prev_points, curr_points, status, error);

    // masking current status
    for(int j = 0; j < status.size(); ++j )
      total_status[j] &= status[j];

    track_points.push_back(curr_points);
  }

  vector<Mat1d> camera_rot;
  vector<Point3f> camera_trans;
  vector<Point3f> points_in_world;
  factorization_orthographic( track_points, camera_rot, camera_trans, points_in_world);

  return 0;
}


