#include "bundle_adjustment.hpp"
#include "Eigen/LU"

inline double square(double a) { return a*a; }

double ba_reproject_x( bundleAdjustment::Solver &s, int i, int j) {
  return ( s.point_x[j] - s.cam_pose_z[i] * s.point_y[j] + s.cam_pose_y[i] ) / s.point_z[j] + s.cam_t_x[i];
}

double ba_reproject_y( bundleAdjustment::Solver &s, int i, int j) {
  return ( s.point_x[j] * s.cam_pose_z[i] + s.point_y[j] - s.cam_pose_x[i] ) / s.point_z[j] + s.cam_t_y[i];
}

double ba_reproject_z( bundleAdjustment::Solver &s, int i, int j) {
  return -s.point_x[j] * s.cam_pose_y[i] + s.cam_pose_z[i] * s.point_y[j] + 1 + s.cam_t_z[i];
}

double ba_get_reproject_gradient_x( bundleAdjustment::Solver &s, int i, int j, int k) {

  if ( k < s.Nc ) { // Txでの微分

    return ( i == k ) ? 1.0 : 0.0;

  } else if ( k < 2*s.Nc ) { // Tyでの微分

    return 0;

  } else if ( k < 3*s.Nc ) { // Tzでの微分

    return 0;

  } else if ( k < 4*s.Nc ) { // pose_xでの微分

    return 0;

  } else if ( k < 5*s.Nc ) { // pose_yでの微分

    k = k - 4*s.Nc;
    return ( i == k ) ? 1.0 / s.point_z[j] : 0.0;

  } else if ( k < 6*s.Nc ) { // pose_zでの微分

    k = k - 5*s.Nc;
    return ( i == k ) ? - s.point_y[j] / s.point_z[j] : 0.0;

  } else if ( k < 6*s.Nc + s.Np ) { // point_xでの微分

    k = k - 6*s.Nc;
    return ( j == k ) ? 1.0 / s.point_z[j] : 0.0;

  } else if ( k < 6*s.Nc + 2 * s.Np ) { // point_yでの微分

    k = k - 6*s.Nc - s.Np;
    return ( j == k ) ? - s.cam_pose_z[i] / s.point_z[j] : 0.0;

  } else { // point_zでの微分

    k = k - 6*s.Nc - 2*s.Np;
    return ( j == k ) ? ( s.point_x[j] - s.cam_pose_z[i] * s.point_y[j] + s.cam_pose_y[i] ) / ( s.point_z[j] * s.point_z[j] ) : 0.0;

  }

}

double ba_get_reproject_gradient_y( bundleAdjustment::Solver &s, int i, int j, int k) {

  if ( k < s.Nc ) { // Txでの微分

    return 0;

  } else if ( k < 2*s.Nc ) { // Tyでの微分

    k = k - s.Nc;
    return ( i == k ) ? 1.0 : 0.0;

  } else if ( k < 3*s.Nc ) { // Tzでの微分

    return 0;

  } else if ( k < 4*s.Nc ) { // pose_xでの微分

    k = k - 3*s.Nc;
    return ( i == k ) ? -1.0 / s.point_z[j] : 0.0;

  } else if ( k < 5*s.Nc ) { // pose_yでの微分

    return 0.0;

  } else if ( k < 6*s.Nc ) { // pose_zでの微分

    k = k - 5*s.Nc;
    return ( i == k ) ? s.point_x[j] / s.point_z[j] : 0.0;

  } else if ( k < 6*s.Nc + s.Np ) { // point_xでの微分

    k = k - 6*s.Nc;
    return ( j == k ) ? s.cam_pose_z[i] / s.point_z[j] : 0.0;

  } else if ( k < 6*s.Nc + 2 * s.Np ) { // point_yでの微分

    k = k - 6*s.Nc - s.Np;
    return ( j == k ) ? 1.0 / s.point_z[j] : 0.0;

  } else { // point_zでの微分

    k = k - 6*s.Nc - 2*s.Np;
    return ( j == k ) ? - ( s.cam_pose_z[i] * s.point_x[j] + s.point_y[j] - s.cam_pose_x[i] ) / (s.point_z[j] * s.point_z[j] ) : 0.0;

  }

}

double ba_get_reproject_gradient_z( bundleAdjustment::Solver &s, int i, int j, int k) {

  if ( k < s.Nc ) { // Txでの微分

    return 0;

  } else if ( k < 2*s.Nc ) { // Tyでの微分

    return 0;

  } else if ( k < 3*s.Nc ) { // Tzでの微分

    k = k - 2*s.Nc;
    return ( i == k ) ? 1.0 : 0.0;

  } else if ( k < 4*s.Nc ) { // pose_xでの微分

    k = k - 3*s.Nc;
    return ( i == k ) ? s.point_y[j] : 0.0;

  } else if ( k < 5*s.Nc ) { // pose_yでの微分

    k = k - 4*s.Nc;
    return ( i == k ) ? -s.point_x[j] : 0.0;

  } else if ( k < 6*s.Nc ) { // pose_zでの微分

    return 0;

  } else if ( k < 6*s.Nc + s.Np ) { // point_xでの微分

    k = k - 6*s.Nc;
    return ( j == k ) ? -s.cam_pose_y[i] : 0.0;

  } else if ( k < 6*s.Nc + 2 * s.Np ) { // point_yでの微分

    k = k - 6*s.Nc - s.Np;
    return ( j == k ) ? s.cam_pose_x[i] : 0.0;

  } else { // point_zでの微分

    return 0.0;

  }

}

double ba_get_gradient( bundleAdjustment::Solver &s, int k) {

  double ret = 0.0;
  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double px = s.captured_x[i][j];
      double py = s.captured_y[i][j];
      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];
      double grad_x = s.grad_reproject_x[i][j][k];
      double grad_y = s.grad_reproject_y[i][j][k];
      double grad_z = s.grad_reproject_z[i][j][k];
      
      ret += ( square(px - qx/qz) * ( qx * grad_z - qz * grad_x ) 
	       + square(py - qy/qz) * ( qy * grad_z - qz * grad_y ) 
	       ) / square(qz);
    }
  }
  return 2.0 * ret;
}

double ba_get_hessian_matrix( bundleAdjustment::Solver &s, int k, int l) {

  double ret = 0.0;
  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];
      double grad_x_k = s.grad_reproject_x[i][j][k];
      double grad_y_k = s.grad_reproject_y[i][j][k];
      double grad_z_k = s.grad_reproject_z[i][j][k];
      double grad_x_l = s.grad_reproject_x[i][j][l];
      double grad_y_l = s.grad_reproject_y[i][j][l];
      double grad_z_l = s.grad_reproject_z[i][j][l];

      ret += ( ( qx*grad_z_k - qz*grad_x_k ) * ( qx*grad_z_l - qz*grad_x_l )
	       + ( qy*grad_z_k - qz*grad_y_k ) * ( qy*grad_z_l - qz*grad_y_l )
	       ) / square(square(qz));

    }
  }
  return 4.0 * ret;
}

Eigen::VectorXd ba_get_update_for_step( bundleAdjustment::Solver &s, vector< vector<double> > hessian_matrix, vector<double> gradient_vector)
{
  Eigen::MatrixXd H(s.K, s.K);
  for(int k1 = 0; k1 < s.K; ++k1 ) {
    for(int k2 = 0; k2 < s.K; ++k2 ) {
      H(k1, k2) = hessian_matrix[k1][k2];
    }
  }

  Eigen::VectorXd grad(s.K);
  for(int k = 0; k < s.K; ++k ) {
    grad(k) = -gradient_vector[k];
  }

  cout << "|H| = " << H.determinant() << endl;

  return H.fullPivLu().solve(grad);

}
