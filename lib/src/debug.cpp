#include "depth_from_sequence.hpp"

void dump_camera(Camera camera){
    cout << camera.t << " " << camera.rot << endl;
}


void print_params(BundleAdjustment::Solver &s) {
    cout << "BundleAdjustment::Solver estimated"  << endl;

    cout << "points"  << endl;
    for(int j = 0; j < s.Np; ++j )
        cout << s.points[j] << endl;

    cout << endl << "camera params : [trans], [rot]" << endl;
    for(int i = 0; i < s.Nc; ++i )
        cout << s.camera_params[i].t << " " << s.camera_params[i].rot << endl;

    cout << endl;
}

void print_ittr_status(BundleAdjustment::Solver &s) {
    cout << "BundleAdjustment::Solver itteration status" << endl;
    cout << "ittr = " << s.ittr << endl;
    cout << "reprojection error = " << s.reprojection_error() << endl;
    cout << "update step size = " << s.c << endl;
    cout << "update norm = " << s.update_norm << endl;

    cout << endl;
}
