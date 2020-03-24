#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

int main(){
    // Dynamic Matrix
    MatrixXd d;
    // OR
    //Matrix<double, Dynamic, Dynamic> d;

    // Fixed size Matrix
    Matrix3d f;
    f << 1, 2, 3, 4, 5, 6, 7, 8, 9; // for specific values
    f = Matrix3d::Random();  // initialize with random values

    d = MatrixXd::Random(4, 4); // Need to specify shape(4, 4) for random initialization of dynamic matrix

    // Matrix Multiplication
    Matrix<double, 3, 1> m1;
    m1 << 1, 2, 3;

    MatrixXd m2 = f * m1;


    Vector2d v1;
    v1<< 1, 2;
    cout<<v1.transpose()<< endl<<endl;
    cout<<m1<< "###"<<m2<<endl;
    cout << f.size() <<endl;   // should return 3 x 3 = 9
    cout<< f<<endl;
    cout<< d<<endl;
    cin.get();
    return 0;
}
