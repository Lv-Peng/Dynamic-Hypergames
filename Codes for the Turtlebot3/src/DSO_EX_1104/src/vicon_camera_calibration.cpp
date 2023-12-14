#include<iostream>
#include<vector>
#include<Eigen/Eigen>
#include<opencv2/core.hpp>
#include<cmath>
#include<yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;


struct CalibrationDual
{
    Vector2d ImagePoint;
    Vector2d SpacePoint;
};

vector<CalibrationDual> CalibrationDuals;
int numPoints;
double width=800,height=600,CameraScalingFactor=1000;


Matrix3Xd CameraScale(Matrix3Xd Points, int mode)
{
    switch (mode)
    {
    case 0:
        for(int i=0;i<numPoints;i++)
        {
            Points(0,i)=(Points(0,i)-width/2)/CameraScalingFactor;
            Points(1,i)=(Points(1,i)-height/2)/CameraScalingFactor;
            Points(2,i)=1;
        }
        break;
    
    case 1:
        for(int i=0;i<numPoints;i++)
        {
            Points(0,i)=Points(0,i)*CameraScalingFactor+width/2;
            Points(1,i)=Points(1,i)*CameraScalingFactor+height/2;
            Points(2,i)=1;
        }
        break;

    default:
        cerr<<"Invalid mode.(0 to Scale, 1 to Unscale)";
        break;
    }
    return Points;

}

Matrix3Xd CameraScale(Matrix2Xd Points_, int mode)
{
    int n=Points_.cols();
    Matrix3Xd Points;
    Points.block(0,0,2,n)=Points_;
    Points.row(2)=VectorXd::Ones(n).transpose();
    
    switch (mode)
    {
    case 0:
        for(int i=0;i<numPoints;i++)
        {
            Points(0,i)=(Points(0,i)-width/2)/CameraScalingFactor;
            Points(1,i)=(Points(1,i)-height/2)/CameraScalingFactor;
            Points(2,i)=1;
        }
        break;
    
    case 1:
        for(int i=0;i<numPoints;i++)
        {
            Points(0,i)=Points(0,i)*CameraScalingFactor+width/2;
            Points(1,i)=Points(1,i)*CameraScalingFactor+height/2;
            Points(2,i)=1;
        }
        break;

    default:
        cerr<<"Invalid mode.(0 to Scale, 1 to Unscale)";
        break;
    }
    return Points;

}

MatrixX3d Differential(Matrix3Xd U)
{
    int n=U.cols();
    Matrix3Xd U_=CameraScale(U,0);
    MatrixX3d diff;
    diff.resize(2*n,3);
    for(int i=0;i<n;i++)
    {
        double x=(U_(0,i)-width/2)/CameraScalingFactor,y=(U_(1,i)-height/2)/CameraScalingFactor;
        double r=sqrt(x*x+y*y);
        diff.row(2*i)=Vector3d(pow(r,2)*x,pow(r,4)*x,pow(r,6)*x).transpose();
        diff.row(2*i+1)=Vector3d(pow(r,2)*y,pow(r,4)*y,pow(r,6)*y).transpose();
    }
    return diff;
}

Matrix3Xd distort(Matrix2Xd Points_, VectorXd p, int mode)
{
    Matrix3Xd Points=CameraScale(Points_,0);
    switch (mode)
    {
    case 0:
        for(int i=0;i<numPoints;i++)
        {
            double x=Points(0,i),y=Points(1,i),r=sqrt(x*x+y*y);
            Points(0,i)=x*(1+p(0)*r*r+p(1)*r*r*r*r+p(2)*r*r*r*r*r*r);
            Points(1,i)=y*(1+p(0)*r*r+p(1)*r*r*r*r+p(2)*r*r*r*r*r*r);
        }
        Points=CameraScale(Points,1);
        break;

    case 1:
        for(int i=0;i<numPoints;i++)
        {
            double x=Points(0,i),y=Points(1,i),r=sqrt(x*x+y*y);
            Points(0,i)=x*(1-p(0)*r*r-p(1)*r*r*r*r-p(2)*r*r*r*r*r*r);
            Points(1,i)=y*(1-p(0)*r*r-p(1)*r*r*r*r-p(2)*r*r*r*r*r*r);
        }
        Points=CameraScale(Points,1);
        break;
    
    default:
        cerr<<"Invalid mode.(0 to distort, 1 to undistort)";
        break;
    }
    return Points;
}

Matrix3Xd distort(Matrix3Xd Points, VectorXd p, int mode)
{
    Points=CameraScale(Points,0);
    switch (mode)
    {
    case 0:
        for(int i=0;i<numPoints;i++)
        {
            double x=Points(0,i),y=Points(1,i),r=sqrt(x*x+y*y);
            Points(0,i)=x*(1+p(0)*r*r+p(1)*r*r*r*r+p(2)*r*r*r*r*r*r);
            Points(1,i)=y*(1+p(0)*r*r+p(1)*r*r*r*r+p(2)*r*r*r*r*r*r);
        }
        Points=CameraScale(Points,1);
        break;

    case 1:
        for(int i=0;i<numPoints;i++)
        {
            double x=Points(0,i),y=Points(1,i),r=sqrt(x*x+y*y);
            Points(0,i)=x*(1-p(0)*r*r-p(1)*r*r*r*r-p(2)*r*r*r*r*r*r);
            Points(1,i)=y*(1-p(0)*r*r-p(1)*r*r*r*r-p(2)*r*r*r*r*r*r);
        }
        Points=CameraScale(Points,1);
        break;
    
    default:
        cerr<<"Invalid mode.(0 to distort, 1 to undistort)";
        break;
    }
    return Points;
}

Matrix3d getTransformation(MatrixXd P,MatrixXd Q)
{
    Vector2d p_mean=Vector2d(P.row(0).mean(),P.row(1).mean());
    Vector2d q_mean=Vector2d(Q.row(0).mean(),Q.row(1).mean());
    Matrix2Xd p=P.block(0,0,2,numPoints),q=Q.block(0,0,2,numPoints);
    double p_scale=0,q_scale=0;
    for(int i=0;i<numPoints;i++)
    {
        p.col(i)-=p_mean;
        q.col(i)-=q_mean;
        p_scale+=p.col(i).norm();
        q_scale+=q.col(i).norm();
    }
    p_scale/=numPoints;
    q_scale/=numPoints;
    double scalingFactor=q_scale/p_scale;
    
    Matrix2d M=scalingFactor*p*q.transpose();
    JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
    Matrix2d U=svd.matrixU(),V=svd.matrixV();
    Vector2d S=svd.singularValues();
    Matrix2d S_;S_(0,0)=S(0);S_(1,1)=S(1);
    if(S_.determinant()>0)
    {
        Matrix2d R=scalingFactor*V*U.transpose();
        Vector2d t=q_mean-R*p_mean;
        Matrix3d T;
        T.block(0,0,2,2)=R;T.block(0,2,2,1)=t;T.block(2,0,1,2)=Vector2d(0,0).transpose();T(2,2)=1;
        return T;
    }
    else
        cout<<"S.det()="<<S_.determinant()<<endl;
        cerr<<"Unable to get the Transformation, plz reverse the image and retry.\n\n";
}

int main(int argc,char** argv)
{
    YAML::Node config=YAML::LoadFile("./VCCaliPoints.yaml");
    numPoints=config["num"].as<int>();
    for(int i=0;i<numPoints;i++)
    {
        vector<double> temp;
        CalibrationDual temp_dual;
        temp=config["Points"]["P"+to_string(i+1)].as<vector<double>>();
        temp_dual.ImagePoint(0)=temp[0];temp_dual.ImagePoint(1)=temp[1];
        temp_dual.SpacePoint(0)=temp[2];temp_dual.SpacePoint(1)=temp[3];
        CalibrationDuals.push_back(temp_dual);
        // cout<<"P"<<i+1<<":\nImage:"<<temp_dual.ImagePoint.transpose()<<"\nSpace:"<<temp_dual.SpacePoint.transpose()<<"\n\n";
    }

    double kk=1e-5;
    double e_past=9999,e_total,e_incre;
    int cnt=0;

    VectorXd p=VectorXd::Zero(3);
    Matrix3d M_hat;
    numPoints=CalibrationDuals.size();
    Matrix3Xd U_=Matrix3Xd::Ones(3,numPoints),U=Matrix3Xd::Ones(3,numPoints),X=Matrix3Xd::Ones(3,numPoints);
    for(int i=0;i<numPoints;i++)
    {
        U_.block(0,i,2,1)=CalibrationDuals[i].ImagePoint;
        X.block(0,i,2,1)=CalibrationDuals[i].SpacePoint;
    }

    for(int i=0;i<1e4;i++)
    {
        U=distort(U_,p,1);
        M_hat=getTransformation(U,X);
        // if(i==0)cout<<M_hat<<endl<<endl;
        Matrix3Xd X_=M_hat.inverse()*X;
        Matrix3Xd U_hat=distort(X_,p,0);
        Matrix3Xd e_hat=U_-U_hat;
        MatrixX3d diff=Differential(U_hat);
        VectorXd de;
        de.resize(2*numPoints,1);
        for(int j=0;j<U_hat.cols();j++)
        {
            de(2*j)=e_hat(0,j);
            de(2*j+1)=e_hat(1,j);
        }
        VectorXd dp=(diff.transpose()*diff).inverse()*diff.transpose()*de;
        p+=dp*kk;

        e_total=de.cwiseAbs().mean();
        // if(i<100)
        // cout<<i<<"th e:\n"<<e_total<<"\n\n";
        e_incre=e_past-e_total;
        e_past=e_total;

        if(abs(e_incre)<1e-3)cnt+=1;
        else cnt=0;

        if(cnt>=20 || e_total<=1e-6)break;

    }

    cout<<"rotationMatrix:\n"<<M_hat<<"\n\n";
    cout<<"p:\n"<<p.transpose()<<"\n\n";
    cout<<"e:\n"<<e_total<<"\n\n";

    return 0;
}