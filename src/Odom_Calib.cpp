#include "msf/Odom_Calib.h"
#include <iostream>
#include <fstream>

using namespace std;

//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<data_len)
    {
        // Odom(2)=1;
        // scan(2)=1;
        //TODO: 构建超定方程组
        
        A(now_len%data_len*3,0)=Odom(0);
        A(now_len%data_len*3,1)=Odom(1);
        A(now_len%data_len*3,2)=Odom(2);
        A(now_len%data_len*3+1,3)=Odom(0);
        A(now_len%data_len*3+1,4)=Odom(1);
        A(now_len%data_len*3+1,5)=Odom(2);
        A(now_len%data_len*3+2,6)=Odom(0);
        A(now_len%data_len*3+2,7)=Odom(1);
        A(now_len%data_len*3+2,8)=Odom(2);

        b(now_len%data_len*3)=scan(0);
        b(now_len%data_len*3+1)=scan(1);
        b(now_len%data_len*3+2)=scan(2);
        
        /*
        Eigen::Vector4d tmp_odom;
        tmp_odom(0) = Odom(0);
        tmp_odom(1) = Odom(1);
        tmp_odom(2) = Odom(2);
        tmp_odom(3) = 1.0; 
        A.block<1,4>(now_len*3,0)=tmp_odom.transpose();
        A.block<1,4>(now_len*3+1,4)=tmp_odom.transpose();
        A.block<1,4>(now_len*3+2,8)=tmp_odom.transpose();
        b.block<3,1>(now_len*3,0)=scan;
        */

        // A.block<1,3>(now_len*3,0)=Odom.transpose();
        // A.block<1,3>(now_len*3+1,3)=Odom.transpose();
        // A.block<1,3>(now_len*3+2,6)=Odom.transpose();
        // b.block<3,1>(now_len*3,0)=scan;
        
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}
/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}
/* 解最小二乘
 * 返回最小二乘矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    
    Eigen::Matrix3d correct_matrix;

    //TODO:求解线性最小二乘
    //correct_matrix.conservativeResize(3,3);
   


    Eigen::VectorXd correct_vector =  A.colPivHouseholderQr().solve(b);

    /*
    correct_matrix << correct_vector(0),correct_vector(1),correct_vector(2),correct_vector(3),
                      correct_vector(4),correct_vector(5),correct_vector(6),correct_vector(7),
                      correct_vector(8),correct_vector(9),correct_vector(10),correct_vector(11);
    std::cout<<"correct_matrix:"<<std::endl<<correct_matrix<<std::endl;
    */
    correct_matrix << correct_vector(0),correct_vector(1),correct_vector(2),
                      correct_vector(3),correct_vector(4),correct_vector(5),
                      correct_vector(6),correct_vector(7),correct_vector(8);

    // std::cout<<"correct_matrix:"<<std::endl<<correct_matrix<<std::endl;
    // ofstream out("/home/lie/catkin_ws/src/odom_ws/src/calib_odom/myresult.txt");
    // out<<correct_matrix<<std::endl;
    // out.close();
    // std::cout<<"write over"<<std::endl;

    return correct_matrix;
}
/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
