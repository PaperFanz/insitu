#include <Manipulation/Manipulation.hpp>
#include <Manipulation/Manipulation_dialog.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Manipulation::Manipulation(void) : tfListener_(tfBuffer_)
{
     
}

void Manipulation::onInit(void)
{
    settingsDialog = new ManipulationDialog(this);

}

void Manipulation::onDelete(void)
{
    
}

const cv::Mat Manipulation::apply(void)
{
    /*
        Create an empty box image 
        TODO - subscribe to the image stream and overlay on box
    */
    cv::Mat ret = cv::Mat(settings.get("height", 250).asInt(),
                          settings.get("width", 250).asInt(), CV_8UC4,
                          cv::Scalar(0, 0, 0, 0)
                                     );

    //get TF from ROS buffer
    try
    {
      transformStamped_ = tfBuffer_.lookupTransform("base_footprint", "right_ur5_ee_link",
                                                    ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    quat_msg_ = transformStamped_.transform.rotation;
    geometry_msgs::Vector3 trans = transformStamped_.transform.translation;

    tf2::fromMsg(quat_msg_, quat_eigen);
    tf2::fromMsg(trans, trans_eigen);
    
    //calculate scaling for matrices
    double dist_from_base_raw =(trans_eigen(0)-0.6732);
    double distraw_to_meter= 100 * dist_from_base_raw + 45;
    double dist_to_ImagePlane_X=int((31.0/60.0)*(15-distraw_to_meter)+376);
    double dist_to_ImagePlane_Y=int((113.0/60.0)*(15-distraw_to_meter)+1172);
    double scale_percent = 100-int((dist_from_base_raw)*100);
   
   //pixel scaling, scale lines: 2500 rows, 1360 cols
    double scale_x = 1920.0 / 2500.0;
    double scale_y = 1080.0 / 1300.0;

    // imageROI = ret mat canvas dimensions, apply scaling
    //   double width = double (imageROI *scale_percent/100); 
    //   double height = double(imageROI *scale_percent/100);

    double height = 250; double width = 250;  //hard coded width = 619,height = 825 based on imageROI

    std::vector<double> dim;
    dim.push_back(width);
    dim.push_back(height);
 
    // create a matrix from dx and dy  
    // TODO change to function 
    Eigen::Matrix3d trans_matrix_3;
    trans_matrix_3 << 1, 0, -dim[0],
        0, 1, -dim[1],
        0, 0, 1;

    Eigen::Matrix3d trans_matrix_4;
    trans_matrix_4 << 1, 0, dim[0],
        0, 1, dim[1],
        0, 0, 1;

    //scale rot_vector elements and convert back to rot mat 
    Eigen::Matrix3d rot_matrix = quat_eigen.normalized().toRotationMatrix();
  
    Eigen::AngleAxisd rotation_vector2;

    rotation_vector2.fromRotationMatrix(rot_matrix);
    double rot_vector_angle = rotation_vector2.angle(); 
    Eigen::Vector3d rot_vector_axis = rotation_vector2.axis().transpose(); 
    Eigen::Vector3d rot_vector = rot_vector_angle * rot_vector_axis;
    
    double zrot = rot_vector[2];
    double yrot = rot_vector[1];

    Eigen::AngleAxisd rot_vec_z1 (-zrot/90, Eigen::Vector3d(0, 1, 0));  //about y
    Eigen::AngleAxisd rot_vec_z2 (-zrot/3000, Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd rot_vec_y1 (-yrot/90, Eigen::Vector3d(1, 0, 0));  //about x
    Eigen::AngleAxisd rot_vec_y2 (-yrot/3000, Eigen::Vector3d(1, 0, 0));  

    Eigen::Matrix3d Rz1 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rz2 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Ry1 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Ry2 = Eigen::Matrix3d::Identity();

    Rz1 = rot_vec_z1.matrix();
    Rz2 = rot_vec_z2.matrix();
    Ry1 = rot_vec_y1.matrix();
    Ry2 = rot_vec_y2.matrix();

    Eigen::Matrix3d rot_shifted = Rz2 *(trans_matrix_3); 
    Eigen::Matrix3d rot_shifted2 = -trans_matrix_4*rot_shifted; 
    Eigen::Vector3d Rz2_row1 = Rz2.row(0);
    Eigen::Vector3d Ry2_row2 = Ry2.row(1);

    /*
    TODO: change dim variable
    */

    //convert eigen mat to cv mat for warp transformation 
    Eigen::Matrix3d warp_Rz2Ry2_pos1 = rot_shifted2*trans_matrix_4*Ry2*trans_matrix_3;
    Eigen::Matrix3d warp_Rz2Ry2_neg1 = rot_shifted2* -Ry2;
    Eigen::Matrix3d warp_Rz2Ry2_neg2 = Rz2*Ry2;
    Eigen::Matrix3d warp_Rz2Ry2_pos2 = Rz2*trans_matrix_4*-Ry2*trans_matrix_3;

    cv::Mat warp_mat_Rz2Ry2_pos1; cv::Mat warp_mat_Rz2Ry2_neg1;
    cv::Mat warp_mat_Rz2Ry2_neg2, warp_mat_Rz2Ry2_pos2;

    cv::eigen2cv(warp_Rz2Ry2_pos1, warp_mat_Rz2Ry2_pos1);
    cv::eigen2cv(warp_Rz2Ry2_neg1, warp_mat_Rz2Ry2_neg1);
    cv::eigen2cv(warp_Rz2Ry2_neg2, warp_mat_Rz2Ry2_neg2);
    cv::eigen2cv(warp_Rz2Ry2_pos2, warp_mat_Rz2Ry2_pos2);

    Size dim_size = Size2d(width,height);

    if (Rz2_row1 [2] > 0){
        if (Ry2_row2 [2] > 0){
            warpPerspective(ret,ret,warp_mat_Rz2Ry2_pos1,dim_size);
        }
        if (Ry2_row2 [2] < 0){
            warpPerspective(ret,ret,warp_mat_Rz2Ry2_neg1,dim_size);
        }

    }
    if (Rz2_row1 [2] < 0){
        if (Ry2_row2 [2] < 0){
            warpPerspective(ret,ret,warp_mat_Rz2Ry2_neg2,dim_size);
        }
        if (Ry2_row2 [2] > 0){
            warpPerspective(ret,ret,warp_mat_Rz2Ry2_pos2,dim_size);
        }
    }

 //TODO change rotation boundary during rotation
    return ret;
}

}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Manipulation, insitu::Filter);
