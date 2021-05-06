#include <test_wc.h>
#include <math.h>

double runningAverage(double avg, double new_sample, int samples) {

    avg -= avg / samples;
    avg += new_sample / samples;

    return avg;
}

void evaluate_path(tesseract_common::VectorIsometry3d tool_poses, 
                   std::vector<std::vector<Eigen::Isometry3d>> path)
{
    assert(tool_poses.size() == path.size()-1);

    double average_rot_cost = 0;
    double average_trans_cost = 0;

    double average_disp_rot_cost = 0;
    double average_disp_trans_cost = 0;

    double average_acc_rot_cost = 0;
    double average_acc_trans_cost = 0;

    int pose_num = 0;
    Eigen::Isometry3d transformsub1, transformsub2;
    double length = 0, lengthsub1;
    double disp_rot_cost_sub1, disp_trans_cost_sub1, disp_rot_cost, disp_trans_cost;
    double y_translation_sub1;
    for(std::size_t i = 0; i < path.size(); i++)
    {
        auto segment_start = tool_poses[i];
        auto segment_end = tool_poses[i+1];

        auto end_transform = segment_start.inverse()*segment_end;
        double segment_y_trans = end_transform.translation().y();
        double segment_length = std::sqrt(std::pow(end_transform.translation().x(),2.0)
                                +std::pow(end_transform.translation().z(),2.0));

        double length_acum;
        for(std::size_t j = 0; j < path[i].size(); j++)
        {
            pose_num++;
            auto transform = segment_start.inverse()*path[i][j];
            auto translation = transform.translation();
           
            // Length
            if(pose_num > 1)
            {
                length = std::sqrt(std::pow(transformsub1.translation().x()-transform.translation().x(),2.0)
                    +std::pow(transformsub1.translation().z()-transform.translation().z(),2.0));
                length_acum+=length;
            }
            
            // Position
            double y_rotation = transform.rotation().eulerAngles(0, 1, 2)[1];
            average_rot_cost = runningAverage(average_rot_cost, std::abs(y_rotation), pose_num);
            double y_translation = translation.y()-segment_y_trans*(length_acum/segment_length);
            std::cout << "Y translation: " << y_translation << " Y trans: " << translation.y() << " Y seg: " << segment_y_trans 
            << " length: "<< length << " seg length: " << segment_length << std::endl;
            average_trans_cost = runningAverage(average_trans_cost, std::abs(y_translation), pose_num);

            if(pose_num > 1) // Displacement
            {
                const double y_rotation_sub1 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                disp_rot_cost = y_rotation_sub1-y_rotation;
                average_disp_rot_cost = runningAverage(average_disp_rot_cost, std::abs(disp_rot_cost/length), pose_num-1);

                disp_trans_cost = y_translation_sub1-y_translation; 
                average_disp_trans_cost = runningAverage(average_disp_trans_cost, std::abs(disp_trans_cost/length), pose_num-1);
            }
            if(pose_num > 2) // Change in displacement
            {
                const double y_rotation_sub1 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                const double y_rotation_sub2 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                double acc_rot_cost = std::abs(disp_rot_cost_sub1-disp_rot_cost);

                double average_length = (length+lengthsub1)/2.0;
                average_acc_rot_cost = runningAverage(average_disp_rot_cost, acc_rot_cost/average_length, pose_num-2);

                double acc_trans_cost = std::abs(disp_trans_cost_sub1-disp_trans_cost);
                average_acc_trans_cost = runningAverage(average_disp_trans_cost, acc_trans_cost/average_length, pose_num-2);
            }
            
            y_translation_sub1 = y_translation;
            lengthsub1 = length;
            disp_rot_cost_sub1 = disp_rot_cost;
            disp_trans_cost_sub1 = disp_trans_cost;
            transformsub1 = transform;
            transformsub2 = transformsub1;
        }

        std::cout << "Rot cost: " << average_rot_cost << " Trans cost: " << average_trans_cost << std::endl;
        std::cout << "Rot disp cost: " << average_disp_rot_cost << " Trans disp cost: " << average_disp_trans_cost << std::endl;
        std::cout << "Rot acc cost: " << average_acc_rot_cost << " Trans acc cost: " << average_acc_trans_cost << std::endl;
    }
}