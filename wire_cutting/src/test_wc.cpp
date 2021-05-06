#include <test_wc.h>
#include <math.h>

double runningAverage(double avg, double new_sample, int samples) {

    avg -= avg / samples;
    avg += new_sample / samples;

    return avg;
}

void evaluate_path(tesseract_common::VectorIsometry3d tool_poses, 
                   std::vector<tesseract_common::VectorIsometry3d> path)
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
    for(std::size_t i = 0; i < path.size(); i++)
    {
        auto segment_start = tool_poses[i];
        auto segment_end = tool_poses[i+1];
   
        for(std::size_t j = 0; j < path[i].size(); j++)
        {
            pose_num++;
            auto transform = segment_start.inverse()*path[i][j];
            auto translation = transform.translation();

            // Position
            const double y_rotation = transform.rotation().eulerAngles(0, 1, 2)[1];
            average_rot_cost = runningAverage(average_rot_cost, std::abs(y_rotation), pose_num);
            average_trans_cost = runningAverage(average_trans_cost, std::abs(translation.y()), pose_num);
            
            if(pose_num > 1) // Displacement
            {
                const double y_rotation_sub1 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                double disp_rot_cost = std::abs(y_rotation-y_rotation_sub1);
                average_disp_rot_cost = runningAverage(average_disp_rot_cost, disp_rot_cost, pose_num);

                double disp_trans_cost = std::abs(transform.translation().y()-transformsub1.translation().y());
                average_disp_trans_cost = runningAverage(average_disp_trans_cost, disp_trans_cost, pose_num);
            }
            if(pose_num > 2) // Change in displacement
            {
                const double y_rotation_sub1 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                const double y_rotation_sub2 = transformsub1.rotation().eulerAngles(0, 1, 2)[1];
                double acc_rot_cost = std::abs(y_rotation-2*y_rotation_sub1+y_rotation_sub2);
                average_acc_rot_cost = runningAverage(average_disp_rot_cost, acc_rot_cost, pose_num);

                double acc_trans_cost = std::abs(transform.translation().y()-2*transformsub1.translation().y()+transformsub2.translation().y());
                average_acc_trans_cost = runningAverage(average_disp_trans_cost, acc_trans_cost, pose_num);
            }
            
            transformsub1 = transform;
            transformsub2 = transformsub1;
        }

        std::cout << "Rot cost: " << average_rot_cost << " Trans cost: " << average_trans_cost << std::endl;
        std::cout << "Rot disp cost: " << average_disp_rot_cost << " Trans disp cost: " << average_disp_trans_cost << std::endl;
        std::cout << "Rot acc cost: " << average_acc_rot_cost << " Trans acc cost: " << average_acc_trans_cost << std::endl;
    }
}