#include <memory>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/serial_container.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

// Function to create the pick and place task
moveit::task_constructor::Task createPickPlaceTask() {
    moveit::task_constructor::Task task;

    // Create Connect stage to move to pick
    auto stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>(
        "move to pick",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    // Initialize attach_object_stage
    moveit::task_constructor::Stage* attach_object_stage = nullptr;

    // Create SerialContainer for pick stages
    {
        auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Create MoveRelative stage to approach the object
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.15);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // Create GenerateGraspPose stage
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);
            grasp->insert(std::move(stage));
        }

        // Compute IK for grasp pose
        {
            // Define transformation matrix
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.1;

            // Create ComputeIK stage
            auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }

        // Allow collision between the hand and object
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("object", task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), true);
            grasp->insert(std::move(stage));
        }

        // Close the hand
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        // Lift the object
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // Add the grasp stages to the task
        task.add(std::move(grasp));
    }

    // Define the place stages similarly as above
    // ...

    return task;
}

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node node("pick_place_node");
    auto task = createPickPlaceTask();

    // More ROS setup...

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
