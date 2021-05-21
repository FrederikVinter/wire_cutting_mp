clear;
close all;

name1 = "cut_basic";
name2 = "accelvel";
name3 = "cut_joint_only";
name4 = "cut_rot_accel";
name5 = "jointtranscost";
name6 = "accelpos";
name7 = "accelveltrans";
name8 = "jointtranscost_descartes";
name9 = "joint_only_rot_only"

names = ["j" "jp" "jpv" "jpva"];
shownnames = ["LJI p_{j}" "LJI p_{j,p}" "LJI p_{j,p,v}" "LJI p_{j,p,v,a}"];

%names = [name3 name1 name2];
%x = plot_redundancy(names, ["Joint cost" "Pose cost" "Acceleration + Velocity"],"square");
%names = [name3 name5 name9 name2 "All"];
x = plot_redundancy(names, shownnames,"saddel", "Hyperbolic Paraboloid");


%names = [name1 name2 name3];
%x = plot_redundancy(names, ["Pose" "Acceleration + velocity", "Joint only"],"Stairs4m");

%names = [name3 name5 name9];
%x = plot_redundancy(names, ["Joint cost" "Translation cost + joint cost" "Joint cost + Rotation only"],"saddel_short");

