clear;
close all;

name1 = "cut_basic";
name2 = "cut_trans_pos";
name3 = "cut_joint_only";
name4 = "cut_rot_accel";

path1 = "/redundancy_util_";
path2 = "ex";
path3 = ".txt";

A1 = readmatrix(name1+path1+path2+path3);
A2 = readmatrix(name2+path1+path2+path3);
A3 = readmatrix(name3+path1+path2+path3);
A4 = readmatrix(name4+path1+path2+path3);

figure(1)
plot(A1(3,:),A1(2,:));
hold on
plot(A2(3,:),A2(2,:));
hold on
plot(A3(3,:),A3(2,:));
hold on
plot(A4(3,:),A4(2,:));
hold off

figure(2)
plot(A1(3,:),A1(1,:));
hold on
plot(A2(3,:),A2(1,:));
hold on
plot(A3(3,:),A3(1,:));
hold on
plot(A4(3,:),A4(1,:));