%
%
close all
clear all
clc

GenerateFunctions(true); %% Generates supporting functions
GenerateObstacleData; %% Generate points on the surface of the upper half of the table
GenerateTrajectory; %% Change gains inside the file

q=importdata('Kulkarni_Abhijeet.txt');

Drawplots(q);