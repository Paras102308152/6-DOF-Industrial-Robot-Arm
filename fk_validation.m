
% =========================================================================
% 6-DOF INDUSTRIAL ROBOT FORWARD KINEMATICS VALIDATION
% =========================================================================
clc; clear; close all;

% 1. Load URDF Model
urdf_file = 'RobotArm.urdf';
if ~exist(urdf_file, 'file') && exist('robotarm.urdf', 'file')
    urdf_file = 'robotarm.urdf';
end


robot = importrobot(urdf_file);
robot.DataFormat = 'column';

% Target the final end-effector link in the kinematic tree
ee_link = robot.BodyNames{end};

% 2. Define 6-DOF Joint Configurations (Degrees)
% Columns correspond to [J1, J2, J3, J4, J5, J6]
pose_matrix_deg = [
      0,  0,   0,   0,  0,  0 ;  % Home
     30, 20,  15,   0,  0,  0 ;  % Pose 1
      0, 45, -15,  30, 20, 10 ;  % Pose 2
    -30, 30,  20, -45, 15, 60    % Pose 3
];

pose_names = {'Home', 'Pose 1', 'Pose 2', 'Pose 3'};
num_poses = size(pose_matrix_deg, 1);
results = zeros(num_poses, 3); % Stores calculated [X, Y, Z]

% 3. Calculate Forward Kinematics for Each 6-Axis Pose
for i = 1:num_poses
    % Convert joint angles from degrees to radians
    q_deg = pose_matrix_deg(i, :)';
    q_rad = deg2rad(q_deg);
    
    % Populate joint configuration vector
    q_config = homeConfiguration(robot);
    n_joints = min(numel(q_config), numel(q_rad));
    q_config(1:n_joints) = q_rad(1:n_joints);
    
    % Compute End-Effector Homogeneous Transformation Matrix (T_0E)
    T = getTransform(robot, q_config, ee_link);
    
    % Extract End-Effector X, Y, Z Position (4th column of T matrix)
    results(i, :) = T(1:3, 4)';
end

% 4. Display Formatted Validation Table in Command Window
fprintf('\n======================= 6-DOF KINEMATIC VALIDATION =======================\n');
fprintf('%-8s | %-5s %-5s %-5s %-5s %-5s %-5s | %-8s %-8s %-8s\n', ...
    'Test', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X (m)', 'Y (m)', 'Z (m)');
fprintf('-------------------------------------------------------------------------\n');

for i = 1:num_poses
    fprintf('%-8s | %4.0f° %4.0f° %4.0f° %4.0f° %4.0f° %4.0f° | %8.4f %8.4f %8.4f\n', ...
        pose_names{i}, ...
        pose_matrix_deg(i,1), pose_matrix_deg(i,2), pose_matrix_deg(i,3), ...
        pose_matrix_deg(i,4), pose_matrix_deg(i,5), pose_matrix_deg(i,6), ...
        results(i,1), results(i,2), results(i,3));
end
fprintf('=========================================================================\n\n');

% 5. Plot 3D End-Effector Trajectory &amp; Poses
figure('Name', '6-DOF Workspace Poses', 'Color', 'w');
plot3(results(:,1), results(:,2), results(:,3), '-o', ...
    'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Color', [0.8500 0.3250 0.0980]);
grid on; axis equal;
xlabel('X Position (m)', 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontWeight', 'bold');
zlabel('Z Position (m)', 'FontWeight', 'bold');
title('6-DOF End-Effector Position Trajectory');

% Annotate pose names on the 3D plot
for i = 1:num_poses
    text(results(i,1) + 0.01, results(i,2) + 0.01, results(i,3) + 0.01, ...
        sprintf(' %s', pose_names{i}), 'FontSize', 9, 'FontWeight', 'bold');
end

%% % ========================================================================= % CROSS-VALIDATION: ANALYTICAL FK vs URDF MODEL % ========================================================================= clc; clear; close all; % 1\. Load URDF Model (Method B) robot = importrobot('RobotArm.urdf'); robot.DataFormat = 'column'; ee\_link = robot.BodyNames{end}; % 2\. Define Link Dimensions measured from CAD (in meters) L1 = 0.100; % Base to shoulder height L2 = 0.200; % Upper arm length L3 = 0.150; % Forearm length L4 = 0.080; % Wrist/End-effector length % 3\. Joint Test Configurations [J1, J2, J3, J4, J5, J6] (in degrees) pose\_matrix\_deg = [ 0, 0, 0, 0, 0, 0 ; % Home 30, 20, 15, 0, 0, 0 ; % Pose 1 0, 45, -15, 30, 20, 10 ; % Pose 2 -30, 30, 20, -45, 15, 60 % Pose 3 ]; pose\_names = {'Home', 'Pose 1', 'Pose 2', 'Pose 3'}; num\_poses = size(pose\_matrix\_deg, 1); fprintf('\\n================ CROSS-VALIDATION RESULTS ================\\n'); fprintf('%-8s | %-18s | %-18s | %-10s\\n', ... 'Pose', 'Analytical (m)', 'URDF Model (m)', 'Error (mm)'); fprintf('----------------------------------------------------------\\n'); for i = 1:num\_poses q\_deg = pose\_matrix\_deg(i, :); q\_rad = deg2rad(q\_deg); % --- METHOD A: Analytical FK (Simplified Trigo/Transform) --- % Compute coordinates using link lengths L1..L4 x\_ana = (L2\*cos(q\_rad(2)) + L3\*cos(q\_rad(2)+q\_rad(3)) + L4\*cos(q\_rad(2)+q\_rad(3)+q\_rad(4))) \* cos(q\_rad(1)); y\_ana = (L2\*cos(q\_rad(2)) + L3\*cos(q\_rad(2)+q\_rad(3)) + L4\*cos(q\_rad(2)+q\_rad(3)+q\_rad(4))) \* sin(q\_rad(1)); z\_ana = L1 + L2\*sin(q\_rad(2)) + L3\*sin(q\_rad(2)+q\_rad(3)) + L4\*sin(q\_rad(2)+q\_rad(3)+q\_rad(4)); P\_ana = [x\_ana, y\_ana, z\_ana]; % --- METHOD B: URDF Model via getTransform --- q\_config = homeConfiguration(robot); n = min(numel(q\_config), numel(q\_rad)); q\_config(1:n) = q\_rad(1:n); T\_urdf = getTransform(robot, q\_config, ee\_link); P\_urdf = T\_urdf(1:3, 4)'; % --- Calculate Cartesian Error --- err\_mm = norm(P\_ana - P\_urdf) \* 1000; fprintf('%-8s | [%.3f,%.3f,%.3f] | [%.3f,%.3f,%.3f] | %6.2f mm\\n', ... pose\_names{i}, P\_ana(1), P\_ana(2), P\_ana(3), ... P\_urdf(1), P\_urdf(2), P\_urdf(3), err\_mm); end fprintf('==========================================================\\n\\n');
