%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        3D RRT SEARCH ALGORITHM FOR MOTION PLANNING OF ROBOT           %
%                       BY SWAPNEEL MEHTA                               %
%                  ADVISOR: Dr. MINGHUI ZHENG                           %
%                          CAL@UB                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;

% Boundary for the map or reach space of robot 
x_max = 600;
y_max = 600;
z_max = 600;
% Plotting to see the way it converges and finds an optimal path for robot
figure()
axis([0 x_max 0 y_max 0 z_max])
title('Visual Simulation of Algorithm finding the path')
xlabel('X')
ylabel('Y')
zlabel('Z')

hold on

compare = 40; % The Value used to restrict the movement within certain
Max_num_Nodes = 1600; 

q_start.config = [0 0 0]; % initial position of the robot
q_start.cost = 0; % Cost to reach to node
q_start.parent = 0; % parent is node through which it passed 
q_goal.config = [600 500 100]; % Desired position
q_goal.cost = 0; % Initialization of cost
nodes(1) = q_start; % Defining the first node as starting position

 plot3(q_start.config(1), q_start.config(2), q_start.config(3), 'o',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','b',...
    'MarkerSize',14);

 plot3(q_goal.config(1), q_goal.config(2), q_goal.config(3), 'o',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','r',...
    'MarkerSize',14);
  
% OBSTACLES DEFINED HERE
% Adding new obstacles and modifying any existing obstacles is pretty
% simple and fast, by just changing parameters of any obstacle
p_obstacle=[150,50,0];
size_obstacle=[120,100,200];
p_obstacle2=[350,350,0];
size_obstacle2=[100,100,180];
obstacle(size_obstacle,p_obstacle,1,[0 0 1]);
hold on
obstacle(size_obstacle2,p_obstacle2,1,[0 0 1]);
hold on
R0=40;x0=200;y0=400;z0=0;h1=180;
R02=30;x02=500;y02=150;z02=0;h2=190;
R03=50;x03=500;y03=300;z03=0;h3=160;
R04=40;x04=350;y04=250;z04=0;h4=190;
cyl_obs(x0,y0,z0,R0,h1)
cyl_obs(x02,y02,z02,R02,h2)
cyl_obs(x03,y03,z03,R03,h3)
cyl_obs(x04,y04,z04,R04,h4)

% Providing data of each of the obstacles in a matrix form to use later 
ob(1,1:6)=[p_obstacle(1), p_obstacle(2),p_obstacle(3), size_obstacle(1), size_obstacle(2), size_obstacle(3)];
ob(2,1:6)=[x0-R0, y0-R0,z0-R0 2*R0, 2*R0, h1];
ob(3,1:6)=[x02-R02, y02-R02, z02-R02 2*R02, 2*R02, h2];
ob(4,1:6)=[x03-R03, y03-R03, z03-R03, 2*R03, 2*R03, h3];
ob(5,1:6)=[p_obstacle2(1), p_obstacle2(2), p_obstacle2(3), size_obstacle2(1), size_obstacle2(2), size_obstacle2(3)];
ob(6,1:6)=[x04-R04, y04-R04, z04-R04, 2*R04, 2*R04, h4];
tic
% {RRT Algoithm starts here, the loop goes on until the number of nodes
% has not reached to maximum allowed by input}
for i = 1:1:Max_num_Nodes
    q_rand = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410]);
    hold on
    legend({'Start Node','Goal Node'},'Location','northeast')
   % {A condition, using break command, to exit from the loop if the robot has reached to 
   % goal node and met the requirements}  
    for j = 1:1:length(nodes)
        if nodes(j).config == q_goal.config
            break
        end
    end
    % {Comparing distance between current nodes and new nodes to find
    % nearest node out of a group of random nodes}
    n_dist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        dist_tmp = euc_dist_3d(n.config, q_rand);
        n_dist = [n_dist dist_tmp];
    end
    [val, idx] = min(n_dist);
    q_near = nodes(idx);
    q_new.config = move(q_rand, q_near.config, val, compare);
    % {The following 'if' Condition checks whether the edge found collides
    % with any obstacle using the function provided at the end} 
    if ob_avoidance(q_rand, q_near.config, ob(1,:)) && ob_avoidance(q_rand, q_near.config, ob(2,:)) && ob_avoidance(q_rand, q_near.config, ob(3,:)) && ob_avoidance(q_rand, q_near.config, ob(4,:)) && ob_avoidance(q_rand, q_near.config, ob(5,:)) && ob_avoidance(q_rand, q_near.config, ob(6,:))
        line([q_near.config(1), q_new.config(1)], [q_near.config(2), q_new.config(2)], [q_near.config(3), q_new.config(3)], 'Color', 'm', 'LineWidth', 2);
        drawnow; % This command shows actual plotting of convergence 
        hold on
        q_new.cost = euc_dist_3d(q_new.config, q_near.config) + q_near.cost;
        q_nearest = [];
        r = 50;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if (euc_dist_3d(nodes(j).config, q_new.config)) <= r && ob_avoidance(q_rand, q_near.config, ob(1,:)) && ob_avoidance(q_rand, q_near.config, ob(2,:)) && ob_avoidance(q_rand, q_near.config, ob(3,:)) && ob_avoidance(q_rand, q_near.config, ob(4,:)) && ob_avoidance(q_rand, q_near.config, ob(5,:)) && ob_avoidance(q_rand, q_near.config, ob(6,:)) 
                q_nearest(neighbor_count).config = nodes(j).config;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        % Comparing the costs to travel through different nodes to nearest
        % node, and finally selecting the least cost node
        q_min = q_near;
        C_min = q_new.cost;
        for k = 1:1:length(q_nearest)
            if q_nearest(k).cost + euc_dist_3d(q_nearest(k).config, q_new.config) < C_min && ob_avoidance(q_rand, q_near.config, ob(1,:)) && ob_avoidance(q_rand, q_near.config, ob(2,:)) && ob_avoidance(q_rand, q_near.config, ob(3,:)) && ob_avoidance(q_rand, q_near.config, ob(4,:)) && ob_avoidance(q_rand, q_near.config, ob(5,:)) && ob_avoidance(q_rand, q_near.config, ob(6,:))
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + euc_dist_3d(q_nearest(k).config, q_new.config);
                line([q_min.config(1), q_new.config(1)], [q_min.config(2), q_new.config(2)], [q_min.config(3), q_new.config(3)], 'Color', 'b');
                hold on
            end
        end
        
        for j = 1:1:length(nodes)
            if nodes(j).config == q_min.config
                q_new.parent = j;
            end
        end
        nodes = [nodes q_new];
    end
end
toc
% Finding the shortest destance path by forming a vector with distances 
D = [];
for j = 1:1:length(nodes)
    tmp_dist = euc_dist_3d(nodes(j).config, q_goal.config);
    D = [D tmp_dist];
end

% Comparing the elements of vector formed above and finding the minimum
% distance out of it to get final path
% If in case, the last node selected is not goal node, we replace it and
% add at the end, goal node, in nodes list
[val, idx] = min(D);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
% {The following loop helps in connecting the path from last node to goal
% node if, in case maximum number of nodes are not sufficient to reach}
while q_end.parent ~= 0
    start = q_end.parent;
    data1=line([q_end.config(1), nodes(start).config(1)], [q_end.config(2), nodes(start).config(2)], [q_end.config(3), nodes(start).config(3)], 'Color', 'g', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
end
  label(data1,'Final Path')
% {This is the most important function in the code used to avoid all the
% obstacles if present in the new direction found by nearest node strategy}  

function nc = ob_avoidance(n2, n1, ob_data)

A = [n1(1) n1(2) n1(3)]; % Node from where edge is to be formed
B = [n2(1) n2(2) n2(3)]; % Node to be reached from previous one
obs = [ob_data(1) ob_data(2) ob_data(3) ob_data(1)+ob_data(4) ob_data(2)+ob_data(5) ob_data(3)+ob_data(6)];

C1 = [obs(1),obs(2)];
D1 = [obs(1),obs(5)];
C2 = [obs(1),obs(2)];
D2 = [obs(4),obs(2)];
C3 = [obs(4),obs(5)];
D3 = [obs(4),obs(2)];
C4 = [obs(4),obs(5)];
D4 = [obs(1),obs(5)];
C5 = [obs(1) obs(3)];
D5 = [obs(1) obs(6)];
C6 = [obs(1) obs(3)];
D6 = [obs(4) obs(3)];
C7 = [obs(4) obs(6)];
D7 = [obs(4) obs(3)];
C8 = [obs(4) obs(6)];
D8 = [obs(1) obs(6)];
C9 = [obs(2) obs(3)];
D9 = [obs(2) obs(6)];
C10 = [obs(2) obs(3)];
D10 = [obs(5) obs(3)];
C11 = [obs(5) obs(6)];
D11 = [obs(5) obs(3)];
C12 = [obs(5) obs(6)];
D12 = [obs(2) obs(6)];

% Checking intersection using geometry rules for each edge and sides
val_con_1 = condition(A,C1,D1) ~= condition(B,C1,D1) && condition(A,B,C1) ~= condition(A,B,D1);
val_con_2 = condition(A,C2,D2) ~= condition(B,C2,D2) && condition(A,B,C2) ~= condition(A,B,D2);
val_con_3 = condition(A,C3,D3) ~= condition(B,C3,D3) && condition(A,B,C3) ~= condition(A,B,D3);
val_con_4 = condition(A,C4,D4) ~= condition(B,C4,D4) && condition(A,B,C4) ~= condition(A,B,D4);
val_con_5 = condition(A,C5,D5) ~= condition(B,C5,D5) && condition(A,B,C5) ~= condition(A,B,D5);
val_con_6 = condition(A,C6,D6) ~= condition(B,C6,D6) && condition(A,B,C6) ~= condition(A,B,D6);
val_con_7 = condition(A,C7,D7) ~= condition(B,C7,D7) && condition(A,B,C7) ~= condition(A,B,D7);
val_con_8 = condition(A,C8,D8) ~= condition(B,C8,D8) && condition(A,B,C8) ~= condition(A,B,D8);
val_con_9 = condition(A,C9,D9) ~= condition(B,C9,D9) && condition(A,B,C9) ~= condition(A,B,D9);
val_con_10 = condition(A,C10,D10) ~= condition(B,C10,D10) && condition(A,B,C10) ~= condition(A,B,D10);
val_con_11 = condition(A,C11,D11) ~= condition(B,C11,D11) && condition(A,B,C11) ~= condition(A,B,D11);
val_con_12 = condition(A,C12,D12) ~= condition(B,C12,D12) && condition(A,B,C12) ~= condition(A,B,D12);

% It only allows the movement which is safe without any collision
if val_con_1==0 && val_con_2==0 && val_con_3==0 && val_con_4==0 && val_con_5==0 && val_con_6==0 && val_con_7==0 && val_con_8==0 && val_con_9==0 && val_con_10==0 && val_con_11==0 && val_con_12==0
    nc = 1;
else
    nc = 0;
end
end

% {This function acts as a condition to check binary value of any possible
% intersection, using geometry of comparing distance between coordinates}
% {The value obtained by this function is sent to the function ob_avoidance
% to compare and decide whether the obstacle is interfering the new edge} 
function val = condition(A,B,C)
val = (C(2)-A(2)) * (B(1)-A(1)) > (B(2)-A(2)) * (C(1)-A(1));
end

% Simple function to calculate euclidean distance between two given nodes
function d = euc_dist_3d(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2);
end

%{ This function uses coordinates, factor of value and epsilon criteria
% to move the branch further in the direction of new node selected by
% algorithm. If distance is very large, it will form a new node in the same
% direction as the nearest node selected to move before}
function A = move(qr, qn, val, eps)
q_new = [0 0];
if val >= eps
    q_new(1) = qn(1) + ((qr(1)-qn(1))*eps)/euc_dist_3d(qr,qn);
    q_new(2) = qn(2) + ((qr(2)-qn(2))*eps)/euc_dist_3d(qr,qn);
    q_new(3) = qn(3) + ((qr(3)-qn(3))*eps)/euc_dist_3d(qr,qn);
else
    q_new(1) = qr(1);
    q_new(2) = qr(2);
    q_new(3) = qr(3);
end
A = [q_new(1), q_new(2), q_new(3)];
end

%{Function used to create graphical image of a cuboid of any size
% It takes coordinates of one corner and side lengths in x,y and z
% direction as input and produces image as output}
function obstacle(varargin)
inArgs = { ...
    [10 56 100] , ...
    [10 10  10] , ...
    .7          , ...
    [1 0 0]       ...
    };
inArgs(1:nargin) = varargin;
[edges,origin,alpha,clr] = deal(inArgs{:});
XYZ = { ...
    [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ...
    [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
    [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
    [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
    [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
    [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
    };
XYZ = mat2cell(...
    cellfun( @(x,y,z) x*y+z , ...
    XYZ , ...
    repmat(mat2cell(edges,1,[1 1 1]),6,1) , ...
    repmat(mat2cell(origin,1,[1 1 1]),6,1) , ...
    'UniformOutput',false), ...
    6,[1 1 1]);
cellfun(@patch,XYZ{1},XYZ{2},XYZ{3},...
    repmat({clr},6,1),...
    repmat({'FaceAlpha'},6,1),...
    repmat({alpha},6,1)...
    );
view(3);
end

% {Function used to create the cylinder shaped obstacle by taking
% coordinates of center(x,y,z), Base-Radius(R) and cylinder height(h)
% as input and output is the plot in figure}
function cyl_obs(x0,y0,z0,R,h)
[x,y,z]=cylinder(R);
x=x+x0;
y=y+y0;
z=z*h+z0;
surf(x,y,z)
hold on
end