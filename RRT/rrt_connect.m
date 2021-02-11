%{
Eric Miller
Edm54
EECS 499
Project 2
RRT Connect
%}
clear all
close all

% Format graph 
set(0, 'DefaultAxesFontWeight', 'normal', ...
      'DefaultAxesFontSize', 20, ...
      'DefaultAxesFontAngle', 'normal', ... 
      'DefaultAxesFontWeight', 'normal', ... 
      'DefaultAxesTitleFontWeight', 'bold', ...
      'DefaultAxesTitleFontSizeMultiplier', 1.3) ;
set(groot,'defaultLineLineWidth',3)

% Define map parameters
global circle_centers radius step_size step_length arm_length;

circle_centers = [[8, 0]; [8, 8]; [8, -8]];
radius = 2;
step_size = .001;
step_length = .75;
arm_length =6;

init_config = [0, 5.3559, .9273];
goal_config = [0, .9273, 5.3559];

[success, q_connect, ta, tb, ta_list, tb_list, ta_map, tb_map] = rrt_connect_algo(init_config, goal_config)

% Determine which tree is the start tree
if ta_list(1, :) == init_config
    start_list = ta_list
    start_map = ta_map
    start_g = ta
    
    end_list = tb_list
    end_map = tb_map
    end_g = tb    
else
    start_list = tb_list
    start_map = tb_map
    start_g = tb
    
    end_list = ta_list
    end_map = ta_map
    end_g = ta
end   

% Check if a path has been find
if length(q_connect) == 3
    start_name = get_config_name(init_config, start_list);
    goal_name = get_config_name(goal_config, end_list);
    start_connect_name = get_config_name(q_connect, start_list);
    end_connect_name = get_config_name(q_connect, end_list);

    start_path = shortestpath(start_g, start_name, start_connect_name);
    end_path = shortestpath(end_g, end_connect_name, goal_name);

    start_config_path = [];
    for i = 1:length(start_path)
        config = start_map(start_path(i));
        start_config_path = [start_config_path ;config];
    end

    end_config_path = [];
    for i = 1:length(end_path)
        config = end_map(end_path(i));
        end_config_path = [end_config_path ;config];
    end

    plot_path(start_config_path, end_config_path, init_config, goal_config)    
    make_video(start_config_path, end_config_path, start_map, end_map)
    merge_trees(start_g, end_g, start_connect_name, end_connect_name)    
end

function [success, q_connect, ta, tb, ta_list, tb_list, ta_map, tb_map] = rrt_connect_algo(init_config, goal_config)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RRT connect algorithm
    % Param:
    %   init_config: starting configuration of the robot
    %   goal_config: goal configuration of the robot
    % Return:
    %   sucess: True if path has been found
    %   q_connect: configuration where the two trees meet
    %   ta: Tree 1
    %   tb: Tree 2
    %   ta_list: list of configurations in Tree 1
    %   tb_list: list of configurations in Tree 2
    %   ta_map: maps the name of each config in ta to the config
    %   tb_map: maps the name of each config in tb to the config
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Maps name to a config
    ta_map = containers.Map;
    tb_map = containers.Map;
    
    ta_list = []; % stores configs in order to track the name of the config
    tb_list = []; % stores configs in order to track the name of the config

    % Add starting node to ta
    ta = graph;
    [ta, ta_map, ta_list] = add_node_update(ta, init_config, ta_map, ta_list);

    % Add goal node to tb
    tb = graph;
    [tb, tb_map, tb_list] = add_node_update(tb, goal_config, tb_map, tb_list);

    success = false;
    maxnodes = 5e4;
    
    for i = 1:maxnodes
       q_rand = sample_configuration(); % Generate random configuration
       
       % Move towards q_rand
       [result, ta, q_target, ta_map, ta_list] = rrt_extend_single(ta, q_rand, ta_map, ta_list); 
        
        if result 
            % Try to connect tb to q_target
            [result2, tb, q_connect, tb_map, tb_list] = rrt_extend_multiple(tb, q_target, tb_map, tb_list);
            if result2
                success = true
                return
            end         
        end
        
        [ta, tb] = swap(ta, tb);
        [ta_list, tb_list] = swap(ta_list, tb_list); 
        [ta_map, tb_map] = swap(ta_map, tb_map); 
    end
end

function [tb, ta] = swap(ta, tb)
    return
end

function [sucess, t, q_target, t_map, t_list] = rrt_extend_single(t, q_rand, t_map, t_list)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RRT Extend Single algorithm
    % Param:
    %   t: tree 
    %   q_rand: random config to move towards
    %   t_map: hashmap for t
    %   t_list: list of configurations in t
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    global step_length;
    q_target = [];
    
    % Get closest configuration to q_rand
    [name, q_near] = get_closest_config(q_rand, t_map);
    q_new = step_towards(step_length, q_near, q_rand);
    sucess = local_planner(q_near, q_new);  
    
    if sucess % Update map, tree and list
        [t, t_map, t_list] = add_node_update(t, q_new, t_map, t_list);
        q_new_name = get_config_name(q_new, t_list);
        q_near_name = get_config_name(q_near, t_list);
        
        t = addedge(t, [q_new_name], [q_near_name]);        
        q_target = q_new;
    end
end

function [result, t, q_connect, t_map, t_list] = rrt_extend_multiple(t, q_target, t_map, t_list)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RRT Extend Multiple Algorithm
    % Param:
    %   t: tree
    %   q_target: target configuration to move to
    %   t_map: hashmap for tree
    %   t_list: list of configs for tree
    % Return:
    %   result: True if path from t to q_target
    %   q_connect: config where paths met
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    global step_length;
    q_connect = [];
    % find closest configuration
    [name, q_near] = get_closest_config(q_target, t_map);
    q_int = step_towards(step_length, q_near, q_target);
    q_last = q_near;
    
    dist = get_config_dist(q_near, q_target);
    num_steps = ceil(dist/step_length);
    
    % Go as far as possible towards q_target
    for i = 1:num_steps
        result = local_planner(q_last, q_int);   
        
        if ~result
            return
        end
        
        % Add to tree, map and list
        [t, t_map, t_list] = add_node_update(t, q_int, t_map, t_list);
        q_int_name = get_config_name(q_int, t_list);
        q_last_name = get_config_name(q_last, t_list);
        t = addedge(t, [q_int_name], [q_last_name]);     
        
        q_connect = q_int;       
        q_last = q_int;
        
        % Make sure last step does not over step target
        if i == num_steps - 1
            q_int = q_target;
        else     
            q_int = step_towards(step_length, q_int, q_target);
        end        
    end
    result = true;
end

function [name] = get_config_name(config, config_list)
    % Gets name of config from config list
    
    index = -1;
    for i = 1:length(config_list)
        if config_list(i, :) == config
           index = i;
           break
        end     
    end
    if index == -1
        config
        error("Config not found")
    end
    name = string(index);
end

function [q_new] = step_towards(step_length, q_near, q_rand)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % steps q_near to q_rand by step_length
    % Param:
    %   step_length: step size to move by
    %   q_near: nearest config to q_rand
    %   q_rand: random configuration to move towards
    % Return:
    %   q_new: new configuration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [step_1, step_2, step_3, num_steps] = get_step_size(q_near, q_rand, step_length);
    q_new = q_near + [step_1 step_2 step_3];
    
    for i = 1:length(q_new)
        if q_new(i) < 0
            q_new(i) = q_new(i) + 2 * pi;
        end
        
        if q_new(i) > 2 * pi
            q_new(i) = q_new(i) - 2 * pi;
        end
    end
end

function [graph, name_to_config_map, config_list] = add_node_update(graph, config, name_to_config_map, config_list)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Adds config to the graph, hash map and list 
    % Param:
    %   graph: tree to add config to
    %   config: configuration to add 
    %   name_to_config_map: Hash map that maps the name of a config to the config
    %   config_list: list of configurations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [num_nodes, a] = size(graph.Nodes);
    name = string(num_nodes + 1);
    graph = addnode(graph, [name]);
    name_to_config_map(name) = config;
    config_list  = [config_list; config];
end

function [min_name, min_config] = get_closest_config(added_config, config_map)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % Finds the closest configuration in config_map to added_config
    % Param:
    %   added_config: target configuration
    %   config_map: hash map that maps the name of a config to the value of
    %   the config
    % Return:
    %   min_name: name of configuration that is closest to added_config
    %   min_config: value of configuration that is closest to added_config
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    keys = config_map.keys;
    values = config_map.values;
    min_dist = inf;
    min_name = '-1';
    min_config = [-100, -100, -100];
    
    for i = 1:length(keys)
        config = values(i);
        config_dist = get_config_dist(config{1}, added_config);
        
        if config_dist < min_dist
            min_dist = config_dist;
            min_name = keys(i);
            min_config = config{1};
        end
    end
end

function [dist] = get_config_dist(config_1, config_2)
    % Distance from config1 to config2
    dist = sqrt((angdiff(config_1(1),config_2(1)))^2 + (angdiff(config_1(2),config_2(2)))^2 + (angdiff(config_1(3),config_2(3)))^2);
end

function [step_1, step_2, step_3, num_steps] = get_step_size(config_1, config_2, step_size)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Finds the step size for theta_1, theta_2 and theta_3
    % I have set the step size from one config to the other, but this 
    % function sets the step size for each angle to acheive the total step
    % size
    % Param:
    %   Config_1: Initial configuration
    %   Config_2: Target configuration
    %   Step_size: distance to travel from config1 to config2
    
    % Returns:
    %   Distance to move in each direction
    %   num_steps: number of steps needed to make it from config_1 to
    %              config_2 via step_size
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dist = get_config_dist(config_1, config_2);
    
    dist_1 = angdiff(config_1(1),config_2(1));
    dist_2 = angdiff(config_1(2),config_2(2));
    dist_3 = angdiff(config_1(3),config_2(3));
   
    num_steps = ceil(dist/step_size);
    step_1 = dist_1/num_steps;
    step_2 = dist_2/num_steps;
    step_3 = dist_3/num_steps;
    
end

function [sucess] = local_planner(config_1, config_2)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Finds a path from config1 to config2 on a straight line
    % Param:
    %   Config_1: starting config  
    %   Config_2: ending config
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    global step_size
    
    [step_1, step_2, step_3, num_steps] = get_step_size(config_1, config_2, step_size);    
    current_config = config_1;
    collision = false;
    
    for i = 1:num_steps
        current_config(1) = wrapTo2Pi([current_config(1) + step_1]);
        current_config(2) = wrapTo2Pi([current_config(2) + step_2]);
        current_config(3) = wrapTo2Pi([current_config(3) + step_3]);      
        collision = check_collision(current_config) | collision;
    end    
    sucess = ~collision ;       
end

function [] = plot_map(config)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plots the robot arm in the map on a new figure
    % Param:
    %   config: Configuration of robot 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    t1 = config(1);
    t2 = config(2);
    t3 = config(3);

    figure
    hold on
    plot_circles()
    plot_robot_arm(config)
    grid minor
end

function [] = plot_robot_arm(config, color)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plots the robot arm on the current figure
    % Param:
    %   config: Configuration of robot 
    %   color: color to plot robot arm
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    t1 = config(1);
    t2 = config(2);
    t3 = config(3);
    
    % Plot link one
    [x1, y1] = find_end(0, 0, t1);
    plot([0, x1], [0, y1], 'Color', color,  'LineWidth', 3)

    % Plot link two
    [x2, y2] = find_end(x1, y1, t1 +  t2);
    plot([x1, x2], [y1, y2], 'Color', color,  'LineWidth', 3)

    % Plot link three
    [x3, y3] = find_end(x2, y2, t1 +  t2 + t3);
    plot([x2, x3], [y2, y3], 'Color', color,  'LineWidth', 3)
end

function [] = circle(x,y,r)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plots circle 
    % Param:  
    %   x: x value of circle center  	
    %   y: y value of circle center  
    %   r: Radius of circle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    fill(xunit, yunit, 'k');
end

function [x_final, y_final] = find_end(x_start, y_start, theta)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Finds the end of a line segment based on theta
    % Params:
    %   x_start: initial X value of line segment
    %   y_start: initial Y value of line segment
    %   theta: angle of line segment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    global arm_length
    x_final = arm_length*cos(theta) + x_start;
    y_final = arm_length* sin(theta) + y_start;
end

function [config] = sample_configuration()
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Randomly samples a random configuration 
    % Randomly samples from 0: 2*pi
    % Returns:
    %   config: Configuration of robot (theta1, theta2, theta3)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    collision = true ;  
    while collision
        t1 = rand * 2 * pi;
        t2 = rand * 2 * pi;
        t3 = rand * 2 * pi;
        config = [t1, t2, t3];
        collision = check_collision(config);
    end
end

function [collision] = check_collision(config)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Checks if a configuration is in collision with any obstacle
    % Params:
    %   config: Configuration of robot (theta1, theta2, theta3)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    global circle_centers
    
    t1 = config(1);
    t2 = config(2);
    t3 = config(3);
    
    [x1, y1] = find_end(0, 0, t1);
    [x2, y2] = find_end(x1, y1, t1 +  t2);
    [x3, y3] = find_end(x2, y2, t1 +  t2 + t3);    
    start_point = [0, 0];
    
    % Define circle centers
    point_1 = [x1, y1];
    point_2 = [x2, y2];
    point_3 = [x3, y3];
    
    collision = false;
    
    % Check collisions with all obstacles
    for i=1:length(circle_centers)
        x = circle_centers(i, 1);
        y = circle_centers(i, 2);
        
        circle_center = [x, y];     
        % Check collision with each link of the robot
        c1 = check_collision_circle(circle_center, start_point, point_1);
        c2 = check_collision_circle(circle_center, point_1, point_2);
        c3 = check_collision_circle(circle_center, point_2, point_3);
        
        if c1|c2|c3
            collision = true;
        end       
    end    
end

function [collision] = check_collision_circle(pt, v1, v2)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Checks if a line segment collides with a circle
    % Params:
    %   pt: center of circle 
    %   v1: vertex one of line segment
    %   v2: verex two of line segment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    global radius
    dist = check_distance_to_circle(pt, v1, v2);
    
    if dist <= radius
        collision  = true;
    else
        collision = false;
    end
end

function distance=check_distance_to_circle(pt, v1, v2)
    % Finds the distance from the line segment from v1 to v2 to the circle
    % center
    % Params:
    %   pt: center of circle
    %   v1: vertex one of line segment
    %   v2: verex two of line segment
    
    % --------------------------------------------------
    % Taken from: https://www.mathworks.com/matlabcentral/answers/260593-distance-between-points-and-a-line-segment-in-3d
    % --------------------------------------------------------------------------------------------------
    
    distance=zeros(1,1);
    dsp=sqrt( (pt(1)-v1(1)).^2 + (pt(2)-v1(2))^2); % Distance between the start and center point
    dep=sqrt( (pt(1)-v2(1)).^2 + (pt(2)-v2(2)).^2); % Distance between the end and center point
    dse=sqrt(sum((v1-v2).^2)); % Distance between start and end
    
    % ------------------------------------------------------------
    % for Type 3 we can use the point_to_line_distance FEX submission
    % https://www.mathworks.com/matlabcentral/fileexchange/64396-point-to-line-distance
    % ------------------------------------------------------------
    dpl=point_to_line_distance(pt, v1, v2); % Distance between P and the projection on the entire line
    
    type1= sqrt(dse^2+dep.^2)<=dsp ; % Type 1: Center point is closer to the end-point than to the rest of the line
    type2= sqrt(dse^2+dsp.^2)<=dep ; % Type 2: Center point is closer to the start-point than to the rest of the line
    type3= ~type1 & ~type2; % Type 3: Center point is closer to some other point on the segment 
    
    distance(type1)=dep(type1);
    distance(type2)=dsp(type2);
    distance(type3)=dpl(type3);
end

function distance = point_to_line_distance(pt, v1, v2)
    % Finds the distance from the line segment from v1 to v2 to the point pt
    % Params:
    %   pt: point 
    %   v1: vertex one of line segment
    %   v2: verex two of line segment

    % --------------------------------------------------
    % Taken from: https://github.com/thrynae/point_to_line_distance/blob/master/point_to_line_distance.m
    % --------------------------------------------------------------------------------------------------

    %prepare inputs
    v1=v1(:)';%force 1x3 or 1x2
    v2=v2(:)';%force 1x3 or 1x2
    if length(v1)==2,v1(3)=0;  end % Extend 1x2 to 1x3 if needed
    if length(v2)==2,v2(3)=0;  end % Extend 1x2 to 1x3 if needed
    if size(pt,2)==2,pt(1,3)=0;end % Extend nx2 to nx3 if needed
    v1_ = repmat(v1,size(pt,1),1); 
    v2_ = repmat(v2,size(pt,1),1);

    a = v1_ - v2_;
    b = pt - v2_;
    distance = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
end

function [] = merge_trees(gs, ge, start_connect, end_connect)
    % Merges the resulting trees for ploting 
    
    global step_length
    g = graph;
    
    start_name_map = containers.Map;
    [start_sz, a] = size(gs.Nodes);
    [start_edges, a] = size(gs.Edges);
    
    [end_sz, a] = size(ge.Nodes);
    [end_edges, a] = size(ge.Edges);
    
    for i = 1:start_sz
        tmp = gs.Nodes{i, 1};
        g = addnode(g, ["s" + string(tmp{1})]); 
    end
    
    for i = 1:start_edges
        tmp = gs.Edges{i, 1};
        g = addedge(g, "s" + string(tmp{1}), "s" + string(tmp{2}));
    end
     
    for i = 1:end_sz
        tmp = ge.Nodes{i, 1};
        g = addnode(g, ["g" + string(tmp{1})]); 
    end
    
    for i = 1:end_edges
        tmp = ge.Edges{i, 1};
        g = addedge(g, "g" + string(tmp{1}), "g" + string(tmp{2}));
    end
    
    g = addedge(g, "s" + start_connect , "g" + end_connect);
    
    figure 
    hold on
%     plot(g,'Layout','force', 'NodeLabel',g.Nodes.Name, 'LineWidth', 4, 'NodeFontSize', 14, 'LineWidth', 2)
    plot(g,'Layout','force', 'MarkerSize', 4, 'LineWidth', 4, 'NodeFontSize', 10, 'LineWidth', 2)
    title('Final Connected Tree, Step Length =' + string(step_length))
    grid minor

end

function [] = make_video(start_path, end_path, start_map, end_map)
    % Makes video of path
    
    frames = length(path);
    xlimit = 14;
    ylimit = 10;

    vidObj=VideoWriter('robot_motion5.avi');
    vidObj.FrameRate=15;
    vidObj.Quality=100;
    vidobj.VideoBitsPerPixel=48;
    open(vidObj);

    figure
    hold on
    axis equal;
    axis([-2 14 -10 10]);
    for i = 1:length(start_path(:, 1)) 

        hold on
        config = start_path(i, :);
        plot_robot_arm(config, [10/255, 48/255, 78/255])
        axis equal;
        axis([-2 14 -10 10]);

        plot_circles()
        text(xlimit/2,0.95*ylimit,strcat('t=',num2str(i)));
        set(gca,'box','on');
        M(:,i) = getframe(gca);
        currFrame = getframe(gcf);
        writeVideo(vidObj,currFrame);
        clf
    end
    for i = 1:length(end_path(:, 1))   
        hold on
        config = end_path(i, :);
        plot_robot_arm(config, [10/255, 48/255, 78/255])

        axis equal;
        axis([-2 14 -10 10]);

        plot_circles()
        text(xlimit/2,0.95*ylimit,strcat('t=',num2str(i)));
        set(gca,'box','on');
        M(:,i) = getframe(gca);
        currFrame = getframe(gcf);
        writeVideo(vidObj,currFrame);
        clf
    end
    close(vidObj);
end

function [] = plot_circles()
    global radius circle_centers
    
    for i=1:length(circle_centers)
        x = circle_centers(i, 1);
        y = circle_centers(i, 2);
        circle(x, y, radius)
    end
end

function [] = plot_path(start_config_path, end_config_path, init, goal)
    % Plots the path of the robot
    
    global step_length arm_length
    figure
    hold on

    plot_circles()
    path_length = length(start_config_path(:, 1)) + length(end_config_path(:, 1)) - 1
    
    color_index = 1; 
    colormap(autumn(path_length))
    greycustom = autumn(path_length)
    
    for i = 1:length(start_config_path(:, 1))
        config = start_config_path(i, :);        
        plot_robot_arm(config, greycustom(color_index, :))   
        color_index = color_index + 1;
    end
    
    color_index = color_index - 1
    for i = 1:length(end_config_path(:, 1))
        config = end_config_path(i, :);        
        plot_robot_arm(config, greycustom(color_index, :))       
        color_index = color_index + 1;
    end
    
    text(12, -3, 'Start', 'FontSize', 18)
    text(12, 5, 'Goal','FontSize', 18)

    grid minor
    colormap(autumn(path_length))
    colorbar
    caxis([0 path_length])
    title('Final Path, Step Length =' + string(step_length) + ', Arm Length = ' + string(arm_length))
end

