clc; clear; close all;

%% Grey Wolf Optimizer Parameters
num_drones = 10;
num_targets = 5;
max_iterations = 1000;
operating_height = 15;  % Operating height (m)
ascent_speed = 1;      % Speed for initial ascent (m/s)
dt = 0.2;             % Timestep (s)
max_speed = 10;       % Maximum UAV speed (m/s)
detection_radius = 3.0;  % Target detection radius (m)
obstacle_margin = 3.0;   % Safety margin for obstacles (m)

% Environment parameters
grid_size = [100, 100, 60];  % Search space size (m)
ObstaclePositions = [50 0 0; 20 50 0; 50 70 0];
ObstacleHeights = [25, 35, 20];  % Different heights for each obstacle
ObstaclesWidth = 3;

%% Initialize UAVs
% Initialize UAVs in a circular pattern
radius = 6;  % meters
positions = zeros(num_drones, 3);
InitialPosition = [10 10 0];  % Start position
for i = 1:num_drones
    angle = 2*pi*(i-1)/num_drones;
    x = radius*cos(angle);
    y = radius*sin(angle);
    positions(i, :) = InitialPosition + [x, y, 0];  % Start at ground level
end

%% Initialize Targets
targets = [];
while size(targets, 1) < num_targets
    % Generate targets at ground level (z=0)
    candidate = [randi(grid_size(1)), randi(grid_size(2)), 0];
    % Expanded obstacle check
    is_safe = true;
    for obs = 1:size(ObstaclePositions,1)
        obs_pos = ObstaclePositions(obs,:);
        if norm(candidate(1:2) - obs_pos(1:2)) < ObstaclesWidth * 2
            is_safe = false;
            break;
        end
    end
    % Check distance from other targets
    for t = 1:size(targets,1)
        if norm(candidate(1:2) - targets(t,1:2)) < 5
            is_safe = false;
            break;
        end
    end
    if is_safe
        targets = [targets; candidate];
    end
end

%% Initialize Visualization
figure('Position', [100 100 800 600]);
hold on;

% Draw obstacles
for i = 1:size(ObstaclePositions,1)
    x = ObstaclePositions(i,1);
    y = ObstaclePositions(i,2);
    w = ObstaclesWidth;
    h = ObstacleHeights(i);
    % Draw base
    vertices = [x-w/2, y-w/2, 0;
               x+w/2, y-w/2, 0;
               x+w/2, y+w/2, 0;
               x-w/2, y+w/2, 0;
               x-w/2, y-w/2, 0];
    plot3(vertices(:,2), vertices(:,1), zeros(size(vertices,1)), 'k-', 'LineWidth', 2);
    % Draw vertical edges
    for v = 1:4
        plot3([vertices(v,2) vertices(v,2)], ...
              [vertices(v,1) vertices(v,1)], ...
              [0 h], 'k--', 'LineWidth', 1);
    end
    % Add top edges
    top_vertices = [x-w/2, y-w/2, h;
                   x+w/2, y-w/2, h;
                   x+w/2, y+w/2, h;
                   x-w/2, y+w/2, h;
                   x-w/2, y-w/2, h];
    plot3(top_vertices(:,2), top_vertices(:,1), top_vertices(:,3), 'k-', 'LineWidth', 1);
end

% Plot targets (with handle for color updates)
target_plots = gobjects(num_targets, 1);
for i = 1:num_targets
    target_plots(i) = scatter3(targets(i,2), targets(i,1), 0, 200, 'r', 'filled', 'Marker', 'o', 'LineWidth', 2);
end

% Initialize UAV plots and trajectories
uav_plots = gobjects(num_drones, 1);
trajectory_plots = cell(num_drones, 1);  % Store trajectory line handles
trajectories = cell(num_drones, 1);
for i = 1:num_drones
    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    trajectories{i} = positions(i,:);
    % Initialize trajectory plot
    trajectory_plots{i} = plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0 0.5 1], 'LineWidth', 1.5);
end

% Set up the view
grid on;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
title('Grey Wolf Optimizer-based UAV Search Progress');
view(45, 30);

% Set axis limits
xlim([0 grid_size(2)]);
ylim([0 grid_size(1)]);
zlim([0 grid_size(3)]);
axis equal;

%% Initial Ascent Phase
fprintf('Starting initial ascent phase...\n');
ascending = true(1, num_drones);
while any(ascending)
    for d = 1:num_drones
        if ascending(d)
            if positions(d,3) < operating_height
                positions(d,3) = min(positions(d,3) + ascent_speed*dt, operating_height);
                % Update visualization
                set(uav_plots(d), 'XData', positions(d,2), 'YData', positions(d,1), 'ZData', positions(d,3));
                trajectories{d} = [trajectories{d}; positions(d,:)];
                % Update trajectory visualization
                set(trajectory_plots{d}, 'XData', trajectories{d}(:,2), ...
                                       'YData', trajectories{d}(:,1), ...
                                       'ZData', trajectories{d}(:,3));
            else
                ascending(d) = false;
                positions(d,3) = operating_height;  % Ensure exact height
            end
        end
    end
    drawnow;
    pause(0.01);
end

%% Grey Wolf Optimizer Search
fprintf('Initial ascent complete. Starting GWO search...\n');
targets_found = false(num_targets, 1);
start_time = tic;

% Initialize alpha, beta, and delta positions
wolf_scores = inf(num_drones, 1);
alpha_pos = zeros(1, 3);
beta_pos = zeros(1, 3);
delta_pos = zeros(1, 3);

for iter = 1:max_iterations
    % Score all wolves
    for i = 1:num_drones
        available_targets = targets(~targets_found, :);
        if ~isempty(available_targets)
            wolf_scores(i) = min(vecnorm(available_targets(:,1:2) - positions(i,1:2), 2, 2));
        end
    end
    
    % Update alpha, beta, and delta
    [~, sorted_indices] = sort(wolf_scores);
    alpha_pos = positions(sorted_indices(1),:);
    beta_pos = positions(sorted_indices(2),:);
    delta_pos = positions(sorted_indices(3),:);
    
    % Update positions
    a = 2 - iter * (2/max_iterations);  % Linearly decreased from 2 to 0
    
    for i = 1:num_drones
        prev_pos = positions(i,:);
        
        % Calculate movement towards alpha, beta, and delta
        r1 = rand(1,3); r2 = rand(1,3);
        A1 = 2 * a * r1 - a;
        C1 = 2 * r2;
        D_alpha = abs(C1 .* alpha_pos - positions(i,:));
        X1 = alpha_pos - A1 .* D_alpha;
        
        r1 = rand(1,3); r2 = rand(1,3);
        A2 = 2 * a * r1 - a;
        C2 = 2 * r2;
        D_beta = abs(C2 .* beta_pos - positions(i,:));
        X2 = beta_pos - A2 .* D_beta;
        
        r1 = rand(1,3); r2 = rand(1,3);
        A3 = 2 * a * r1 - a;
        C3 = 2 * r2;
        D_delta = abs(C3 .* delta_pos - positions(i,:));
        X3 = delta_pos - A3 .* D_delta;
        
        % Calculate new position
        movement = (X1 + X2 + X3)/3 - positions(i,:);
        
        % Maintain height
        movement(3) = 0.5 * (operating_height - positions(i,3));
        
        % Apply speed limit
        speed = norm(movement);
        if speed > max_speed
            movement = movement * max_speed / speed;
        end
        
        % Update position
        new_pos = positions(i,:) + movement * dt;
        
        % Keep within bounds
        new_pos = max([1, 1, operating_height], min([grid_size(1:2), operating_height], new_pos));
        
        % Check obstacle collision
        is_safe = true;
        for obs = 1:size(ObstaclePositions,1)
            if norm(new_pos(1:2) - ObstaclePositions(obs,1:2)) < ObstaclesWidth * obstacle_margin
                is_safe = false;
                break;
            end
        end
        
        if is_safe
            positions(i,:) = new_pos;
            positions(i,3) = operating_height;  % Maintain height
            
            % Update visualization
            set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
            trajectories{i} = [trajectories{i}; positions(i,:)];
            % Update trajectory visualization
            set(trajectory_plots{i}, 'XData', trajectories{i}(:,2), ...
                                   'YData', trajectories{i}(:,1), ...
                                   'ZData', trajectories{i}(:,3));
        else
            % If collision detected, apply random perturbation
            movement = max_speed * (2 * rand(1, 3) - 1);
            movement(3) = 0;  % Maintain height
        end
        
        % Check for target finding
        for t = 1:num_targets
            if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                targets_found(t) = true;
                fprintf('Target %d found at iteration %d (%.2f sec)\n', t, iter, toc(start_time));
                
                % Update target color to green with larger marker
                set(target_plots(t), 'CData', [0 1 0], ...
                                    'MarkerFaceColor', [0 1 0], ...
                                    'MarkerEdgeColor', [0 0.6 0]);
                
                % Add visual effect for target discovery
                discovery_marker = scatter3(targets(t,2), targets(t,1), 0, 300, 'g', 'filled', 'MarkerFaceAlpha', 0.3);
                pause(0.1);
                delete(discovery_marker);
            end
        end
    end
    
    drawnow;
    pause(0.01);
    
    if all(targets_found)
        fprintf('All targets found in %.2f seconds!\n', toc(start_time));
        break;
    end
end

if ~all(targets_found)
    fprintf('Search completed. Found %d/%d targets in %.2f seconds.\n', ...
        sum(targets_found), num_targets, toc(start_time));
end 