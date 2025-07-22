clc; clear; close all;

%% Comparison of PSO, Firefly Algorithm (FA), Grey Wolf Optimizer (GWO), Ant Colony Optimization (ACO), and Tabu Search (TS) for UAV Search
num_simulations = 5;
num_drones = 10;
num_targets = 5;
max_iterations = 1000;
operating_height = 15;  % Operating height (m)
ascent_speed = 1;      % Speed for initial ascent (m/s)

% Environment parameters
grid_size = [100, 100, 60];  % Search space size (m)
ObstaclePositions = [50 0 0; 20 50 0; 50 70 0];
ObstacleHeights = [25, 35, 20];  % Different heights for each obstacle
ObstaclesWidth = 3;

% Initialize targets at ground level
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

% Create figure with subplots for each method
methods = {'PSO', 'FA', 'GWO', 'ACO', 'TS'};

% Initialize results structure
results = struct('Method', {}, 'Simulation', {}, 'Time', {}, 'Distance', {}, 'Iterations', {}, 'SuccessRate', {});

% Run simulations
for sim = 1:num_simulations
    fprintf('\nStarting Simulation %d/%d\n', sim, num_simulations);
    
    % Run each method sequentially
    for m = 1:length(methods)
        % Close any existing figures
        close all;
        
        % Create new figure for current method
        figure_handle = figure('Name', methods{m}, 'Position', [100 100 800 600]);
        hold on;
        
        fprintf('\nRunning %s (Simulation %d/%d)\n', methods{m}, sim, num_simulations);
        
        % Initialize UAVs in a circular pattern
        radius = 6;  % Increased radius like in PSOswarm3D
        positions = zeros(num_drones, 3);
        InitialPosition = [10 10 0];  % Start position
        for i = 1:num_drones
            angle = 2*pi*(i-1)/num_drones;
            x = radius*cos(angle);
            y = radius*sin(angle);
            positions(i, :) = InitialPosition + [x, y, 0];  % Start at ground level
        end
        
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
        
        % Plot targets
        scatter3(targets(:,2), targets(:,1), zeros(size(targets,1)), 100, 'r', '*', 'LineWidth', 2);
        
        % Initialize UAV plots and trajectories
        uav_plots = gobjects(num_drones, 1);
        trajectories = cell(num_drones, 1);
        for i = 1:num_drones
            uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
            trajectories{i} = positions(i,:);
        end
        
        % Set up the view
        grid on;
        xlabel('East (m)');
        ylabel('North (m)');
        zlabel('Up (m)');
        title(sprintf('%s Search Progress (Sim %d/%d)', methods{m}, sim, num_simulations));
        view(3);
        
        % Set axis limits first, then make equal
        xlim([0 grid_size(2)]);
        ylim([0 grid_size(1)]);
        zlim([0 grid_size(3)]);
        axis equal;
        
        % Adjust camera view
        view(45, 30);
        
        % Run optimization method
        fprintf('Starting initial ascent phase...\n');
        % Initial ascent phase before running the optimization
        ascending = true(1, num_drones);
        while any(ascending)
            for d = 1:num_drones
                if ascending(d)
                    if positions(d,3) < operating_height
                        positions(d,3) = min(positions(d,3) + ascent_speed, operating_height);
                        % Update visualization
                        if isvalid(uav_plots(d))
                            set(uav_plots(d), 'XData', positions(d,2), 'YData', positions(d,1), 'ZData', positions(d,3));
                        else
                            uav_plots(d) = plot3(positions(d,2), positions(d,1), positions(d,3), 'bo', 'MarkerFaceColor', 'b');
                        end
                        trajectories{d} = [trajectories{d}; positions(d,:)];
                        plot3(positions(d,2), positions(d,1), positions(d,3), '-', 'Color', [0.8 0.8 1]);
                    else
                        ascending(d) = false;
                        positions(d,3) = operating_height;  % Ensure exact height
                    end
                end
            end
            drawnow;
            pause(0.01);
        end
        fprintf('Initial ascent complete. Starting optimization...\n');
        
        % Run the selected optimization method
        [time, distance, iterations, success, final_positions, final_trajectories] = runUAVSearch(methods{m}, positions, targets, num_drones, num_targets, max_iterations, operating_height, ObstaclePositions, ObstacleHeights, ObstaclesWidth, figure_handle, uav_plots, trajectories);
        
        % Store results
        results(end+1) = struct('Method', methods{m}, 'Simulation', sim, 'Time', time, 'Distance', distance, 'Iterations', iterations, 'SuccessRate', success);
        
        % Display current results
        fprintf('%s completed: Time=%.2f, Distance=%.2f, Success=%.2f%%\n', ...
                methods{m}, time, distance, success*100);
        
        % Pause briefly between methods
        pause(1);
    end
end

% Display final results table
results_table = struct2table(results);
disp('Final Results:');
disp(results_table);

%% Function for UAV Search using different optimization algorithms
function [time, distance, iterations, success, positions, trajectories] = runUAVSearch(method, positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, figure_handle, uav_plots, trajectories)
    dt = 0.2;
    max_speed = 10;
    detection_radius = 3.0;
    obstacle_margin = 3.0;
    targets_found = false(num_targets, 1);
    distance = 0;
    grid_size = [100, 100, 60];  % Define grid_size here for ACO
    
    % Ensure all UAVs are at operating height before starting
    positions(:,3) = operating_height;
    
    % Update visualization before starting
    figure(figure_handle);
    for i = 1:num_drones
        if isvalid(uav_plots(i))
            set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
        else
            uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
        end
    end
    drawnow;
    
    switch method
        case 'PSO'
            [time, distance, iterations, success, positions, trajectories] = runPSO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories);
        case 'FA'
            [time, distance, iterations, success, positions, trajectories] = runFA(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories);
        case 'GWO'
            [time, distance, iterations, success, positions, trajectories] = runGWO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories);
        case 'ACO'
            [time, distance, iterations, success, positions, trajectories] = runACO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories);
        case 'TS'
            [time, distance, iterations, success, positions, trajectories] = runTS(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories);
    end
end

%% Particle Swarm Optimization (PSO)
function [time, distance, iterations, success, positions, trajectories] = runPSO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories)
    velocities = zeros(num_drones, 3);
    pBest = positions;
    pBestScores = inf(num_drones, 1);
    gBest = zeros(1, 3);
    gBestScore = inf;
    targets_found = false(num_targets, 1);
    distance = 0;
    success = 0;
    
    % Ensure we're plotting in the correct figure
    figure(figure_handle);
    
    for iter = 1:max_iterations
        % Update personal and global best
        for i = 1:num_drones
            available_targets = targets(~targets_found, :);
            if isempty(available_targets)
                success = 1;
                time = iter;
                iterations = iter;
                return;
            end
            
            distances = vecnorm(available_targets(:,1:2) - positions(i,1:2), 2, 2);
            current_score = min(distances);
            
            if current_score < pBestScores(i)
                pBestScores(i) = current_score;
                pBest(i,:) = positions(i,:);
            end
            
            if current_score < gBestScore
                gBestScore = current_score;
                gBest = positions(i,:);
            end
            
            % Check for target finding
            for t = 1:num_targets
                if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                    targets_found(t) = true;
                end
            end
        end
        
        % Update positions
        for i = 1:num_drones
            prev_pos = positions(i,:);
            
            % Calculate new velocity
            r1 = rand(1,3);
            r2 = rand(1,3);
            velocities(i,:) = 0.7 * velocities(i,:) + ...
                            1.5 * r1 .* (pBest(i,:) - positions(i,:)) + ...
                            1.5 * r2 .* (gBest - positions(i,:));
            
            % Maintain operating height
            velocities(i,3) = 0.5 * (operating_height - positions(i,3));
            
            % Apply speed limit
            speed = norm(velocities(i,:));
            if speed > max_speed
                velocities(i,:) = velocities(i,:) * max_speed / speed;
            end
            
            % Update position
            new_pos = positions(i,:) + velocities(i,:) * dt;
            
            % Check obstacle collision
            is_safe = true;
            for obs = 1:size(obstacles,1)
                if norm(new_pos(1:2) - obstacles(obs,1:2)) < obstacle_width * obstacle_margin
                    is_safe = false;
                    break;
                end
            end
            
            if is_safe
                positions(i,:) = new_pos;
                positions(i,3) = operating_height;  % Maintain height
                distance = distance + norm(positions(i,:) - prev_pos);
                
                % Update visualization - recreate plot if invalid
                try
                    set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
                catch
                    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
                end
                
                % Update trajectory
                trajectories{i} = [trajectories{i}; positions(i,:)];
                plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0.8 0.8 1]);
            end
        end
        
        drawnow;
        pause(0.01);
        
        if all(targets_found)
            success = 1;
            time = iter;
            iterations = iter;
            return;
        end
    end
    
    time = max_iterations;
    iterations = max_iterations;
    success = sum(targets_found)/num_targets;
end

%% Firefly Algorithm (FA)
function [time, distance, iterations, success, positions, trajectories] = runFA(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories)
    % FA parameters
    alpha = 0.5;     % Randomization parameter
    beta0 = 1.0;     % Attractiveness at distance = 0
    gamma = 0.1;     % Light absorption coefficient
    targets_found = false(num_targets, 1);
    distance = 0;
    success = 0;
    
    % Ensure we're plotting in the correct figure
    figure(figure_handle);
    
    for iter = 1:max_iterations
        % Update firefly positions
        for i = 1:num_drones
            prev_pos = positions(i,:);
            movement = zeros(1,3);
            
            % Compare with all other fireflies
            for j = 1:num_drones
                if i ~= j
                    % Calculate distance between fireflies
                    r = norm(positions(i,1:2) - positions(j,1:2));
                    
                    % If j is brighter (closer to targets), move towards it
                    j_dist = min(vecnorm(targets(~targets_found,1:2) - positions(j,1:2), 2, 2));
                    i_dist = min(vecnorm(targets(~targets_found,1:2) - positions(i,1:2), 2, 2));
                    
                    if j_dist < i_dist
                        % Calculate attractiveness
                        beta = beta0 * exp(-gamma * r^2);
                        
                        % Update movement
                        movement = movement + beta * (positions(j,:) - positions(i,:));
                    end
                end
            end
            
            % Add randomization
            movement = movement + alpha * (rand(1,3) - 0.5);
            
            % Maintain height
            movement(3) = 0.5 * (operating_height - positions(i,3));
            
            % Apply speed limit
            speed = norm(movement);
            if speed > max_speed
                movement = movement * max_speed / speed;
            end
            
            % Update position
            new_pos = positions(i,:) + movement * dt;
            
            % Check obstacle collision
            is_safe = true;
            for obs = 1:size(obstacles,1)
                if norm(new_pos(1:2) - obstacles(obs,1:2)) < obstacle_width * obstacle_margin
                    is_safe = false;
                    break;
                end
            end
            
            if is_safe
                positions(i,:) = new_pos;
                positions(i,3) = operating_height;  % Maintain height
                distance = distance + norm(positions(i,:) - prev_pos);
                
                % Update visualization - recreate plot if invalid
                try
                    set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
                catch
                    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
                end
                
                % Update trajectory
                trajectories{i} = [trajectories{i}; positions(i,:)];
                plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0.8 0.8 1]);
            end
            
            % Check for target finding
            for t = 1:num_targets
                if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                    targets_found(t) = true;
                end
            end
        end
        
        drawnow;
        pause(0.01);
        
        if all(targets_found)
            success = 1;
            time = iter;
            iterations = iter;
            return;
        end
    end
    
    time = max_iterations;
    iterations = max_iterations;
    success = sum(targets_found)/num_targets;
end

%% Grey Wolf Optimizer (GWO)
function [time, distance, iterations, success, positions, trajectories] = runGWO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories)
    targets_found = false(num_targets, 1);
    distance = 0;
    success = 0;
    
    % Ensure we're plotting in the correct figure
    figure(figure_handle);
    
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
            
            % Check obstacle collision
            is_safe = true;
            for obs = 1:size(obstacles,1)
                if norm(new_pos(1:2) - obstacles(obs,1:2)) < obstacle_width * obstacle_margin
                    is_safe = false;
                    break;
                end
            end
            
            if is_safe
                positions(i,:) = new_pos;
                positions(i,3) = operating_height;  % Maintain height
                distance = distance + norm(positions(i,:) - prev_pos);
                
                % Update visualization - recreate plot if invalid
                try
                    set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
                catch
                    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
                end
                
                % Update trajectory
                trajectories{i} = [trajectories{i}; positions(i,:)];
                plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0.8 0.8 1]);
            end
            
            % Check for target finding
            for t = 1:num_targets
                if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                    targets_found(t) = true;
                end
            end
        end
        
        drawnow;
        pause(0.01);
        
        if all(targets_found)
            success = 1;
            time = iter;
            iterations = iter;
            return;
        end
    end
    
    time = max_iterations;
    iterations = max_iterations;
    success = sum(targets_found)/num_targets;
end

%% Ant Colony Optimization (ACO)
function [time, distance, iterations, success, positions, trajectories] = runACO(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories)
    % ACO parameters
    evaporation_rate = 0.1;
    pheromone_strength = 1.0;
    targets_found = false(num_targets, 1);
    distance = 0;
    success = 0;
    
    % Initialize pheromone matrix
    pheromone = ones(grid_size(1), grid_size(2)) * 0.1;
    
    % Ensure we're plotting in the correct figure
    figure(figure_handle);
    
    for iter = 1:max_iterations
        % Update each ant (drone)
        for i = 1:num_drones
            prev_pos = positions(i,:);
            
            % Get available directions (8 directions)
            directions = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
            valid_moves = [];
            valid_probs = [];
            
            % Check each direction
            for d = 1:size(directions,1)
                test_pos = positions(i,1:2) + directions(d,:) * max_speed * dt;
                
                % Check if move is valid (within bounds and not colliding)
                is_valid = true;
                if any(test_pos < 1) || any(test_pos > [grid_size(1), grid_size(2)])
                    is_valid = false;
                end
                
                % Check obstacle collision
                for obs = 1:size(obstacles,1)
                    if norm(test_pos - obstacles(obs,1:2)) < obstacle_width * obstacle_margin
                        is_valid = false;
                        break;
                    end
                end
                
                if is_valid
                    valid_moves = [valid_moves; directions(d,:)];
                    pos_idx = round(test_pos);
                    valid_probs = [valid_probs; pheromone(pos_idx(1), pos_idx(2))];
                end
            end
            
            if ~isempty(valid_moves)
                % Select direction based on pheromone levels
                probs = valid_probs / sum(valid_probs);
                cumprobs = cumsum(probs);
                selected = find(rand < cumprobs, 1);
                
                % Calculate movement
                movement = [valid_moves(selected,:), 0] * max_speed * dt;
                movement(3) = 0.5 * (operating_height - positions(i,3));  % Height adjustment
                
                % Update position
                new_pos = positions(i,:) + movement;
                positions(i,:) = new_pos;
                positions(i,3) = operating_height;  % Maintain height
                distance = distance + norm(movement);
                
                % Update visualization - recreate plot if invalid
                try
                    set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
                catch
                    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
                end
                
                % Update trajectory
                trajectories{i} = [trajectories{i}; positions(i,:)];
                plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0.8 0.8 1]);
                
                % Update pheromone
                pos_idx = round(positions(i,1:2));
                pheromone(pos_idx(1), pos_idx(2)) = pheromone(pos_idx(1), pos_idx(2)) + pheromone_strength;
            end
            
            % Check for target finding
            for t = 1:num_targets
                if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                    targets_found(t) = true;
                    % Add extra pheromone near found target
                    target_pos = round(targets(t,1:2));
                    pheromone(max(1,target_pos(1)-1):min(grid_size(1),target_pos(1)+1), ...
                             max(1,target_pos(2)-1):min(grid_size(2),target_pos(2)+1)) = ...
                        pheromone(max(1,target_pos(1)-1):min(grid_size(1),target_pos(1)+1), ...
                                max(1,target_pos(2)-1):min(grid_size(2),target_pos(2)+1)) + pheromone_strength * 2;
                end
            end
        end
        
        % Evaporate pheromone
        pheromone = pheromone * (1 - evaporation_rate);
        
        drawnow;
        pause(0.01);
        
        if all(targets_found)
            success = 1;
            time = iter;
            iterations = iter;
            return;
        end
    end
    
    time = max_iterations;
    iterations = max_iterations;
    success = sum(targets_found)/num_targets;
end

%% Tabu Search (TS)
function [time, distance, iterations, success, positions, trajectories] = runTS(positions, targets, num_drones, num_targets, max_iterations, operating_height, obstacles, obstacle_heights, obstacle_width, dt, max_speed, detection_radius, obstacle_margin, figure_handle, uav_plots, trajectories)
    % TS parameters
    tabu_list_size = 10;
    neighborhood_size = 8;  % 8 directions
    targets_found = false(num_targets, 1);
    distance = 0;
    success = 0;
    
    % Initialize tabu lists for each drone
    tabu_lists = cell(num_drones, 1);
    for i = 1:num_drones
        tabu_lists{i} = zeros(tabu_list_size, 3);
    end
    
    % Ensure we're plotting in the correct figure
    figure(figure_handle);
    
    for iter = 1:max_iterations
        for i = 1:num_drones
            prev_pos = positions(i,:);
            
            % Generate neighborhood moves (8 directions)
            directions = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
            valid_moves = [];
            move_scores = [];
            
            % Evaluate each possible move
            for d = 1:size(directions,1)
                test_pos = positions(i,1:2) + directions(d,:) * max_speed * dt;
                
                % Check if move is valid and not in tabu list
                is_valid = true;
                if any(test_pos < 1) || any(test_pos > [grid_size(1), grid_size(2)])
                    is_valid = false;
                end
                
                % Check obstacle collision
                for obs = 1:size(obstacles,1)
                    if norm(test_pos - obstacles(obs,1:2)) < obstacle_width * obstacle_margin
                        is_valid = false;
                        break;
                    end
                end
                
                % Check if move is in tabu list
                for t = 1:size(tabu_lists{i},1)
                    if norm(test_pos - tabu_lists{i}(t,1:2)) < 1
                        is_valid = false;
                        break;
                    end
                end
                
                if is_valid
                    valid_moves = [valid_moves; directions(d,:)];
                    % Score based on distance to nearest unfound target
                    available_targets = targets(~targets_found,:);
                    if ~isempty(available_targets)
                        min_dist = min(vecnorm(available_targets(:,1:2) - test_pos, 2, 2));
                        move_scores = [move_scores; -min_dist];  % Negative because we want to minimize distance
                    else
                        move_scores = [move_scores; 0];
                    end
                end
            end
            
            if ~isempty(valid_moves)
                % Select best non-tabu move
                [~, best_idx] = max(move_scores);
                movement = [valid_moves(best_idx,:), 0] * max_speed * dt;
                movement(3) = 0.5 * (operating_height - positions(i,3));  % Height adjustment
                
                % Update position
                new_pos = positions(i,:) + movement;
                positions(i,:) = new_pos;
                positions(i,3) = operating_height;  % Maintain height
                distance = distance + norm(movement);
                
                % Update tabu list
                tabu_lists{i} = [positions(i,:); tabu_lists{i}(1:end-1,:)];
                
                % Update visualization - recreate plot if invalid
                try
                    set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
                catch
                    uav_plots(i) = plot3(positions(i,2), positions(i,1), positions(i,3), 'bo', 'MarkerFaceColor', 'b');
                end
                
                % Update trajectory
                trajectories{i} = [trajectories{i}; positions(i,:)];
                plot3(positions(i,2), positions(i,1), positions(i,3), '-', 'Color', [0.8 0.8 1]);
            end
            
            % Check for target finding
            for t = 1:num_targets
                if ~targets_found(t) && norm(positions(i,1:2) - targets(t,1:2)) < detection_radius
                    targets_found(t) = true;
                end
            end
        end
        
        drawnow;
        pause(0.01);
        
        if all(targets_found)
            success = 1;
            time = iter;
            iterations = iter;
            return;
        end
    end
    
    time = max_iterations;
    iterations = max_iterations;
    success = sum(targets_found)/num_targets;
end