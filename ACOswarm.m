clc; clear; close all;

%% Ant Colony Optimization Parameters
num_drones = 10;
num_targets = 5;
max_iterations = 1000;
operating_height = 15;  % Operating height (m)
ascent_speed = 1;      % Speed for initial ascent (m/s)
dt = 0.2;             % Timestep (s)
max_speed = 10;       % Maximum UAV speed (m/s)
detection_radius = 3.0;  % Target detection radius (m)
obstacle_margin = 3.0;   % Safety margin for obstacles (m)

% ACO specific parameters
evaporation_rate = 0.05;    % Reduced evaporation rate for better path persistence
pheromone_strength = 2.0;   % Increased base pheromone deposit
min_distance = 4;          % Minimum distance between drones
target_weight = 2.0;       % Weight for target attraction
exploration_rate = 0.2;    % Probability of random exploration

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
title('Ant Colony Optimization-based UAV Search Progress');
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

%% ACO Search
fprintf('Initial ascent complete. Starting ACO search...\n');
targets_found = false(num_targets, 1);
start_time = tic;

% Initialize pheromone matrix with higher initial values
pheromone = ones(grid_size(1), grid_size(2)) * 0.5;
directions = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];  % 8 possible directions

% Initialize target influence matrix
target_influence = zeros(grid_size(1), grid_size(2));
for t = 1:num_targets
    [X, Y] = meshgrid(1:grid_size(1), 1:grid_size(2));
    distances = sqrt((X - targets(t,1)).^2 + (Y - targets(t,2)).^2);
    target_influence = target_influence + 1./(1 + distances);
end

for iter = 1:max_iterations
    % Update each ant (drone)
    for i = 1:num_drones
        prev_pos = positions(i,:);
        
        % Calculate repulsion from other drones
        repulsion = zeros(1,2);
        for j = 1:num_drones
            if i ~= j
                diff_vec = positions(i,1:2) - positions(j,1:2);
                dist = norm(diff_vec);
                if dist < min_distance
                    repulsion = repulsion + diff_vec * (min_distance - dist) / min_distance;
                end
            end
        end
        
        % Get available directions
        valid_moves = [];
        move_probs = [];
        
        % Evaluate each possible direction
        for d = 1:size(directions,1)
            test_pos = positions(i,1:2) + directions(d,:) * max_speed * dt;
            
            % Check if move is valid
            if test_pos(1) >= 1 && test_pos(1) <= grid_size(1) && ...
               test_pos(2) >= 1 && test_pos(2) <= grid_size(2)
                
                % Check obstacle collision
                is_safe = true;
                for obs = 1:size(ObstaclePositions,1)
                    if norm(test_pos - ObstaclePositions(obs,1:2)) < ObstaclesWidth * obstacle_margin
                        is_safe = false;
                        break;
                    end
                end
                
                if is_safe
                    valid_moves = [valid_moves; directions(d,:)];
                    pos_idx = round(test_pos);
                    
                    % Calculate move probability based on multiple factors
                    pheromone_factor = pheromone(pos_idx(1), pos_idx(2))^2;  % Squared for stronger influence
                    
                    % Target attraction with distance-based scaling
                    target_factor = 0;
                    for t = 1:num_targets
                        if ~targets_found(t)
                            dist_to_target = norm(test_pos - targets(t,1:2));
                            target_factor = target_factor + target_weight / (1 + dist_to_target^0.5);
                        end
                    end
                    
                    % Add target influence from pre-calculated matrix
                    target_factor = target_factor + target_influence(pos_idx(1), pos_idx(2));
                    
                    % Combine factors
                    move_prob = pheromone_factor * (1 + target_factor);
                    
                    % Add repulsion influence
                    repulsion_factor = norm(repulsion);
                    if repulsion_factor > 0
                        move_prob = move_prob * (1 - dot(normalize(repulsion), directions(d,:)));
                    end
                    
                    move_probs = [move_probs; max(0.1, move_prob)];  % Ensure minimum probability
                end
            end
        end
        
        if ~isempty(valid_moves)
            % Random exploration with probability exploration_rate
            if rand < exploration_rate
                selected = randi(size(valid_moves, 1));
            else
                % Select direction based on probabilities
                move_probs = move_probs / sum(move_probs);
                cumprobs = cumsum(move_probs);
                selected = find(rand < cumprobs, 1);
            end
            
            % Calculate movement
            movement = [valid_moves(selected,:), 0] * max_speed;
            
            % Update position
            new_pos = positions(i,:) + movement * dt;
            new_pos(3) = operating_height;  % Maintain height
            positions(i,:) = new_pos;
            
            % Update pheromone with distance-based deposit
            pos_idx = round(positions(i,1:2));
            deposit_amount = pheromone_strength;
            
            % Deposit pheromone in a small radius
            radius = 2;
            for x = -radius:radius
                for y = -radius:radius
                    px = pos_idx(1) + x;
                    py = pos_idx(2) + y;
                    if px >= 1 && px <= grid_size(1) && py >= 1 && py <= grid_size(2)
                        dist = sqrt(x^2 + y^2);
                        if dist <= radius
                            pheromone(px, py) = pheromone(px, py) + deposit_amount * (1 - dist/radius);
                        end
                    end
                end
            end
            
            % Update visualization
            set(uav_plots(i), 'XData', positions(i,2), 'YData', positions(i,1), 'ZData', positions(i,3));
            trajectories{i} = [trajectories{i}; positions(i,:)];
            % Update trajectory visualization
            set(trajectory_plots{i}, 'XData', trajectories{i}(:,2), ...
                                   'YData', trajectories{i}(:,1), ...
                                   'ZData', trajectories{i}(:,3));
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
                
                % Add strong pheromone near found target
                target_pos = round(targets(t,1:2));
                range = -3:3;  % Increased range
                for x = range
                    for y = range
                        px = target_pos(1) + x;
                        py = target_pos(2) + y;
                        if px >= 1 && px <= grid_size(1) && py >= 1 && py <= grid_size(2)
                            dist = sqrt(x^2 + y^2);
                            pheromone(px, py) = pheromone(px, py) + pheromone_strength * 4 * (1 - dist/length(range));
                        end
                    end
                end
            end
        end
    end
    
    % Evaporate pheromone with distance-based decay
    pheromone = pheromone * (1 - evaporation_rate);
    
    % Add small amount of pheromone to maintain minimum level
    pheromone = max(pheromone, 0.1);
    
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
end 