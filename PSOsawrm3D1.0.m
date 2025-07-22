clc; clear; close all;

%% Create UAV Scenario
Scenario = uavScenario("UpdateRate",100,"ReferenceLocation",[0 0 0],"MaxNumFrames",50);

% Add marker for UAV start position
addMesh(Scenario,"cylinder",{[0 0 0.5], [0 1]},[0 1 0]);

%% Define UAVs and Initial Setup
c1 = 1.5 ;  % Cognitive coefficient
c2 = 1.0;  % Social coefficient
num_drones = 10;
InitialPosition = [0 0 -5];
InitialOrientation = [0 0 0];
positions = zeros(num_drones, 3);
platUAV = uavPlatform.empty(num_drones, 0); % Initialize empty array

for i = 1:num_drones
    positions(i, :) = InitialPosition + [rand*2, rand*2, 0];
    platUAV(i) = uavPlatform(sprintf("UAV%d",i),Scenario, ...
                            "ReferenceFrame","NED", ...
                            "InitialPosition", positions(i, :), ...
                            "InitialOrientation", eul2quat(InitialOrientation));
    % Add quadrotor mesh with proper transform
    updateMesh(platUAV(i), "quadrotor", {1.2}, [1.0, 0.0, 0.0], eul2tform([0 0 pi]));
end

%% Configure Lidar Sensor
for i = 1:num_drones
    % Create unique Lidar model for each UAV
    LidarModel = uavLidarPointCloudGenerator("UpdateRate",10, ...
                                           "MaxRange",7, ...
                                           "RangeAccuracy",3, ...
                                           "AzimuthResolution",0.5, ...
                                           "ElevationResolution",2, ...
                                           "AzimuthLimits",[-179 179], ...
                                           "ElevationLimits",[-15 15], ...
                                           "HasOrganizedOutput",true);
    
    % Mount sensor on UAV
    uavSensor("Lidar",platUAV(i),LidarModel, ...
              "MountingLocation",[0 0 -0.4], ...
              "MountingAngles",[0 0 180]);
end

%% Add Obstacles
ObstaclePositions = [10 0 0; 20 10 0; 10 20 0];
ObstacleHeight = 15;
ObstaclesWidth = 3;
for i = 1:size(ObstaclePositions,1)
    addMesh(Scenario,"polygon", ...
        {[ObstaclePositions(i,1)-ObstaclesWidth/2, ObstaclePositions(i,2)-ObstaclesWidth/2; ...
          ObstaclePositions(i,1)+ObstaclesWidth/2, ObstaclePositions(i,2)-ObstaclesWidth/2; ...
          ObstaclePositions(i,1)+ObstaclesWidth/2, ObstaclePositions(i,2)+ObstaclesWidth/2; ...
          ObstaclePositions(i,1)-ObstaclesWidth/2, ObstaclePositions(i,2)+ObstaclesWidth/2], ...
        [-ObstacleHeight 0]}, 0.651*ones(1,3));  % Changed height range to be negative for upward direction
end

%% Show Environment
% Remove show3D visualization
setup(Scenario);

%% Simulation Parameters
num_targets = 5;
grid_size = [30, 30, 15];
max_iterations = 300;
max_speed = 3;        % Increased for better movement
dt = 0.2;            % Smaller timestep for smoother motion
inertia = 0.8;       % Increased inertia for better momentum
c1 = 1.5;           % Personal best influence
c2 = 2.0;           % Global best influence
detection_radius = 2.0;  % Increased detection radius
obstacle_margin = 2.0;   % Safety margin for obstacles

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
        if norm(candidate(1:2) - targets(t,1:2)) < 5  % Minimum distance between targets (only check x,y)
            is_safe = false;
            break;
        end
    end
    if is_safe
        targets = [targets; candidate];
        % Add marker for each target with proper cylinder dimensions at ground level
        addMesh(Scenario, "cylinder", {[candidate(2) candidate(1) 0.5], [0 1]}, [1 0 0]);
    end
end

%% PSO-Based UAV Search
velocities = max_speed * (2 * rand(num_drones, 3) - 1);
pBest = positions;
pBestScores = inf(num_drones, 1);
gBest = zeros(1, 3);
gBestScore = inf;
targets_found = false(num_targets, 1);
rescue_times = NaN(num_targets, 1);
trajectories = cell(num_drones, 1);
for i = 1:num_drones
    trajectories{i} = positions(i, :);
end

% Create custom visualization
figure;
h = plot3(nan, nan, nan, 'b-');
hold on;

% Plot UAVs
uav_plots = gobjects(num_drones, 1);
for i = 1:num_drones
    uav_plots(i) = plot3(positions(i,2), positions(i,1), -positions(i,3), 'bo', 'MarkerFaceColor', 'b');
end

% Draw obstacles as rectangles
for i = 1:size(ObstaclePositions,1)
    x = ObstaclePositions(i,1);
    y = ObstaclePositions(i,2);
    w = ObstaclesWidth;
    h = ObstacleHeight;
    % Draw base
    vertices = [x-w/2, y-w/2, 0;
               x+w/2, y-w/2, 0;
               x+w/2, y+w/2, 0;
               x-w/2, y+w/2, 0;
               x-w/2, y-w/2, 0];
    plot3(vertices(:,2), vertices(:,1), zeros(size(vertices,1)), 'k-', 'LineWidth', 2);
    % Draw vertical edges with correct height direction (negative is up)
    for v = 1:4
        plot3([vertices(v,2) vertices(v,2)], ...
              [vertices(v,1) vertices(v,1)], ...
              [0 -h], 'k--', 'LineWidth', 1);  % Changed to plot upward
    end
    
    % Add top edges for better visualization
    top_vertices = [x-w/2, y-w/2, -h;
                   x+w/2, y-w/2, -h;
                   x+w/2, y+w/2, -h;
                   x-w/2, y+w/2, -h;
                   x-w/2, y-w/2, -h];
    plot3(top_vertices(:,2), top_vertices(:,1), top_vertices(:,3), 'k-', 'LineWidth', 1);
end

% Plot targets at ground level
scatter3(targets(:,2), targets(:,1), zeros(size(targets,1)), 100, 'r', '*', 'LineWidth', 2);

grid on;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
title('UAV Search Progress');
legend('UAVs', 'Obstacles', 'Targets');
view(3);
axis equal;

start_time = tic;
for iter = 1:max_iterations
    fprintf('Iteration %d/%d\n', iter, max_iterations);
    
    % Update personal and global best positions
    for d = 1:num_drones
        % Calculate fitness (distance to nearest unfound target)
        available_targets = targets(~targets_found, :);
        if ~isempty(available_targets)
            distances = vecnorm(available_targets - positions(d, :), 2, 2);
            current_score = min(distances);
            
            % Update personal best
            if current_score < pBestScores(d)
                pBestScores(d) = current_score;
                pBest(d, :) = positions(d, :);
            end
            
            % Update global best
            if current_score < gBestScore
                gBestScore = current_score;
                gBest = positions(d, :);
            end
        end
        
        % Check for target finding with increased detection radius
        for t = 1:num_targets
            if ~targets_found(t) && norm(positions(d,1:2) - targets(t,1:2)) < detection_radius  % Only check x,y distance
                targets_found(t) = true;
                rescue_times(t) = toc(start_time);
                fprintf('Target %d found at iteration %d (%.2f sec)\n', t, iter, rescue_times(t));
            end
        end
    end
    
    if all(targets_found)
        fprintf('All targets found in %.2f seconds!\n', max(rescue_times));
        break;
    end
    
    % Update velocities and positions
    for d = 1:num_drones
        available_targets = targets(~targets_found, :);
        if isempty(available_targets)
            continue;
        end
        
        % Find closest target (only consider x,y distance)
        distances = vecnorm(available_targets(:,1:2) - positions(d,1:2), 2, 2);
        [~, closest_target_idx] = min(distances);
        target_position = available_targets(closest_target_idx,:);
        target_position(3) = positions(d,3);  % Keep UAV's current height
        
        % Calculate repulsive force from obstacles
        repulsive_force = zeros(1, 3);
        for obs = 1:size(ObstaclePositions,1)
            obs_pos = ObstaclePositions(obs,:);
            diff_vec = positions(d,:) - [obs_pos(1:2), positions(d,3)];
            dist = norm(diff_vec);
            if dist < ObstaclesWidth * 2
                repulsive_force = repulsive_force + diff_vec/dist * (ObstaclesWidth * 2 - dist);
            end
        end
        
        % Update velocity with obstacle avoidance
        r1 = rand(1, 3);
        r2 = rand(1, 3);
        velocities(d, :) = inertia * velocities(d, :) + ...
            c1 * r1 .* (pBest(d, :) - positions(d, :)) + ...
            c2 * r2 .* (target_position - positions(d, :)) + ...
            0.5 * repulsive_force;  % Added repulsive force
        
        % Apply velocity constraints
        speed = norm(velocities(d, :));
        if speed > max_speed
            velocities(d, :) = (velocities(d, :) / speed) * max_speed;
        end
        
        % Update position
        next_pos = positions(d, :) + velocities(d, :) * dt;
        next_pos = max([1, 1, 1], min(grid_size, next_pos));
        
        % Check obstacle collision
        is_safe = true;
        for obs = 1:size(ObstaclePositions,1)
            obs_pos = ObstaclePositions(obs,:);
            if norm(next_pos(1:2) - obs_pos(1:2)) < ObstaclesWidth * obstacle_margin
                is_safe = false;
                break;
            end
        end
        
        if is_safe
            positions(d, :) = next_pos;
            trajectories{d} = [trajectories{d}; next_pos];
            
            % Update visualization
            set(uav_plots(d), 'XData', positions(d,2), 'YData', positions(d,1), 'ZData', -positions(d,3));
            % Draw trajectory
            plot3(trajectories{d}(:,2), trajectories{d}(:,1), -trajectories{d}(:,3), '-', 'Color', [0.8 0.8 1]);
        else
            % If collision detected, apply random perturbation to escape
            velocities(d, :) = max_speed * (2 * rand(1, 3) - 1);
        end
    end
    
    % Update UAV visualization in scenario
    for i = 1:num_drones
        if i <= numel(platUAV) && ~isempty(platUAV(i))
            motion = zeros(1, 16);
            ned_pos = [positions(i,1), positions(i,2), -positions(i,3)];
            ned_vel = [velocities(i,1), velocities(i,2), -velocities(i,3)];
            
            motion(1:3) = ned_pos;
            motion(4:6) = ned_vel;
            motion(7:9) = [0 0 0];
            motion(10:13) = eul2quat([0 0 0]);
            motion(14:16) = [0 0 0];
            
            move(platUAV(i), motion);
        end
    end
    advance(Scenario);
    drawnow;
    pause(0.05);
end

if any(~targets_found)
    fprintf('Max iterations reached. Some targets were not found.\n');
else
    fprintf('All targets found in %.2f seconds!\n', max(rescue_times));
end

