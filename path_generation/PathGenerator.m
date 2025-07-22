clf; close all; clear all;

% Load vehicle parameters
% Get the path to the current script
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
yamlFile = fullfile(projectRoot, 'vehicle_params.yaml');
vehicle_params = readyaml(yamlFile);

% Check if a configuration file exists (.mat file)
configFile = 'generator_config.mat';
if isfile(configFile)
    load(configFile, 'generator');
    delete(configFile); % Clean up the configuration file after loading
else
    % Load generation config
    config = readyaml('generation_config.yaml');

    generator = struct;
    generator.use_pre_load_map = config.use_pre_load_map;
    generator.total_paths_to_generate = config.total_paths_to_generate;
    generator.manual_path_generation = config.manual_path_generation;
    generator.testing = config.testing;
    generator.inflationRadius = config.inflation_radius;
    generator.generate_easy_path = strcmp(config.path_difficulty, 'easy');
    generator.generate_hard_path = strcmp(config.path_difficulty, 'hard');
    % Moderate will automatically selected if path_difficulty is neither easy nor hard
    % generator.generate_moderate_path = ~(generator.generate_easy_path || generator.generate_hard_path);

    % Prompt user to select map location
    disp('Select map location:');
    disp('1. Physics Building');
    disp('2. EE Building');
    % map_choice = input('Enter the number corresponding to the map location: ');
    % Set map location based on config.map_choice
    if config.map_choice == 1
        generator.map_location = 'physics_building';
    elseif config.map_choice == 2
        generator.map_location = 'ee_building';
    else
        error('Invalid map_choice in generation_config.yaml! Use 1 or 2.');
    end

    % Get current date, hour, and minute as a string (e.g., '20240427_1532')
    datetime_str = datestr(now, 'yyyymmdd_HHMM');

    % Determine the difficulty subfolder
    if generator.generate_hard_path
        difficulty_folder = 'hard';
    elseif generator.generate_easy_path
        difficulty_folder = 'easy';
    else
        difficulty_folder = 'moderate';
    end

    % Build the full folder path: generated_path/<datetime>/<difficulty>
    base_output_folder = fullfile(scriptDir, 'generated_path');
    datetime_folder = fullfile(base_output_folder, datetime_str);
    difficulty_folder_path = fullfile(datetime_folder, difficulty_folder);

    % Create the folders if they do not exist
    if ~exist(base_output_folder, 'dir')
        mkdir(base_output_folder);
    end
    if ~exist(datetime_folder, 'dir')
        mkdir(datetime_folder);
    end
    if ~exist(difficulty_folder_path, 'dir')
        mkdir(difficulty_folder_path);
    end

    % Set the saving path for generated paths
    generator.saving_path_folder = difficulty_folder_path;

    % Dump vehicle_params.yaml and generation_config.yaml into the <datetime> folder
    copyfile(fullfile(scriptDir, '../vehicle_params.yaml'), fullfile(datetime_folder, 'vehicle_params.yaml'));
    copyfile(fullfile(scriptDir, 'generation_config.yaml'), fullfile(datetime_folder, 'generation_config.yaml'));
end

% If folder does not exist, create it
if ~exist(generator.saving_path_folder, 'dir')
    mkdir(generator.saving_path_folder);
end

if generator.manual_path_generation
    generator.saving_path_folder = generator.saving_path_folder + "_manual";
end

% Load map based on selected location
if ~generator.use_pre_load_map
    %% Read Image Map
    if strcmp(generator.map_location, 'physics_building')
        image_map = imread(fullfile(scriptDir, 'map_file', 'physics_building_binary.png'));
    elseif strcmp(generator.map_location, 'ee_building')
        image_map = imread(fullfile(scriptDir, 'map_file', 'ee_building.png'));
    else
        error('Invalid map location');
    end
    image_map = image_map(:,:,1);
    imageOccupancy = 1 - image_map;

    map = occupancyMap(imageOccupancy, 20);
    map.GridLocationInWorld = [-50,-50];
    inflate(map, 0.8);
    % show(map);
    
    % Collision Checking Parameters
    % Car dimensions
    length_car = vehicle_params.length; % meters
    width = vehicle_params.width; % meters
    wheelbase = vehicle_params.wheelbase; % meters
    height = vehicle_params.height; % meters

    vehicle_dim = vehicleDimensions(length_car, width, height, 'Wheelbase', wheelbase);
    vehicle_collision_config = inflationCollisionChecker(vehicle_dim, 3);
    vehicle_collision_config.InflationRadius = generator.inflationRadius; %0.8 meters

    % State Validator
    validator = validatorOccupancyMap(stateSpaceSE2);
    validator.Map = map;
    mapData = occupancyMatrix(map);

    map = vehicleCostmap(imageOccupancy, 'CellSize', 0.05, 'mapLocation', [-50 -50], 'CollisionChecker',vehicle_collision_config);
    map_name = 'vehicleCostmap0.05_inflation' + string(vehicle_collision_config.InflationRadius) + '_' + generator.map_location + '.mat';
    save(map_name, 'map');
else
    map_name = 'vehicleCostmap0.05_inflation' + string(generator.inflationRadius) + '_' + generator.map_location + '.mat';
    map = load(map_name, 'map');
    map = map.map;
end

use_code_generation = false;

if use_code_generation
    % Code generation (Check if there is code generation folder)
    code_generation_folder = scriptDir;
    if ~exist(code_generation_folder, 'dir')
        map = vehicleCostmap(imageOccupancy, 'CellSize', 0.05, 'mapLocation', [-50 -50], 'CollisionChecker',vehicle_collision_config);    
        % Create a state space object
        stateSpace = stateSpaceSE2;        
        % Update state space bounds to be the same as map limits.
        stateSpace.StateBounds = [map.MapExtent(1:2); map.MapExtent(3:4) ;[-pi pi]];
        % Construct a state validator object using the statespace and map object
        sv = validatorVehicleCostmap(stateSpace,Map=map);
        sv.ValidationDistance = 0.01;
        startPose = [0 0 0]; % [meters, meters, radians]
        goalPose = [5 0 pi/4];
%        % Run code generation
        % codegen -config:mex codegenPathPlanner_rev -args {imageOccupancy,startPose,goalPose}
        codegen -config:mex codegenPathPlanner_rev -args {sv,startPose,goalPose}
    end
end

%% A* Hybrid Planner
% ss = stateSpaceSE2;
% ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
% sv = validatorOccupancyMap(ss);
% sv.Map = map;

if ~use_code_generation
    % Create a state space object
    stateSpace = stateSpaceSE2;

    % Hybrid A Star Params (Adjust these parameters to change the path generation if the vehicle parameters are changed)
    motionPrimitiveLength = 7.85398; % 8.7265; 
    numofMotionPrimitives = 21;
    interpolationDistance = 0.05; % Grid size cell
    current_min_turning_radius = vehicle_params.min_turning_radius; % 0.18
    
    % Update state space bounds to be the same as map limits.
    stateSpace.StateBounds = [map.MapExtent(1:2); map.MapExtent(3:4) ;[-pi pi]];
    
    % Construct a state validator object using the statespace and map object
    sv = validatorVehicleCostmap(stateSpace,Map=map);
    sv.ValidationDistance = 0.05;
    
    planner = plannerHybridAStar(sv, ...
                             MinTurningRadius=current_min_turning_radius, ...
                             MotionPrimitiveLength=motionPrimitiveLength, ... 
                             NumMotionPrimitives=numofMotionPrimitives, ...
                             InterpolationDistance=interpolationDistance);
end

if generator.testing
    startPose = [0 0 0]; % [meters, meters, radians]
    goalPose = [5 0 pi/3];

    refpath = plan(planner,startPose,goalPose,SearchMode='greedy');     

    x = refpath.States(:,1);
    y = refpath.States(:,2);

    x_new = linspace(min(x), max(x), 100);
    y_new = spline(x, y, x_new);

    show(planner)

    % Plot original path
    figure
    plot(x, y, 'r', 'LineWidth', 2)

    % Plot interpolated path
    hold on
    plot(x_new, y_new, 'b', 'LineWidth', 2)
    legend('Original Path', 'Interpolated Path')
    hold off

    % Calculate the curvature of the path
    convert_to_waypoints = @(x, y) [x(:), y(:)];
    waypoints = convert_to_waypoints(x, y);
    % Frenet Coordinate System
    refPathObj = referencePathFrenet(waypoints);
    % For each x, y point, calculate the curvature
    ch = CurvaturesHelper();
    curvatures = ch.calculate_curvature_using_circle(x, y);
    % Plot the path with the curvature
    figure
    plot(x, y, 'r', 'LineWidth', 2)
    hold on
    plot(x, y, 'bo')
    quiver(x, y, cos(refPathObj.Theta), sin(refPathObj.Theta), 'b')
    quiver(x, y, -sin(refPathObj.Theta), cos(refPathObj.Theta), 'g')
    hold off
    title('Path with Curvature')
    xlabel('X (m)')
    ylabel('Y (m)')
    axis equal
end

% If the folder does not exist, create it
if ~exist(generator.saving_path_folder, 'dir')
    mkdir(generator.saving_path_folder);
    total_path_generated = 0;
else
    % Check how many paths have been generated
    files = dir(fullfile(generator.saving_path_folder, '*.csv'));
    total_path_generated = size(files, 1);
    disp(['Total paths generated: ', num2str(total_path_generated)]);
end

% For filtering paths based on difficulty
generator.use_curvature_based_difficulty_filter = true;
ch = CurvaturesHelper();

%% Generate Path
while total_path_generated < generator.total_paths_to_generate
    % Defining Drivable Area for generatiing random start and goal points
    if strcmp(generator.map_location, 'ee_building')
        if generator.generate_hard_path
            mode = 1;
            if mode == 0
                % Starting pose (Bottom right corner)
                starting_pose_br = [28.3862210389997, -46.730297817433, 0.097952, 0.0, 0.0, 1.6459158149431];
                % Starting pose (Top left corner)
                starting_pose_tl = [-8, 20.044, 0.097952, 0.0, 0.0, -1.58];
                % Starting pose (Bottom left corner)
                starting_pose_bl = [-33.744, -12.153, 0.097952, 0.0, 0.0, -0.008];
                % Predefined initial position for generating hard paths
                starting_pose_locations = {starting_pose_br, starting_pose_tl, starting_pose_bl};
                % Get starting pose length
                leng_starting_pose = length(starting_pose_locations);
                starting_pose_location_choice = randi([0, leng_starting_pose - 1]);

                % Starting pose boundaries (X, Y)
                if starting_pose_location_choice == 0
                    % Starting pose boundaries (Bottom right corner)
                    starting_pose_boundaries = {[27.7, 31.0], [-45, -31], [pi/2 - 0.11, pi/2 + 0.11]};
                elseif starting_pose_location_choice == 1
                    % Starting pose boundaries (Top left corner)
                    starting_pose_boundaries = {[-10, -6], [13.35, 26], [-pi/2 - 0.26, -pi/2 + 0.26]};
                elseif starting_pose_location_choice == 2
                    % Starting pose boundaries (Bottom left corner)
                    starting_pose_boundaries = {[-39, -28], [-13.5, -10], [-pi/10, pi/10]};
                end

                % Goal poses
                % Goal pose (Bottom right corner)
                goal_pose_br = [-19.894691357716,-11.198840172631, 2.45431486246556];
                % Goal pose (Top left corner)
                goal_pose_tl = [28.3862210389997, -46.730297817433, 1.6459158149431];
                % Goal pose (Bottom left corner)
                goal_pose_bl = [-33.744, -12.153, -0.008];
                
                % Goal poses boundaries (X, Y)
                % Goal pose boundaries (Bottom right corner)
                if starting_pose_location_choice == 0
                    gp1 = {[-13,3, 10], [-14, -11.5], [pi - pi/8, pi + pi/8]};
                    gp2 = {[1.51, 15.4], [-1.69, 4.023], [pi - pi/8, pi + pi/8]};
                    goal_pose_boundaries = {gp1, gp2};
                    % Choose one of the two goal poses using random choice
                    goal_pose_choice = randi([0, 1]);
                    goal_pose_boundaries = goal_pose_boundaries{goal_pose_choice + 1};
                % Goal pose boundaries (Top left corner)
                elseif starting_pose_location_choice == 1
                    gp1 = {[9.17, 19.09], [-14, -11.5], [-pi/8, pi/8]};
                    gp2 = {[-36,-26], [-13.5, -10], [pi-pi/8, pi+pi/8]};
                    goal_pose_choice = randi([0, 1]);
                    goal_pose_boundaries = {gp1, gp2};
                    goal_pose_boundaries = goal_pose_boundaries{goal_pose_choice + 1};
                elseif starting_pose_location_choice == 2
                    goal_pose_boundaries = {[11.5, 23.8], [-1.26, 2.82], [-pi/7, pi/7]};
                    %goal_pose_boundaries = {[-10, -6], [13.35, 26], [-pi/2 - 0.26, pi/2 + 0.26]};
                end
                x_start = starting_pose_boundaries{1}(1) + (starting_pose_boundaries{1}(2) - starting_pose_boundaries{1}(1)) * rand;
                y_start = starting_pose_boundaries{2}(1) + (starting_pose_boundaries{2}(2) - starting_pose_boundaries{2}(1)) * rand;
                yaw_angle_start = starting_pose_boundaries{3}(1) + (starting_pose_boundaries{3}(2) - starting_pose_boundaries{3}(1)) * rand;

                start_point = [x_start, y_start, yaw_angle_start];

                % Sample a random position from the goal pose boundaries
                x_goal = goal_pose_boundaries{1}(1) + (goal_pose_boundaries{1}(2) - goal_pose_boundaries{1}(1)) * rand;
                y_goal = goal_pose_boundaries{2}(1) + (goal_pose_boundaries{2}(2) - goal_pose_boundaries{2}(1)) * rand;
                yaw_angle_goal = goal_pose_boundaries{3}(1) + (goal_pose_boundaries{3}(2) - goal_pose_boundaries{3}(1)) * rand;
            else
                % Hard Path Generation
                leng_starting_pose = 3;
                starting_pose_location_choice = randi([0, leng_starting_pose - 1]);
                if starting_pose_location_choice == 0
                    % Starting pose boundaries (X, Y, Yaw) (Middle of parking lot)
                    starting_pose_boundaries = {[4.52, 16.1], [-2.7, 3.0], [-pi/8, pi/8]};
                elseif starting_pose_location_choice == 1
                    % Starting pose boundaries (X, Y, Yaw) (Middle of parking lot, facing left)
                    starting_pose_boundaries = {[4.52, 16.1], [-2.7, 3.0], [pi - pi/8, pi + pi/8]};
                else
                    % Starting pose boundaries (X, Y, Yaw) (Upper part of parking lot)
                    starting_pose_boundaries = {[-6.5, -5.4], [5.7, 14.8], [-pi/2 - pi/10, -pi/2 + pi/10]};
                end

                % Goal pose boundaries (X, Y, Yaw)
                if starting_pose_location_choice == 0
                    goal_pose_total = 2;
                    choice_goal = randi([0, goal_pose_total - 1]);
                    % Goal pose boundaries (X, Y, Yaw) (Middle of parking lot)
                    if choice_goal == 0 % UTURN
                        goal_pose_boundaries = {[5, 13], [-14, -12.5], [pi - pi/8, pi + pi/8]};
                    else % PICKUP
                        goal_pose_boundaries = {[28.7, 30.3], [-29.1, -17.14], [-pi/2 - pi/10, -pi/2 + pi/10]};
                    end
                elseif starting_pose_location_choice == 1 || starting_pose_location_choice == 2
                    goal_pose_boundaries = {[6, 15], [-14.0, -11.5], [-pi/8, pi/8]};
                    if starting_pose_location_choice == 1
                        total_goal_choices = 2;
                        % Additional goal pose boundaries (Middle of parking lot, facing left)
                        choice_goal = randi([0, total_goal_choices - 1]);
                        if choice_goal == 0 % TURNING RIGHT TO UPPER LEFT PART
                            goal_pose_boundaries = {[-10, -5], [16, 26], [pi/2 - pi/9, pi/2 + pi/9]};
                        else
                            goal_pose_boundaries = goal_pose_boundaries;            
                        end
                    end
                end

                % Sample a random position from the starting pose boundaries
                x_start = starting_pose_boundaries{1}(1) + (starting_pose_boundaries{1}(2) - starting_pose_boundaries{1}(1)) * rand;
                y_start = starting_pose_boundaries{2}(1) + (starting_pose_boundaries{2}(2) - starting_pose_boundaries{2}(1)) * rand;
                yaw_angle_start = starting_pose_boundaries{3}(1) + (starting_pose_boundaries{3}(2) - starting_pose_boundaries{3}(1)) * rand;
                
                start_point = [x_start, y_start, yaw_angle_start];

                % Sample a random position from the goal pose boundaries
                x_goal = goal_pose_boundaries{1}(1) + (goal_pose_boundaries{1}(2) - goal_pose_boundaries{1}(1)) * rand;
                y_goal = goal_pose_boundaries{2}(1) + (goal_pose_boundaries{2}(2) - goal_pose_boundaries{2}(1)) * rand;
                yaw_angle_goal = goal_pose_boundaries{3}(1) + (goal_pose_boundaries{3}(2) - goal_pose_boundaries{3}(1)) * rand;
            end
        else
            % Moderate Path Generation
            choices_total_start = 4;
            choice = randi([0, choices_total_start - 1]);
            if choice == 0
                % Starting pose boundaries (X, Y, Yaw) (Middle of parking lot)
                starting_pose_boundaries = {[0.0, 15.0], [-1.5, 1.5], [-pi/8, pi/8]};
            elseif choice == 1
                % Starting pose boundaries (X, Y, Yaw) (Top left corner)
                starting_pose_boundaries = {[-10, -6], [13.35, 26], [-pi/2 - 0.26, -pi/2 + 0.26]};
            elseif choice == 2
                % Starting pose boundaries (X, Y, Yaw) (Bottom left corner)
                starting_pose_boundaries = {[-39, -28], [-13.5, -10], [-pi/10, pi/10]};
            else
                % Starting pose boundaries (X, Y, Yaw) (Bottom right corner)
                starting_pose_boundaries = {[27.7, 31.0], [-45, -31], [pi/2 - 0.11, pi/2 + 0.11]};
            end

            % Sample a random position from the starting pose boundaries
            x_start = starting_pose_boundaries{1}(1) + (starting_pose_boundaries{1}(2) - starting_pose_boundaries{1}(1)) * rand;
            y_start = starting_pose_boundaries{2}(1) + (starting_pose_boundaries{2}(2) - starting_pose_boundaries{2}(1)) * rand;
            yaw_angle_start = starting_pose_boundaries{3}(1) + (starting_pose_boundaries{3}(2) - starting_pose_boundaries{3}(1)) * rand;
            % Goal pose boundaries (X, Y, Yaw)
            line_length = 10.0;
            line_max_length = 15.0;
            line_distance = line_length + (line_max_length - line_length) * rand;
            % If yaw is positive then also add positive yaw to the initial yaw to make curve path
            if generator.generate_easy_path
                if yaw_angle_start > 0
                    yaw_to_add_range = [0, 0];
                else
                    yaw_to_add_range = [0, 0];
                end
            else
                if yaw_angle_start > 0
                    yaw_to_add_range = [pi/3, pi/6];
                else
                    yaw_to_add_range = [-pi/3, -pi/6];
                end
            end
            yaw_angle_goal = yaw_angle_start + yaw_to_add_range(1) + (yaw_to_add_range(2) - yaw_to_add_range(1)) * rand;
            projected_goal_points = [x_start + line_distance * cos(yaw_angle_start), y_start + line_distance * sin(yaw_angle_start), yaw_angle_start];
            x_goal = projected_goal_points(1);
            y_goal = projected_goal_points(2);
            if choice == 0
                % Goal pose boundaries (X, Y, Yaw) (Middle of parking lot)
                goal_pose_boundaries = {[0.0, 27.0], [-6.4, 6.5]};
            elseif choice == 1
                % Goal pose boundaries (X, Y, Yaw) (Top left corner)
                goal_pose_boundaries = {[-4, -10], [-5, 26]};
            elseif choice == 2
                % Goal pose boundaries (X, Y, Yaw) (Bottom left corner)
                goal_pose_boundaries = {[-39, -20], [-14, -10]};
            else
                % Goal pose boundaries (X, Y, Yaw) (Bottom right corner)
                goal_pose_boundaries = {[23, 32.0], [-30, -10]};
            end

            % Check if the goal points are within the goal pose boundaries
            x_goal = max(goal_pose_boundaries{1}(1), min(goal_pose_boundaries{1}(2), x_goal));
            y_goal = max(goal_pose_boundaries{2}(1), min(goal_pose_boundaries{2}(2), y_goal));
            start_point = [x_start, y_start, yaw_angle_start];
        end
    elseif strcmp(generator.map_location, 'physics_building')
        % Drivable Area
        % Define the areas with their identifiers
        areas = struct('id', {}, 'points', {});

        % Area 1 (Bottom Right Corner)
        areas(1).id = 1;
        areas(1).points = [28.48, -5.875; 29.075, -23.725; 31.38, -23.73; 31.43, -5.675];

        % Area 2 (Center bottom)
        areas(2).id = 2;
        areas(2).points = [-3.575, 1.425; -3.525, -2.825; 30.125, -2.075; 30.075, 1.425];

        % Area 3 (Center left)
        areas(3).id = 3;
        areas(3).points = [-9.275, 27.775; -5.875, 6.325; -3.725, 6.475; -7.075, 28.075];

        % Area 4 (Center Top)
        areas(4).id = 4;
        areas(4).points = [-15.68, 30.88; -15.22, 28.32; 38.125, 36.775; 37.575, 40.675];

        % Area 5 (Center right)
        areas(5).id = 5;
        areas(5).points = [28.875, 33.725; 28.82, 4.425; 31.73, 4.675; 32.175, 33.725];
    
        % Areas Heading (Correspond to clockwise direction, assuming the common driving direction for setting the orinetation before adding random heading)
        yaw_angles = [pi/2, pi, pi/2, 0, -pi/2]; % Area 1, Area 2, Area 3, Area 4, Area 5

        % Create random reverse direction flag
        reverse_direction = randi([0, 1]);

        % Reverse heading if reverse_direction is true
        if reverse_direction
            yaw_angles = -yaw_angles;
        end

        % Generate hard path
        if generator.generate_hard_path
            % Generate hard path
            % Randomly select the start area
            start_area_choice = randi([1, 5]);
            start_area = areas(start_area_choice);
            start_point = ch.samplePointFromArea(start_area);
            max_yaw_angle = pi/3;
            min_yaw_angle = -pi/3;
            yaw_angle_start = yaw_angles(start_area_choice) + min_yaw_angle + (max_yaw_angle - min_yaw_angle) * rand;

            % Randomly select the goal area
            goal_area_choice = randi([1, 5]);
            while goal_area_choice == start_area_choice
                goal_area_choice = randi([1, 5]);
            end
            goal_area = areas(goal_area_choice);
            goal_point = ch.samplePointFromArea(goal_area);
            
            % Set the start and goal points
            start_point = [start_point(1), start_point(2), yaw_angle_start];
            x_goal = goal_point(1);
            y_goal = goal_point(2);
            yaw_angle_goal = yaw_angles(goal_area_choice) + min_yaw_angle + (max_yaw_angle - min_yaw_angle) * rand;
        else
            % Generate easy path
            % Randomly select the area
            area_choice = randi([1, 5]);
            % Length of the line for the path
            line_length = 10.0;
            line_max_length = 15.0;
            % Randomly select the point within the area
            start_selected_point = ch.samplePointFromArea(areas(area_choice));
            x_start = start_selected_point(1);
            y_start = start_selected_point(2);
            if generator.generate_easy_path
                % Randomly select the point within the area
                % Small random yaw angle for easy path
                max_yaw_angle = pi/10;
                min_yaw_angle = -pi/10;
                line_distance = line_length + (line_max_length - line_length) * rand;
                yaw_angle_start = yaw_angles(area_choice) + min_yaw_angle + (max_yaw_angle - min_yaw_angle) * rand;
                yaw_angle_goal = yaw_angle_start; % Not adding random in easy path to keep it straight
                projected_goal_points = [x_start + line_distance * cos(yaw_angle_start), y_start + line_distance * sin(yaw_angle_start), yaw_angle_start];
                x_goal = projected_goal_points(1);
                y_goal = projected_goal_points(2);
            else
                % Randomly select the point within the area
                % Random yaw angle for moderate path
                max_yaw_angle = pi/3;
                min_yaw_angle = -pi/3;
                line_distance = line_length + (line_max_length - line_length) * rand;
                yaw_angle_start = yaw_angles(area_choice) + min_yaw_angle + (max_yaw_angle - min_yaw_angle) * rand;
                yaw_angle_goal = yaw_angle_start + min_yaw_angle + (max_yaw_angle - min_yaw_angle) * rand;
                projected_goal_points = [x_start + line_distance * cos(yaw_angle_start), y_start + line_distance * sin(yaw_angle_start), yaw_angle_start];
                x_goal = projected_goal_points(1);
                y_goal = projected_goal_points(2);
            end
            start_point = [x_start, y_start, yaw_angle_start];
        end
    else
        error('Invalid map location');
    end
    goal_point = [x_goal, y_goal, yaw_angle_goal];
    
    % Manual Path Generation
    if generator.manual_path_generation
        if generator.generate_easy_path
            % Easy Path
            if total_path_generated == 0
                start_point = [5.0, -13.0, 0.0];
                goal_point = [18.0, -13.0, 0.0];
            else
                start_point = [-7.5, 16.0, -pi/2];
                goal_point = [-7.5, 1.2, -pi/2];
            end
        elseif generator.generate_hard_path
            % Hard Path
            if total_path_generated == 0
                % start_point = [9.3, -13.0, 0.0];
                % goal_point = [30.0, -35.0, -pi/1.8];
                start_point = [-9.09359061856203, 16.923232391773, -1.45342545158202];
                goal_point = [-34.5479743320204, -12.5410439513444, -2.93338061227461];
            else
                start_point = [15.0, 1.0, -pi/2];
                goal_point = [29.5, -29.0, -pi/1.9];
            end
        end
    end

    % Generate path
    % Print current start and goal points
    disp(['Start Point: ', num2str(start_point)]);
    disp(['Goal Point: ', num2str(goal_point)]);
    
    % Check if the start point is valid
    if isStateValid(sv, goal_point)
        disp('Start Point is valid');
    else
        disp('Goal point is invalid');
        continue;
    end
    
    % Check if the goal point is valid
    if isStateValid(sv, goal_point)
        disp('Goal point is valid');
    else
        disp('Goal point is invalid');
        continue;
    end
    
    % Generate path
    try 
        if use_code_generation
            refpath = codegenPathPlanner_rev_mex(mapData, start_point, goal_point);
        else
            refpath = plan(planner, start_point, goal_point);
            refpath = refpath.States;
        end
    catch ME
        disp(ME.message);
        disp('Failed to generate path, finding new goal point');
        continue;
    end
    
    if ~isempty(refpath)
        % Extract path coordinates and yaw
        ref_x = refpath(:, 1);
        ref_y = refpath(:, 2);
        ref_yaw = refpath(:, 3);
        
        if generator.use_curvature_based_difficulty_filter
            % Handle cusp points in the path
            [ref_x, ref_y, ref_yaw] = ch.handleCuspPoints(ref_x, ref_y, ref_yaw);
            
            % Validate the difficulty of the path
            if generator.generate_easy_path
                expected_difficulty = 'easy';
            elseif generator.generate_hard_path
                expected_difficulty = 'hard';
            else
                expected_difficulty = 'moderate';
            end

            is_valid = ch.validate_difficulty(ref_x, ref_y, expected_difficulty);
            
            if is_valid
                total_path_generated = total_path_generated + 1;
                % Save the path as csv
                % Create a table from the data
                T = table(ref_x, ref_y, ref_yaw, 'VariableNames', {'ref_x', 'ref_y', 'ref_yaw'});
                % Write the table to a CSV file
                writetable(T, fullfile(generator.saving_path_folder, ['path_', num2str(total_path_generated), '.csv']));
                disp('Total path generated: ' + string(total_path_generated));
            else
                disp('Generated path does not match the expected difficulty criteria.');
                continue;
            end
        else
            total_path_generated = total_path_generated + 1;
            % Save the path as csv
            % Create a table from the data
            T = table(ref_x, ref_y, ref_yaw, 'VariableNames', {'ref_x', 'ref_y', 'ref_yaw'});
            % Write the table to a CSV file
            writetable(T, fullfile(generator.saving_path_folder, ['path_', num2str(total_path_generated), '.csv']));
            disp('Total path generated: ' + string(total_path_generated));
        end
    end
end