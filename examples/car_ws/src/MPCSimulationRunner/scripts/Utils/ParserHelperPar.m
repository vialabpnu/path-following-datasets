classdef ParserHelperPar < handle
    properties
        table_data;
        whole_path_table_data_no_filtering;
        % TODO: Variable for loading path data (done)
        path_data_from_file;
        % Directory for exporting metrics file
        plot_export_dir;
        metrics_export_dir;
        % Verdict of the path
        path_verdict;
        variable_sampling_total;
        base_types;
        % Joint table for the metrics
        joint_path_metrics_table;
        % Curvature Limits
        maximum_curve;
        % Table for difficulty mapping
        use_path_mapping_table;
        path_difficulty_table_mapping;
        difficulty_metrics_table;
        path_lists_struct;
        % Failure threshold 
        lateral_distance_max;
        % Fig file saving folder
        fig_file_saving_folder;
    end
    methods
        % Constructor for the ParserHelper class
        function obj = ParserHelperPar(plots_export_dir, metrics_export_dir, path_data_from_file, use_path_mapping_table)
            obj.table_data = table();
            obj.path_data_from_file = path_data_from_file;
            obj.metrics_export_dir = metrics_export_dir;
            obj.plot_export_dir = plots_export_dir;
            obj.variable_sampling_total = 8;
            obj.use_path_mapping_table = use_path_mapping_table;
            % Based on variable sampling total, generate the base types (DRL1, DRL2, ...)
            obj.base_types = [];
            obj.maximum_curve = 0.2;
            for i = 1:obj.variable_sampling_total
                obj.base_types = [obj.base_types; append("DRL", num2str(i))];
            end
            obj.joint_path_metrics_table = [];
            obj.lateral_distance_max = 1.0; %m
        end
        %% Setter for the table data %%
        function obj = set_table_data(obj, data)
            obj.table_data = data;
        end
        %% Setter for the Path Difficulty Mapping Table %%
        function obj = set_path_difficulty_mapping_table(obj, path_difficulty_table_mapping)
            obj.path_difficulty_table_mapping = path_difficulty_table_mapping;
        end
        %% Setter for the metrics export directory %%
        function obj = set_metrics_export_dir(obj, dir)
            obj.metrics_export_dir = dir;
        end
        %% Setter for the path data from file %%
        function obj = set_path_data_from_file(obj, path_data)
            obj.path_data_from_file = path_data;
        end
        %% Setter for the path verdict %%
        function obj = set_path_verdict(obj, path_verdict)
            % Check if the path_verdict is string or not
            if ~isstring(path_verdict) && ~ischar(path_verdict)
                error("The path_verdict must be a string")
            end

            % Remove the old path_verdict from the directories
            if (ischar(obj.path_verdict) || isstring(obj.path_verdict)) && ~isempty(obj.path_verdict)
                obj.plot_export_dir = replace(obj.plot_export_dir, obj.path_verdict, '');
                obj.metrics_export_dir = replace(obj.metrics_export_dir, obj.path_verdict, '');
            end
        
            % Set the new path_verdict
            obj.path_verdict = path_verdict;
        
            % Append the new path_verdict to the directories
            obj.plot_export_dir = append(obj.plot_export_dir, path_verdict, '/');
            obj.metrics_export_dir = append(obj.metrics_export_dir, path_verdict, '/');
            obj.fig_file_saving_folder = append(obj.plot_export_dir, 'fig_file');
            % If the directories do not exist, create them
            if ~exist(obj.plot_export_dir, 'dir')
                mkdir(obj.plot_export_dir)
            end
            if ~exist(obj.metrics_export_dir, 'dir')
                mkdir(obj.metrics_export_dir)
            end
            % Fig file saving folder
            if ~exist(obj.fig_file_saving_folder, 'dir')
                mkdir(obj.fig_file_saving_folder)
            end
        end
        %% Setter for prefix of figure export directory (same as path verdict setter but it can be set anything)
        function obj = set_plot_export_dir(obj, desired_dir_name)
            % Check if the desired_dir_name is string or not
            if ~isstring(desired_dir_name) && ~ischar(desired_dir_name)
                error("The desired_dir_name must be a string")
            end

            % Append the new desired name to the directories
            obj.plot_export_dir = append(obj.plot_export_dir, desired_dir_name, '/');
            
            % If the directories do not exist, create them
            if ~exist(obj.plot_export_dir, 'dir')
                mkdir(obj.plot_export_dir)
            end
        end
        %% Failure rate component checking
        function obj = get_failure_rate_detailed_proportion_value(obj, types)
            % Obtain the current table 
            table_data_all = obj.whole_path_table_data_no_filtering;
            % Struct for storing the failure rates for each type
            failure_percentage = struct();
            % For each type identify the reason of failure for the whole failure paths
            for i = 1:length(types)
                % Get the type
                current_type = types(i);
                % Get the data for the current type
                type_data = table_data_all(table_data_all.type == current_type, :);
                % Get the failure paths for the current type
                failure_paths = type_data(type_data.verdict == "failure", :);
                % Get the success paths count
                success_path_count = type_data(type_data.verdict == "success", :);
                % Get the total number of failure paths
                total_failure_paths = height(failure_paths);
                % Get total number of paths
                num_paths = height(type_data);
                total_success_paths = height(success_path_count);
                % Get the failure rate for each failure reason
                failures = struct('lateral_deviation', {}, 'collision', {}, 'both', {});
                % For each path that fails
                for j = 1:total_failure_paths
                    % Get the current path data
                    current_path_data = failure_paths(j, :);
                    % Get the failure reason for the current path
                    % Read the lateral deviation ("lateral_error_raw")
                    lateral_deviation = cell2mat(current_path_data.lateral_error_raw(1,:));
                    % Read the collision flag ("collision_flag")
                    collision_flag = cell2mat(current_path_data.collision_flag(1,:));
                    % Check if lateral deviation is greateer than the threshold
                    lateral_deviation_failure = any(lateral_deviation > obj.lateral_distance_max);
                    % Check the collision flag
                    collision_failure = any(collision_flag == 1);
                    % Both failure flag
                    lateral_collision_failure = lateral_deviation_failure & collision_failure;

                    failures(j).lateral_deviation = lateral_deviation_failure;
                    failures(j).collision = collision_failure;
                    failures(j).both = lateral_collision_failure;
                end
                % Calculate the percentage of each failure type
                percentage_lateral_deviation_failures = sum([failures.lateral_deviation]) / num_paths * 100;
                percentage_collision_failures = sum([failures.collision]) / num_paths * 100;
                percentage_both_failures = sum([failures.both]) / num_paths * 100;
                % Subtract the lateral deviation and collison with the intersection
                percentage_lateral_deviation_failures = percentage_lateral_deviation_failures - percentage_both_failures;
                percentage_collision_failures = percentage_collision_failures - percentage_both_failures;
                % Calcualte faiilure and success results
                percentage_of_failure = total_failure_paths / num_paths * 100;
                percentage_of_success = total_success_paths / num_paths * 100;
                % Store the failure rate for the current type
                failure_percentage.(current_type) = struct('lateral_devation_percentage', percentage_lateral_deviation_failures, ...
                    'collision_percentage', percentage_collision_failures, 'both_percentage', percentage_both_failures, 'failure_percentage', percentage_of_failure, ...
                    'success_percentage', percentage_of_success);
            end
            % Print the failure rates
            for i = 1:length(types)
                current_type = types(i);
                fprintf("Failure Rates for the Type: %s\n", current_type)
                fprintf("Failure Rate: %.2f\n", failure_percentage.(current_type).failure_percentage)
                fprintf("Lateral Deviation Failure Rate: %.2f\n", failure_percentage.(current_type).lateral_devation_percentage)
                fprintf("Collision Failure Rate: %.2f\n", failure_percentage.(current_type).collision_percentage)
                fprintf("Both Failure Rate: %.2f\n", failure_percentage.(current_type).both_percentage)
                fprintf("\n")
            end
        end
        %% Comparison of the success rate of the types
        function obj = compare_success_rate_for_each_type(obj, types)
            % Get the full lists of path table
            full_table = obj.whole_path_table_data_no_filtering;
            % Get the total path available by counting the unique path names
            total_paths = unique(full_table.path_name);
            total_paths_number = length(total_paths);
            % Filter the types list to only incude the type that has DRL on it
            types = types(contains(types, "DRL"));
            % From the types list create a comparison combination for each type
            comparison_combinations = nchoosek(types, 2);
            % Get the default plot export dir for setting the default
            default_export_dir = obj.plot_export_dir;
            % For each combinations get the comparison percentage
            for i = 1:height(comparison_combinations)
                % Get the current comparison combination
                current_combination = comparison_combinations(i, :);
                % Get the data for the first type
                first_type = current_combination(1);
                first_type_data = full_table(full_table.type == first_type, :);
                % Get the data for the second type
                second_type = current_combination(2);
                second_type_data = full_table(full_table.type == second_type, :);
                % Get the success paths for the first type
                first_type_success = first_type_data(first_type_data.verdict == "success", :);
                % Get the success paths for the second type
                second_type_success = second_type_data(second_type_data.verdict == "success", :);
                % Get the success paths on the first type but not in second type based on the path_name
                first_type_success_not_in_second = setdiff(first_type_success.path_name, second_type_success.path_name);
                % Get the success paths on the second type but not in first type based on the path_name
                second_type_success_not_in_first = setdiff(second_type_success.path_name, first_type_success.path_name);
                % Get the common success paths for both types
                common_success_paths = intersect(first_type_success.path_name, second_type_success.path_name);
                % Table for the success path in first type but not in second type
                % Obtain the indices of the rows in first_type_success where path_name is in first_type_success_not_in_second
                first_type_success_indices = ismember(first_type_success.path_name, first_type_success_not_in_second);
                % Get the subset of first_type_success that matches the condition
                first_type_success_subset = first_type_success(first_type_success_indices, :);
                % Table for the success path in second type but not in first type
                % Get the indices of the rows in second_type_success where path_name is in second_type_success_not_in_first
                second_type_success_indices = ismember(second_type_success.path_name, second_type_success_not_in_first);
                % Obtain the subset of second_type_success that matches the condition
                second_type_success_subset = second_type_success(second_type_success_indices, :);
                % Table for the common success paths
                % Get the indices of the rows in both types
                both_type_success_indices = ismember(full_table.path_name, common_success_paths);
                % Unfiltered both success path table
                both_type_success_table_unfiltered = full_table(both_type_success_indices, :);
                % Index that only have first_type and second type
                first_type_indices = ismember(both_type_success_table_unfiltered.type, first_type);
                second_type_indices = ismember(both_type_success_table_unfiltered.type, second_type);
                % Get the subset of the full table that matches the condition
                first_type_subset = full_table(first_type_indices, :);
                second_type_subset = full_table(second_type_indices, :);
                % Combine the subset of the first type and second type on the paths that success on both
                combined_subset = [first_type_subset; second_type_subset];

                % Get the percentage of each type
                first_type_percentage = height(first_type_success_subset) / total_paths_number * 100;
                second_type_percentage = height(second_type_success_subset) / total_paths_number * 100;
                % Get the percentage of the common success paths
                common_success_percentage = height(combined_subset) / (2 * total_paths_number) * 100;

                % Build a table that has the lists of success path on the first type and not in the second while getting the failure results in second types
                first_type_success_second_type_failure = ismember(full_table.path_name, first_type_success_not_in_second);
                first_type_success_second_type_failure_table = full_table(first_type_success_second_type_failure, :);
                % Only get the table that consists of first type and second type 
                first_type_success_second_type_failure_table = first_type_success_second_type_failure_table(ismember(first_type_success_second_type_failure_table.type, [first_type, second_type]), :);
                second_type_success_first_type_failure = ismember(full_table.path_name, second_type_success_not_in_first);
                second_type_success_first_type_failure_table = full_table(second_type_success_first_type_failure, :);
                % Only get the table that consists of first type and second type
                second_type_success_first_type_failure_table = second_type_success_first_type_failure_table(ismember(second_type_success_first_type_failure_table.type, [first_type, second_type]), :); 
                
                % Plotting the Current Types (for comparison)
                type_for_metrics = [first_type, second_type];
                path_name_for_metrics = ["path_323.csv"];
                % Set the plot export dir
                % For plotting the graph where first type is success but second type is not
                desired_dir_name = sprintf('%s_success_%s_not_success', first_type, second_type); 
                obj.set_plot_export_dir(desired_dir_name);
                % Set the table_data to the first_type_success_second_type_failure_table
                obj.set_table_data(first_type_success_second_type_failure_table);
                which_graphs = ["lateral_deviation_signed", "path_tracking", "plot_path_projection", "heading_error", "acceleration", "velocity", "steering_angle_rate"];
                % obj.plot_which_data(path_name_for_metrics, type_for_metrics, which_graphs, false, true, false); 
                obj.plot_export_dir = default_export_dir;
                
                % For Plotting the graph where second type is success but first type is not
                desired_dir_name = sprintf('%s_success_%s_not_success', second_type, first_type);
                obj.set_plot_export_dir(desired_dir_name);
                % Set the table_data to the second_type_success_first_type_failure_table
                obj.set_table_data(second_type_success_first_type_failure_table);
                % obj.plot_which_data(path_name_for_metrics, type_for_metrics, which_graphs, false, true, false); 
                obj.plot_export_dir = default_export_dir;

                % Print the comparison results
                fprintf("Comparison of the Success Rate for %s and %s\n", first_type, second_type)
                fprintf("Success Rate for %s that is not in %s: %.2f\n", first_type, second_type, first_type_percentage)
                fprintf("Total Success Path for %s and %s: %.2f\n", first_type, second_type, height(first_type_success_not_in_second))
                fprintf("Success Rate for %s that is not in %s: %.2f\n", second_type, first_type, second_type_percentage)
                fprintf("Total Success Path for %s and %s: %.2f\n", first_type, second_type, height(second_type_success_not_in_first))
                fprintf("Common Success Rate: %.2f\n", common_success_percentage)
                fprintf("Total Common Paths: %d\n", height(combined_subset) / 2)
                fprintf("\n")
            end
        end
        %% Classify the Type difficulty curvatures and sampling times based on the path difficulties
        function obj = classify_difficulty_metrics(obj)
            % Separate paths based on the difficulty
            difficulty_table = obj.difficulty_metrics_table;
            obj.path_lists_struct = struct();
            % Create a dictionary of difficulties
            difficulty_mapping = containers.Map(["E", "M", "H"], ["easy", "moderate", "hard"]);
            % Assign the path names to the struct according to the difficulty
            for i = 1:height(difficulty_table)
                path_name = difficulty_table.path_name(i);
                difficulty = difficulty_table.difficulty(i);
                difficulty = difficulty_mapping(difficulty);
                if ~isfield(obj.path_lists_struct, difficulty)
                    obj.path_lists_struct.(difficulty) = [];
                end
                obj.path_lists_struct.(difficulty) = [obj.path_lists_struct.(difficulty); path_name];
            end
        end
        %% Distribution plotter of sampling times for each difficulty
        function obj = plot_overall_sampling_time_distribution_all_types(obj, types, path_list_to_plot)
            % Plot for the entire path difficulties
            % Get the path lists for the current difficulty
            % Get the data for the current type
            type_data = obj.table_data(startsWith(obj.table_data.type, 'DRL'), :);            
            % Filter only types that has DRL
            % Extract the unique types that have 'DRL'
            types_to_consider = string(unique(type_data.type));

            % Difficulties 
            difficulty_table = obj.path_difficulty_table_mapping;
            difficulty_table = difficulty_table(ismember(difficulty_table.path_name, path_list_to_plot), :);
            difficulty_mapping = containers.Map(["E", "M", "H"], ["easy", "moderate", "hard"]);

            obj.path_lists_struct = struct();
            for i = 1:height(difficulty_table)
                path_name = difficulty_table.path_name(i);
                difficulty = difficulty_table.difficulty(i);
                difficulty = difficulty_mapping(string(difficulty));
                if ~isfield(obj.path_lists_struct, difficulty)
                    obj.path_lists_struct.(difficulty) = [];
                end
                obj.path_lists_struct.(difficulty) = [obj.path_lists_struct.(difficulty); path_name];
            end

            % Get the sampling times for the current type for each difficulty
            sampling_times = struct();
    
            % For each difficulty
            difficulties = fieldnames(obj.path_lists_struct);
            for d = 1:length(difficulties)
                difficulty = difficulties{d};
                path_lists = obj.path_lists_struct.(difficulty);
                
                % For each type in types_to_consider
                for i = 1:length(types_to_consider)
                    % Get the data for the current type
                    type_data = obj.table_data(strcmp(obj.table_data.type, types_to_consider{i}), :);
                    
                    % Initialize an array to hold the sampling times for the current type and difficulty
                    type_sampling_times = [];
                    
                    % For each path in the path list
                    for j = 1:length(path_lists)
                        path_name = path_lists{j};
                        path_data = type_data(strcmp(type_data.path_name, path_name), :);
                        % Get the sampling times for the current path
                        dt_array = path_data.dt_array{1};
                        % Remove brackets and convert to numbers
                        dt_array = cellfun(@(x) str2num(x(2:end-1)), dt_array, 'UniformOutput', false);
                        dt_array = cell2mat(dt_array);
                        type_sampling_times = [type_sampling_times; dt_array];
                    end
                    
                    % Assign the sampling times to the struct
                    if ~isfield(sampling_times, difficulty)
                        sampling_times.(difficulty) = struct();
                    end
                    sampling_times.(difficulty).(types_to_consider{i}) = type_sampling_times;
                end
            end

            % Obtain average sampling time for each dt array for all type for each difficulty
            for d = 1:length(difficulties)
                % Take averages for each element in sampling time for each type
                difficulty = difficulties{d};
                % FOr each type obtain average sampling time in curent diffiucltie
                for i = 1:length(types_to_consider)
                    % Get the current type
                    current_type = types_to_consider{i};
                    % Get the sampling times for the current type and difficulty
                    current_sampling_times = sampling_times.(difficulty).(current_type);
                    % Get the average sampling time for each dt array
                    average_sampling_times = mean(current_sampling_times, 1);
                    % Assign the average sampling times to the struct
                    sampling_times.(difficulty).(current_type) = average_sampling_times;
                end

                % Filter types to only include those with the prefix 'DRL'
                filtered_types = types_to_consider(startsWith(types_to_consider, 'DRL'));

                % Initialize variables for positions and labels
                bar_data = [];
                labels = {};
                group_counter = 0;

                % Collect data for bar plot
                for col = 1:size(sampling_times.(difficulty).(filtered_types{1}), 2)
                    temp_data = [];
                    for i = 1:length(filtered_types)
                        current_type = filtered_types{i};
                        average_sampling_times = sampling_times.(difficulty).(current_type);
                        % Collect average sampling times for the current column
                        temp_data = [temp_data; average_sampling_times(:, col)'];
                    end
                    bar_data = [bar_data; temp_data'];
                    labels{end+1} = sprintf('τ_{%d}', group_counter);
                    group_counter = group_counter + 1;
                end

                % Create bar plot
                figure;
                bar(bar_data);
                xticks(1:size(bar_data, 1));
                xticklabels(labels); % Label each group sequentially
                set(gca, 'XTickLabel', labels, 'FontSize', 14); % Adjust the font size as needed
                xlabel('Time Slot', 'FontSize', 16);
                ylabel('Average Sampling Intervals (s)', 'FontSize', 16);
                % title('Bar Plots for Each Type and Difficulty');
                legend(["HIDRL-MPC-6", "NUDRL-MPC"], 'Location', 'NorthWest', 'FontSize', 12);

                % Save files for the bar plot
                fig_name_path = "bar_plot_sampling_time_distribution.png";
                fig_name_export = strcat(difficulty, '_', fig_name_path);
                filename = append(obj.plot_export_dir, '/', fig_name_export);
                saveas(gcf, filename);
                % Close the figure
                close(gcf);
            end
        end
        %% Histogram Plotter for Plotting the Sampling Times Distribution 
        function obj = plot_overall_sampling_time_distribution(obj, types, paths_list_to_plot)
            % From the path difficulty_table filter the table to only have path that is in the paths_list_to_plot
            difficulty_table = obj.path_difficulty_table_mapping;
            difficulty_table = difficulty_table(ismember(difficulty_table.path_name, paths_list_to_plot), :);
            difficulty_mapping = containers.Map(["E", "M", "H"], ["easy", "moderate", "hard"]);
            mode_changes_limit = containers.Map(["easy", "moderate", "hard"], [1, 2, 3])
            % For each difficulty get the path lists based on the difficulty
            is_plot_sampling_time_per_difficulty = false;
            obj.path_lists_struct = struct();
            for i = 1:height(difficulty_table)
                path_name = difficulty_table.path_name(i);
                difficulty = difficulty_table.difficulty(i);
                difficulty = difficulty_mapping(string(difficulty));
                if ~isfield(obj.path_lists_struct, difficulty)
                    obj.path_lists_struct.(difficulty) = [];
                end
                obj.path_lists_struct.(difficulty) = [obj.path_lists_struct.(difficulty); path_name];
            end
            % For each types in the table data
            for i = 1:length(types)
                % For each difficulty in the path_lists_struct
                fprintf("Current Type is %s \n", types(i))
                if is_plot_sampling_time_per_difficulty
                    for difficulty = fieldnames(obj.path_lists_struct)'
                        % Get the path lists for the current difficulty
                        path_lists = obj.path_lists_struct.(difficulty{1});
                        % Get the data for the current type
                        type_data = obj.table_data(obj.table_data.type == types(i), :);
                        % Get the sampling times for the current type
                        sampling_times = [];
                        mode_changes_curvature = [];
                        for j = 1:length(path_lists)
                            path_name = path_lists(j);
                            path_data = type_data(type_data.path_name == path_name, :);
                            % Get the sampling times for the current path
                            dt_array = path_data.dt_array{1};
                            % Remove brackets and convert to numbers
                            dt_array = cellfun(@(x) str2num(x(2:end-1)), dt_array, 'UniformOutput', false);
                            dt_array = cell2mat(dt_array);
                            sampling_times = [sampling_times; dt_array];
                            % Get the mode curvature changes
                            mode_changes_curvature = [mode_changes_curvature; cell2mat(path_data.mode_changes_curvature)];
                        end
                        % For each unique mode changes curvature get the sampling times and plot the sampling times histogram
                        mode_changes_curvature = mode_changes_curvature(mode_changes_curvature <= mode_changes_limit(difficulty{1}));
                        unique_mode_changes_curvature = unique(mode_changes_curvature);
                        for k = 1:length(unique_mode_changes_curvature)
                            mode_change_curvature = unique_mode_changes_curvature(k);
                            % Get the sampling times for the current mode change curvature
                            sampling_times_mode_change = sampling_times(mode_changes_curvature == mode_change_curvature,:);

                            % The numbers you want to count
                            choices = [0.1, 0.2, 0.4, 0.8];

                            % Initialize a matrix to store the counts
                            counts = zeros(length(choices), size(sampling_times_mode_change, 2));

                            % For each column in the data
                            for j = 1:size(sampling_times_mode_change, 2)
                                % Count the occurrence of the choices in the column and normalize to get probabilities
                                counts(:, j) = histcounts(sampling_times_mode_change(:, j), [choices, max(choices)+1], 'Normalization', 'probability');
                            end

                            % Define the names for each column
                            column_names = arrayfun(@(n) ['ST' num2str(n)], 1:size(sampling_times_mode_change, 2), 'UniformOutput', false);

                            % Plot the grouped bar graph
                            figure();
                            b = bar(categorical(column_names), counts', 'BarWidth', 0.75, 'EdgeColor', 'k', 'LineWidth', 1.5);
                            title("Sampling Times of " + obj.get_label_name(types(i)) + " with Mode Change " + mode_change_curvature)
                            xlabel("Sampling Times Choices")
                            ylabel("Probability")
                            legend('0.1s', '0.2s', '0.4s', '0.8s', 'AutoUpdate', 'off'); 

                            % Change the color of the bars
                            for k = 1:length(b)
                                b(k).FaceColor = 'flat';
                            end
                            
                            % Save the histogram plot
                            fig_name_path = "sampling_time_distribution.png";
                            fig_name_export = strcat(types(i), '_', difficulty{1}, '_', "mode_change_curvature_", num2str(mode_change_curvature), '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            
                            % Close the figure
                            close(gcf);

                            %% Plot the total prediciton horizon
                            % Sum up the sampling times for each group
                            total_prediction_horizon = sum(sampling_times_mode_change, 2);

                            % Plot the histogram of the total prediction horizons
                            figure();
                            % Define the bin edges
                            maximum_horizon = max(choices) * obj.variable_sampling_total;
                            minimum_horizon = min(choices) * obj.variable_sampling_total;
                            % Calculate the number of bins using the square-root choice
                            num_bins = 14;
                            % Calculate the bin width
                            bin_width = (maximum_horizon - minimum_horizon) / num_bins;
                            bin_edges = minimum_horizon:bin_width:maximum_horizon;
                            histogram(total_prediction_horizon, bin_edges, 'Normalization', 'probability', 'EdgeColor', 'k', 'LineWidth', 0.75);
                            title("Total Prediction Horizon for " + obj.get_label_name(types(i)) + " with Mode Change " + mode_change_curvature)
                            xlabel("Total Prediction Horizon (s)")
                            ylabel("Probability")
                            legend("Total Prediction Horizon", 'Location', 'Best')
                            xlim([minimum_horizon, maximum_horizon]);
                            % Save the histogram plot
                            fig_name_path = "total_prediction_horizon_distribution.png";
                            fig_name_export = strcat(types(i), '_', difficulty{1}, '_', "mode_change_curvature_", num2str(mode_change_curvature), '_', "type_", types(i), '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);

                            % Close the figure
                            close(gcf);

                            % Stem Plot of Prediction Horizon
                            maximum_horizon = max(choices) * obj.variable_sampling_total;
                            minimum_horizon = min(choices) * obj.variable_sampling_total;
                            % Calculate the number of bins using the square-root choice
                            num_bins = 14;
                            % Calculate the bin width
                            bin_width = (maximum_horizon - minimum_horizon) / num_bins;
                            bin_edges = minimum_horizon:bin_width:maximum_horizon;
                            % histogram(total_prediction_horizon, bin_edges, 'Normalization', 'probability', 'EdgeColor', 'k', 'LineWidth', 0.75);
                            
                            % Increase the figure size
                            % figure('Position', [100 100 1200 600]);

                            % Calculate the unique values and their counts
                            [unique_values, ~, idx] = unique(total_prediction_horizon);
                            counts = accumarray(idx, 1);

                            % Calculate the probabilities
                            probabilities = counts / sum(counts);

                            % Create a stem plot
                            stem(unique_values, probabilities, 'Marker', 'none', 'LineWidth', 2.5);
                            % Decrease the font size
                            set(gca, 'FontSize', 9);
                            hold on;
                            % xlim([minimum_horizon - 0.1 * minimum_horizon, maximum_horizon + 0.1 * maximum_horizon]);

                            % Select the unique values with the top 3 probabilities
                            [~, idx] = sort(probabilities, 'descend');
                            top_candidates = 3;
                            if length(unique_values) < top_candidates
                                top_values = unique_values(idx(1:end)); % Select all the values
                            else
                                top_values = unique_values(idx(1:top_candidates)); % Select the top values
                            end

                            % Get the current x-ticks
                            current_xticks = get(gca, 'XTick');

                            % Ensure that both current_xticks and top_values are row vectors
                            current_xticks = current_xticks(:)';
                            top_values = top_values(:)';

                            % Add the top values to the current x-ticks
                            new_xticks = [current_xticks, top_values];
                            % Ensure the x-ticks are unique and sorted          
                            new_xticks = unique(sort(new_xticks));
                            % Set the x-ticks
                            xticks(new_xticks);

                            % Create labels for the x-ticks
                            xticklabels(arrayfun(@num2str, new_xticks, 'UniformOutput', false));
                            % Rotate the x-tick labels
                            xtickangle(45);

                            % Label the axes
                            xlabel('Prediction Horizon Total (s)');
                            ylabel('Probability');

                            % Add a title
                            title("Total Prediction Horizon for " + obj.get_label_name(types(i)) + " with Mode Change " + mode_change_curvature);
                            legend("Total Prediction Horizon", 'Location', 'Best')
                            % Save the stem plot
                            fig_name_path = "total_prediction_horizon_distribution_stem.png";
                            fig_name_export = strcat(types(i), '_', difficulty{1}, '_', "mode_change_curvature_", num2str(mode_change_curvature), '_', "type_", types(i), '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            close(gcf);
                        end
                    end
                else
                    % Plot for the entire path difficulties
                    % Get the path lists for the current difficulty
                    path_lists = path_name;
                    % Get the data for the current type
                    type_data = obj.table_data(obj.table_data.type == types(i), :);
                    % Get the sampling times for the current type
                    sampling_times = [];
                    mode_changes_curvature = [];
                    for j = 1:length(path_lists)
                        path_name = path_lists(j);
                        path_data = type_data(type_data.path_name == path_name, :);
                        % Get the sampling times for the current path
                        dt_array = path_data.dt_array{1};
                        % Remove brackets and convert to numbers
                        dt_array = cellfun(@(x) str2num(x(2:end-1)), dt_array, 'UniformOutput', false);
                        dt_array = cell2mat(dt_array);
                        sampling_times = [sampling_times; dt_array];
                        % Get the mode curvature changes
                        mode_changes_curvature = [mode_changes_curvature; cell2mat(path_data.mode_changes_curvature)];
                    end
                    % Get the distribution of sampling times for the current type
                    % The numbers you want to count
                    choices = [0.1, 0.2, 0.4, 0.8];

                    % Initialize a matrix to store the counts
                    counts = zeros(length(choices), size(sampling_times, 2));

                    % For each column in the data
                    for j = 1:size(sampling_times, 2)
                        % Count the occurrence of the choices in the column and normalize to get probabilities
                        counts(:, j) = histcounts(sampling_times(:, j), [choices, max(choices)+1], 'Normalization', 'probability');
                    end

                    % Define the names for each column
                    column_names = arrayfun(@(n) ['T' num2str(n-1)], 1:size(sampling_times, 2), 'UniformOutput', false);

                    % Plot the grouped bar graph
                    figure();
                    b = bar(categorical(column_names), counts', 'BarWidth', 0.75, 'EdgeColor', 'k', 'LineWidth', 1.5);
                    % title("Sampling Times of " + obj.get_label_name(types(i)) + " with Mode Change " + mode_change_curvature)
                    xlabel("Sampling Times Choices Distribution")
                    ylabel("Probability")
                    legend('0.1s', '0.2s', '0.4s', '0.8s', 'AutoUpdate', 'off'); 
                    
                    % Set font size
                    set(gca, 'FontSize', 12); % Set the font size for the axes
                    set(get(gca, 'XLabel'), 'FontSize', 14); % Set the font size for the x-axis label
                    set(get(gca, 'YLabel'), 'FontSize', 14); % Set the font size for the y-axis label
                    set(get(gca, 'Title'), 'FontSize', 16); % Set the font size for the title
                    set(legend, 'FontSize', 12); % Set the font size for the legend

                    % Change the color of the bars
                    for k = 1:length(b)
                        b(k).FaceColor = 'flat';
                    end
                    
                    % Save the histogram plot
                    fig_name_path = "sampling_time_distribution_overall.png";
                    fig_name_export = strcat(types(i), '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    
                    % Close the figure
                    close(gcf);

                    %% Plot the total prediciton horizon
                    % Sum up the sampling times for each group
                    total_prediction_horizon = sum(sampling_times, 2);

                    % Plot the histogram of the total prediction horizons
                    figure();
                    % Define the bin edges
                    maximum_horizon = max(choices) * obj.variable_sampling_total;
                    minimum_horizon = min(choices) * obj.variable_sampling_total;
                    % Calculate the number of bins using the square-root choice
                    num_bins = 14;
                    % Calculate the bin width
                    bin_width = (maximum_horizon - minimum_horizon) / num_bins;
                    bin_edges = minimum_horizon:bin_width:maximum_horizon;
                    histogram(total_prediction_horizon, bin_edges, 'Normalization', 'probability', 'EdgeColor', 'k', 'LineWidth', 0.75);
                    % title("Total Prediction Horizon for " + obj.get_label_name(types(i)) + " with Mode Change " + mode_change_curvature)
                    xlabel("Total Prediction Horizon (s)")
                    ylabel("Probability")
                    legend("Total Prediction Horizon", 'Location', 'Best')
                    xlim([minimum_horizon, maximum_horizon]);
                    
                    % Set font size
                    set(gca, 'FontSize', 12); % Set the font size for the axes
                    set(get(gca, 'XLabel'), 'FontSize', 14); % Set the font size for the x-axis label
                    set(get(gca, 'YLabel'), 'FontSize', 14); % Set the font size for the y-axis label
                    set(get(gca, 'Title'), 'FontSize', 16); % Set the font size for the title
                    set(legend, 'FontSize', 12); % Set the font size for the legend

                    % Save the histogram plot
                    fig_name_path = "total_prediction_horizon_distribution.png";
                    fig_name_export = strcat(types(i), '_', "type_", types(i), '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);

                    % Close the figure
                    close(gcf);

                    % Stem Plot of Prediction Horizon
                    maximum_horizon = max(choices) * obj.variable_sampling_total;
                    minimum_horizon = min(choices) * obj.variable_sampling_total;
                    % Calculate the number of bins using the square-root choice
                    num_bins = 14;
                    % Calculate the bin width
                    bin_width = (maximum_horizon - minimum_horizon) / num_bins;
                    bin_edges = minimum_horizon:bin_width:maximum_horizon;
                    % histogram(total_prediction_horizon, bin_edges, 'Normalization', 'probability', 'EdgeColor', 'k', 'LineWidth', 0.75);
                    
                    % Increase the figure size
                    % figure('Position', [100 100 1200 600]);

                    % Calculate the unique values and their counts
                    [unique_values, ~, idx] = unique(total_prediction_horizon);
                    counts = accumarray(idx, 1);

                    % Calculate the probabilities
                    probabilities = counts / sum(counts);

                    % Create a stem plot
                    stem(unique_values, probabilities, 'Marker', 'none', 'LineWidth', 2.5);
                    % Decrease the font size
                    set(gca, 'FontSize', 9);
                    hold on;
                    % xlim([minimum_horizon - 0.1 * minimum_horizon, maximum_horizon + 0.1 * maximum_horizon]);

                    % Select the unique values with the top 3 probabilities
                    [~, idx] = sort(probabilities, 'descend');
                    top_candidates = 3;
                    if length(unique_values) < top_candidates
                        top_values = unique_values(idx(1:end)); % Select all the values
                    else
                        top_values = unique_values(idx(1:top_candidates)); % Select the top values
                    end

                    % Get the current x-ticks
                    current_xticks = get(gca, 'XTick');

                    % Ensure that both current_xticks and top_values are row vectors
                    current_xticks = current_xticks(:)';
                    top_values = top_values(:)';

                    % Add the top values to the current x-ticks
                    new_xticks = [current_xticks, top_values];
                    % Ensure the x-ticks are unique and sorted          
                    new_xticks = unique(sort(new_xticks));
                    % Set the x-ticks
                    xticks(new_xticks);

                    % Create labels for the x-ticks
                    xticklabels(arrayfun(@num2str, new_xticks, 'UniformOutput', false));
                    % Rotate the x-tick labels
                    xtickangle(45);

                    % Label the axes
                    xlabel('Prediction Horizon Total (s)');
                    ylabel('Probability');

                    % Add a title
                    % title("Total Prediction Horizon for " + obj.get_label_name(types(i)));
                    legend("Total Prediction Horizon", 'Location', 'Best')

                    set(gca, 'FontSize', 12); % Set the font size for the axes
                    set(get(gca, 'XLabel'), 'FontSize', 14); % Set the font size for the x-axis label
                    set(get(gca, 'YLabel'), 'FontSize', 14); % Set the font size for the y-axis label
                    set(get(gca, 'Title'), 'FontSize', 16); % Set the font size for the title
                    set(legend, 'FontSize', 12); % Set the font size for the legend

                    % Save the stem plot
                    fig_name_path = "total_prediction_horizon_distribution_stem.png";
                    fig_name_export = strcat(types(i), '_', "type_", types(i), '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    close(gcf);
                end
            end
        end

        % Update table to have the curvature mode change per time step in the filtered results for each type
        function obj = get_mode_changes_of_curvature(obj)
            mode_change_curvature_thres = 0.04;
            mode_counts = {};
            % For each index in the table data
            table_data = obj.table_data;
            for i = 1:height(table_data)
                curv_data_all = [];
                % Get the curvature data (path_curv, path_curv_1, ..., path_curv_7)
                for j = 0:obj.variable_sampling_total-1
                    if j == 0
                        curv_name = 'path_curv';
                    else
                        curv_name = ['path_curv_', num2str(j)];
                    end
                    curv_data = table_data.(curv_name)(i);
                    % Concatenate the data from the current curvature into curv_data_all
                    curv_data_all = [curv_data_all, curv_data];
                end  
                curv_data_2d = [];
                % Loop over each cell in curv_data_all
                for k = 1:numel(curv_data_all)
                    % Concatenate the data from the current cell into curv_data_2d
                    curv_data_2d = [curv_data_2d, curv_data_all{k}];
                end
                % Get the mode of the curvature data
                curv_changes = diff(curv_data_2d, 1, 2);
                mode_count = sum(abs(curv_changes) > mode_change_curvature_thres, 2);
                % Stacking the mode counts
                mode_counts = [mode_counts; {mode_count}];
            end                
            % Add the mode counts to the table data
            obj.table_data = addvars(obj.table_data, mode_counts, 'After', 'path_curv_7', 'NewVariableNames', 'mode_changes_curvature');
        end
        %% Function to plot and get the data for the specified path %%
        function done = plot_which_data(obj, path_lists_to_plot, type_lists, which_graphs, plot_all_graphs, plot_all_paths, verbose)
            % Set default values for the plot_all_graphs and plot_all_paths
            if nargin < 5 || isempty(plot_all_graphs)
                plot_all_graphs = false;
            end
            if nargin < 6 || isempty(plot_all_paths)
                plot_all_paths = false;
            end
            if nargin < 7 || isempty(verbose)
                verbose = false;
            end
            % which_graphs options: 'path_tracking', 'horizon_changes', 'steering', 'heading', 'speed', 'curvature'
            % Check which graph to plot
            % Define the list of graphs
            if plot_all_graphs
                graph_list = ["path_tracking", "horizon_changes", "steering", "heading", ...
                            "box_plot_error", "velocity", "acceleration", "curvature", "reward_function", ...
                            "velocity_versus_distance", "heading_versus_distance", "error_over_time", "velocity_heading_curvature_in_one", ...
                            "steering_versus_distance", "steering_angle_rate", "steering_velocity_in_one", "heading_curvature_in_one", "reward_function", ...
                            "plot_path_projection", "sampling_time_spikes", "sampling_time_distribution", "objective_function", "objective_function_components", "lateral_deviation_signed", "heading_error",
                            "problem_status", "path_tracking_paper", "velocity_heading_in_one", "sampling_time_average"];
            else
                graph_list = which_graphs;
            end
            if plot_all_paths
                path_lists_to_plot = obj.table_data.path_name;
                % Get the unique path names
                path_lists_to_plot = unique(path_lists_to_plot, 'stable');
            end
            % Check if each graph is in the list
            is_plot_path_tracking = ismember("path_tracking", graph_list);
            is_plot_path_tracking_paper = ismember("path_tracking_paper", graph_list);
            is_plot_horizon_changes = ismember("horizon_changes", graph_list);
            is_plot_steering = ismember("steering", graph_list);
            is_plot_steering_angle_rate = ismember("steering_angle_rate", graph_list);
            is_plot_heading = ismember("heading", graph_list);
            is_plot_box_plot_error = ismember("box_plot_error", graph_list);
            is_plot_velocity = ismember("velocity", graph_list);
            is_plot_curvature = ismember("curvature", graph_list);
            is_plot_velocity_in_distance = ismember("velocity_versus_distance", graph_list);
            is_plot_heading_in_distance = ismember("heading_versus_distance", graph_list);
            is_plot_steering_in_distance = ismember("steering_versus_distance", graph_list);
            is_plot_steering_velocity_in_one = ismember("steering_velocity_in_one", graph_list);
            is_plot_velocity_heading_in_one = ismember("velocity_heading_in_one", graph_list);
            is_plot_heading_curvature_in_one = ismember("heading_curvature_in_one", graph_list);
            is_plot_velocity_heading_curvature_in_one = ismember("velocity_heading_curvature_in_one", graph_list);
            is_plot_error_over_time = ismember("error_over_time", graph_list);
            is_plot_reward_function = ismember("reward_function", graph_list);
            is_plot_path_projection = ismember("plot_path_projection", graph_list);
            is_plot_objective_function = ismember("objective_function", graph_list);
            is_plot_objective_function_components = ismember("objective_function_components", graph_list);
            is_plot_acceleration = ismember("acceleration", graph_list);
            is_plot_sampling_time_spikes = ismember("sampling_time_spikes", graph_list);
            is_plot_sampling_time_distribution = ismember("sampling_time_distribution", graph_list);
            is_plot_lateral_error_over_time = ismember("lateral_deviation_signed", graph_list);
            is_plot_heading_error_over_time = ismember("heading_error", graph_list);
            is_plot_problem_status = ismember("problem_status", graph_list);
            is_plot_sampling_time_average_per_difficulty = ismember("sampling_time_average", graph_list);
            type_lists = string(type_lists);
            
            % Check if the path_lists_to_plot is a string or a string array, if not raise an error
            if ~isstring(path_lists_to_plot) && ~iscellstr(path_lists_to_plot)
                error("The path_lists_to_plot must be a string or a string array")
            end 

            parfor i = 1:length(path_lists_to_plot)
                try
                    % Get the path name
                    current_path = path_lists_to_plot(i);
                    if ~plot_all_paths
                        % Check if the current path is exist in the table data if not raise an error
                        if ~ismember(current_path, obj.table_data.path_name)
                            error("The path name is not exist in the table data")
                        end
                    end
                    % Plot the path tracking data
                    if is_plot_path_tracking
                        obj.plot_path_tracking_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the path tracking data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the path tracking data for the paper
                    if is_plot_path_tracking_paper
                        obj.plot_path_tracking_data(current_path, type_lists, true);
                        if verbose
                            fprintf("Plotting the path tracking data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the horizon data
                    if is_plot_horizon_changes
                        obj.plot_horizon_data(current_path, type_lists, true);
                        if verbose
                            fprintf("Plotting the horizon data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the steering data
                    if is_plot_steering 
                        obj.plot_steering_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the steering data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the heading data
                    if is_plot_heading
                        obj.plot_heading_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the heading data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the velocity data
                    if is_plot_velocity
                        obj.plot_velocity_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the velocity data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the velocity data categorized
                    if is_plot_velocity_in_distance
                        obj.plot_velocity_data_categorized(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the velocity data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the heading data categorized
                    if is_plot_heading_in_distance
                        obj.plot_heading_data_categorized(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the heading data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the steering data categorized
                    if is_plot_steering_in_distance
                        obj.plot_steering_data_categorized(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the steering data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the steering rate data
                    if is_plot_steering_angle_rate
                        obj.plot_steering_rate_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the steering rate data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the velocity and steering data
                    if is_plot_steering_velocity_in_one
                        obj.plot_velocity_steering_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the velocity and steering data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the velocity and heading data in one
                    if is_plot_velocity_heading_in_one
                        obj.plot_velocity_heading_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting Velocity and Heading data")
                        end
                    end
                    % Plot the heading and curvature data
                    if is_plot_heading_curvature_in_one
                        obj.plot_heading_curvature_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the heading and curvature data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the velocity, heading and curvature data
                    if is_plot_velocity_heading_curvature_in_one
                        obj.plot_velocity_heading_curvature_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the velocity, heading and curvature data in distance for the path: %s\n", current_path)
                        end
                    end
                    % Plot the acceleration
                    if is_plot_acceleration
                        obj.plot_accel_rate_in_one(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the acceleration data for the path: %s\n", current_path)
                        end
                    end
                    % Plot the error over time
                    if is_plot_error_over_time
                        obj.plot_error_over_time(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the error over time for the path: %s\n", current_path)
                        end
                    end
                    % Plot Projection of the path
                    if is_plot_path_projection
                        obj.plot_path_projection_data(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the path projection for the path: %s\n", current_path)
                        end
                    end
                    % Plot Objective function
                    if is_plot_objective_function
                        obj.plot_objective_function(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the objective function for the path: %s\n", current_path)
                        end
                    end
                    % Plot the objective function components
                    if is_plot_objective_function_components
                        obj.plot_objective_function_components_proportion(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the objective function components for the path: %s\n", current_path)
                        end
                    end
                    % Plot Sampling Time Spikes
                    if is_plot_sampling_time_spikes
                        obj.plot_tracking_with_sampling_time_spikes_marker(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the tracking with sampling time spikes for the path: %s\n", current_path)
                        end
                    end
                    % Plot the objective function
                    if is_plot_objective_function
                        obj.plot_objective_function(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the objective function for the path: %s\n", current_path)
                        end
                    end
                    % Plot the lateral deviation signed
                    if is_plot_lateral_error_over_time
                        obj.plot_lateral_error_over_time(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the Signed Lateral Deviation for the path: %s\n", current_path)
                        end
                    end
                    % Plot the heading error
                    if is_plot_heading_error_over_time
                        obj.plot_heading_error_over_time(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the Heading Error for the path:%s\n", current_path)
                        end
                    end
                    % Plot the problem status
                    if is_plot_problem_status
                        obj.plot_solution_status(current_path, type_lists);
                        if verbose
                            fprintf("Plotting the Problem Status for the path: %s\n", current_path)
                        end
                    end
                    % Plot the curvature data
                    % TODO: Add the curvature data
                    % if is_plot_curvature
                    %     obj.plot_curvature_data(current_path, type_lists);
                    % end
                    % Plot the box plot error
                    if is_plot_box_plot_error
                        obj.plot_box_plot_error(current_path, type_lists);
                    end

                catch ME
                    fprintf("Error in plotting the data for the path: %s\n", current_path)
                    fprintf("Error Message: %s\n", ME.message)
                    fprintf("Error Identifier: %s\n", ME.identifier)
                    fprintf("Error Report:\n%s\n", getReport(ME))
                end
            end
            if is_plot_sampling_time_distribution
                obj.plot_overall_sampling_time_distribution(type_lists, path_lists_to_plot);
            end
            if is_plot_sampling_time_average_per_difficulty
                obj.plot_overall_sampling_time_distribution_all_types(type_lists, path_lists_to_plot);
            end
            if is_plot_reward_function
                obj.plot_reward_function(4, 5, 2, 8);
            end
            done = true;
            fprintf("Done plotting and evaluating the trajectories data\n")
        end
        % Function to plot error over time
        function obj = plot_error_over_time(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            % For plotting the data for the specified type
            label = [];
            figure()
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    grid on
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the error data
                    error_data = cell2mat(current_data.error_over_time(1,:));
                    % Get the odom data
                    odom_x = cell2mat(current_data.odom_x(1,:));
                    odom_y = cell2mat(current_data.odom_y(1,:));
                    % Get the path data
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    error_from_path = sqrt((odom_x-path_x).^2 + (odom_y-path_y).^2);

                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    plot(time_data, error_data, '-', 'LineWidth', 1.0);
                    hold on;
                    label = [label; obj.get_label_name(current_type)];
                end
            end
            % plot(time_data, error_from_path, '--', 'LineWidth', 1.0);
            title("Error Over Time")
            xlabel("Time (s)")
            ylabel("Error (m)")

            legend(label, 'Location', 'Best')
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "error_over_time.png";
            fig_name_export = append(prefix_path_name, "_", fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot the velocity, heading and curvature data in one plot
        function obj = plot_velocity_heading_curvature_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the target velocity data
                    target_velocity_data = cell2mat(current_data.path_vel_1(1,:));
                    % Get the velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    % Get the path heading data
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % Get the heading data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    % Get the path curvature data
                    path_curvature_data = cell2mat(current_data.path_curv(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure()
                    % Subplot for the velocity data
                    subplot(3,1,1)
                    plot(time_data, target_velocity_data, '-', 'LineWidth', 1.0);
                    hold on
                    plot(time_data, velocity_data, '-', 'LineWidth', 1.0);
                    title("Velocity Profile")
                    xlabel("Time Step")
                    ylabel("Velocity (m/s)")
                    legend("Target Velocity", "Velocity", 'Location', 'Best')
                    % Subplot for the heading data
                    subplot(3,1,2)
                    plot(time_data, path_heading_data, '-', 'LineWidth', 1.0);
                    hold on
                    plot(time_data, heading_data, '-', 'LineWidth', 1.0);
                    title("Heading")
                    xlabel("Time Step")
                    ylabel("Heading (rad)")
                    legend("Path Heading", "Heading", 'Location', 'Best')
                    % Subplot for the curvature data
                    subplot(3,1,3)
                    plot(time_data, path_curvature_data, '-', 'LineWidth', 1.0);
                    title("Curvature")
                    xlabel("Time Step")
                    ylabel("Curvature (1/m)")
                    legend("Path Curvature", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "velocity_heading_curvature_in_one.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        %% PLotter for solution status in MPC results
        function obj = plot_solution_status(obj, path_name, type_lists)  
            % Create mapping of status into integer
            status_mapping = containers.Map(["undefined", "optimal", "infeasible", "unbounded", "optimal_inaccurate", "infeasible_inaccurate", "unbounded_inaccurate"], [0, 1, 2, 3, 4, 5, 6]);
            % Get the index of current 
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    cur_data = obj.table_data(cur_idx, :);
                    problem_status = cur_data.problem_status;
                    problem_status = string(problem_status{1});
                    % Map to the number
                    num_status = cellfun(@(x) status_mapping(x), problem_status);
                    % Get the time data
                    time_data = cur_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    % Plot the solution status (in discrete way)
                    figure()
                    stairs(time_data, num_status, 'LineWidth', 1.5);
                    title("Solution Status")
                    xlabel("Time Step")
                    ylabel("Solution Number")
                    % Add text legend about the number mapping
                    % Add legend that tells about the map
                    str = {'1: Optimal', '2: Infeasible', '3: Unbounded', '4: Optimal Inaccurate', '5: Infeasible Inaccurate', '6: Unbounded Inaccurate'};
                    annotation('textbox', [0.5, 0.7, 0.2, 0.2], 'String', str, 'FitBoxToText', 'on');

                    % Save the image
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "solution_status.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function to plot trajectory and mark the sudden spikes in the sampling times in the trajectory plot
        function obj = plot_tracking_with_sampling_time_spikes_marker(obj, path_name, type_lists)
            % Mark the x,y point that has sudden curvature changes and also get the curvature plots
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            path_name_file = append(obj.path_data_from_file,'/', path_name, '.csv');
            path_data = readtable(path_name_file);
            path_x_data = table2array(path_data(:,1));
            path_y_data = table2array(path_data(:,2));
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    cur_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(cur_data.path_x(1,:));
                    path_y = cell2mat(cur_data.path_y(1,:));
                    data_len = length(cur_data.dt_array{1});
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    odom_x = cell2mat(cur_data.odom_x(1,:));
                    odom_y = cell2mat(cur_data.odom_y(1,:));
                    skip_points = 12;
                    odom_x_skip = odom_x(1:skip_points:end);
                    odom_y_skip = odom_y(1:skip_points:end);
                    % Curvature data
                    curvature_data = cell2mat(cur_data.path_curv(1,:));
                    % Get the time data
                    time_data = cur_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    % Get the number of inner cell arrays
                    data_len = length(cur_data.dt_array{1});
                    N_RL = zeros(data_len, 1);
                    sum_horizon = zeros(data_len, 1);
                    sum_sparse_horizon = zeros(data_len, 1);
                    match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                    var_DRL = false;
                    if ~isempty(match_exp)    
                        n_total = str2double(match_exp{1});
                        for j = 1:n_total
                            eval(strcat("N_",num2str(j)," = zeros(data_len, 1);"));
                        end
                        var_DRL = true;
                    else
                        n_total = 4;
                        for j = 1:n_total
                            eval(strcat("N_",num2str(j)," = zeros(data_len, 1);"));
                        end
                    end
                    dt_array = cur_data.dt_array{1};
                    % Remove brackets and convert to numbers
                    dt_array = cellfun(@(x) str2num(x(2:end-1)), dt_array, 'UniformOutput', false);
                    dt_array = cell2mat(dt_array);
                    % Find mode of the dt_array
                    mode_dt = mode(dt_array);
                    % Get the indices of the dt_array that is not equal to the mode_dt
                    idx = find(~all(dt_array == mode_dt, 2));
                    % Plot the trajectory data and mark specific points
                    figure()
                    % Plot the reference trajectory
                    plot(path_x_data, path_y_data, 'k', 'LineWidth', 2.0);
                    hold on
                    axis equal
                    % Start and end points
                    plot(path_x_data(1), path_y_data(1), 'Marker', 'o', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g', 'MarkerSize', 8.0, 'LineStyle', 'none');
                    plot(path_x_data(end), path_y_data(end), 'Marker', 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'MarkerSize', 8.0, 'LineStyle', 'none');
                    % Plot the trajectory data
                    plot(odom_x_skip, odom_y_skip, 'LineStyle', '--', 'Marker', 'v', 'Color', 'b');
                    % Mark the sudden spikes in the sampling times (with X)
                    plot(odom_x(idx), odom_y(idx), 'Marker', 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 4.0, 'LineStyle', 'none');
                    title("Path Tracking Results")
                    xlabel("X (m)")
                    ylabel("Y (m)")
                    current_label = obj.get_label_name(current_type);
                    legend("Ref. Path", "Start Point", "Goal Point", current_label, "Sudden Sampling Time Spikes", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "path_tracking_with_sampling_time_spikes_marker.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                    % Plot the curvature and also mark the sudden spikes in the sampling times
                    figure()
                    plot(dist_travelled, curvature_data, 'b', 'LineWidth', 1.0);
                    hold on
                    grid on
                    % Mark the sudden spikes in the sampling times (with X)
                    plot(dist_travelled(idx), curvature_data(idx), 'r', 'Marker', 'x', 'LineWidth', 4.0, 'LineStyle', 'none');
                    title("Curvature Profile")
                    xlabel("Distance Travelled (m)")
                    ylabel("Curvature (1/m)")
                    legend("Curvature", "Sudden Sampling Time Spikes", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "curvature_with_sampling_time_spikes_marker.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function to plot the velocity data for the specified path based on the distance travelled
        function obj = plot_velocity_data_categorized(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            figure()
            label = [];
            % grid on
            hold on
            colors = {'g', 'r', 'b', 'm', 'c', 'y', 'k', 'r', 'b'};
            label = [];
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.odom_x(1,:));
                    path_y = cell2mat(current_data.odom_y(1,:));
                    % Get the path velocity data
                    path_velocity_data = cell2mat(current_data.path_vel_1(1,:));
                    % Get the velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    data_len = length(velocity_data);
                    % Distance travelled
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    plot(dist_travelled, velocity_data, colors{i}, 'LineWidth', 1.5);
                    label = [label; obj.get_label_name(current_type)];
                end
            end

            xlabel('Distance Travelled (m)', 'FontSize', 18);
            ylabel('Speed (m/s)', 'FontSize', 18);
            legend(label, 'Location', 'Best', 'FontSize', 14);
            box on;
            set(gca, 'FontSize', 18);
            ylim([0.0 1.25]);

            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "velocity_categorized.png";
            fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot the heading data for the specified path based on the distance travelled
        function obj = plot_heading_data_categorized(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Use degree or rad
            use_deg_instead_of_rad = true;
            % Get the time data
            figure()
            label = [];
            % grid on
            hold on
            colors = {'g', 'r', 'b', 'm', 'c', 'y', 'k', 'r', 'b'};
            label = [];

            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.odom_x(1,:));
                    path_y = cell2mat(current_data.odom_y(1,:));
                    % Get the path heading data
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % Get the heading data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    data_len = length(heading_data);
                    % Distance travelled
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    if use_deg_instead_of_rad
                        heading_data = rad2deg(heading_data);
                        path_heading_data = rad2deg(path_heading_data);
                    end
                    plot(dist_travelled, heading_data, colors{i}, 'LineWidth', 1.5);
                    label = [label; obj.get_label_name(current_type)];
                end
            end
            % title("Heading")
            xlabel("Distance Travelled (m)")
            if use_deg_instead_of_rad
                ylabel("Heading (deg)")
            else
                ylabel("Heading (rad)")
            end
            box on;
            % Set font size to 18
            set(gca, 'FontSize', 18);


            legend(label, 'Location', 'Best', 'FontSize', 14);
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "heading_categorized.png";
            fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot steering data for the specified path based on the distance travelled
        function obj = plot_steering_data_categorized(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            figure()
            label = [];
            grid on
            hold on
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    % Get the steering data
                    steering_data = cell2mat(current_data.mpc_steer(1,:));
                    data_len = length(steering_data);
                    % Distance travelled
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    plot(dist_travelled, steering_data, '-', 'LineWidth', 1.0);
                    label = [label; obj.get_label_name(current_type)];
                end
            end
            title("Steering Angle")
            xlabel("Distance Travelled (m)")
            ylabel("Steering (rad)")

            legend(label, 'Location', 'Best')
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "steering_categorized.png";
            fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot the objective cost over time 
        function obj = plot_objective_function(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the objective function data
                    state_cost_data = cell2mat(current_data.state_cost(1,:));
                    control_cost_data = cell2mat(current_data.control_cost(1,:));
                    control_diff_cost_data = cell2mat(current_data.control_diff_cost(1,:));
                    objective_function_data = cell2mat(current_data.objective_cost_total(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    % Figure for plotting total objective function
                    figure()
                    plot(time_data, objective_function_data, '-', 'LineWidth', 1.0);
                    title("Cost of Objective Function Over Time")
                    xlabel("Time Step")
                    ylabel("Objective Function Cost")
                    legend("Objective Function Cost", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "objective_function.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                    % Figure for plotting state cost, control cost and control diff cost
                    figure()
                    subplot(4,1,1)
                    plot(time_data, state_cost_data, '-', 'LineWidth', 1.0);
                    title("State Cost Over Time")
                    xlabel("Time Step")
                    ylabel("Cost")
                    legend("State Cost", 'Location', 'Best')
                    subplot(4,1,2)
                    plot(time_data, control_cost_data, '-', 'LineWidth', 1.0);
                    title("Control Cost Over Time")
                    xlabel("Time Step")
                    ylabel("Cost")
                    legend("Control Cost", 'Location', 'Best')
                    subplot(4,1,3)
                    plot(time_data, control_diff_cost_data, '-', 'LineWidth', 1.0);
                    title("Control Difference Cost Over Time")
                    xlabel("Time Step")
                    ylabel("Cost")
                    legend("Control Diff. Cost", 'Location', 'Best')
                    subplot(4,1,4)
                    plot(time_data, objective_function_data, '-', 'LineWidth', 1.0);
                    title("Objective Function Total Over Time")
                    xlabel("Time Step")
                    ylabel("Cost")
                    legend("Objective Function Cost", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "objective_function_components.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function to plot steering rate
        function obj = plot_steering_rate_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % MAX D STEER
            max_d_steer = 0.14 * 0.1;
            % Get the time data
            label = [];
            color_arr = ['g', 'r', 'b'];
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    % Get the steering data
                    steering_data = cell2mat(current_data.mpc_steer(1,:));
                    steering_data = steering_data(1:end-1);
                    data_len = length(steering_data);
                    % Get the steering rate
                    steering_rate = steering_data(2:data_len) - steering_data(1:data_len-1);
                    % Clip the steering rate
                    steering_rate = min(max(steering_rate, -max_d_steer), max_d_steer);
                    % Pad the first element with first steering data
                    steering_rate = [steering_rate(1); steering_rate];
                    % Distance travelled
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]); 
                    % Steps for the steering rate
                    steps = 0:1:data_len-1;
                    % Subplot for the steering data
                    figure()
                    subplot(2,1,1)
                    plot(steps, steering_data, color_arr(i), 'LineWidth', 1.5);
                    % grid on
                    axis tight
                    hold on
                    title("Steering Angle")
                    % legend("Steering Angle", 'Location', 'Best')
                    xlabel("Time Step")
                    ylabel("Steering (rad)")
                    % Subplot for the steering rate
                    subplot(2,1,2)
                    title("Steering Angle Rate")
                    % grid on
                    axis tight
                    hold on
                    % Plot maximum steering rate and minimum steering rate
                    plot(steps, steering_rate, color_arr(i), 'LineWidth', 1.5);
                    % Plot maximum steering rate and minimum steering rate
                    plot(steps, max_d_steer*ones(data_len, 1), 'k--', 'LineWidth', 1.5);
                    plot(steps, -max_d_steer*ones(data_len, 1), 'k--', 'LineWidth', 1.5);
                    fontsize(12, 'points')
                    if path_name == "path_1191" || path_name == "path_150"
                        legend("Steering Rate", "Max Steering Rate", "Min Steering Rate", 'Location', 'NorthWest')
                    else
                        legend("Steering Rate", "Max Steering Rate", "Min Steering Rate", 'Location', 'Best')
                    end
                    xlabel("Time Step")
                    ylabel("Steering Rate (rad/s)")
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "steering_angle_rate.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function to plot velocity and steering data for the specified path based on the distance travelled
        function obj = plot_velocity_steering_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            figure()
            label = [];
            % For plotting the data for the specified type
            color_arr = ['g', 'r', 'b'];
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Color 
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    odom_x = cell2mat(current_data.odom_x(1,:));
                    odom_y = cell2mat(current_data.odom_y(1,:));
                    % Get the steering data
                    steering_data = cell2mat(current_data.mpc_steer(1,:));
                    % Exclude last steering data (usually 0 if goal is reached) and make the value as the previous steering angle
                    steering_data(end) = steering_data(end-1);
                    data_len = length(steering_data);
                    % Get Velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    % Distance travelled
                    dist_travelled = sqrt((odom_x(2:data_len)-odom_x(1:data_len-1)).^2 + (odom_y(2:data_len)-odom_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    % % Change to time step
                    % time_step = 0:1:data_len-1;
                    % dist_travelled = time_step;
                    % Subplot for the velocity data
                    subplot(2,1,1)
                    plot(dist_travelled, velocity_data, color_arr(i), 'LineWidth', 1.5);
                    % grid on
                    axis tight
                    hold on
                    % Subplot for the steering data
                    subplot(2,1,2)
                    plot(dist_travelled, steering_data, color_arr(i), 'LineWidth', 1.5);
                    label = [label; obj.get_label_name(current_type)];
                    % grid on
                    axis tight
                    hold on
                end
            end
            fontsize(12, 'points')
            % Label for the velocity subplot
            subplot(2,1,1)
            title("Velocity Profile")
            xlabel("Distance Travelled (m)")
            % xlabel("Time Step")
            ylabel("Velocity (m/s)")
            legend(label, 'Location', 'NorthEast');

            % Label for the steering subplot
            subplot(2,1,2)
            title("Steering Angle")
            xlabel("Distance Travelled (m)")
            % xlabel("Time Step")
            ylabel("Steering (rad)")
            legend(label, 'Location', 'Best')
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "steering_categorized.png";
            fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot velocity and steering data for the specified path based on the distance travelled
        function obj = plot_velocity_heading_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % Get the time data
            figure()
            label = [];
            % For plotting the data for the specified type
            color_arr = ['g', 'r', 'b'];
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Color 
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    odom_x = cell2mat(current_data.odom_x(1,:));
                    odom_y = cell2mat(current_data.odom_y(1,:));
                    % Get the steering data
                    steering_data = cell2mat(current_data.odom_yaw(1,:));
                    % Exclude last steering data (usually 0 if goal is reached) and make the value as the previous steering angle
                    steering_data(end) = steering_data(end-1);
                    data_len = length(steering_data);
                    % Get Speed data
                    speed_data = cell2mat(current_data.odom_vel(1,:));
                    % Distance travelled
                    dist_travelled = sqrt((odom_x(2:data_len)-odom_x(1:data_len-1)).^2 + (odom_y(2:data_len)-odom_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);

                    % Plot the speed profile
                    figure;
                    plot(dist_travelled, speed_data, color_arr(i), 'LineWidth', 1.5);
                    xlabel('Distance Travelled (m)');
                    ylabel('Speed (m/s)');
                    % title('Speed Profile');
                    axis tight;
                    hold on;


                    % Plot the heading angle profile
                    figure;
                    plot(dist_travelled, steering_data, color_arr(i), 'LineWidth', 1.5);
                    xlabel('Distance Travelled (m)');
                    ylabel('Heading Angle (rad)');
                    % title('Heading Angle Profile');
                    axis tight;
                    hold on;

                    label = [label; obj.get_label_name(current_type)];
                end
            end
            fontsize(12, 'points')
            % Label for the velocity subplot
            subplot(2,1,1)
            title("Velocity Profile")
            xlabel("Distance Travelled (m)")
            % xlabel("Time Step")
            ylabel("Velocity (m/s)")
            legend(label, 'Location', 'Best')

            % Label for the steering subplot
            subplot(2,1,2)
            title("Steering Angle")
            xlabel("Distance Travelled (m)")
            % xlabel("Time Step")
            ylabel("Steering (rad)")
            legend(label, 'Location', 'Best')
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            prefix_type = current_type;
            fig_name_path = "steering_categorized.png";
            fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot the histogram / box plot of the sampling times and horizon
        function obj = plot_sampling_time_horizon_histogram(obj, path_name, type_lists, dt_min)
            % Default argument for dt_min
            if nargin < 4 || isempty(dt_min)
                dt_min = 0.1;
            end
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
        end
        % Function to plot the horizon data for the specified path
        function obj = plot_horizon_data(obj, path_name, type_lists, dt_min, what_to_plot, use_stairs)
        % what_to_plot options: 'sampling_time', 'horizon'
            % Default argument for dt_min and what_to_plot
            if nargin < 4 || isempty(dt_min)
                dt_min = 0.1;
            end
            if nargin < 5 || isempty(what_to_plot)
                what_to_plot = 'horizon';
            end
            if nargin < 6 || isempty(use_stairs)
                use_stairs = false;
            end
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            split_horizon_plot = true;
            path_index = find(ismember(path_name_array, path_name));
            if ~split_horizon_plot
                figure;
                hold on;
            end
            label = [];
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                if ismember(current_type, type_lists)
                    cur_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(cur_data.path_x(1,:));
                    path_y = cell2mat(cur_data.path_y(1,:));
                    data_len = length(cur_data.dt_array{1});
                    dist_travelled = sqrt((path_x(2:data_len)-path_x(1:data_len-1)).^2 + (path_y(2:data_len)-path_y(1:data_len-1)).^2);
                    dist_travelled = cumsum([0.0; dist_travelled]);
                    % Get the number of inner cell arrays
                    data_len = length(cur_data.dt_array{1});
                    N_RL = zeros(data_len, 1);
                    sum_horizon = zeros(data_len, 1);
                    sum_sparse_horizon = zeros(data_len, 1);
                    match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                    var_DRL = false;
                    if ~isempty(match_exp)    
                        n_total = str2double(match_exp{1});
                        for j = 1:n_total
                            eval(strcat("N_",num2str(j)," = zeros(data_len, 1);"));
                        end
                        var_DRL = true;
                    else
                        n_total = 4;
                        for j = 1:n_total
                            eval(strcat("N_",num2str(j)," = zeros(data_len, 1);"));
                        end
                    end
                    dt_array = cur_data.dt_array{1};
                    for j = 1:data_len
                        dt_arr = str2num(cell2mat(dt_array(j)));
                        sum_horizon(j) = sum(dt_arr);
                        dt_arr = dt_arr(length(dt_arr)-n_total+1:end);
                        sum_sparse_horizon(j) = sum(dt_arr);
                        if strcmp(what_to_plot, 'horizon')
                            for i = 1:n_total
                                eval(strcat("N_",num2str(i),"(j) = dt_arr(",num2str(i),");"));
                            end
                            total_arr = sum(dt_arr);
                            N_RL(j) = total_arr/dt_min;
                        else
                            for i = 1:n_total
                                eval(strcat("N_",num2str(i),"(j) = dt_arr(",num2str(i),");"));
                            end
                        end
                    end
                    markers = ['o', 's', 'd', '^', 'v', 'p', 'h', 'x', '+', '*'];
                    if ~split_horizon_plot
                        for j = 1:length(dt_arr)
                            if use_stairs
                                if current_type ~= "DRL" && ~var_DRL
                                    stairs(dist_travelled, N_1);
                                    break
                                else
                                    eval(strcat("stairs(dist_travelled, ","N_",num2str(j),")"));
                                end
                            else
                                if current_type ~= "DRL" && ~var_DRL
                                    plot(dist_travelled, N_1, 'LineWidth', 1.0);
                                    break
                                else
                                    eval(strcat("plot(dist_travelled, ","N_",num2str(j),", 'LineWidth', 1.0, 'Marker', markers(j))"));
                                end
                            end
                        end
                        if current_type ~= "DRL" && ~var_DRL
                            label = [label; obj.get_label_name(current_type)];
                        elseif var_DRL
                            for i = 1:n_total
                                label = [label; append("NUDRL-MPC S", num2str(i))];
                            end
                        else
                            label = [label; "NUDRL-MPC S1"; "NUDRL-MPC S2"; "NUDRL-MPC S3"; "NUDRL-MPC S4"];
                        end
                    end

                    % Separate dt based on the DRL type
                    if split_horizon_plot && var_DRL
                        if ~strcmp(what_to_plot, 'horizon')
                            hor_num_to_split_fig = containers.Map({'2', '4', '6', '8'}, {1, 2, 3, 4});
                            % Create separate figures with the total based on the mapped value
                            % fprintf("N Total %d\n", n_total);
                            if hor_num_to_split_fig.isKey(string(n_total))
                                hor_num_to_split = hor_num_to_split_fig(string(n_total));
                                if hor_num_to_split >= 1
                                    % Divide split into n parts
                                    n = hor_num_to_split;
                                    dt_splits = ones(1, n_total) * floor(n_total / n);
                                    dt_splits(1:mod(n_total, n)) = dt_splits(1:mod(n_total, n)) + 1;
                                    % Initialize an empty cell array to hold the subarrays
                                    subarrays = cell(1, hor_num_to_split);

                                    % Initialize the start index
                                    start_idx = 1;
                                    % Loop over dt_splits to create the subarrays
                                    arr = 1:n_total;
                                    for i = 1:hor_num_to_split
                                        % Calculate the end index
                                        end_idx = start_idx + dt_splits(i) - 1;
                                        % Create the subarray
                                        subarrays{i} = arr(start_idx:end_idx);
                                        % Update the start index for the next iteration
                                        start_idx = end_idx + 1;
                                    end
                                    for i = 1:hor_num_to_split
                                        figure;
                                        hold on;
                                        label = [];
                                        cur_dt_arr = subarrays{i};
                                        for j = cur_dt_arr(1):cur_dt_arr(end)
                                            if use_stairs
                                                if current_type ~= "DRL" && ~var_DRL
                                                    stairs(dist_travelled, N_1);
                                                    break
                                                else
                                                    eval(strcat("stairs(dist_travelled, ","N_",num2str(j),")"));
                                                end
                                            else
                                                if current_type ~= "DRL" && ~var_DRL
                                                    plot(dist_travelled, N_1, 'LineWidth', 1.0);
                                                    break
                                                else
                                                    eval(strcat("plot(dist_travelled, ","N_",num2str(j),", 'LineWidth', 1.0, 'Marker', markers(j))"));
                                                end
                                            end
                                            % Get label name
                                            label = [label; append("NUDRL-MPC ST", num2str(j))];
                                        end
                                        title("Sampling Time Over Time - " + "NUDRL-MPC S" + num2str(n_total) + " - " + "Part " + num2str(i))
                                        xlabel("Distance Travelled (m)")
                                        ylabel("Sampling Time (s)")
                                        grid on;
                                        ylim([0 0.8])
                                        legend(label, 'Location', 'Best')
                                        prefix_path_name = path_name_array(cur_idx);
                                        if obj.use_path_mapping_table
                                            % If the path name is in the path difficulty table mapping
                                            path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                            % Get the difficulty of the path
                                            if ~isempty(path_difficulty_index)
                                                path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                                prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                            end
                                        end
                                        fig_name_path = current_type + "_horizon_over_time_figure_" + num2str(i) + ".png";
                                        fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                                        saveas(gcf, filename);
                                        % Close the figure
                                        close(gcf);
                                    end    
                                end
                            end   
                        else
                            use_stairs = true;
                            dt_min = 0.1;
                            %Plot total horizon all (seconds)
                            figure;
                            hold on;
                            grid on;
                            if use_stairs
                                stairs(dist_travelled, sum_horizon, 'LineWidth', 1.0);
                            else
                                plot(dist_travelled, sum_horizon, 'LineWidth', 1.0);
                            end
                            title("Horizon Over Time - " + "NUDRL-MPC S" + num2str(n_total))
                            xlabel("Distance Travelled (m)")
                            ylabel("Horizon Length (s)")
                            legend("NUDRL-MPC S" + num2str(n_total), 'Location', 'Best')
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = current_type + "_horizon_over_time_figure.png";
                            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot total horizon sparse (seconds)
                            figure;
                            hold on;
                            grid on;
                            if use_stairs
                                stairs(dist_travelled, sum_sparse_horizon, 'LineWidth', 1.0);
                            else
                                plot(dist_travelled, sum_sparse_horizon, 'LineWidth', 1.0);
                            end
                            title("Sparse Horizon Over Time - " + "NUDRL-MPC S" + num2str(n_total))
                            xlabel("Distance Travelled (m)")
                            ylabel("Horizon Length (s)")
                            legend("NUDRL-MPC S" + num2str(n_total), 'Location', 'Best')
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = current_type + "_horizon_sparse_over_time_figure.png";
                            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot total horizon (slots)
                            figure;
                            hold on;
                            grid on;
                            slot_horizon = sum_horizon / dt_min;
                            if use_stairs
                                stairs(dist_travelled, slot_horizon, 'LineWidth', 1.0);
                            else
                                plot(dist_travelled, slot_horizon, 'LineWidth', 1.0);
                            end
                            title("Horizon Over Time - " + "NUDRL-MPC S" + num2str(n_total))
                            xlabel("Distance Travelled (m)")
                            ylabel("Horizon Number")
                            legend("NUDRL-MPC S" + num2str(n_total), 'Location', 'Best')
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = current_type + "_horizon_slot_over_time_figure.png";
                            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot total horizon (slots)
                            figure;
                            hold on;
                            grid on;
                            sprarse_slot_horizon = sum_sparse_horizon / dt_min;
                            if use_stairs
                                stairs(dist_travelled, sprarse_slot_horizon, 'LineWidth', 1.0);
                            else
                                plot(dist_travelled, sprarse_slot_horizon, 'LineWidth', 1.0);
                            end
                            title("Sparse Horizon Over Time - " + "NUDRL-MPC S" + num2str(n_total))
                            xlabel("Distance Travelled (m)")
                            ylabel("Horizon Number")
                            legend("NUDRL-MPC S" + num2str(n_total), 'Location', 'Best')
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = current_type + "_horizon_sparse_slot_over_time_figure.png";
                            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot total horizon over time steps
                            figure;
                            hold on;
                            grid on;
                            if use_stairs
                                stairs(0:data_len-1, sum_horizon, 'LineWidth', 1.0);
                            else
                                plot(0:data_len-1, sum_horizon, 'LineWidth', 1.0);
                            end
                            title("Horizon Over Time - " + "NUDRL-MPC S" + num2str(n_total))
                            xlabel("Time Step")
                            ylabel("Horizon Length (s)")
                            legend("NUDRL-MPC S" + num2str(n_total), 'Location', 'Best')
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = current_type + "_horizon_over_time_steps_figure.png";
                            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);


                        end                         
                    end
                end
            end
            grid on
            if strcmp(what_to_plot, 'horizon')
                title("Horizon Over Time")
                xlabel("Distance Travelled (m)")
                ylabel("Horizon Number")
                legend(label, 'Location', 'Best')
                ylim([0 10]);
            else
                title("Sampling Time Over Time")
                xlabel("Distance Travelled (m)")
                ylabel("Sampling Time (s)")
                legend(label, 'Location', 'Best')
                ylim([0.0 1.0])
            end
            prefix_path_name = obj.table_data.path_name(cur_idx,:);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            if strcmp(what_to_plot, "horizon")
                fig_name_path = "horizon_over_time.png";
            else
                fig_name_path = "sampling_time_over_time.png";
            end
            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Close the figure
            close(gcf);
        end
        % Function to plot sampling times distribution over the path
        function obj = plot_sampling_time_box_plot(obj, type_lists)
            % Generate the empty array for the sampling times per type
            % Replace the subtypes with the base type
            obj.base_types = obj.base_types;
            types = type_lists;
            for i = 1:length(obj.base_types)
                types = replace(types, obj.base_types(i) + "1", obj.base_types(i));
                types = replace(types, obj.base_types(i) + "2", obj.base_types(i));
            end
            drl_types = types(startsWith(types, "DRL"));
            unique_drl_types = unique(drl_types, 'stable');
            drl_types = string(drl_types);
            types_list_drl_only = type_lists(startsWith(type_lists, "DRL"));
            for i = 1:length(unique_drl_types)
                cur_type = unique_drl_types(i);
                eval(strcat("sampling_times_", cur_type, '= [];'));
            end
            % Init the array for saving each time step sampling time
            for i = 1:obj.variable_sampling_total
                eval(strcat("sampling_times_var", num2str(i), '= [];'));
            end
            % Generate a table for saving the sampling times number
            for i = 1:length(unique_drl_types)
                cur_type = unique_drl_types(i);
                eval(strcat("sampling_times_table_", cur_type, '= table();'));
            end
            % For each path index, get the sampling times
            % Obtain the path names
            path_name_array = obj.table_data.path_name;
            path_name_array = string(unique(path_name_array, 'stable'));
            % For each path name, get the sampling times for each type
            for y = 1:length(unique_drl_types)
                cur_drl_type = unique_drl_types(y);
                % Obtain all types that belong to the current drl type
                type_list_to_get = types_list_drl_only(types_list_drl_only.startsWith(cur_drl_type));
                for i = 1:length(path_name_array)
                    for j = 1:length(type_list_to_get)
                        cur_type = type_list_to_get(j);
                        % Get the data for each type on the current path
                        current_path = path_name_array(i);
                        table_data_with_current_path_type = obj.table_data(strcmp(string(obj.table_data.path_name), current_path) & strcmp(string(obj.table_data.type), cur_type), :);
                        % Get the dt array
                        dt_array = table_data_with_current_path_type.dt_array{1};
                        % Get the dt from current time until the end
                        dt_arr_length = length(dt_array);
                        % Obtain the dt array from the current time until the end
                        for dt_len = 1:dt_arr_length
                            dt_array_conv = str2num(cell2mat(dt_array(dt_len)));
                            for k = 1:obj.variable_sampling_total
                                if ~isempty(dt_array_conv)
                                    eval(strcat("sampling_times_var", num2str(k), '= [sampling_times_var', num2str(k), '; dt_array_conv(', num2str(k), ')];'));
                                end
                            end
                        end
                    end
                end
                % Store each sampling times to the table
                for i = 1:obj.variable_sampling_total
                    if ~isempty(eval(strcat("sampling_times_var", num2str(i))))
                        eval(strcat("sampling_times_table_", cur_drl_type, '= [sampling_times_table_', cur_drl_type, '; sampling_times_var', num2str(i),'];'));
                    end
                end
                % Plot the box plot for the sampling times
                figure()
                sampling_times = [];
            end
        end
        % Function to plot the acceleration and acceleration rate
        function obj = plot_accel_rate_in_one(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            plot_with_speed = true;
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    % Get the acceleration data
                    accel_data = cell2mat(current_data.mpc_accel(1,:));
                    data_len = length(accel_data);
                    % Get the acceleration rate
                    accel_rate = accel_data(2:data_len) - accel_data(1:data_len-1);
                    accel_rate = [0; accel_rate];
                    % Time Steps
                    steps = 0:1:data_len-1;
                    % Subplot for the acceleration data
                    figure()
                    subplot(2,1,1)
                    hold on
                    title("Acceleration and Acceleration Rate")
                    plot(steps, accel_data, '-', 'LineWidth', 1.0);
                    grid on
                    legend("Acceleration", 'Location', 'Best')
                    xlabel("Time Step")
                    ylabel("Acceleration (m/s^2)")
                    % Subplot for the acceleration rate
                    subplot(2,1,2)
                    grid on
                    hold on
                    plot(steps, accel_rate, '-', 'LineWidth', 1.0);
                    legend("Acceleration Rate", 'Location', 'Best')
                    xlabel("Time Step")
                    ylabel("Acceleration Rate (m/s^2)")
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    fig_name_path = "acceleration_rate.png";
                    fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                    % If Plot with speed 
                    if plot_with_speed
                        % Get the MPC Speed data
                        speed_data = cell2mat(current_data.mpc_vel(1,:));
                        % Subplot for the speed data
                        figure()
                        subplot(3,1,1)
                        hold on
                        title("Velocity and Acceleration")
                        plot(steps, speed_data, '-', 'LineWidth', 1.0);
                        grid on
                        hold on
                        legend("Velocity", 'Location', 'Best')
                        xlabel("Time Step")
                        ylabel("Velocity (m/s)")
                        % Subplot for the acceleration data
                        subplot(3,1,2)
                        grid on
                        hold on
                        plot(steps, accel_data, '-', 'LineWidth', 1.0);
                        legend("Acceleration", 'Location', 'Best')
                        xlabel("Time Step")
                        ylabel("Acceleration (m/s^2)")
                        % Subplot for the acceleration rate
                        subplot(3,1,3)
                        grid on
                        hold on
                        plot(steps, accel_rate, '-', 'LineWidth', 1.0);
                        legend("Acceleration Rate", 'Location', 'Best')
                        xlabel("Time Step")
                        ylabel("Acceleration Rate (m/s^2)")
                        prefix_path_name = path_name_array(cur_idx);
                        if obj.use_path_mapping_table
                            % If the path name is in the path difficulty table mapping
                            path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                            % Get the difficulty of the path
                            if ~isempty(path_difficulty_index)
                                path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                            end
                        end
                        fig_name_path = "velocity_acceleration_rate.png";
                        fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        % Close the figure
                        close(gcf);
                    end
                end
            end
        end
        % Function to plot the lateral error (signed)
        function obj = plot_lateral_error_over_time(obj, path_name, type_lists)
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                if ismember(current_type, type_lists)
                    current_data = obj.table_data(cur_idx, :);
                    % Lateral deviation data over time
                    lateral_deviation_signed = cell2mat(current_data.lateral_error_signed(1,:));
                    % Plot the lateral deviation over time
                    figure()
                    plot(lateral_deviation_signed, 'LineWidth', 1.0);
                    grid on
                    xlabel("Time Step")
                    ylabel("Lateral Deviation")
                    title("Lateral Deviation Over Time")
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    fig_name_path = "lateral_deviation_over_time.png";
                    fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    lateral_deviation_raw = cell2mat(current_data.lateral_error_raw(1,:));
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                    % Plot the steering angle and steering angle rate, lateral deviation signed 
                    % Get the steering data
                    % MAX D STEER
                    max_d_steer = 0.14 * 0.1;
                    steering_data = cell2mat(current_data.mpc_steer(1,:));
                    steering_data = steering_data(1:end-1);
                    data_len = length(steering_data);
                    % Get the steering rate
                    steering_rate = steering_data(2:data_len) - steering_data(1:data_len-1);
                    % Clip the steering rate
                    steering_rate = min(max(steering_rate, -max_d_steer), max_d_steer);
                    % Pad the first element with first steering data
                    steering_rate = [steering_rate(1); steering_rate];
                    % Plot the steering angle and steering rate
                    subplot(4,1,1)
                    plot(steering_data, 'LineWidth', 1.0);
                    grid on
                    xlabel("Time Step")
                    ylabel("Steering Angle")
                    title("Steering Angle Over Time")
                    subplot(4,1,2)
                    plot(steering_rate, 'LineWidth', 1.0);
                    grid on
                    xlabel("Time Step")
                    ylabel("Steering Rate")
                    title("Steering Rate Over Time")
                    subplot(4,1,3)
                    plot(lateral_deviation_signed, 'LineWidth', 1.0);
                    grid on
                    xlabel("Time Step")
                    ylabel("Lateral Deviation (m)")
                    title("Lateral Deviation EMA Over Time")
                    subplot(4,1,4)
                    plot(lateral_deviation_raw, 'LineWidth', 1.0);
                    grid on
                    xlabel("Time Step")
                    ylabel("Lateral Deviation (m)")
                    title("Lateral Deviation Over Time")
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    fig_name_path = "steering_angle_rate_lateral_deviation.png";
                    fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function to plot the proportion of objective function cost
        function obj = plot_objective_function_components_proportion(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            plot_per_component_wise_cost = true;
            line_plot_the_cost = true;
            plot_normalized = false;
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the objective function data
                    state_cost_data = cell2mat(current_data.state_cost(1,:));
                    control_cost_data = cell2mat(current_data.control_cost(1,:));
                    control_diff_cost_data = cell2mat(current_data.control_diff_cost(1,:));
                    objective_function_data = cell2mat(current_data.objective_cost_total(1,:));

                    if plot_normalized
                        % Nomalize the cost data to 100%
                        state_cost_data_norm = state_cost_data ./ objective_function_data * 100;
                        control_cost_data_norm = control_cost_data ./ objective_function_data * 100;
                        control_diff_cost_data_norm = control_diff_cost_data ./ objective_function_data * 100;
                    end

                    % Combine the data into a single matrix
                    if plot_normalized
                        data = [state_cost_data_norm, control_cost_data_norm, control_diff_cost_data_norm];
                    else
                        data = [state_cost_data, control_cost_data, control_diff_cost_data];
                    end

                    % Create a stacked area plot of the data with specified colors
                    if ~isempty(data) && ~all(isnan(data(:)))
                        % Create a stacked area plot of the data with specified colors
                        figure()
                        h = area(data, 'LineStyle', ':');
                        h(1).FaceColor = [0.8 0.4 0.4];  % Red
                        h(2).FaceColor = [0.4 0.8 0.4];  % Green
                        h(3).FaceColor = [0.4 0.4 0.8];  % Blue
                    else
                        warning('Data is empty or contains only NaN values. Cannot create plot.');
                    end

                    % Add the total objective function data as a separate line
                    hold on;
                    grid on;
                    % plot(objective_function_data, 'k', 'LineWidth', 2);

                    % Add labels and title
                    if ~plot_normalized
                        title('Cost Components and Total Objective Cost');
                        xlabel('Time Step');
                        ylabel('Objective Cost Total');
                    else
                        title('Cost Components Proportion and Total Objective Cost Proportion');
                        xlabel('Time Step');
                        ylabel('Objective Cost Total Proportion (%)');
                    end
                    % Add legend
                    legend('State Cost', 'Control Cost', 'Control Diff Cost');

                    % Save the plot to a file
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    if ~plot_normalized
                        fig_name_path = "objective_function_components_proportion.png";
                    else
                        fig_name_path = "objective_function_components_proportion_normalized.png";
                    end
                    fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                    
                    %% TODO: Implement the parsing for the components of weight values
                    if plot_per_component_wise_cost
                        % 'x_cost', 'y_cost', 'yaw_cost', 'v_cost', 'accel_cost', 'accel_diff_cost', 'steer_cost', 'steer_diff_cost'
                        % Get the cost value of x
                        cost_x = cell2mat(current_data.x_cost(1,:));
                        % Get the cost value of y
                        cost_y = cell2mat(current_data.y_cost(1,:));
                        % Get the cost value of yaw
                        cost_yaw = cell2mat(current_data.yaw_cost(1,:));
                        % Get the cost value of v
                        cost_v = cell2mat(current_data.v_cost(1,:));
                        % Get the cost value of steer
                        cost_steer = cell2mat(current_data.steer_cost(1,:));
                        % Get the cost value of steer rate
                        cost_steer_rate = cell2mat(current_data.steer_diff_cost(1,:));
                        % Get the cost value of accel
                        cost_accel = cell2mat(current_data.accel_cost(1,:));
                        % Get the cost value of accel_rate
                        cost_accel_rate = cell2mat(current_data.accel_diff_cost(1,:));

                        if plot_normalized
                            % Normalize the cost data to 100%
                            cost_x_norm = cost_x ./ objective_function_data * 100;
                            cost_y_norm = cost_y ./ objective_function_data * 100;
                            cost_yaw_norm = cost_yaw ./ objective_function_data * 100;
                            cost_v_norm = cost_v ./ objective_function_data * 100;
                            cost_steer_norm = cost_steer ./ objective_function_data * 100;
                            cost_steer_rate_norm = cost_steer_rate ./ objective_function_data * 100;
                            cost_accel_norm = cost_accel ./ objective_function_data * 100;
                            cost_accel_rate_norm = cost_accel_rate ./ objective_function_data * 100;
                        end

                        % Combine the data into a single matrix
                        if plot_normalized
                            data = [cost_x_norm, cost_y_norm, cost_yaw_norm, cost_v_norm, cost_steer_norm, cost_steer_rate_norm, cost_accel_norm, cost_accel_rate_norm];
                        else
                            data = [cost_x, cost_y, cost_yaw, cost_v, cost_steer, cost_steer_rate, cost_accel, cost_accel_rate];
                        end

                        figure();
                        % Create a stacked area plot of the data with specified colors
                        h = area(data, 'LineStyle', ':');
                        h(1).FaceColor = [0.8 0.4 0.4];  % Red
                        h(2).FaceColor = [0.4 0.8 0.4];  % Green
                        h(3).FaceColor = [0.4 0.4 0.8];  % Blue
                        h(4).FaceColor = [0.8 0.8 0.4];  % Yellow
                        h(5).FaceColor = [0.8 0.4 0.8];  % Magenta
                        h(6).FaceColor = [0.4 0.8 0.8];  % Cyan
                        h(7).FaceColor = [0.4 0.4 0.4];  % Black
                        h(8).FaceColor = [0.8 0.8 0.8];  % Gray

                        % Add labels and title
                        if ~plot_normalized
                            title('Cost Components Wise and Total Objective Function');
                            xlabel('Time Step');
                            ylabel('Objective Cost Total');
                        else
                            title('Cost Components Wise and Total Objective Function Proportion'); 
                            xlabel('Time Step');
                            ylabel('Objective Cost Total Proportion (%)');
                        end
                        
                        % Add legend
                        legend('Cost X', 'Cost Y', 'Cost Yaw', 'Cost Velocity', 'Cost Steer', 'Cost Steer Rate', 'Cost Accel', 'Cost Accel Rate', 'Location', 'Best');
                        
                        prefix_path_name = path_name_array(cur_idx);
                        if obj.use_path_mapping_table
                            % If the path name is in the path difficulty table mapping
                            path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                            % Get the difficulty of the path
                            if ~isempty(path_difficulty_index)
                                path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                            end
                        end
                        if ~plot_normalized
                            fig_name_path = "objective_function_components_wise_proportion.png";
                        else
                            fig_name_path = "objective_function_components_wise_proportion_normalized.png";
                        end
                        fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        % Close the figure
                        close(gcf);
                        
                        if line_plot_the_cost
                            % Plot per state components
                            data_len = length(cost_x);
                            steps = 0:1:data_len-1;
                            % State Cost
                            figure()
                            subplot(4,1,1)
                            plot(steps, cost_x, '-', 'LineWidth', 1.0);
                            title("State Cost Components")
                            grid on
                            hold on
                            legend("Cost X", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost X")
                            % Subplot for the y data
                            subplot(4,1,2)
                            grid on
                            hold on
                            plot(steps, cost_y, '-', 'LineWidth', 1.0);
                            legend("Cost Y", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Y")
                            % Subplot for the yaw data
                            subplot(4,1,3)
                            grid on
                            hold on
                            plot(steps, cost_yaw, '-', 'LineWidth', 1.0);
                            legend("Cost Yaw", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Yaw")
                            % Subplot for the v data
                            subplot(4,1,4)
                            grid on
                            hold on
                            plot(steps, cost_v, '-', 'LineWidth', 1.0);
                            legend("Cost Velocity", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Velocity")
                            prefix_path_name = path_name_array(cur_idx);
                            if obj.use_path_mapping_table
                                % If the path name is in the path difficulty table mapping
                                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                                % Get the difficulty of the path
                                if ~isempty(path_difficulty_index)
                                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                                end
                            end
                            fig_name_path = "state_cost_components.png";
                            fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot per control components
                            % Control Cost
                            figure()
                            subplot(2, 1, 1)
                            plot(steps, cost_steer, '-', 'LineWidth', 1.0);
                            title("Control Cost Components")
                            grid on
                            hold on
                            legend("Cost Steer", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Steer")
                            % Subplot for the Acceleration data
                            subplot(2, 1, 2)
                            grid on
                            hold on
                            plot(steps, cost_accel, '-', 'LineWidth', 1.0);
                            legend("Cost Accel", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Acceleration")
                            fig_name_path = "control_cost_components.png";
                            fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            % Close the figure
                            close(gcf);

                            % Plot per control rate components
                            % Control Diff Cost
                            figure()
                            subplot(2, 1, 1)
                            plot(steps, cost_steer_rate, '-', 'LineWidth', 1.0);
                            title("Control Diff Cost Components")
                            grid on
                            hold on
                            legend("Cost Steer Rate", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Steer Rate")
                            % Subplot for the Acceleration Rate data
                            subplot(2, 1, 2)
                            grid on
                            hold on
                            plot(steps, cost_accel_rate, '-', 'LineWidth', 1.0);
                            legend("Cost Accel Rate", 'Location', 'Best')
                            xlabel("Time Step")
                            ylabel("Cost Acceleration Rate")
                            fig_name_path = "control_diff_cost_components.png";
                            fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                            filename = append(obj.plot_export_dir, '/', fig_name_export);
                            saveas(gcf, filename);
                            close(gcf);
                        end

                        % Plot area stacked plot for each cost component
                        figure();
                        % State Cost
                        if plot_normalized
                            cost_x_norm = cost_x ./ state_cost_data * 100;
                            cost_y_norm = cost_y ./ state_cost_data * 100;
                            cost_yaw_norm = cost_yaw ./ state_cost_data * 100;
                            cost_v_norm = cost_v ./ state_cost_data * 100;
                            state_cost_data_stack = [cost_x_norm, cost_y_norm, cost_yaw_norm, cost_v_norm];
                        else
                            state_cost_data_stack = [cost_x, cost_y, cost_yaw, cost_v];
                        end
                        h = area(state_cost_data_stack, 'LineStyle', ':');
                        h(1).FaceColor = [0.8 0.4 0.4];  % Red
                        h(2).FaceColor = [0.4 0.8 0.4];  % Green
                        h(3).FaceColor = [0.4 0.4 0.8];  % Blue
                        h(4).FaceColor = [0.8 0.8 0.4];  % Yellow
                        % Add labels and title
                        if ~plot_normalized
                            title('State Cost Components');
                            xlabel('Time Step');
                            ylabel('Cost');
                        else
                            title('State Cost Components Proportion');
                            xlabel('Time Step');
                            ylabel('Cost (%)');
                        end
                        % Add legend
                        legend('Cost X', 'Cost Y', 'Cost Yaw', 'Cost Velocity');
                        % Save the plot to a file
                        prefix_path_name = path_name_array(cur_idx);
                        if obj.use_path_mapping_table
                            % If the path name is in the path difficulty table mapping
                            path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                            % Get the difficulty of the path
                            if ~isempty(path_difficulty_index)
                                path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                            end
                        end
                        if plot_normalized
                            fig_name_path = "state_cost_components_proportion_normalized.png";
                        else
                            fig_name_path = "state_cost_components_proportion.png";
                        end
                        fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        % Close the figure
                        close(gcf);

                        % Control cost
                        if plot_normalized
                            cost_steer_norm = cost_steer ./ control_cost_data * 100;
                            cost_accel_norm = cost_accel ./ control_cost_data * 100;
                            control_cost_data_stack = [cost_steer_norm, cost_accel_norm];
                        else
                            control_cost_data_stack = [cost_steer, cost_accel];
                        end
                        figure();
                        h = area(control_cost_data_stack, 'LineStyle', ':');
                        h(1).FaceColor = [0.8 0.4 0.4];  % Red
                        h(2).FaceColor = [0.4 0.8 0.4];  % Green
                        % Add labels and title
                        if ~plot_normalized
                            title('Control Cost Components');
                            xlabel('Time Step');
                            ylabel('Cost');
                        else
                            title('Control Cost Components Proportion');
                            xlabel('Time Step');
                            ylabel('Cost (%)');
                        end
                        hold on;
                        % Add legend
                        legend('Cost Steer', 'Cost Accel');
                        % Save the plot to a file
                        if plot_normalized
                            fig_name_path = "control_cost_components_proportion_normalized.png";
                        else
                            fig_name_path = "control_cost_components_proportion.png";
                        end
                        fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        % Close the figure
                        close(gcf);

                        % Control Rate Cost
                        if plot_normalized
                            control_steer_rate_norm = cost_steer_rate ./ control_diff_cost_data * 100;
                            control_accel_rate_norm = cost_accel_rate ./ control_diff_cost_data * 100;
                            control_diff_cost_data_stack = [control_steer_rate_norm, control_accel_rate_norm];
                        else
                            control_diff_cost_data_stack = [cost_steer_rate, cost_accel_rate];
                        end
                        figure();
                        h = area(control_diff_cost_data_stack, 'LineStyle', ':');
                        h(1).FaceColor = [0.8 0.4 0.4];  % Red
                        h(2).FaceColor = [0.4 0.8 0.4];  % Green
                        % Add labels and title
                        if ~plot_normalized
                            title('Control Diff Cost Components');
                            xlabel('Time Step');
                            ylabel('Cost');
                        else
                            title('Control Diff Cost Components Proportion');
                            xlabel('Time Step');
                            ylabel('Cost (%)');
                        end
                        hold on;
                        % Add legend
                        legend('Cost Steer Rate', 'Cost Accel Rate');
                        % Save the plot to a file
                        if plot_normalized
                            fig_name_path = "control_diff_cost_components_proportion_normalized.png";
                        else
                            fig_name_path = "control_diff_cost_components_proportion.png";
                        end
                        fig_name_export = strcat(prefix_path_name, '_', current_type, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        % Close the figure
                        close(gcf);
                    end
                end
            end
        end

        % Function to plot the box plot error for the specified path
        function obj = plot_box_plot_error(obj, path_name, type_lists, log_scale)
            % Check if the log_scale is empty or not (default is false)
            if nargin < 4 || isempty(log_scale)
                log_scale = false;
            end
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            error_data_array = [];
            error_data_group = [];
            labels = [];
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Add current type to the labels
                    % label = obj.get_label_name(current_type);
                    label = current_type;
                    labels = [labels; label];
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the error data
                    error_data = cell2mat(current_data.error_over_time(1,:));
                    if log_scale
                        error_data = log(error_data);
                    end
                    error_data_array = [error_data_array; error_data];
                    error_data_group = [error_data_group; i.*ones(length(error_data), 1)];
                end
            end
            % Start plotting the box plot
            figure()
            boxplot(error_data_array, error_data_group, 'Labels', labels)
            if log_scale
                title("Log Tracking Distance Error of MPC Path Tracking")
                ylabel("Log Tracking Distance Error (m)")
            else
                title("Tracking Distance Error of MPC Path Tracking")
                ylabel("Tracking Distance Error (m)")
            end
            % xlabel("MPC Approaches")
            xlabel("MPC Algorithms")
            grid on
            axis tight
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            if log_scale
                fig_name_path = "log_error_box_plot.png";
            else
                fig_name_path = "error_box_plot.png";
            end
            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            close(gcf);
        end
        %TODO: Add the original path to the plot (done)
        %% Function to plot the path tracking data for the specified path
        function obj = plot_path_tracking_data(obj, path_name, type_lists, plot_without_title)
            if nargin < 4 || isempty(plot_without_title)
                plot_without_title = false;
            end
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            path_name_file = append(obj.path_data_from_file,'/', path_name, '.csv');
            path_data = readtable(path_name_file);
            label = [];
            path_x = table2array(path_data(:,1));
            path_y = table2array(path_data(:,2));
            % Plot the path tracking data
            figure()
            hold on
            % grid on
            % Extend Axis with the 10% so it doesnt 
            label = [label; "Ref. Path"; "Start"; "Goal"];
            % plot(path_x, path_y, 'k-', 'LineWidth', 0.5)
            plot(path_x, path_y, 'k-', 'LineWidth', 2.0)
            % Plot the start and end point of the path
            plot(path_x(1), path_y(1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r')
            plot(path_x(end), path_y(end), 'b^', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');            plot_idx = 1;
            % Calculate the range of x and y data
            xMin = min(path_x);
            xMax = max(path_x);
            yMin = min(path_y);
            yMax = max(path_y);

            % Determine the range
            xRange = xMax - xMin;
            yRange = yMax - yMin;

            % Add a margin of 10% to the range
            xMargin = 0.1 * xRange;
            yMargin = 0.1 * yRange;

            % Set the x and y limits with the margin
            xlim([xMin - xMargin, xMax + xMargin]);
            ylim([yMin - yMargin, yMax + yMargin]);

            % Add top

            axis equal;
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the x and y data
                    x_data = cell2mat(current_data.odom_x(1,:));
                    y_data = cell2mat(current_data.odom_y(1,:));
                    % Skip some points so the plot is not too crowded
                    x_data = x_data(1:12:end);
                    y_data = y_data(1:12:end);
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    % label = [label; current_type];
                    label = [label; obj.get_label_name(current_type)];
                    plot_color_rgb = {[1 0 0], [0 1 0], [0 1 1], [0.9100 0.4100 0.1700]};
                    % plot_type = ['k-', 'r--', 'b:', 'g-'. 'mo-'. 'cd--', 'y:'];
                    % Define line properties
                    lineStyles = {'-.', '--', ':', '-.', '-', '--', ':', '-.', '-', '--'};
                    colors = {'g', 'r', 'b', 'm', 'c', 'y', 'k', 'r', 'b'};
                    markers = {'o', 's', 'd', '^', 'v', '>', '<', 'p', 'h', '*'};
                    % line_style = "--";
                    line_width = 1.5;
                    % plot(x_data, y_data, 'Color', colors{plot_idx}, 'LineWidth', 0.5)
                    plot(x_data, y_data, 'LineStyle', lineStyles{plot_idx}, 'Color', colors{plot_idx}, 'Marker', markers{plot_idx}, 'LineWidth', 1.5);
                    plot_idx = plot_idx + 1;
                else
                    continue;
                end
            end
            if plot_without_title
                title("")
            else
                title("Path Tracking Comparison Results")
            end
            % Add border to top and right side
            box on
            xlabel("X(m)")
            ylabel("Y(m)")
            fontsize(18, 'points')
            % Set the axis limits
            if path_name == "manualpath_10"
               xlim([-10 -6]) 
            end
            % Set the legend location based on the path name
            path_names_se = ["path_572", "manualpath_8", "path_592", "path_1019", "path_102", "path_148", "path_210", "path_262", "path_289", "path_388", "path_980", "path_1268", "path_1613", "path_1687", "path_1876"];
            % if any(path_name == path_names_se)
            %     legend(label, 'Location', 'South East')
            % elseif path_name == "manualpath_1"
            %     legend(label, 'Location', 'South West')
            % else
            %     legend(label, 'Location', 'North East')
            % end
            legend(label, 'Location', 'Best', 'FontSize', 14);
            prefix_path_name = path_name_array(cur_idx);
            if obj.use_path_mapping_table
                % If the path name is in the path difficulty table mapping
                path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                % Get the difficulty of the path
                if ~isempty(path_difficulty_index)
                    path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                    prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                end
            end
            fig_name_path = "path_tracking.png";
            fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
            filename = append(obj.plot_export_dir, '/', fig_name_export);
            saveas(gcf, filename);
            % Save the fig file
            fig_name_export = strcat(prefix_path_name, '_', fig_name_path, '.fig');
            filename = append(obj.fig_file_saving_folder, '/', fig_name_export);
            savefig(gcf, filename);
            close(gcf);
        end
        % Function to plot the path projection data at the current time step
        function obj = plot_path_projection_data(obj, path_name, type_lists, time_step)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            path_name_file = append(obj.path_data_from_file,'/', path_name, '.csv');
            path_data = readtable(path_name_file);
            label = [];
            ori_path_x = table2array(path_data(:,1));
            ori_path_y = table2array(path_data(:,2));
            plot_with_time_step_as_color = true;
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Plot the path tracking data
                    figure()
                    hold on
                    grid on
                    axis equal
                    label = [label; "Ref. Path"; "Start"; "Goal"];
                    plot(ori_path_x, ori_path_y, 'k-', 'LineWidth', 2.5)
                    % Plot the start and end point of the path
                    plot(ori_path_x(1), ori_path_y(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
                    plot(ori_path_x(end), ori_path_y(end), 'bx', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
                    % Add current type to the labels
                    % label = obj.get_label_name(current_type);
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    skip_value = 10;
                    % Get the x and y data
                    x_data = cell2mat(current_data.odom_x(1,:));
                    y_data = cell2mat(current_data.odom_y(1,:));
                    % Get the path data
                    path_x = cell2mat(current_data.path_x(1,:));
                    path_y = cell2mat(current_data.path_y(1,:));
                    % Skip the data
                    x_data = x_data(1:skip_value:end);
                    y_data = y_data(1:skip_value:end);
                    path_x = path_x(1:skip_value:end);
                    path_y = path_y(1:skip_value:end);
                    % plot the path projection data
                    plot(x_data, y_data, 'ro-', 'LineWidth', 1.5)  % Use a red dashed line
                    % Plot the path projection data at the current time step
                    plot(path_x, path_y, 'g--', 'LineWidth', 1.5)  % Use a blue line with circle markers
                    % Connect the path projection data with the path data
                    for j = 1:length(x_data)
                        plot([x_data(j), path_x(j)], [y_data(j), path_y(j)], 'k.-', 'LineWidth', 0.5)
                    end
                    label = ["Ref. Path"; "Start"; "Goal"; "Vehicle Path"; "Path Projection"];
                    title("Path Projection of Reference Path")
                    xlabel("X(m)")
                    ylabel("Y(m)")
                    legend(label, 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    fig_name_path = current_type + "_path_projection.png";
                    fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    close(gcf);
                    %% Plot with time step as color bar
                    if plot_with_time_step_as_color
                        figure()
                        hold on
                        grid on
                        axis equal
                        h1 = plot(ori_path_x, ori_path_y, 'k-', 'LineWidth', 2.5); % Handle for 'Ref. Path'
                        % Plot the start and end point of the path
                        h2 = plot(ori_path_x(1), ori_path_y(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Handle for 'Start'
                        h3 = plot(ori_path_x(end), ori_path_y(end), 'bx', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Handle for 'Goal'                    
                        % Plot the path projection data at the current time step
                        h4 = plot(path_x, path_y, 'g--', 'LineWidth', 1.5);  % Handle for 'Path Projection'
                        % Connect the path projection data with the path data
                        for j = 1:length(x_data)
                            plot([x_data(j), path_x(j)], [y_data(j), path_y(j)], 'k.-', 'LineWidth', 0.5)
                        end
                        % Add the color bar
                        time_data = current_data.index_length(1,:);
                        time_data = 0:1:time_data-1;
                        time_data = time_data(1:skip_value:end);
                        % plot the path projection data
                        colormap(jet(max(time_data))); % Use the jet colormap
                        cmap = colormap; % Get the current colormap
                        h5 = []; % Initialize handle array for 'Vehicle Path'
                        for j = 1:length(x_data)-1
                            color_index = max(1, min(size(cmap, 1), round(time_data(j)+1)));
                            h5(j) = plot(x_data(j:j+1), y_data(j:j+1), 'ro-', 'LineWidth', 1.5, 'Color', cmap(color_index, :));
                            if mod(j, skip_value/2) == 0 
                                text(x_data(j) + 0.05, y_data(j), num2str(time_data(j)), 'FontSize', 8, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
                            end
                        end
                        % Create the color bar
                        cb = colorbar('Ticks', linspace(0, 1, 5), 'TickLabels', round(linspace(min(time_data), max(time_data), 5)))
                        ylabel(cb, 'Time Step');
                        title("Path Projection of Reference Path with Time Step")
                        % Rest of your code...
                        xlabel("X(m)")
                        ylabel("Y(m)")
                        legend([h1, h2, h3, h4, h5(1)], {'Ref. Path', 'Start', 'Goal', 'Path Projection', 'Vehicle Path'}, 'Location', 'Best')
                        prefix_path_name = path_name_array(cur_idx);
                        if obj.use_path_mapping_table
                            % If the path name is in the path difficulty table mapping
                            path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                            % Get the difficulty of the path
                            if ~isempty(path_difficulty_index)
                                path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                                prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                            end
                        end
                        fig_name_path = current_type + "_path_projection_time_step.png";
                        fig_name_export = strcat(prefix_path_name, '_', fig_name_path);
                        filename = append(obj.plot_export_dir, '/', fig_name_export);
                        saveas(gcf, filename);
                        close(gcf);
                    end
                end
            end
        end

        %% Function to plot the steering data for the specified path
        function obj = plot_steering_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx,:);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the steering data
                    steering_data = cell2mat(current_data.mpc_steer(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure()
                    grid on
                    hold on
                    plot(time_data, steering_data,'-', 'LineWidth', 1.0)

                    title("Steering Angle")
                    xlabel("Step")
                    ylabel("Steering(rad)")

                    legend("MPC Steering Angle", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "steering.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        function obj = plot_reward_function(obj, w1, w2, w3, w4)
            %% Reward Function plotter
            % If the weights are not specified, set the default values
            if isempty(w1)
                w1 = 4;
            end
            if isempty(w2)
                w2 = 5;
            end
            if isempty(w3)
                w3 = 2;
            end
            if isempty(w4)
                w4 = 8;
            end

            vel_limit = @(x) -(w1*x).^2;
            vel_ref = @(x) -(w2*x).^2;
            head_ref = @(x) -(w3*x).^2;
            traj_err = @(x) -(w4*x).^2;

            x = -10:0.1:10;
            y_vel_limit = vel_limit(x);
            y_vel_ref = vel_ref(x);
            y_head_ref = head_ref(x);
            y_traj_err = traj_err(x);

            figure()
            grid on
            hold on
            xlim([-3 3]);
            plot(x, y_vel_limit);
            plot(x, y_vel_ref);
            plot(x, y_head_ref);
            plot(x, y_traj_err);
            title("Reward Function");
            legend("Velocity Limit", "Velocity Tracking", "Heading Offset", "Lateral Offset", 'Location', 'Best')
            fig_name_path = "reward_function.png";
            filename = append(obj.plot_export_dir, '/', fig_name_path);
            saveas(gcf, filename);
            close(gcf);
        end
        %% Function to plot the path heading data for the specified path
        function obj = plot_heading_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx, :);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the path heading data
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % Get the heading data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure()
                    plot(time_data, path_heading_data, 'k-', 'LineWidth', 1.0)
                    grid on
                    hold on
                    plot(time_data, heading_data,'-', 'LineWidth', 1.0)
                    title("Heading")
                    xlabel("Step")
                    ylabel("Heading(rad)")

                    legend("Ref. Heading", "Vehicle Heading", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "heading.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        %% Function to plot the path heading data and curvature data for the specified path
        function obj = plot_heading_curvature_data(obj, path_name, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx, :);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the path heading data
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % Get the heading data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    % Get the path curvature data
                    path_curvature_data = cell2mat(current_data.path_curv(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure()
                    subplot(2,1,1)
                    plot(time_data, path_heading_data, 'k-', 'LineWidth', 1.0)
                    grid on
                    hold on
                    plot(time_data, heading_data,'-', 'LineWidth', 1.0)
                    title("Heading")
                    xlabel("Step")
                    ylabel("Heading(rad)")

                    legend("Ref. Heading", "Vehicle Heading", 'Location', 'Best')
                    subplot(2,1,2)
                    plot(time_data, path_curvature_data, 'k-', 'LineWidth', 1.0)
                    hold on;
                    % grid on;
                    % Plot the max curvature threshold
                    max_curvature = obj.maximum_curve;
                    % plot(time_data, max_curvature*ones(length(time_data), 1), 'r--', 'LineWidth', 1.0)
                    % plot(time_data, -max_curvature*ones(length(time_data), 1), 'r--', 'LineWidth', 1.0)

                    title("Curvature")
                    xlabel("Step")
                    ylabel("Curvature(1/m)")

                    % legend("Ref. Curvature", "Curvature Threshold", "Curvature Threshold", 'Location', 'Best')
                    legend("Ref. Curvature", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "heading_curvature.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function for plotting the heading error graph
        function obj = plot_heading_error_over_time(obj, path, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx, :);
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the heading error data
                    heading_error_data = cell2mat(current_data.rl_heading_error(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure()
                    plot(time_data, heading_error_data, 'k-', 'LineWidth', 1.0)
                    grid on
                    title("Heading Error Over Time")
                    xlabel("Time Step")
                    ylabel("Heading Error(rad)")
                    legend("Heading Error", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "heading_error_over_time.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        %% Function to plot the velocity data for the specified path
        function obj = plot_velocity_data(obj, path_name_lists, type_lists)
            % Get the data index for the specified path from table_data
            path_name_array = obj.table_data.path_name;
            path_index = find(ismember(path_name_array, path_name_lists));
            % For plotting the data for the specified type
            for i = 1:length(path_index)
                cur_idx = path_index(i);
                % Get the data for each type
                current_type = obj.table_data.type(cur_idx, :);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the path velocity data
                    path_velocity_data = cell2mat(current_data.path_vel_1(1,:));
                    % Get the velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    % Get the mpc velocity data
                    mpc_velocity_data = cell2mat(current_data.mpc_vel(1,:));
                    % Get the time data
                    time_data = current_data.index_length(1,:);
                    time_data = 0:1:time_data-1;
                    figure;
                    plot(time_data, path_velocity_data, 'k-', 'LineWidth', 1.0)
                    grid on
                    hold on
                    plot(time_data, mpc_velocity_data,'-', 'LineWidth', 1.0)
                    plot(time_data, velocity_data,'b-', 'LineWidth', 1.0)

                    title("Velocity Profile")
                    xlabel("Step")
                    ylabel("Speed(m/s)")

                    legend("Traj. Planner Vel", "MPC Vel", "Vehicle Vel", 'Location', 'Best')
                    prefix_path_name = path_name_array(cur_idx);
                    if obj.use_path_mapping_table
                        % If the path name is in the path difficulty table mapping
                        path_difficulty_index = find(strcmp(string(obj.path_difficulty_table_mapping.path_name), prefix_path_name));
                        % Get the difficulty of the path
                        if ~isempty(path_difficulty_index)
                            path_difficulty = string(obj.path_difficulty_table_mapping.difficulty(path_difficulty_index, :));
                            prefix_path_name = append(path_difficulty, '_', prefix_path_name);
                        end
                    end
                    prefix_type = current_type;
                    fig_name_path = "velocity.png";
                    fig_name_export = strcat(prefix_path_name, '_', prefix_type, '_', fig_name_path);
                    filename = append(obj.plot_export_dir, '/', fig_name_export);
                    saveas(gcf, filename);
                    % Close the figure
                    close(gcf);
                end
            end
        end
        % Function for plotting the curvature data for the specified path
        % function obj = plot_curvature_data(obj, path_name_to_calc, type_lists)
        %     % Get the data index for the specified path from table_data
        % end
        % Function for calculating the assessment metrics (MSE, RMSE, Max Error, Mean Error, Std Dev)
        % And export the results to a file
        function obj = calculate_assesement_metrics(obj, path_name_to_calc, type_lists, get_total_metrics, is_success)
            % Default argument for get_total_metrics
            if nargin < 3 || isempty(get_total_metrics)
                get_total_metrics = true;
            end
            % Default argument for mode
            if nargin < 4 || isempty(is_success)
                is_success = true;
            end
            % Assert the type_lists and path name to calc is a string array
            assert(isa(type_lists, 'string'), "The type_lists must be a string array")
            assert(isa(path_name_to_calc, 'string'), "The path_name_to_calc must be a string")
            % Check if the exporting directory is empty or not
            if isempty(obj.metrics_export_dir)
                error("The metrics export directory is empty")
            end
            % Check if the path_name_to_calc is exist in the table_data if not raise an error
            path_name_array = obj.table_data.path_name;
            % If get_total_metrics is true, calculate the total metrics for the whole path
            if get_total_metrics
                path_index = 1:length(path_name_array);
            % Else calculate the metrics for the specified path
            else
                if ~ismember(path_name_to_calc, path_name_array)
                    error("The path name is not exist in the table data")
                end
                % Get the data index for the specified path from table_data
                path_index_logical = ismember(path_name_array, path_name_to_calc);
                path_index = find(path_index_logical);
            end
            % Initialize arrays for storing the metrics
            path_name_array_logging = [];
            type_array = [];
            mse_array = [];
            rmse_array = [];
            max_error_array = [];
            mean_error_array = [];
            mean_velocity_error_array = [];
            mean_heading_error_array = [];
            std_dev_array = [];
            log_mse_array = [];
            log_rmse_array = [];
            log_max_error_array = [];
            log_mean_error_array = [];
            log_std_dev_array = [];
            % For calculating the data for the specified type
            for i = 1:length(path_index)
                fprintf("Total Path: %d\n", length(path_index));
                cur_idx = path_index(i);
                % Get the data for each type
                current_path_name = obj.table_data.path_name(cur_idx,:);
                current_type = obj.table_data.type(cur_idx,:);
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the path x and y data
                    path_x_data = cell2mat(current_data.path_x(1,:));
                    path_y_data = cell2mat(current_data.path_y(1,:));
                    % Get the x and y data
                    x_data = cell2mat(current_data.odom_x(1,:));
                    y_data = cell2mat(current_data.odom_y(1,:));
                    % Get the velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    path_velocity_data = cell2mat(current_data.path_vel_1(1,:));
                    % Get the heading error data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % End point for failure cases
                    end_point = 5;
                    % 10% of length for data
                    % ratio = 0.05;
                    % end_point = round(length(path_x_data)* ratio);
                    if is_success
                        % Get the veloctity error data
                        velocity_error_data = abs(velocity_data - path_velocity_data);
                        % Get the error data
                        error_data = sqrt((path_x_data-x_data).^2 + (path_y_data-y_data).^2);
                        heading_error_data = abs(obj.calculate_heading_error(path_heading_data, heading_data));
                    else
                        % Get error only for the last end_point data
                        velocity_error_data = abs(velocity_data(end-end_point:end) - path_velocity_data(end-end_point:end));
                        error_data = sqrt((path_x_data(end-end_point:end)-x_data(end-end_point:end)).^2 + (path_y_data(end-end_point:end)-y_data(end-end_point:end)).^2);
                        heading_error_data = abs(obj.calculate_heading_error(path_heading_data(end-end_point:end), heading_data(end-end_point:end)));
                    end
                    % Get the assessment metrics
                    mse = mean(error_data);
                    rmse = sqrt(mse);
                    max_error = max(error_data);
                    mean_error = mean(error_data);
                    mean_velocity_error = mean(velocity_error_data);
                    mean_heading_error = mean(heading_error_data);
                    std_dev = std(error_data);
                    % Get the log assessment metrics
                    enable_log = true;
                    log_error_data = log10(error_data);
                    log_mse = mean(log_error_data);
                    log_rmse = sqrt(log_mse);
                    log_max_error = max(log_error_data);
                    log_mean_error = mean(log_error_data);
                    log_std_dev = std(log_error_data);
                    % Get the metrics for the current type
                    fprintf("Path Name: %s\n", current_path_name)
                    fprintf("Type: %s\n", current_type)
                    fprintf("MSE: %f\n", mse)
                    fprintf("RMSE: %f\n", rmse)
                    fprintf("Max Error: %f\n", max_error)
                    fprintf("Std Dev: %f\n", std_dev)
                    fprintf("Mean Velocity Error: %f\n", mean_velocity_error)
                    fprintf("Mean Heading Error: %f\n", mean_heading_error)
                    fprintf("\n")
                    if enable_log
                        fprintf("Log MSE: %f\n", log_mse)
                        fprintf("Log RMSE: %f\n", log_rmse)
                        fprintf("Log Max Error: %f\n", log_max_error)
                        fprintf("Log Std Dev: %f\n", log_std_dev)
                        fprintf("\n")
                    end
                    % Append the metrics to the arrays
                    path_name_array_logging = [path_name_array_logging; current_path_name];
                    type_array = [type_array; current_type];
                    mse_array = [mse_array; mse];
                    rmse_array = [rmse_array; rmse];
                    max_error_array = [max_error_array; max_error];
                    mean_error_array = [mean_error_array; mean_error];
                    mean_velocity_error_array = [mean_velocity_error_array; mean_velocity_error];
                    mean_heading_error_array = [mean_heading_error_array; mean_heading_error];
                    std_dev_array = [std_dev_array; std_dev];
                    log_mse_array = [log_mse_array; log_mse];
                    log_rmse_array = [log_rmse_array; log_rmse];
                    log_max_error_array = [log_max_error_array; log_max_error];
                    log_mean_error_array = [log_mean_error_array; log_mean_error];
                    log_std_dev_array = [log_std_dev_array; log_std_dev];
                end
            end
            % Export the metrics to a file
            if get_total_metrics
                % Create a table for the metrics
                metrics_table = table(path_name_array_logging, type_array, mse_array, rmse_array, max_error_array, mean_error_array, std_dev_array, ...
                                      log_mse_array, log_rmse_array, log_max_error_array, log_mean_error_array, log_std_dev_array, ...
                                      mean_heading_error_array, mean_velocity_error_array);
                % Check if the current type is in the type_lists
                joint_path_metrics_table = table();
                % init array for storing the metrics
                for i = 1:obj.variable_sampling_total
                    eval(strcat("MSE_avg_DRL", num2str(i), "= [];"));
                    eval(strcat("max_error_avg_DRL", num2str(i), "= [];"));
                    eval(strcat("std_dev_avg_DRL", num2str(i), "= [];"));
                    eval(strcat("vel_error_avg_DRL", num2str(i), "= [];"));
                    eval(strcat("heading_error_avg_DRL", num2str(i), "= [];"));
                end
                % Export the metrics to a file
                filename = append(obj.metrics_export_dir, 'assessment_metrics_total.txt');
                writetable(metrics_table, filename);
                % Print the average MSE for each type
                fprintf("Average MSE and Max Error for each type\n")
                % Export the total Averaged metrics to txt file
                filename = append(obj.metrics_export_dir, 'assessment_metrics_summary.txt');
                avg_metrics_file = fopen(filename, 'w');
                for i = 1:length(type_lists)
                    current_type = type_lists(i);
                    % Get number of DRL (only for DRL type)
                    match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                    % type_array
                    type_index = type_array == current_type;
                    mse_avg = mean(mse_array(type_index));
                    avg_max_error = mean(max_error_array(type_index));
                    std_dev_avg = mean(std_dev_array(type_index));
                    heading_error_avg = mean(mean_heading_error_array(type_index));
                    vel_error_avg = mean(mean_velocity_error_array(type_index));
                    % Assign the average metrics to the array
                    if ~isempty(match_exp)
                        n_total = str2double(match_exp{1});
                        eval(strcat("MSE_avg_DRL", num2str(n_total), "= [MSE_avg_DRL", num2str(n_total), "; mse_avg];"));
                        eval(strcat("max_error_avg_DRL", num2str(n_total), "= [max_error_avg_DRL", num2str(n_total), "; avg_max_error];"));
                        eval(strcat("std_dev_avg_DRL", num2str(n_total), "= [std_dev_avg_DRL", num2str(n_total), "; std_dev_avg];"));
                        eval(strcat("vel_error_avg_DRL", num2str(n_total), "= [vel_error_avg_DRL", num2str(n_total), "; vel_error_avg];"));
                        eval(strcat("heading_error_avg_DRL", num2str(n_total), "= [heading_error_avg_DRL", num2str(n_total), "; heading_error_avg];"));
                    end
                    fprintf("Type: %s, Average MSE: %f\n", current_type, mse_avg)
                    fprintf("Type: %s, Average Max Error: %f\n", current_type, avg_max_error)
                    fprintf("Type: %s, Average Std Dev: %f\n", current_type, std_dev_avg)
                    fprintf("Type: %s, Average Velocity Error: %f\n", current_type, vel_error_avg)
                    fprintf("Type: %s, Average Heading Error: %f\n", current_type, heading_error_avg)
                    % Export the average metrics to a file
                    fprintf(avg_metrics_file, "Type: %s, Average MSE: %f\n", current_type, mse_avg);
                    fprintf(avg_metrics_file, "Type: %s, Average Max Error: %f\n", current_type, avg_max_error);
                    fprintf(avg_metrics_file, "Type: %s, Average Std Dev: %f\n", current_type, std_dev_avg);
                    fprintf(avg_metrics_file, "Type: %s, Average Velocity Error: %f\n", current_type, vel_error_avg);
                    fprintf(avg_metrics_file, "Type: %s, Average Heading Error: %f\n", current_type, heading_error_avg);
                end
                % Print the average MSE for each type
                fprintf("Average MSE and Max Error for each DRL type\n")
                for i = 1:length(type_lists)
                    current_type = type_lists(i);
                    % Get number of DRL
                    match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                    % Get mean MSE for each DRL type
                    if ~isempty(match_exp)
                        n_total = str2double(match_exp{1});
                        eval(strcat("mse_avg = mean(MSE_avg_DRL", num2str(n_total), ");"));
                        eval(strcat("avg_max_error = mean(max_error_avg_DRL", num2str(n_total), ");"));
                        eval(strcat("std_dev_avg = mean(std_dev_avg_DRL", num2str(n_total), ");"));
                        eval(strcat("vel_error_avg = mean(vel_error_avg_DRL", num2str(n_total), ");"));
                        eval(strcat("heading_error_avg = mean(heading_error_avg_DRL", num2str(n_total), ");"));
                        fprintf("Type: %s, Average MSE: %f\n", current_type, mse_avg)
                        fprintf("Type: %s, Average Max Error: %f\n", current_type, avg_max_error)
                        fprintf("Type: %s, Average Std Dev MSE: %f\n", current_type, std_dev_avg)
                        fprintf("Type: %s, Average Velocity Error: %f\n", current_type, vel_error_avg)
                        fprintf("Type: %s, Average Heading Error: %f\n", current_type, heading_error_avg)
                    end
                end
                % Assign the average metrics to the table
                for i = 1:obj.variable_sampling_total
                    if ~isempty(eval(strcat("MSE_avg_DRL", num2str(i))))
                        eval(strcat("joint_path_metrics_table.DRL", num2str(i), "_MSE = MSE_avg_DRL", num2str(i), ";"));
                        eval(strcat("joint_path_metrics_table.DRL", num2str(i), "_max_error = max_error_avg_DRL", num2str(i), ";"));
                        eval(strcat("joint_path_metrics_table.DRL", num2str(i), "_std_dev = std_dev_avg_DRL", num2str(i), ";"));
                    end
                end
                obj.joint_path_metrics_table = joint_path_metrics_table;
            else
                % Create a table for the metrics
                metrics_table = table(path_name_array_logging, type_array, mse_array, rmse_array, max_error_array, std_dev_array, log_mse_array, log_rmse_array, log_max_error_array, log_std_dev_array);
                % Export the metrics to a file
                filename = append(obj.metrics_export_dir, 'assessment_metrics.txt');
                writetable(metrics_table, filename);
            end
        end
         %% Function for calculating metrics for each difficulty level
        function obj = calculate_assesement_metrics_per_difficulty(obj, type_lists, get_total_metrics, is_success)
            % Default argument for get_total_metrics
            if nargin < 3 || isempty(get_total_metrics)
                get_total_metrics = true;
            end
            if nargin < 4 || isempty(is_success)
                is_success = true;
            end
            % Check if the exporting directory is empty or not
            if isempty(obj.metrics_export_dir)
                error("The metrics export directory is empty")
            end
            % Check if the difficulty mapping is empty or not
            if isempty(obj.path_difficulty_table_mapping)
                error("The difficulty mapping is empty")
            end
            % Get the path name array
            path_name_array = obj.table_data.path_name;
            % Get the unique path name
            unique_path_name = unique(path_name_array);
            % Get the unique path name length
            unique_path_name_length = length(unique_path_name);
            % Get the unique type name
            unique_type_name = unique(type_lists);
            % Get the unique type name length
            unique_type_name_length = length(unique_type_name);
            % Initialize the arrays for storing the metrics
            difficulty_lists = ['E', 'M', 'H'];
            VariableTypes = ['string', 'string', repmat({'double'}, 1, 12)];
            % Create a table for storing the metrics
            difficulty_metrics_table = table('Size',[0 14],...
                'VariableTypes', VariableTypes,...
                'VariableNames', {'path_name', 'type', 'difficulty', 'mse', 'rmse', 'max_error', 'mean_error', 'velocity_error', 'heading_error', 'std_error', 'log_mse', 'log_rmse', 'log_max_error', 'log_mean_error'});
            % For calculating the data for the specified type assign the metrics to the table
            % For each path obtain the metrics
            % For calculating the data for the specified type
            for i = 1:length(path_name_array)
                fprintf("Total Path: %d\n", length(path_name_array));
                cur_idx = i;
                % Get the data for each type
                current_path_name = obj.table_data.path_name(cur_idx,:);
                current_type = obj.table_data.type(cur_idx,:);
                % Get difficulty from the path name mapping
                difficulty = string(obj.path_difficulty_table_mapping(strcmp(string(obj.path_difficulty_table_mapping.path_name), current_path_name), :).difficulty);
                % Check if the current type is in the type_lists
                if ismember(current_type, type_lists)
                    % Get the data for the current type
                    current_data = obj.table_data(cur_idx, :);
                    % Get the path x and y data
                    path_x_data = cell2mat(current_data.path_x(1,:));
                    path_y_data = cell2mat(current_data.path_y(1,:));
                    % Get the x and y data
                    x_data = cell2mat(current_data.odom_x(1,:));
                    y_data = cell2mat(current_data.odom_y(1,:));
                    % Get the velocity data
                    velocity_data = cell2mat(current_data.odom_vel(1,:));
                    path_velocity_data = cell2mat(current_data.path_vel_1(1,:));
                    % Get the heading error data
                    heading_data = cell2mat(current_data.odom_yaw(1,:));
                    path_heading_data = cell2mat(current_data.path_yaw(1,:));
                    % End point for failure cases
                    end_point = 5;
                    % 10% of length for data
                    % ratio = 0.05;
                    % end_point = round(length(path_x_data)* ratio);
                    if is_success
                        % Get the veloctity error data
                        velocity_error_data = abs(velocity_data - path_velocity_data);
                        % Get the error data
                        error_data = sqrt((path_x_data-x_data).^2 + (path_y_data-y_data).^2);
                        heading_error_data = abs(obj.calculate_heading_error(path_heading_data, heading_data));
                    else
                        % Get error only for the last end_point data
                        velocity_error_data = abs(velocity_data(end-end_point:end) - path_velocity_data(end-end_point:end));
                        error_data = sqrt((path_x_data(end-end_point:end)-x_data(end-end_point:end)).^2 + (path_y_data(end-end_point:end)-y_data(end-end_point:end)).^2);
                        heading_error_data = abs(obj.calculate_heading_error(path_heading_data(end-end_point:end), heading_data(end-end_point:end)));
                    end
                    % Get the assessment metrics
                    mse = mean(error_data);
                    rmse = sqrt(mse);
                    max_error = max(error_data);
                    mean_error = mean(error_data);
                    mean_velocity_error = mean(velocity_error_data);
                    mean_heading_error = mean(heading_error_data);
                    std_dev = std(error_data);
                    % Get the log assessment metrics
                    enable_log = true;
                    log_error_data = log10(error_data);
                    log_mse = mean(log_error_data);
                    log_rmse = sqrt(log_mse);
                    log_max_error = max(log_error_data);
                    log_mean_error = mean(log_error_data);
                    log_std_dev = std(log_error_data);
                    % Assign each metrics to the table
                    % Create a new row as a table
                    current_path_name = cellstr(current_path_name);
                    current_type = cellstr(current_type);
                    difficulty = difficulty;
                    
                    newData.path_name = current_path_name;
                    newData.type = current_type;
                    newData.difficulty = difficulty;
                    newData.mse = mse;
                    newData.rmse = rmse;
                    newData.max_error = max_error;
                    newData.mean_error = mean_error;
                    newData.velocity_error = mean_velocity_error;
                    newData.heading_error = mean_heading_error;
                    newData.std_error = std_dev;
                    newData.log_mse = log_mse;
                    newData.log_rmse = log_rmse;
                    newData.log_max_error = log_max_error;
                    newData.log_mean_error = log_mean_error;
                    % Add the new row to the table
                    difficulty_metrics_table = [difficulty_metrics_table; struct2table(newData)];
                end
            end
            % Assign to the class properties
            obj.difficulty_metrics_table = difficulty_metrics_table;
            % Categorize the metrics based on the difficulty
            % Create array for DRL type
            for i = 1:obj.variable_sampling_total
                eval(strcat("MSE_avg_DRL", num2str(i), "= [];"));
                eval(strcat("max_error_avg_DRL", num2str(i), "= [];"));
                eval(strcat("std_dev_avg_DRL", num2str(i), "= [];"));
                eval(strcat("vel_error_avg_DRL", num2str(i), "= [];"));
                eval(strcat("heading_error_avg_DRL", num2str(i), "= [];"));
            end
            % For each difficulty level
            for i = 1:length(difficulty_lists)
                current_difficulty = difficulty_lists(i);
                % Get the metrics for the current difficulty
                current_difficulty_metrics = difficulty_metrics_table(strcmp(string(difficulty_metrics_table.difficulty), current_difficulty), :);
                % Get the Mean Metrics for each type
                for j = 1:length(unique_type_name)
                    current_type = unique_type_name(j);
                    % Get the metrics for the current type
                    current_type_metrics = current_difficulty_metrics(strcmp(string(current_difficulty_metrics.type), current_type), :);
                    % Get the mean metrics for the current type
                    mean_mse = mean(current_type_metrics.mse);
                    mean_rmse = mean(current_type_metrics.rmse);
                    mean_max_error = mean(current_type_metrics.max_error);
                    mean_mean_error = mean(current_type_metrics.mean_error);
                    mean_velocity_error = mean(current_type_metrics.velocity_error);
                    mean_heading_error = mean(current_type_metrics.heading_error);
                    mean_std_error = mean(current_type_metrics.std_error);
                    mean_log_mse = mean(current_type_metrics.log_mse);
                    mean_log_rmse = mean(current_type_metrics.log_rmse);
                    mean_log_max_error = mean(current_type_metrics.log_max_error);
                    mean_log_mean_error = mean(current_type_metrics.log_mean_error);
                    % Print the metrics
                    fprintf("Difficulty: %s, Type: %s\n", current_difficulty, current_type)
                    fprintf("MSE: %f\n", mean_mse)
                    fprintf("RMSE: %f\n", mean_rmse)
                    fprintf("Max Error: %f\n", mean_max_error)
                    fprintf("Mean Error: %f\n", mean_mean_error)
                    fprintf("Velocity Error: %f\n", mean_velocity_error)
                    fprintf("Heading Error: %f\n", mean_heading_error)
                    fprintf("Std Dev: %f\n", mean_std_error)
                    fprintf("Log MSE: %f\n", mean_log_mse)
                    fprintf("Log RMSE: %f\n", mean_log_rmse)
                    fprintf("Log Max Error: %f\n", mean_log_max_error)
                    fprintf("Log Mean Error: %f\n", mean_log_mean_error)
                    fprintf("\n")
                    % if type is DRL append to the lists
                    match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                    if ~isempty(match_exp)
                        n_total = str2double(match_exp{1});
                        eval(strcat("MSE_avg_DRL", num2str(n_total), "= [MSE_avg_DRL", num2str(n_total), "; mean_mse];"));
                        eval(strcat("max_error_avg_DRL", num2str(n_total), "= [max_error_avg_DRL", num2str(n_total), "; mean_max_error];"));
                        eval(strcat("std_dev_avg_DRL", num2str(n_total), "= [std_dev_avg_DRL", num2str(n_total), "; mean_std_error];"));
                        eval(strcat("vel_error_avg_DRL", num2str(n_total), "= [vel_error_avg_DRL", num2str(n_total), "; mean_velocity_error];"));
                        eval(strcat("heading_error_avg_DRL", num2str(n_total), "= [heading_error_avg_DRL", num2str(n_total), "; mean_heading_error];"));
                    end
                end
                % For each variable sampling total
                fprintf("DRL Average Metrics for Current Difficulty %s \n", current_difficulty);
                for j = 1:obj.variable_sampling_total
                    if ~isempty(eval(strcat("MSE_avg_DRL", num2str(j))))
                        fprintf("DRL %d \n", j);
                        fprintf("MSE Average %f\n", mean(eval(strcat("MSE_avg_DRL", num2str(j)))));
                        fprintf("Max Error Average %f\n", mean(eval(strcat("max_error_avg_DRL", num2str(j)))));
                        fprintf("Std Dev Average %f\n", mean(eval(strcat("std_dev_avg_DRL", num2str(j)))));
                        fprintf("Velocity Error Average %f\n", mean(eval(strcat("vel_error_avg_DRL", num2str(j)))));
                        fprintf("Heading Error Average %f\n", mean(eval(strcat("heading_error_avg_DRL", num2str(j)))));
                        fprintf("\n");
                    end
                end
            end
        end
        % Obtaining the maximum values accross the state cost in failure case
        function obj = get_the_maximum_values_of_cost_in_failure(obj, type_lists, path_name_lists)
            % Check if the exporting directory is empty or not
            if isempty(obj.metrics_export_dir)
                error("The metrics export directory is empty")
            end
            % If the path name lists is not provided as an input argument, use the whole paths
            if nargin < 2
                type_lists = obj.table_data.type;
                % Get the string unique values of the type
                type_lists = unique(type_lists);
            end
            % If the type lists is not provided as an input argument, use the whole type lists
            if nargin < 3
                path_name_lists = obj.table_data.path_name;
                % Get the string unique values of the path name
                path_name_lists = unique(path_name_lists);
            end
            threshold_val = 0.5;
            window_size = 10;
            max_gap_of_increasing = 2;
            threshold_for_valid_length = 20;
            % Logging files for saving the maximum value
            txt_file_location = append(obj.metrics_export_dir, 'max_state_cost_failure.txt');
            % For each type get the maximum values of the cost
            for i = 1:length(type_lists)
                % Create array for saving the maximum values
                max_array_current_type = [];
                % Get the table for the current type
                current_type = type_lists(i);
                % Get the data index for the current type
                type_index = find(ismember(obj.table_data.type, current_type));
                % Get the data for the current type
                current_type_data = obj.table_data(type_index, :);
                % For each path get the maximum values of the cost
                for j = 1:length(path_name_lists)
                    % Get the current path index
                    current_path_name = path_name_lists(j);
                    % Get the data index for the current path
                    path_index = find(ismember(current_type_data.path_name, current_path_name));
                    % Get the data for the current path
                    current_path_data = current_type_data(path_index, :);
                    % Get the state cost data
                    state_cost_data = cell2mat(current_path_data.state_cost(1,:));
                    % Get the maximum values of the state cost
                    index_state_cost_deviate_lists = obj.detect_changes(state_cost_data, window_size, threshold_val);
                    start_index_to_deviate = obj.find_longest_consecutive_start_dynamic(index_state_cost_deviate_lists, max_gap_of_increasing, threshold_for_valid_length);
                    % Obtain the maximum values of the state cost based on the state before the index of deviation
                    state_max_cost = max(state_cost_data(1:start_index_to_deviate-1));
                    % Append it to the max array type for current type
                    max_array_current_type = [max_array_current_type; state_max_cost];
                end
                % Log the maximum value into txt files for the current type
                fileID = fopen(txt_file_location, 'a');
                fprintf(fileID, "Type: %s\n", current_type);
                fprintf(fileID, "Max State Cost: %f\n", max(max_array_current_type));
                fprintf(fileID, "\n");
                fclose(fileID);
            end
        end
        function obj = calculate_success_rate_per_difficulty(obj, type_lists)
            % Check if the exporting directory is empty or not
            if isempty(obj.metrics_export_dir)
                error("The metrics export directory is empty")
            end
            % Check if the difficulty mapping is empty or not
            if isempty(obj.path_difficulty_table_mapping)
                error("The difficulty mapping is empty")
            end
            % Get the path name array
            path_name_array = obj.table_data.path_name;
            % Get the unique path name
            unique_path_name = unique(path_name_array);
            % Get the unique path name length
            unique_path_name_length = length(unique_path_name);
            % Get the unique type name
            unique_type_name = unique(type_lists);
            % Get the unique type name length
            unique_type_name_length = length(unique_type_name);
            % Initialize the arrays for storing the metrics
            difficulty_lists = ['E', 'M', 'H'];
            VariableTypes = [repmat({'string'}, 1, 4)];
            % Create a table for storing the metrics
            success_metrics_table = table('Size',[0 4],...
                'VariableTypes', VariableTypes,...
                'VariableNames', {'path_name', 'type', 'difficulty', 'verdict'});
            % For each difficulty get the success rate of each type
            for i = 1:length(difficulty_lists)
                current_difficulty = difficulty_lists(i);
                % Get the metrics for the current difficulty
                current_difficulty_metrics = obj.path_difficulty_table_mapping(strcmp(string(obj.path_difficulty_table_mapping.difficulty), current_difficulty), :);
                % Get only current difficulty path list
                current_difficulty_path_list = unique(current_difficulty_metrics.path_name);
                % Get the index of table that contains the current difficulty
                current_difficulty_path_lists_from_table = obj.table_data(ismember(obj.table_data.path_name, current_difficulty_path_list), :);
                % Get length of the table
                table_length = length(current_difficulty_path_lists_from_table.path_name);
                % For each path in the current difficulty
                for j = 1:table_length
                    current_path_name = current_difficulty_path_lists_from_table.path_name(j);
                    current_type = current_difficulty_path_lists_from_table.type(j);
                    current_verdict = current_difficulty_path_lists_from_table.verdict(j);
                    % Append the metrics to the table
                    % Create a new row as a table
                    current_path_name = cellstr(current_path_name);
                    current_type = cellstr(current_type);
                    current_verdict = cellstr(current_verdict);
                    newData.path_name = current_path_name;
                    newData.type = current_type;
                    newData.difficulty = current_difficulty;
                    newData.verdict = current_verdict;
                    % Add the new row to the table
                    success_metrics_table = [success_metrics_table; struct2table(newData)];
                end
            end
            % Count the success rate for each type and difficulty
            % For each difficulty
            for i = 1:length(difficulty_lists)
                current_difficulty = difficulty_lists(i);
                % Get the metrics for the current difficulty
                current_difficulty_metrics = success_metrics_table(strcmp(string(success_metrics_table.difficulty), current_difficulty), :);
                % Get the unique type name
                unique_type_name = unique(current_difficulty_metrics.type);
                % Get the unique type name length
                unique_type_name_length = length(unique_type_name);
                % Create array for storing succes rate of DRL method
                for j = 1:obj.variable_sampling_total
                    eval(strcat("success_rate_DRL", num2str(j), "= [];"));
                end
                % For each type
                for j = 1:unique_type_name_length
                    current_type = unique_type_name(j);
                    % Get the metrics for the current type
                    current_type_metrics = current_difficulty_metrics(strcmp(string(current_difficulty_metrics.type), current_type), :);
                    % Get the success rate
                    success_rate = sum(strcmp(string(current_type_metrics.verdict), "success"))/height(current_type_metrics);
                    % Print the success rate
                    fprintf("Difficulty: %s, Type: %s\n", current_difficulty, current_type)
                    fprintf("Success Rate: %f\n", success_rate)
                    fprintf("\n")
                    if contains(current_type, 'DRL')
                        % Get number of DRL
                        match_exp = regexp(current_type, '^DRL(\d)', 'tokens');
                        if ~isempty(match_exp)
                            n_total = str2double(match_exp{1});
                            eval(strcat("success_rate_DRL", num2str(n_total), "= [success_rate_DRL", num2str(n_total), "; success_rate];"));
                        end
                    end
                end
                % For each variable sampling total
                fprintf("DRL Success Rate for Current Difficulty %s \n", current_difficulty);
                for j = 1:obj.variable_sampling_total
                    % Print Mean Success Rate
                    if ~isempty(eval(strcat("success_rate_DRL", num2str(j))))
                        fprintf("DRL %d \n", j);
                        fprintf("Success Rate Average %f\n", mean(eval(strcat("success_rate_DRL", num2str(j)))));
                        fprintf("\n");
                    end
                end
            end
        end
        %% Helper function to gradually reduce the threshold for getting the starting index
        function start_index = find_longest_consecutive_start_dynamic(obj, indices, max_gap, max_threshold)
            threshold = max_threshold;
            found = false;
          
            while ~found && threshold > 0
              start_index = obj.find_latest_consecutive_start(indices, max_gap, threshold);
              if ~isnan(start_index)
                found = true;
              else
                threshold = threshold - 1;
              end
            end
        end
    end
    methods(Static)
        %% Function to parse the file names to get the type, verdict, and path name %%
        function [type, verdict, path_name] = parse_file_names(file)
            % Parse the file names to get the type, verdict, and path name %%
            % Split the file name with the delimiter '_' (type_pathname_verdict.csv)
            split_results = split(file, '_');
            % Removes the .csv extension and combine the path name with the rest of the split results
            split_results = erase(split_results, '.csv');
            % Combine the path name with the rest of the split results
            split_results(3) = strcat(split_results(3), '_', split_results(4));
            type = split_results(1);
            verdict = split_results(2);
            path_name = split_results(3);
        end
        function heading_error_data = calculate_heading_error(path_heading_data, heading_data)
            % Calculate the heading error data
            heading_error_data = path_heading_data - heading_data;
            % Check for the heading error data that is greater than pi
            heading_error_data(heading_error_data > pi) = heading_error_data(heading_error_data > pi) - 2*pi;
            heading_error_data(heading_error_data < -pi) = heading_error_data(heading_error_data < -pi) + 2*pi;
        end
        %% Get the label name based on the type%%
        function label = get_label_name(type)
            % Match regex for the DRL type
            match_exp = regexp(type, '^DRL(\d)', 'tokens');
            if type == "uniform"
                label = "U-MPC";
            elseif type == "nonunisparsevar"
                % label = "Variable Sparse MPC [1]";
                label = "VSM-MPC";
            elseif type == "DRL"
                label = "DRL-MPC";
            elseif type == "fsmmpc4"
                label = "FS-MPC";
            elseif ~isempty(match_exp)
                n_total = str2double(match_exp{1});
                % label = append("NUDRL-MPC", " S", num2str(n_total));
                label = append("NUDRL-MPC");
            else
                raise(MException('MATLAB:invalidType', 'The type of MPC is invalid'));
            end
        end
        %% Calculate the maximum moving around the objective function to get the maximum values spread around the objective cost
        function change_points = detect_changes(data, window_size, threshold_factor)
            % Calculate maximum values for each window
            max_values = movmax(data, window_size);
            
            % Calculate rate of change of maximum values
            rate_of_change = diff(max_values);
            
            % Calculate threshold (e.g., using standard deviation)
            threshold = mean(rate_of_change) + threshold_factor * std(rate_of_change);
            
            % Find change points
            change_points = find(rate_of_change > threshold);
        end
        %% Helper function for creating lists of model to evaluate 
        function types = get_the_lists_of_types_to_check(type, number_of_model_for_each_type)
            % Count the DRL type based of the number of model
            types = []
            % Let's say type DRL2 has 4 then it will have types of DRL2, DRL21, DRL22, DRL23
            for i = 1:length(type)
                for num = 1:number_of_model_for_each_type(i)
                    if num == 1
                        cur_type = type(i);
                    else
                        cur_type = type(i) + num2str(num-1); 
                    end
                    types = [types, cur_type]
                end
            end
        end
        %% Helper function to dtermine the starting index of the failure paths deviation
        function start_index = find_latest_consecutive_start(indices, max_gap, threshold)
            diff_indices = diff(indices);
            is_consecutive = diff_indices <= max_gap;
            modified_diff = diff_indices;
            %  modified_diff(~is_consecutive) = 0;
          
            % Find group boundaries
            group_starts = find(diff([false, is_consecutive]) == max_gap);
            group_ends = find(diff([is_consecutive, false]) == -1);
          
            % Calculate group lengths
            group_lengths = group_ends - group_starts + 1;
          
             % Find the index of the longest group, prioritizing the latest one
            [max_length, max_index] = max(group_lengths);
            latest_long_group_index = find(group_lengths == max_length, 1, 'last');
            if max_length >= threshold
                start_index = indices(group_starts(latest_long_group_index));
            else
                start_index = NaN; % Or handle this case as needed
            end
        end
    end
end