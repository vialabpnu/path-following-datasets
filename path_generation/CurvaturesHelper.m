classdef CurvaturesHelper
    properties
        max_curvature_threshold = 0.18; % Default maximum curvature
        max_curvature_change_boundary = 0.0881 % From previous datasetds measuraement
        number_of_curvature_changes_boundary = 5.5; % From previous datasets measurement
        min_distance_threshold = 10; % Default minimum distance in meters
        map_for_curvatures_difficulty
        map_for_max_curvature_change
        map_for_number_of_curvature_changes
    end
    methods
        function obj = CurvaturesHelper()
            obj.map_for_curvatures_difficulty = containers.Map(...
                {'easy', 'moderate', 'hard'}, ...
                {[0, 0.1], [0.1, obj.max_curvature_threshold], [obj.max_curvature_threshold, obj.max_curvature_threshold + eps]});
            
            obj.map_for_max_curvature_change = containers.Map(...
                {'easy', 'moderate', 'hard'}, ...
                {[0, obj.max_curvature_change_boundary - obj.max_curvature_change_boundary*0.1], [obj.max_curvature_change_boundary + obj.max_curvature_change_boundary*0.1, 2 * obj.max_curvature_threshold], [obj.max_curvature_change_boundary + obj.max_curvature_change_boundary*0.1, 2 * obj.max_curvature_threshold + eps]});
            
            obj.map_for_number_of_curvature_changes = containers.Map(...
                {'easy', 'moderate', 'hard'}, ...
                {[0, 1], [2, 6], [6, 11]});
        end
        
        function curvatures = calculate_curvature_using_circle(obj, x, y)
            curvatures = [];
            x = x(:)';
            y = y(:)';
            for i = 2:length(x) - 1
                x1 = x(i - 1);
                y1 = y(i - 1);
                x2 = x(i);
                y2 = y(i);
                x3 = x(i + 1);
                y3 = y(i + 1);
                [R, curvature_sign] = obj.find_circle(x1, y1, x2, y2, x3, y3);
                if isempty(R)
                    curvatures = [curvatures, 0];
                else
                    curvatures = [curvatures, curvature_sign / R];
                end
            end
            curvatures = [curvatures, curvatures(end)];
        end
        
        function [R, curvature_sign] = find_circle(~, x1, y1, x2, y2, x3, y3)
            % Calculate A, B, C, D
            A = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2;
            B = (x1^2 + y1^2)*(y3 - y2) + (x2^2 + y2^2)*(y1 - y3) + (x3^2 + y3^2)*(y2 - y1);
            C = (x1^2 + y1^2)*(x2 - x3) + (x2^2 + y2^2)*(x3 - x1) + (x3^2 + y3^2)*(x1 - x2);
            D = (x1^2 + y1^2)*(x3*y2 - x2*y3) + (x2^2 + y2^2)*(x1*y3 - x3*y1) + (x3^2 + y3^2)*(x2*y1 - x1*y2);
            
            % Check if points are collinear (A should not be zero)
            if A == 0
                R = [];
                curvature_sign = 1;
                return;
            end
            
            radius_limit = 0.5;
            
            % Calculate center (xc, yc) and radius R
            xc = -B / (2 * A);
            yc = -C / (2 * A);
            if B^2 + C^2 - 4*A*D < 0
                R = [];
                curvature_sign = 1;
                return;
            else
                R = sqrt((B^2 + C^2 - 4*A*D) / (4*A^2));
            end
            
            % Calculate the vectors
            v1 = [x2 - x1, y2 - y1];
            v2 = [x3 - x2, y3 - y2];
            
            % Calculate the cross product
            cp = v1(1) * v2(2) - v1(2) * v2(1);
            
            % The sign of the z-component of the cross product gives the direction of curvature
            curvature_sign = sign(cp);
            
            if R < radius_limit
                R = [];
            end
        end
        
        function is_valid = validate_difficulty(obj, x, y, expected_difficulty)
            % Calculate curvatures
            curvatures = obj.calculate_curvature_using_circle(x, y);
            
            max_curvature = obj.calculate_max_curvature(curvatures);
            max_curvature_change = obj.calculate_max_curvature_change(curvatures);
            num_curvature_changes = obj.find_mode_change(curvatures);
            path_distance = obj.calculate_path_distance(x, y);
            
            % Floor max curvature to 2 decimal places
            max_curvature = floor(max_curvature * 100) / 100;
            
            % Validate max curvature
            curvature_range = obj.map_for_curvatures_difficulty(expected_difficulty);
            if strcmp(expected_difficulty, 'hard') || strcmp(expected_difficulty, 'moderate')
                is_valid_curvature = max_curvature >= curvature_range(1) && max_curvature <= curvature_range(2);
            else
                is_valid_curvature = max_curvature >= curvature_range(1) && max_curvature < curvature_range(2);
            end
            
            % Validate max curvature change
            curvature_change_range = obj.map_for_max_curvature_change(expected_difficulty);
            if strcmp(expected_difficulty, 'hard') || strcmp(expected_difficulty, 'moderate')
                is_valid_curvature_change = max_curvature_change >= curvature_change_range(1) && max_curvature_change <= curvature_change_range(2);
            else
                is_valid_curvature_change = max_curvature_change >= curvature_change_range(1) && max_curvature_change < curvature_change_range(2);
            end
            
            % Validate number of curvature changes
            curvature_changes_range = obj.map_for_number_of_curvature_changes(expected_difficulty);
            if strcmp(expected_difficulty, 'hard')
                is_valid_curvature_changes = num_curvature_changes >= curvature_changes_range(1) && num_curvature_changes <= curvature_changes_range(2);
            else
                is_valid_curvature_changes = num_curvature_changes >= curvature_changes_range(1) && num_curvature_changes < curvature_changes_range(2);
            end
            
            % Validate minimum distance
            is_valid_distance = path_distance >= obj.min_distance_threshold;
            
            % Overall validation
            is_valid = is_valid_curvature && is_valid_curvature_change && is_valid_curvature_changes && is_valid_distance;
        end
        
        function distance = calculate_path_distance(~, x, y)
            distance = sum(sqrt(diff(x).^2 + diff(y).^2));
        end
        
        function angle = getAngleBetweenTwoLineSeg(~, l1p1, l1p2, l2p1, l2p2)
            l1 = l1p2 - l1p1;
            l1 = l1 / norm(l1);
            l2 = l2p2 - l2p1;
            l2 = l2 / norm(l2);
            dot_product = dot(l1, l2);
            % Clip the dot product to avoid numerical issues with acos
            dot_product = max(min(dot_product, 1.0), -1.0);
            angle = acos(dot_product);
        end
        
        function [pathList, cuspPointIdx] = segmentPathByCuspPoint(obj, path_x, path_y, minCuspPointAngleDeg)
            if nargin < 4
                minCuspPointAngleDeg = 90;
            end
            path = [path_x(:), path_y(:)];
            pathLen = size(path, 1);
            if pathLen < 3
                pathList = {path};
                cuspPointIdx = [];
                return;
            end
            lineSegAngle = arrayfun(@(i) obj.getAngleBetweenTwoLineSeg(path(i, :), path(i+1, :), path(i+1, :), path(i+2, :)), 1:pathLen-2);
            cuspPointIdx = find(lineSegAngle >= deg2rad(minCuspPointAngleDeg)) + 1; % +1 because cusp point is in the middle of two line segments
            cuspPointIdx = cuspPointIdx(:)'; % Ensure cuspPointIdx is a row vector
            pathPoint = [1, cuspPointIdx, pathLen];

            pathList = cell(1, length(pathPoint) - 1);
            for i = 1:length(pathPoint) - 1
                pathList{i} = path(pathPoint(i):pathPoint(i+1), :);
            end
        end
        
        function [path_x, path_y, path_yaw] = handleCuspPoints(obj, path_x, path_y, path_yaw)
            [pathList, cuspPointIdx] = obj.segmentPathByCuspPoint(path_x, path_y);
            if ~isempty(cuspPointIdx)
                disp('Path contains cusp points!');
                % Select the first path segment
                path_x = pathList{1}(:, 1);
                path_y = pathList{1}(:, 2);
                path_yaw = path_yaw(1:length(path_x));
            end
        end
    end
    
    methods(Static)
        function max_curvature = calculate_max_curvature(curvatures)
            max_curvature = abs(max(curvatures));
        end
        function max_curvature_change = calculate_max_curvature_change(curvatures)
            max_curvature_change = abs(max(diff(curvatures)));
        end
        function mode_change = find_mode_change(curvature, threshold_of_change)
            if nargin < 2
                threshold_of_change = 0.04;
            end
            mode_change = 0;
            for i = 2:length(curvature)
                if abs(curvature(i) - curvature(i-1)) >= threshold_of_change
                    mode_change = mode_change + 1;
                end
            end
        end
        % Function to sample a random point within an area using Monte Carlo method
        function sampled_point = samplePointFromArea(area)
            % Get the bounding box of the area
            min_x = min(area.points(:, 1));
            max_x = max(area.points(:, 1));
            min_y = min(area.points(:, 2));
            max_y = max(area.points(:, 2));
            
            % Generate random points until one is inside the polygon
            while true
                x = rand * (max_x - min_x) + min_x;
                y = rand * (max_y - min_y) + min_y;
                
                % Check if the point is inside the polygon
                if inpolygon(x, y, area.points(:, 1), area.points(:, 2))
                    sampled_point = [x, y];
                    break;
                end
            end
        end
    end
end