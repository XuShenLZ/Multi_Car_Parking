%% loadmap: load map information and olot
function fig = loadmap()
	%% Map construction
	mapLayers = loadParkingLotMapLayers;
	plotMapLayers(mapLayers)
	% For simplicity, combine the three layers into a single costmap.
	costmap = combineMapLayers(mapLayers);
	fig = figure;
	plot(costmap, 'Inflation', 'off')
	hold on
	legend off
end

%% Supporting Functions for the map
%%%
% *loadParkingLotMapLayers*
% Load environment map layers for parking lot
function mapLayers = loadParkingLotMapLayers()
%loadParkingLotMapLayers
%   Load occupancy maps corresponding to 3 layers - obstacles, road
%   markings, and used spots.

	mapLayers.StationaryObstacles = imread('stationary.bmp');
	mapLayers.RoadMarkings        = imread('road_markings.bmp');
	mapLayers.ParkedCars          = imread('parked_cars.bmp');
end

%%%
% *plotMapLayers*
% Plot struct containing map layers
function plotMapLayers(mapLayers)
%plotMapLayers
%   Plot the multiple map layers on a figure window.

	figure
	cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
	montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
	title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end

%%%
% *combineMapLayers*
% Combine map layers into a single costmap
function costmap = combineMapLayers(mapLayers)
%combineMapLayers
%   Combine map layers struct into a single vehicleCostmap.

	combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
	    mapLayers.ParkedCars;
	combinedMap = im2single(combinedMap);

	res = 0.5; % meters
	costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end