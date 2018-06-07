%% Image to Binary Occupancy Grid Example
% This example shows how to convert an image to a binary occupancy grid for
% using with the Robotics System Toolbox(R)

% Import Image
image = imread('image.png');

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Use black and white image as matrix input for binary occupancy grid
grid = robotics.BinaryOccupancyGrid(bwimage);

show(grid)

