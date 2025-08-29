clc;
clear;
close all;

% Original image
A = imread("map1.pgm");

figure;
imshow(A);
title('Original Image');

% Image to binary
BW = imbinarize(A);
BW_inverted = ~BW;

% Merge close obstacles
BW_inverted = bwareaopen(BW_inverted, 3);
se = strel('disk', 1);
BW_inverted = imdilate(BW_inverted, se);
figure;
imshow(BW_inverted);

% Control over area to get convex
CC = bwconncomp(BW_inverted, 8);
p = regionprops(CC,"Area");
areas = [p.Area];

% Filter out small areas based on a threshold
areaThreshold = 100;

CH = false(size(BW_inverted));

% Convex image
if true
    for i = 1:length(areas)
        pixelIdx = CC.PixelIdxList{i};
    
        tempBW = false(size(BW_inverted));
        tempBW(pixelIdx) = true;
    
        if areas(i) < areaThreshold
            tempCH = bwconvhull(tempBW, 'objects', 8);
            CH = CH | tempCH; 
        else
            CH = CH | tempBW; 
        end
    end

    figure;
    imshow(CH);
    title('Union Convex Hull');
else
    CH = BW_inverted;
end


% Boundaries

BW_filled = imfill(CH,"holes");
boundaries = bwboundaries(CH);

CF = bwconncomp(CH, 8);

figure;
imshow(A);

tolerance = 0.01;

% Point reduction with Ramer–Douglas–Peucker algorithm

b_reduced_history = cell(1, CF.NumObjects);

for k = 1:CF.NumObjects
   b = boundaries{k};
  

   b_reduced = reducepoly(b,tolerance);

   b_reduced_history{k} = b_reduced;

   hold on;
   plot(b_reduced(:,2),b_reduced(:,1),"g",LineWidth=3);
end
hold off;

% Poligonalization

figure;
for i=1:CF.NumObjects 
    % Set axis limits to match the original image size
    axis([1 size(A, 2) 1 size(A, 1)]);
    axis equal;
    set(gca,'YDir','reverse');
    title('Polygons');
    
    current_b = b_reduced_history{i};
    polygon{i} = polyshape(current_b(:,2),current_b(:,1));

    hold on;
    plot(polygon{i});
end
hold off;

[height, width] = size(A); % Get dimensions of the original image

% Create a blank binary image
pgmImage = false(height, width);

% Rasterize each polygon into the binary image
for i = 1:length(polygon)
    % Convert the polyshape to a binary mask
    mask = createMask(polygon{i}, height, width);
    pgmImage = pgmImage | mask; % Combine the masks
end
% for i=1:1984
%     for j=1:1984
%     if k(i,j)~=0
%         disp([i,j]);
%     end
%     end
% end
M2 = -1*ones(size(A),'int16');

for i=1:1984
    for j=1:1984
        if pgmImage(i,j)==0
            M2(i,j) =65000;
        else
            M2(i,j) =0;
        end
    end
end

figure;
imshow(M2);

% pgm = logical(~pgmImage);     % HxW binaria

% Convert the binary image to uint8 format for PGM output
pgmImage = uint8(pgmImage*255); % Scale to 0-255 for PGM format


figure;
imshow(pgmImage);

% Write the image to a PGM file
imwrite(pgmImage, 'output_map.pgm');
% 
% Function to create a binary mask from a polyshape
function mask = createMask(polygon, height, width)
    % Create a grid of coordinates
    [X, Y] = meshgrid(1:width, 1:height);
    % Check if points are inside the polygon
    mask = inpolygon(X, Y, polygon.Vertices(:,1), polygon.Vertices(:,2));
end