clc; clear; close all;

addpath(genpath('../'))

rosIP   = '10.245.230.25';                 
MASTER  = 'http://10.245.230.87:11311';    
MAP_IN  = '/map_raw';
MAP_OUT = '/map';
% SCAN = '/scan';
% LASER_FRAME = 'laser';

BYPASS  = false;            % true: copia 1:1; false: applica filtro
ACTIVATE_BAG = false;
BAG_READ = false;

LAMBDA = 5e4;

% --- ROS ---
rosshutdown; pause(0.2);
rosinit(MASTER,'NodeHost',rosIP,'NodeName','/matlab_processing_map3');


% tftree = rostf; pause(0.2);
pub = rospublisher(MAP_OUT,'nav_msgs/OccupancyGrid','IsLatching',true,'DataFormat','struct');


bagWriter = rosbagwriter('office_map_out.bag');
cleanup = onCleanup(@() close(bagWriter));

% subScan = rossubscriber(SCAN,'sensor_msgs/LaserScan', @(~,s) assignin('base','LAST_SCAN',s), 'DataFormat','struct');
subProc = rossubscriber(MAP_IN,'nav_msgs/OccupancyGrid', @(~,m)processAndPublish(m,pub,BYPASS,bagWriter,ACTIVATE_BAG,BAG_READ,LAMBDA), ...
    'DataFormat','struct');


disp('Relay attivo: /map_raw -> /map (latched).');

% ====== FUNZIONI ======
function processAndPublish(m,pub,BYPASS,bagWriter,ACTIVATE_BAG,BAG_READ,LAMBDA)

    if ACTIVATE_BAG
        if BAG_READ
            bag = rosbag("office_map.bag");
            sel = select(bag,'Topic','/map_raw');
            msgs = readMessages(sel,'DataFormat','struct');
            if isempty(msgs), return; end
            m = msgs{end};
        else
            write(bagWriter,'/map_raw', m.Header.Stamp, m);    
        end
    end

    W = double(m.Info.Width);  H = double(m.Info.Height);
    if W==0 || H==0, return; end
    M = reshape(int16(m.Data), [W,H])';


    if BYPASS
        M2 = M;
    else
        [M2,r2bMap] = ProcessingDiffeomorphism(M,LAMBDA);
    end

    out = rosmessage(pub);             
    out.Header = m.Header;
    t = rostime('now');
    out.Header.Stamp.Sec  = uint32(t.Sec);
    out.Header.Stamp.Nsec = uint32(t.Nsec);

    out.Info = m.Info;
    out.Data = int8(reshape(M2',[],1));


    % write(bagWriter,'/map', out.Header.Stamp, out);

    send(pub,out);
end

function [M2, r2bMap] = ProcessingDiffeomorphism(M,LAMBDA)
     
    A = zeros(size(M));

    A(M == 100) = 0;
    A(M == 0) = 254;
    A(M == -1) = 205;
    

    realWorld.domain.type = 'qc';
    realWorld.domain.goal = [0;3];
    ballWorld.domain.center = [0;0];
    ballWorld.domain.goal = realWorld.domain.goal;

    realWorld.obstacles = {};
    ballWorld.obstacles = {};

    BW = imbinarize(A);
    BW_inverted = ~BW;
    
    BW_inverted = bwareaopen(BW_inverted, 3);
    se = strel('disk', 1);
    BW_inverted = imdilate(BW_inverted, se);
    
    CC = bwconncomp(BW_inverted, 8);
    p = regionprops(CC,"Area");
    areas = [p.Area];
    
    areaThreshold = 100;
    
    CH = false(size(BW_inverted));
    TotArea = 0;

    for i = 1:length(areas)
        pixelIdx = CC.PixelIdxList{i};
    
        tempBW = false(size(BW_inverted));
        tempBW(pixelIdx) = true;

        if areas(i) < areaThreshold
            tempCH = bwconvhull(tempBW, 'objects', 8);
            CH = CH | tempCH; 
        else
            CH = CH | tempBW;
            TotArea = TotArea + areas(i);

        end
    end
    ballWorld.domain.radius = sqrt(TotArea/pi);
    
    BW_filled = imfill(CH,"holes");
    boundaries = bwboundaries(CH);
    
    CF = bwconncomp(CH, 8);
    tolerance = 0.01;
    
    b_reduced_history = cell(1, CF.NumObjects);

    xmin = inf; ymin = inf;
    xmax = -inf; ymax = -inf;

    for k=1:CF.NumObjects
       b = boundaries{k};
       b = reducepoly(b,tolerance);

       if ~isequal(b(1,:), b(end,:))
           b = [b; b(1,:)];
       end

       dup = all(diff(b)==0,2);
       b(dup,:) = [];

       if size(b,1) < 4
           b_reduced_history{k} = [];
           continue;
       end

       ymin = min(ymin, min(b(:,1)));
       ymax = max(ymax, max(b(:,1)));
       xmin = min(xmin, min(b(:,2)));
       xmax = max(xmax, max(b(:,2)));

    
       b_reduced_history{k} = b;
    end
    
    cx = (xmin + xmax)/2;  cy = (ymin + ymax)/2;
    side = max(xmax - xmin, ymax - ymin);
    half = side/2;
    xv = [cx-half, cx+half, cx+half, cx-half, cx-half];
    yv = [cy-half, cy-half, cy+half, cy+half, cy-half];
    
    realWorld.domain.contour = [yv; xv];    
    wm = WorldMapping(realWorld, ballWorld);


    % obstacle_contours      = cell(1, CF.NumObjects);
    % ball_obstacle_centers  = cell(1, CF.NumObjects);
    % ball_obstacle_radius   = zeros(1, CF.NumObjects);

    polygon = cell(1, CF.NumObjects);
    for i=1:CF.NumObjects 
    
        current_b = b_reduced_history{i};
        polygon{i} = polyshape(current_b(:,2),current_b(:,1), 'KeepCollinearPoints', true, 'Simplify', true);
        
        obstacle_contours{i} = [current_b(:,1)'; current_b(:,2)'];
        [cx,cy] = centroid(polygon{i});
        ball_obstacle_centers{i} = [cx;cy];
        ball_obstacle_radius{i} = sqrt(area(polygon{i})/pi);

        realWorld.obstacles{end+1}.type = 'qc';
        realWorld.obstacles{end}.contour = obstacle_contours{i};
        ballWorld.obstacles{end+1}.center = ball_obstacle_centers{i};
        ballWorld.obstacles{end}.centerOriginal = ballWorld.obstacles{end}.center;
        ballWorld.obstacles{end}.radius = ball_obstacle_radius{i};
        ballWorld.obstacles{end}.radiusOriginal = ballWorld.obstacles{end}.radius;
    end


    % obstacle_encountered = false(numel(obstacle_contours),1);
    % D_OBST_DETECTION = 1;
    % Nobst = numel(realWorld.obstacles);
    
    wm.setRealWorld(realWorld);
    wm.setBallWorld(ballWorld);

    wm.evaluateMappings(LAMBDA);
    [r2bMap,~,~,~] = wm.getMappings();
    
    [height, width] = size(A); 
    pgmImage = false(height, width);
    
    for i = 1:length(polygon)
        mask = createMask(polygon{i}, height, width);
        pgmImage = pgmImage | mask; 
    end
    
    % ====== CONVERSIONE AL FORMATO OccupancyGrid ======
    
    M2 = -1*ones(size(A),'int8');
    
    M2(pgmImage ~= 0) = 100;
    M2(pgmImage == 0) = 0;
   

end


% Function to create a binary mask from a polyshape
function mask = createMask(polygon, height, width)
    [X, Y] = meshgrid(1:width, 1:height);
    mask = inpolygon(X, Y, polygon.Vertices(:,1), polygon.Vertices(:,2));
end
