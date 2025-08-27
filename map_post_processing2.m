clc; clear; close all;

rosIP   = '10.245.230.25';                 
MASTER  = 'http://10.245.230.87:11311';    
MAP_IN  = '/map_raw';
MAP_OUT = '/map';
SCAN = '/scan';
LASER_FRAME = 'laser';

USE_SENSOR_RANGE = false;
BYPASS  = false;            % true: copia 1:1; false: applica filtro
ACTIVATE_BAG = false;
BAG_READ = false;


SHOW_VIEWER = true;

% --- ROS ---
rosshutdown; pause(0.2);
rosinit(MASTER,'NodeHost',rosIP,'NodeName','/matlab_processing_map2');


tftree = rostf; pause(0.2);
pub = rospublisher(MAP_OUT,'nav_msgs/OccupancyGrid','IsLatching',true,'DataFormat','struct');

% % Viewer live
% if SHOW_VIEWER
%     fh = figure('Name','Map Viewer'); tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
%     ax1 = nexttile; title(ax1,'/map_raw');  im1 = imagesc(ax1,0,0,0); axis(ax1,'equal','tight'); set(ax1,'YDir','normal');
%     ax2 = nexttile; title(ax2,'/map');      im2 = imagesc(ax2,0,0,0); axis(ax2,'equal','tight'); set(ax2,'YDir','normal');
%     cmap = [0 0 0; 1 1 1; .8 .8 .8]; colormap(ax1,cmap); colormap(ax2,cmap);
%     subVisRaw = rossubscriber(MAP_IN, 'nav_msgs/OccupancyGrid', @(~,m)drawOcc(ax1,im1,m),'DataFormat','struct'); 
%     subVisOut = rossubscriber(MAP_OUT,'nav_msgs/OccupancyGrid', @(~,m)drawOcc(ax2,im2,m),'DataFormat','struct'); 
% end

bagWriter = rosbagwriter('office_map_out.bag');
cleanup = onCleanup(@() close(bagWriter));

subScan = rossubscriber(SCAN,'sensor_msgs/LaserScan', @(~,s) assignin('base','LAST_SCAN',s), 'DataFormat','struct');
subProc = rossubscriber(MAP_IN,'nav_msgs/OccupancyGrid', @(~,m)processAndPublish(m,pub,BYPASS,bagWriter,ACTIVATE_BAG,BAG_READ,USE_SENSOR_RANGE,LASER_FRAME,tftree), ...
    'DataFormat','struct');


disp('Relay attivo: /map_raw -> /map (latched).');

% ====== FUNZIONI ======
function processAndPublish(m,pub,BYPASS,bagWriter,ACTIVATE_BAG,BAG_READ,USE_SENSOR_RANGE,LASER_FRAME,tftree)

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
        
    % --- TAGLIO RAGGIO D'AZIONE
    if USE_SENSOR_RANGE
        % 1) Info mappa
        res = double(m.Info.Resolution);
        ox  = double(m.Info.Origin.Position.X);
        oy  = double(m.Info.Origin.Position.Y);
        map_frame = char(m.Header.FrameId); 
    
        % 2) Ultimo Scan
        scan = [];
        if evalin('base',"exist('LAST_SCAN','var')"); scan = evalin('base','LAST_SCAN'); end
        if ~isempty(scan)
            R   = double(scan.RangeMax);                    
            a0  = double(scan.AngleMin);                  
            a1  = double(scan.AngleMax);
            % frame LiDAR
            if isfield(scan.Header,'FrameId') && ~isempty(scan.Header.FrameId)
                LASER_FRAME = char(scan.Header.FrameId);
            end
        else
            R = []; a0 = []; a1 = [];
        end
    
        if ~isempty(R)
            % 3) Posa del LiDAR nel frame mappa
            try
                trL = getTransform(tftree, map_frame, LASER_FRAME);
                lx = double(trL.Transform.Translation.X);
                ly = double(trL.Transform.Translation.Y);
                q  = trL.Transform.Rotation;
                yaw = atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z));
            catch
                lx = ox; ly = oy; yaw = 0;
            end
    
            % 4) Settore circolare sulla mappa
            [Xc,Yc] = meshgrid(1:W,1:H);
            Xw = ox + (Xc-0.5)*res;                 
            Yw = oy + (Yc-0.5)*res;                  
            dx = Xw - lx;  dy = Yw - ly;
            dist2 = dx.^2 + dy.^2;
            inRadius = dist2 <= (R.^2);
    
            % angolo cella nel frame del LiDAR
            ang = atan2(dy,dx) - yaw;
            ang = atan2(sin(ang), cos(ang));
            inFOV = (ang >= a0) & (ang <= a1);
    
            maskSensor = inRadius & inFOV;
    
            % 5) Fuori dal raggio -> unknown
            M(~maskSensor) = -1;
        end
    end


    if BYPASS
        M2 = M;
    else
        M2 = applyPostProcessing(M);
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

function M2 = applyPostProcessing(M)
     
    A = zeros(size(M));

    A(M == 100) = 0;
    A(M == 0) = 254;
    A(M == -1) = 205;
    

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
    
    
    BW_filled = imfill(CH,"holes");
    boundaries = bwboundaries(CH);
    
    CF = bwconncomp(CH, 8);
    tolerance = 0.01;
    
    b_reduced_history = cell(1, CF.NumObjects);
    for k=1:CF.NumObjects
       b = boundaries{k};
       b_reduced = reducepoly(b,tolerance);
    
       b_reduced_history{k} = b_reduced;
    end
    
    polygon = cell(1, CF.NumObjects);
    for i=1:CF.NumObjects 
    
        current_b = b_reduced_history{i};
        polygon{i} = polyshape(current_b(:,2),current_b(:,1));
    
    end
    
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



% function drawOcc(ax, hImg, msg)
%     if isempty(ax) || ~isgraphics(ax) || isempty(hImg) || ~isgraphics(hImg,'image')
%         return;
%     end
%     try
%         W = double(msg.Info.Width);  
%         H = double(msg.Info.Height);
%         if W==0 || H==0, return; end
% 
%         res = double(msg.Info.Resolution);
%         M = reshape(int16(msg.Data), [W, H])';
% 
%         img = ones(H,W,'uint8')*3;
%         img(M==0)   = 2;
%         img(M==100) = 1;
% 
%         ox = double(msg.Info.Origin.Position.X);
%         oy = double(msg.Info.Origin.Position.Y);
% 
%         if ~isgraphics(hImg,'image') || ~isvalid(hImg)
%             cla(ax);
%             hImg = imagesc(ax, [ox, ox+W*res], [oy, oy+H*res], img);
%             set(ax,'YDir','normal'); axis(ax,'equal','tight');
%         else
%             set(hImg,'CData',img, ...
%                      'XData',[ox, ox+W*res], ...
%                      'YData',[oy, oy+H*res]);
%         end
% 
%         set(ax,'YDir','normal'); 
%         axis(ax,'equal','tight');
%         title(ax, sprintf('%s  %dx%d  res=%.3f', msg.Header.FrameId, W, H, res));
%         drawnow limitrate;
%     catch ME
%     end
% end
