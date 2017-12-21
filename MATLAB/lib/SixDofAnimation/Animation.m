function fig = Animation(varargin)

    %% Create local variables

    % Required arguments
    p = varargin{1};                % position of body
    R = varargin{2};                % rotation matrix of body
    p2 = varargin{3};               % position of body2
    R2 = varargin{4};               % rotation matrix of body2
    [numSamples dummy] = size(p);

    % Default values of optional arguments
    SamplePlotFreq = 1;
    Trail = 'Off';
    LimitRatio = 1;
    Position = [];
    FullScreen = false;
    View = [30 20];
    AxisLength = 1;
    ShowArrowHead = 'on';
    Xlabel = 'X';
    Ylabel = 'Y';
    Zlabel = 'Z';
    Title = '6DOF Animation';
    ShowLegend = true;
    CreateMP4 = false;
    MP4fileName = '6DOF Animation';
    MP4fileNameEnum = true;
    MP4fps = 30;

    for i = 5:2:nargin
        if  strcmp(varargin{i}, 'SamplePlotFreq'), SamplePlotFreq = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Trail')
            Trail = varargin{i+1};
            if(~strcmp(Trail, 'Off') && ~strcmp(Trail, 'DotsOnly') && ~strcmp(Trail, 'All'))
                error('Invalid argument.  Trail must be ''Off'', ''DotsOnly'' or ''All''.');
            end
        elseif  strcmp(varargin{i}, 'LimitRatio'), LimitRatio = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Position'), Position = varargin{i+1};
        elseif  strcmp(varargin{i}, 'FullScreen'), FullScreen = varargin{i+1};
        elseif  strcmp(varargin{i}, 'View'), View = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AxisLength'), AxisLength = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowArrowHead'), ShowArrowHead = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Xlabel'), Xlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Ylabel'), Ylabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Zlabel'), Zlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Title'), Title = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowLegend'), ShowLegend = varargin{i+1};
        elseif  strcmp(varargin{i}, 'CreateMP4'), CreateMP4 = varargin{i+1};
        elseif  strcmp(varargin{i}, 'MP4fileName'), MP4fileName = varargin{i+1};
        elseif  strcmp(varargin{i}, 'MP4fileNameEnum'), MP4fileNameEnum = varargin{i+1};
        elseif  strcmp(varargin{i}, 'MP4fps'), MP4fps = varargin{i+1};
        else error('Invalid argument.');
        end
    end;

    %% Reduce data to samples to plot only

    p = p(1:SamplePlotFreq:numSamples, :);
    p2 = p2(1:SamplePlotFreq:numSamples, :);
    R = R(:, :, 1:SamplePlotFreq:numSamples) * AxisLength;
    R2 = R2(:, :, 1:SamplePlotFreq:numSamples) * AxisLength;
    if(numel(View) > 2)
        View = View(1:SamplePlotFreq:numSamples, :);
    end
    [numPlotSamples dummy] = size(p);

    %% Setup MP4 file

    mp4obj = [];                                                            	% create null object
    if(CreateMP4)
        fileName = strcat(MP4fileName, '.mp4');
        if(exist(fileName, 'file'))
            if(MP4fileNameEnum)                                              	% if file name exists and enum enabled
                i = 0;
                while(exist(fileName, 'file'))                                  % find un-used file name by appending enum
                    fileName = strcat(MP4fileName, sprintf('%i', i), '.mp4');
                    i = i + 1;
                end
            else                                                                % else file name exists and enum disabled
                fileName = [];                                                  % file will not be created
            end
        end
        if(isempty(fileName))
            sprintf('MP4 file not created as file already exists.')
        else
            % mp4file discontinued
            % mp4obj = mp4file(fileName, 'fps', MP4fps, 'compression', 'Cinepak', 'quality', 100);
            mp4obj = VideoWriter(fileName,'MPEG-4');
            mp4obj.FrameRate = MP4fps;
        end
    end

    %% Setup figure and plot

    % Create figure
    fig = figure('NumberTitle', 'off', 'Name', '6DOF Animation');
    if(FullScreen)
        screenSize = get(0, 'ScreenSize');
        set(fig, 'Position', [0 0 screenSize(3) screenSize(4)]);
    elseif(~isempty(Position))
        set(fig, 'Position', Position);
    end
    % set(gca, 'drawmode', 'fast');
    lighting phong;
    set(gcf, 'Renderer', 'zbuffer');
    hold on;
    axis equal;
    grid on;
    view(View(1, 1), View(1, 2));
    title(i);
    xlabel(Xlabel);
    ylabel(Ylabel);
    zlabel(Zlabel);

    % Create plot data arrays
    if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
        x = zeros(numPlotSamples, 2);
        y = zeros(numPlotSamples, 2);
        z = zeros(numPlotSamples, 2);
    end
    if(strcmp(Trail, 'All'))
        ox = zeros(numPlotSamples, 2);
        oy = zeros(numPlotSamples, 2);
        oz = zeros(numPlotSamples, 2);
        ux = zeros(numPlotSamples, 2);
        vx = zeros(numPlotSamples, 2);
        wx = zeros(numPlotSamples, 2);
        uy = zeros(numPlotSamples, 2);
        vy = zeros(numPlotSamples, 2);
        wy = zeros(numPlotSamples, 2);
        uz = zeros(numPlotSamples, 2);
        vz = zeros(numPlotSamples, 2);
        wz = zeros(numPlotSamples, 2);
    end
    x(1,1:2) = [p(1,1) p2(1,1)];
    y(1,1:2) = [p(1,2) p2(1,2)];
    z(1,1:2) = [p(1,3) p2(1,3)];
    ox(1,1:2) = x(1,1:2);
    oy(1,1:2) = y(1,1:2);
    oz(1,1:2) = z(1,1:2);
    ux(1,1:2) = [R(1,1,1) R2(1,1,1)];
    vx(1,1:2) = [R(2,1,1) R2(2,1,1)];
    wx(1,1:2) = [R(3,1,1) R2(3,1,1)];
    uy(1,1:2) = [R(1,2,1) R2(1,2,1)];
    vy(1,1:2) = [R(2,2,1) R2(2,2,1)];
    wy(1,1:2) = [R(3,2,1) R2(3,2,1)];
    uz(1,1:2) = [R(1,3,1) R2(1,3,1)];
    vz(1,1:2) = [R(2,3,1) R2(2,3,1)];
    wz(1,1:2) = [R(3,3,1) R2(3,3,1)];

    % Create graphics handles
    orgHandle = plot3(x(:,1), y(:,1), z(:,1), 'k.');
    orgHandle2 = plot3(x(:,2), y(:,2), z(:,2), 'k.');
    if(ShowArrowHead)
        ShowArrowHeadStr = 'on';
    else
        ShowArrowHeadStr = 'off';
    end
    quivXhandle = quiver3(ox(:,1), oy(:,1), oz(:,1), ux(:,1), vx(:,1), wx(:,1),  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle = quiver3(ox(:,1), oy(:,1), oz(:,1), uy(:,1), vy(:,1), wy(:,1),  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle = quiver3(ox(:,1), ox(:,1), oz(:,1), uz(:,1), vz(:,1), wz(:,1),  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivXhandle2 = quiver3(ox(:,2), oy(:,2), oz(:,2), ux(:,2), vx(:,2), wx(:,2),  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle2 = quiver3(ox(:,2), oy(:,2), oz(:,2), uy(:,2), vy(:,2), wy(:,2),  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle2 = quiver3(ox(:,2), ox(:,2), oz(:,2), uz(:,2), vz(:,2), wz(:,2),  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

    % Create legend
    if(ShowLegend)
        legend('Origin', 'X', 'Y', 'Z');
    end

    % Set initial limits
    Xlim = [min(x(1,1:2))-AxisLength max(x(1,1:2))+AxisLength] * LimitRatio;
    Ylim = [min(y(1,1:2))-AxisLength max(y(1,1:2))+AxisLength] * LimitRatio;
    Zlim = [min(z(1,1:2))-AxisLength max(z(1,1:2))+AxisLength] * LimitRatio;

    % Fix z-axis
    % Zlim = [0 0.8];

    set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);

    % Set initial view
    view(View(1, :));

    %% Plot one sample at a time

    for i = 1:numPlotSamples

        % Update graph title
        if(strcmp(Title, ''))
            titleText = sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples);
        else
            titleText = strcat(Title, ' (', sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples), ')');
        end
        title(titleText);

        % Plot body x y z axes
        if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
            x(1:i,1:2) = [p(1:i,1) p2(1:i,1)];
            y(1:i,1:2) = [p(1:i,2) p2(1:i,2)];
            z(1:i,1:2) = [p(1:i,3) p2(1:i,3)];
        else
            x = [p(i,1) p2(i,1)];
            y = [p(i,2) p2(i,2)];
            z = [p(i,3) p2(i,3)];
        end
        if(strcmp(Trail, 'All'))
            ox(1:i,1:2) = [p(1:i,1) p2(1:i,1)];
            oy(1:i,1:2) = [p(1:i,2) p2(1:i,2)];
            oz(1:i,1:2) = [p(1:i,3) p2(1:i,3)];
            ux(1:i,1:2) = [R(1,1,1:i) R2(1,1,1:i)];
            vx(1:i,1:2) = [R(2,1,1:i) R2(2,1,1:i)];
            wx(1:i,1:2) = [R(3,1,1:i) R2(3,1,1:i)];
            uy(1:i,1:2) = [R(1,2,1:i) R2(1,2,1:i)];
            vy(1:i,1:2) = [R(2,2,1:i) R2(2,2,1:i)];
            wy(1:i,1:2) = [R(3,2,1:i) R2(3,2,1:i)];
            uz(1:i,1:2) = [R(1,3,1:i) R2(1,3,1:i)];
            vz(1:i,1:2) = [R(2,3,1:i) R2(2,3,1:i)];
            wz(1:i,1:2) = [R(3,3,1:i) R2(3,3,1:i)];
        else
            ox = [p(i,1) p2(i,1)];
            oy = [p(i,2) p2(i,2)];
            oz = [p(i,3) p2(i,3)];
            ux = [R(1,1,i) R2(1,1,i)];
            vx = [R(2,1,i) R2(2,1,i)];
            wx = [R(3,1,i) R2(3,1,i)];
            uy = [R(1,2,i) R2(1,2,i)];
            vy = [R(2,2,i) R2(2,2,i)];
            wy = [R(3,2,i) R2(3,2,i)];
            uz = [R(1,3,i) R2(1,3,i)];
            vz = [R(2,3,i) R2(2,3,i)];
            wz = [R(3,3,i) R2(3,3,i)];
        end
        set(orgHandle, 'xdata', x(:,1), 'ydata', y(:,1), 'zdata', z(:,1));
        set(quivXhandle, 'xdata', ox(:,1), 'ydata', oy(:,1), 'zdata', oz(:,1),'udata', ux(:,1), 'vdata', vx(:,1), 'wdata', wx(:,1));
        set(quivYhandle, 'xdata', ox(:,1), 'ydata', oy(:,1), 'zdata', oz(:,1),'udata', uy(:,1), 'vdata', vy(:,1), 'wdata', wy(:,1));
        set(quivZhandle, 'xdata', ox(:,1), 'ydata', oy(:,1), 'zdata', oz(:,1),'udata', uz(:,1), 'vdata', vz(:,1), 'wdata', wz(:,1));
        set(orgHandle2, 'xdata', x(:,2), 'ydata', y(:,2), 'zdata', z(:,2));
        set(quivXhandle2, 'xdata', ox(:,2), 'ydata', oy(:,2), 'zdata', oz(:,2),'udata', ux(:,2), 'vdata', vx(:,2), 'wdata', wx(:,2));
        set(quivYhandle2, 'xdata', ox(:,2), 'ydata', oy(:,2), 'zdata', oz(:,2),'udata', uy(:,2), 'vdata', vy(:,2), 'wdata', wy(:,2));
        set(quivZhandle2, 'xdata', ox(:,2), 'ydata', oy(:,2), 'zdata', oz(:,2),'udata', uz(:,2), 'vdata', vz(:,2), 'wdata', wz(:,2));

        % Adjust axes for snug fit and draw
        axisLimChanged = false;
        if((min(p(i,1),p2(i,1)) - AxisLength) < Xlim(1)), Xlim(1) = min(p(i,1),p2(i,1)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((min(p(i,2),p2(i,2)) - AxisLength) < Ylim(1)), Ylim(1) = min(p(i,2),p2(i,2)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((min(p(i,3),p2(i,3)) - AxisLength) < Zlim(1)), Zlim(1) = min(p(i,3),p2(i,3)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,1),p2(i,1)) + AxisLength) > Xlim(2)), Xlim(2) = max(p(i,1),p2(i,1)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,2),p2(i,2)) + AxisLength) > Ylim(2)), Ylim(2) = max(p(i,2),p2(i,2)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,3),p2(i,3)) + AxisLength) > Zlim(2)), Zlim(2) = max(p(i,3),p2(i,3)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if(axisLimChanged), set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); end
        drawnow;

        % Fix z-axis
        % Zlim = [0 0.8];

        % Adjust view
        if(numel(View) > 2)
            view(View(i, :));
        end

        % Add frame to MP4 object
        if(~isempty(mp4obj))
            frame = getframe(fig);
            % mp4obj = addframe(mp4obj, frame);
            open(mp4obj);
            writeVideo(mp4obj,frame);
        end

    end

    hold off;

    % Close MP4 file
    if(~isempty(mp4obj))
        % mp4obj = close(mp4obj);
        close(mp4obj);
    end

end
