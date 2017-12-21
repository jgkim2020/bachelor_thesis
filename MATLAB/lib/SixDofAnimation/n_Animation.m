function fig = n_Animation(varargin)

    %% Create local variables

    % Required arguments
    n_object = varargin{1};             % number of object to draw
    for i = 1:n_object
      p(:,:,i) = varargin{2*i};         % position of body
      R(:,:,:,i) = varargin{2*i + 1};   % rotation matrix of body
    end
    [numSamples dummy] = size(p(:,:,1));

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

    for i = 2*n_object + 2:2:nargin
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

    p = p(1:SamplePlotFreq:numSamples, :, :);
    R = R(:, :, 1:SamplePlotFreq:numSamples, :) * AxisLength;

    if(numel(View) > 2)
        View = View(1:SamplePlotFreq:numSamples, :);
    end
    [numPlotSamples dummy] = size(p(:,:,1));

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
        x = zeros(numPlotSamples, n_object);
        y = zeros(numPlotSamples, n_object);
        z = zeros(numPlotSamples, n_object);
    end
    if(strcmp(Trail, 'All'))
        ox = zeros(numPlotSamples, n_object);
        oy = zeros(numPlotSamples, n_object);
        oz = zeros(numPlotSamples, n_object);
        ux = zeros(numPlotSamples, n_object);
        vx = zeros(numPlotSamples, n_object);
        wx = zeros(numPlotSamples, n_object);
        uy = zeros(numPlotSamples, n_object);
        vy = zeros(numPlotSamples, n_object);
        wy = zeros(numPlotSamples, n_object);
        uz = zeros(numPlotSamples, n_object);
        vz = zeros(numPlotSamples, n_object);
        wz = zeros(numPlotSamples, n_object);
    end
    for i = 1:n_object
      x(1,i) = p(1,1,i);
      y(1,i) = p(1,2,i);
      z(1,i) = p(1,3,i);
      ox(1,i) = x(1,i);
      oy(1,i) = y(1,i);
      oz(1,i) = z(1,i);
      ux(1,i) = R(1,1,1,i);
      vx(1,i) = R(2,1,1,i);
      wx(1,i) = R(3,1,1,i);
      uy(1,i) = R(1,2,1,i);
      vy(1,i) = R(2,2,1,i);
      wy(1,i) = R(3,2,1,i);
      uz(1,i) = R(1,3,1,i);
      vz(1,i) = R(2,3,1,i);
      wz(1,i) = R(3,3,1,i);
    end

    % Create graphics handles
    orgHandle_cell = cell(1,n_object);
    for i = 1:n_object
      orgHandle_cell{i} = plot3(x(:,i), y(:,i), z(:,i), 'k.');
    end

    if(ShowArrowHead)
        ShowArrowHeadStr = 'on';
    else
        ShowArrowHeadStr = 'off';
    end

    quivXhandle_cell = cell(1,n_object);
    quivYhandle_cell = cell(1,n_object);
    quivZhandle_cell = cell(1,n_object);
    for i = 1:n_object
      quivXhandle_cell{i} = quiver3(ox(:,i), oy(:,i), oz(:,i), ux(:,i), vx(:,i), wx(:,i),  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
      quivYhandle_cell{i} = quiver3(ox(:,i), oy(:,i), oz(:,i), uy(:,i), vy(:,i), wy(:,i),  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
      quivZhandle_cell{i} = quiver3(ox(:,i), ox(:,i), oz(:,i), uz(:,i), vz(:,i), wz(:,i),  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    end

    % Create legend
    if(ShowLegend)
        legend('Origin', 'X', 'Y', 'Z');
    end

    % Set initial limits
    Xlim = [min(x(1,1:n_object))-AxisLength max(x(1,1:n_object))+AxisLength] * LimitRatio;
    Ylim = [min(y(1,1:n_object))-AxisLength max(y(1,1:n_object))+AxisLength] * LimitRatio;
    Zlim = [min(z(1,1:n_object))-AxisLength max(z(1,1:n_object))+AxisLength] * LimitRatio;

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
          for j = 1:n_object
            x(1:i,j) = p(1:i,1,j);
            y(1:i,j) = p(1:i,2,j);
            z(1:i,j) = p(1:i,3,j);
          end
        else
          x = zeros(1,n_object);
          y = zeros(1,n_object);
          z = zeros(1,n_object);
          for j = 1:n_object
            x(1,j) = p(i,1,j);
            y(1,j) = p(i,2,j);
            z(1,j) = p(i,3,j);
          end
        end
        if(strcmp(Trail, 'All'))
          for j = 1:n_object
            ox(1:i,j) = p(1:i,1,j);
            oy(1:i,j) = p(1:i,2,j);
            oz(1:i,j) = p(1:i,3,j);
            ux(1:i,j) = R(1,1,1:i,j);
            vx(1:i,j) = R(2,1,1:i,j);
            wx(1:i,j) = R(3,1,1:i,j);
            uy(1:i,j) = R(1,2,1:i,j);
            vy(1:i,j) = R(2,2,1:i,j);
            wy(1:i,j) = R(3,2,1:i,j);
            uz(1:i,j) = R(1,3,1:i,j);
            vz(1:i,j) = R(2,3,1:i,j);
            wz(1:i,j) = R(3,3,1:i,j);
          end
        else
          ox = zeros(1,n_object);
          oy = zeros(1,n_object);
          oz = zeros(1,n_object);
          ux = zeros(1,n_object);
          vx = zeros(1,n_object);
          wx = zeros(1,n_object);
          uy = zeros(1,n_object);
          vy = zeros(1,n_object);
          wy = zeros(1,n_object);
          uz = zeros(1,n_object);
          vz = zeros(1,n_object);
          wz = zeros(1,n_object);
          for j = 1:n_object
            ox(1,j) = p(i,1,j);
            oy(1,j) = p(i,2,j);
            oz(1,j) = p(i,3,j);
            ux(1,j) = R(1,1,i,j);
            vx(1,j) = R(2,1,i,j);
            wx(1,j) = R(3,1,i,j);
            uy(1,j) = R(1,2,i,j);
            vy(1,j) = R(2,2,i,j);
            wy(1,j) = R(3,2,i,j);
            uz(1,j) = R(1,3,i,j);
            vz(1,j) = R(2,3,i,j);
            wz(1,j) = R(3,3,i,j);
          end
        end

        for j = 1:n_object
          set(orgHandle_cell{j}, 'xdata', x(:,j), 'ydata', y(:,j), 'zdata', z(:,j));
          set(quivXhandle_cell{j}, 'xdata', ox(:,j), 'ydata', oy(:,j), 'zdata', oz(:,j),'udata', ux(:,j), 'vdata', vx(:,j), 'wdata', wx(:,j));
          set(quivYhandle_cell{j}, 'xdata', ox(:,j), 'ydata', oy(:,j), 'zdata', oz(:,j),'udata', uy(:,j), 'vdata', vy(:,j), 'wdata', wy(:,j));
          set(quivZhandle_cell{j}, 'xdata', ox(:,j), 'ydata', oy(:,j), 'zdata', oz(:,j),'udata', uz(:,j), 'vdata', vz(:,j), 'wdata', wz(:,j));
        end

        % Adjust axes for snug fit and draw
        axisLimChanged = false;
        if((min(p(i,1,:)) - AxisLength) < Xlim(1)), Xlim(1) = min(p(i,1,:)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((min(p(i,2,:)) - AxisLength) < Ylim(1)), Ylim(1) = min(p(i,2,:)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((min(p(i,3,:)) - AxisLength) < Zlim(1)), Zlim(1) = min(p(i,3,:)) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,1,:)) + AxisLength) > Xlim(2)), Xlim(2) = max(p(i,1,:)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,2,:)) + AxisLength) > Ylim(2)), Ylim(2) = max(p(i,2,:)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((max(p(i,3,:)) + AxisLength) > Zlim(2)), Zlim(2) = max(p(i,3,:)) + LimitRatio*AxisLength; axisLimChanged = true; end
        if(axisLimChanged), set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); end
        drawnow;

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
