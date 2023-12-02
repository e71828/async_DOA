classdef helperAsyncAngleOnlyTrackingDisplay < matlab.System
    % This is a helper class for the example showing passive angle-only
    % tracking with async sensors using GM-PHD tracker. It may be removed
    % or modified in a future release.

    % Copyright 2022 The MathWorks, Inc.

    properties
        Parent
        SnapTimes = [1/3 2/3 3/3 5 60];
        TheaterPlot
        TrackPlotter
        DetectionPlotter
        ReceiverPlotter
        EmitterPlotter
        PHDPlotter
        ColorOrder = darkColorOrder();
    end

    properties (Access = protected)
        Snaps = cell(0,1);
    end
    
    methods (Access = protected)
        function setupImpl(obj, scenario)
            if isempty(obj.Parent)
                obj.Parent = figure('Visible','on','Units','normalized','Position',[0.1 0.1 0.95 0.63]);
            end

            if isPublishing(obj)
                obj.Parent.Visible = 'off';
            end

            ax = axes(obj.Parent,'OuterPosition',[0 0 0.5 1]);
            axis(ax,"square")
            ax.Color = 0.9*[1 1 1];
            tp = theaterPlot('Parent',ax,'XLim',[-8e3 8e3],'YLim',[-3e3 13e3],...
                'AxesUnits',["km","km","km"]);
            receiverPlotter = platformPlotter(tp,'DisplayName','Receivers','Marker','diamond','MarkerFaceColor',obj.ColorOrder(3,:));
            emitterPlotter = platformPlotter(tp,'DisplayName','Emitters','Marker','^','MarkerFaceColor',obj.ColorOrder(2,:));
            hold on;
            l = legend;
            l.AutoUpdate = 'on';
            detectionPlotter = plot3(nan,nan,nan,'LineWidth',1,'Color',obj.ColorOrder(3,:),'DisplayName','Detections');
            trkP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor',obj.ColorOrder(4,:),'ConnectHistory','on','ColorizeHistory','on');
            obj.TheaterPlot = tp;
            obj.TrackPlotter = trkP;
            obj.DetectionPlotter = detectionPlotter;
            obj.ReceiverPlotter = receiverPlotter;
            obj.EmitterPlotter = emitterPlotter;
            hold on;
            xData = linspace(-8e3,8e3,500);
            yData = linspace(-3e3,13e3,500);
            ax2 = axes(obj.Parent,'OuterPosition',[0.5 0 0.5 1]);
            ax2.DataAspectRatio = [1 1 1];
            obj.PHDPlotter = imagesc(ax2,zeros(500,500),'XData',xData,'YData',yData,'AlphaData',1);
            uistack(obj.PHDPlotter,'bottom')
            l.Location = 'northeast';
            l.Orientation = 'vertical';
            % l.Position = [0.0469    0.9429    0.4125    0.0500];
            ax2.YDir = 'normal';
            ax2.XTickLabel = ax.XTickLabel;
            ax2.YTickLabel = ax.YTickLabel;
            ax2.XLabel = copy(ax.XLabel);
            ax2.YLabel = copy(ax.YLabel);
            axis(ax2,"square")
            title(ax2,'Probability Hypothesis Density');
            % Plot trajectories
            scenario = clone(scenario);
            scenario.UpdateRate = 1;
            r = record(scenario,'IncludeSensors',false,'IncludeEmitters',false);
            pos = repmat({zeros(0,3)},8,1);            
            for i = 1:numel(r)
                for k = 1:numel(r(i).Poses)
                    idx = r(i).Poses(k).PlatformID;
                    pos{idx}(end+1,:) = r(i).Poses(k).Position;
                end
            end
            trajPlotter = trajectoryPlotter(tp,'LineStyle','-','LineWidth',2);
            trajPlotter.plotTrajectory(pos);
        end

        function stepImpl(obj, scenario, tracks, detections, phd)
            % obj.DetectionBuffer = [obj.DetectionBuffer;detections];
            % detections = obj.DetectionBuffer;
            isReceiver = cellfun(@(x)~isempty(x.Sensors),scenario.Platforms);
            isTarget = ~isReceiver;
            poses = repmat(pose(scenario.Platforms{1}),numel(scenario.Platforms),1);
            for i = 1:numel(scenario.Platforms)
                poses(i) = pose(scenario.Platforms{i});
            end
            tgtPoses = poses(isTarget);
            recPoses = poses(isReceiver);
            tgtPositions = vertcat(tgtPoses.Position);
            recPositions = vertcat(recPoses.Position);
            obj.EmitterPlotter.plotPlatform(tgtPositions);
            obj.ReceiverPlotter.plotPlatform(recPositions);
            pos = getLines(obj, detections);
            obj.DetectionPlotter.XData = pos(1,:);
            obj.DetectionPlotter.YData = pos(2,:);
            obj.DetectionPlotter.ZData = 0*pos(3,:); % Only 2-d plot
            if isempty(tracks)
                pos = zeros(0,3);
                cov = zeros(3,3,0);
                labels = string.empty(0,1);
            else
                [pos, cov] = getTrackPositions(tracks, 'constvel');
                labels = string([tracks.TrackID]);    
            end
            pos(:,3) = 5;
            cov(3,3,:) = 0;
            obj.TrackPlotter.plotTrack(pos, cov, labels);
            components = extractState(phd,0);
            if ~isempty(components)
                [pos, cov] = getTrackPositions(components,'constvel');
                xData = linspace(-8e3,8e3,500);
                yData = linspace(-3e3,13e3,500);
                phdImage = getPositionPHD(pos', cov, phd.Weights, xData, yData);
            else
                phdImage = zeros(500,500);
            end
            obj.PHDPlotter.CData = phdImage;
            takeSnaps(obj, scenario);
        end

        function pos = getLines(obj, detections)
            pos = zeros(3,0);
            for i = 1:numel(detections)
                fullDetection = objectDetection(0,[detections{i}.Measurement;0],...
                    'MeasurementNoise',blkdiag(detections{i}.MeasurementNoise,1),...
                    'MeasurementParameters',detections{i}.MeasurementParameters,...
                    'SensorIndex',detections{i}.SensorIndex,...
                    'ObjectClassID',detections{i}.ObjectClassID);
                fullDetection.MeasurementParameters(1).HasRange = true;
                fullDetection.Measurement(end) = 0;
                startPos = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(fullDetection,'display','double');
                fullDetection.Measurement(end) = 1e6;
                endPos = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(fullDetection,'display','double');
                pos = [pos startPos endPos nan(3,1)];
            end
        end

        function takeSnaps(obj, scenario)
            time = scenario.SimulationTime;
            snapTimes = obj.SnapTimes;
            if any(abs(time - snapTimes) < 0.05)
                obj.Snaps{end+1} = takesnap(obj.Parent);
            end
        end

        function tf = isPublishing(~)
            tf = false;
            try
                s = numel(dbstack);
                tf = s > 6;
            catch
            end
        end
    end

    methods
        function showSnaps(obj, time)
            idx = find(abs(time - obj.SnapTimes) < 0.05,1,'first');
            showsnap(obj.Snaps{idx});
        end
    end

    methods (Static)
        function plotRangeParameterization()
            figure();
            detection = objectDetection(0,[30;0],'MeasurementParameters',struct('Frame','spherical','HasRange',false,'OriginPosition',[156;225;0]));
            filter = initrpekf(detection,10,[600 5000]);
            plot(156,225,'^');
            hold on;
            plot(156 + [0 5000*cosd(30)],225 + [0 5000*sind(30)],'-','Color','r','LineWidth',2);
            bpLog = zeros(2,0);
            for i = 1:numel(filter.TrackingFilters)
                means = filter.TrackingFilters{i}.State([1 3]);
                C = filter.TrackingFilters{i}.StateCovariance([1 3],[1 3]);
                tt=linspace(0,2*pi,50)';
                x = cos(tt); y=sin(tt);
                ap = [x(:) y(:)]';
                [v,d]=eig(C);
                if any(d(:) < 0)
                    warning(message('shared_tracking:plotgaussian:invalidEigenvalues'));
                    d = max(d,0);
                end
                d = 1.8*sqrt(d); % convert variance to sdwidth*sd
                bp = (v*d*ap) + repmat(means, 1, size(ap,2));
                bpLog = [bp nan(2,1) bpLog];
            end
            plot(bpLog(1,:),bpLog(2,:),'b-','LineWidth',2);
            xlabel('X'); ylabel('Y'); grid on;
            l = legend('Sensor','Detection','Range-parameterized components');
            l.Location = 'northoutside';
            l.Orientation = 'horizontal';
        end
    end
end

function phd = getPositionPHD(m, P, w, x, y)
[X, Y] = meshgrid(x,y);
pos = [X(:)';Y(:)'];
phd = zeros(size(X));
for i = 1:numel(w)
    e = m(1:2,i) - pos;
    S = P(1:2,1:2,i);
    eTSinv = e'/S;
    eT = e';
    thisPHD = w(i)*(1/sqrt(det(2*pi*S))*exp(-1/2*dot(eTSinv,eT,2)));
    phd(:) = phd(:) + thisPHD;
end
end

function colorOrder = darkColorOrder
colorOrder = [1.0000    1.0000    0.0667
    0.0745    0.6235    1.0000
    1.0000    0.4118    0.1608
    0.3922    0.8314    0.0745
    0.7176    0.2745    1.0000
    0.0588    1.0000    1.0000
    1.0000    0.0745    0.6510];

colorOrder(8,:) = [1 1 1];
colorOrder(9,:) = [0 0 0];
colorOrder(10,:) = 0.7*[1 1 1];
end

function snap = takesnap(f)
children = f.Children;
snap = {copy(children)};
snap{2} = f.Position;
end

function h = showsnap(snap)
h = figure("IntegerHandle","off","Units","normalized","Position",snap{2});
children = snap{1};
copyobj(children,h);
end