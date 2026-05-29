classdef (Abstract) MBLinkVisualization < handle & matlab.mixin.Heterogeneous
    %% General class to visualize a rigid or flexible link in a multibody system
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    properties
        % Link object that defines the link's properties
        linkDef         (1,1) MBLinkDefinition = MBLinkDefinitionRigid;
    end
    properties (Access=public, SetObservable)
        % link Color
        % TODO can be both (1,3) array or string?
        Color           (3,1) double = lines(1);

        % Link name
        Name            (1,1) string

        % Draw the link's joint?
        ShowJoint       (1,1) matlab.lang.OnOffSwitchState = true;

        % Draw the link's TCP frame (if defined)?
        ShowTCPFrame    (1,1) matlab.lang.OnOffSwitchState = true;
    end
    properties (SetAccess=protected)
        % Coordinate system object for the joint
        cSysJ           (1,1) coordSysSE3

        % Coordinate system object for the TCP
        cSysTCP         (1,1) coordSysSE3

        % Object for the joint circle visualization
        jointPatch      (1,1) matlab.graphics.primitive.Patch
    end

    methods(Abstract)
        % update method that must be specified in the subclasses
        obj = updateConfiguration(obj)
    end

    %% General drawing methods
    methods(Access=protected)
        function obj = drawTCPFrame(obj, parent)
            obj.cSysTCP = coordSysSE3( obj.linkDef.g_B_TCP, ...
                'scale', 0.07, ...
                'name', "TCP", ...
                'AxisColors', repmat(obj.Color.', [3,1]), ...
                "parent", parent, ...
                "Visible", obj.linkDef.hasTCP && obj.ShowTCPFrame ...
                );
        end
        function obj = drawJoint(obj, parent)
            arguments
                obj         (1,1)
                % Parent objects for the joints
                % Usually transform objects that describe the joints'
                % configuration
                parent      (2,1)
            end

            % Draw joint frames
            coordSysScale = 0.075;
            obj.cSysJ = coordSysSE3( ...
                inv(obj.linkDef.g_J_B), ...
                "scale", coordSysScale, ...
                "name", sprintf("$J_{%s}$",obj.Name),  ...
                "AxisColors", repmat(obj.Color.', [3,1]), ...
                "parent", parent(1), ...
                "Visible", obj.ShowJoint);

            % For revolute joints: Draw joint
            if any(obj.linkDef.jointAxis(1:3))
                pointsAbs = obj.computeJointCirclePoints;
                obj.jointPatch = patch( ...
                    'XData', pointsAbs(1,:), 'YData', pointsAbs(2,:), 'ZData', pointsAbs(3,:), ...
                    'EdgeColor', obj.Color, 'FaceAlpha', 0, ...
                    'HandleVisibility', 'off','LineWidth', 1, ...
                    "Parent", parent(1)...
                    );
            end
            obj.jointPatch.Visible = obj.ShowJoint;
        end
        function pointsAbs = computeJointCirclePoints(obj)
            % Compute the absolute points for the circle that visualizes
            % the joint axis

            % Compute transformation that defines the joint plane in
            % the inertial system
            e_z = obj.linkDef.jointAxis(1:3) / norm(obj.linkDef.jointAxis(1:3));

            % Find suitable x axis that is perpendicular to e_z
            I = eye(3);
            for ii = 1:3
                if any(cross(obj.linkDef.jointAxis(1:3), I(:,ii)))
                    e_x = cross(obj.linkDef.jointAxis(1:3), I(:,ii));
                    break
                end
            end
            e_y = cross(e_x, e_z);
            g_J1_xy = SE3Matrix([e_x, e_y, -e_z], zeros(3,1));

            % Compute circle points in local xy plane
            r = 0.05;
            nPoints = 20;
            th = 0:pi/nPoints*2:2*pi;
            xunit = r * cos(th);
            yunit = r * sin(th);

            % Circle points in the plane written as homogeneous points
            % (4 dimensional, with appended 1)
            planePointsHomog = [xunit.',yunit.',zeros(nPoints+1,1), ones(nPoints+1,1)];
            pointsAbs = obj.linkDef.g_J_B \ g_J1_xy * planePointsHomog.';
        end
    end
end