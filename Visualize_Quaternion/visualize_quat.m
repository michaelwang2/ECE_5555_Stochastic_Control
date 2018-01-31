function visualize_quat()
% Quaternion Visulaizer 
clear; close; clc;
% parameters
e = [0; 0; 1]; e = e./norm(e); % spin axis
v = [1; 0; 1];  % vector to rotate
theta = 3*pi/4;

% plot parameters
axes_length = 1.4;
% arrow parameters
lineWidth = 6; arrowSize = 0.17; arrowBase = 0.09;
blineWidth = 4; barrowSize = 0.1; barrowBase = 0.05; blength = 1.3;% basis
% plane parameter
scale1 = 1; extension1 = 0.2; transparency1 = 0.4; color1 = 'b';
scale2 = 1; extension2 = 0.2; transparency2 = 0.4; color2 = 'm';
scale3 = 1; extension3 = 0.15; transparency3 = 0.4; color3 = 'g';

% plot
figure;
ax = gca;
ax.Units = 'pixels';
ax.Position(1) = 200;
ax.Position(3) = 500;
ax.Position(4) = 500;

azimuth_ang = 135;
ele_ang = 20;

sliderWidth = 300;
sliderHeight = 30;
topleftX = 900;
topleftY = 525;
spacing = 35;
limSpacing = 50;
% Create theta slider
sld = uicontrol('Style', 'slider',...
    'Min',-pi,'Max',pi,'Value',3*pi/4,...
    'Position', [topleftX, topleftY - spacing, sliderWidth, sliderHeight],...
    'Callback', @adjust_theta); 

% Add a text to theta slider
txt_sld = uicontrol('Style','text',...
    'Position',[topleftX, topleftY, sliderWidth, sliderHeight],...
    'String','Rotation Angle','FontSize',20);
txt_sld_min = uicontrol('Style','text',...
    'Position',[topleftX - limSpacing, topleftY - spacing + 5, limSpacing, sliderHeight],...
    'String','-3.14 ','FontSize',17);
txt_sld_max = uicontrol('Style','text',...
    'Position',[topleftX + sliderWidth, topleftY - spacing + 5, limSpacing, sliderHeight],...
    'String',' 3.14','FontSize',17);

% Create azimuth view slider
az = uicontrol('Style', 'slider',...
    'Min',0,'Max',360,'Value',135,...
    'Position',[topleftX, topleftY - spacing*3, sliderWidth, sliderHeight],...
    'Callback', @adjust_az);

% Add a text to azimuth view slider
txt_az = uicontrol('Style','text',...
    'Position',[topleftX, topleftY - spacing*2, sliderWidth, sliderHeight],...
    'String','Azimuth View Angle','FontSize',20);
txt_az_min = uicontrol('Style','text',...
    'Position',[topleftX - limSpacing, topleftY - spacing*3 + 5, limSpacing, sliderHeight],...
    'String','0','FontSize',17);
txt_az_max = uicontrol('Style','text',...
    'Position',[topleftX + sliderWidth, topleftY - spacing*3 + 5, limSpacing, sliderHeight],...
    'String',' 360','FontSize',17);

% Create elemation view slider
ele = uicontrol('Style', 'slider',...
    'Min',0,'Max',90,'Value',20,...
    'Position',[topleftX, topleftY - spacing*5, sliderWidth, sliderHeight],...
    'Callback', @adjust_ele);

% Add a text to azimuth view slider
txt_ele = uicontrol('Style','text',...
    'Position',[topleftX, topleftY - spacing*4, sliderWidth, sliderHeight],...
    'String','Elevation View Angle','FontSize',20);
txt_ele_min = uicontrol('Style','text',...
    'Position',[topleftX - limSpacing, topleftY - spacing*5 + 5, limSpacing, sliderHeight],...
    'String','0','FontSize',17);
txt_ele_max = uicontrol('Style','text',...
    'Position',[topleftX + sliderWidth, topleftY - spacing*5 + 5, limSpacing, sliderHeight],...
    'String',' 90','FontSize',17);

% quaternions
q = [e.*sin(theta/2); cos(theta/2)]; % quaternion operator (right-hand rule)
v = [v; 0]; % pure quaternion
v_alt = q(4)*v(1:3);
v_e_cross = cross(q(1:3), v(1:3));
proj = v_alt + v_e_cross;
e_alt = dot(q(1:3), v(1:3))*q(1:3);
proj_alt = q(4)*proj;
proj_e_cross = cross(proj, -q(1:3));
vp = e_alt + proj_alt + proj_e_cross;

% vectarrow(zeros(3, 1),e, arrowColor, lineColor, lineWidth, arrowSize, arrowBase);

% plot axes
xplus = vectarrow(zeros(3, 1),[blength;0;0], 'k', 'k:', blineWidth, barrowSize, barrowBase);
yplus = vectarrow(zeros(3, 1),[0;blength;0], 'k', 'k:', blineWidth, barrowSize, barrowBase);
zplus = vectarrow(zeros(3, 1),[0;0;blength], 'k', 'k:', blineWidth, barrowSize, barrowBase);
% actual vectors
original = vectarrow(zeros(3, 1),v(1:3), 'b', 'b-', lineWidth, arrowSize, arrowBase);
original_alt = vectarrow(zeros(3, 1),v_alt, 'c', 'c-', lineWidth, arrowSize, arrowBase);
oalt_e_cross = vectarrow(zeros(3, 1),v_e_cross, 'c', 'c-', lineWidth, arrowSize, arrowBase);
projection = vectarrow(zeros(3, 1),proj, 'c', 'c:', lineWidth, arrowSize, arrowBase);
transformed = vectarrow(zeros(3, 1),vp, 'g', 'g-', lineWidth, arrowSize, arrowBase);
saxis = vectarrow(zeros(3, 1),e, 'k', 'k-', lineWidth, arrowSize, arrowBase);
saxis_alt = vectarrow(zeros(3, 1),e_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
projection_alt = vectarrow(zeros(3, 1),proj_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
projection_plus_saxis_alt = vectarrow(zeros(3, 1),proj_alt+e_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
projection_saxis_cross = vectarrow(zeros(3, 1),proj_e_cross, 'm', 'm-', lineWidth, arrowSize, arrowBase);

% planes
plane1 = createPlane(zeros(3,1), v_alt, zeros(3,1), v_e_cross, scale1, extension1, transparency1, color1);
plane2 = createPlane(zeros(3,1), e_alt, zeros(3,1), proj_alt, scale2, extension2, transparency2, color2);
plane3 = createPlane(zeros(3,1), proj_alt+e_alt, zeros(3,1), proj_e_cross, scale3, extension3, transparency3, color3);

grid on; box on; axis equal; axis square;
xlabel('X');
ylabel('Y');
zlabel('Z','Rotation',0);
title('Quaternion Visualizer');
axis([-1, 1, -1, 1, -1, 1].*axes_length);
leg = legend([saxis, original, projection, transformed], 'Spin Axis', 'Original Vector', 'Intermediate Quaternion Projection in R^3', 'Transformed Vector');
leg.Units = 'pixels';
legendWidth = 350;
leg.Position = [topleftX + (sliderWidth - legendWidth)/2, 100, legendWidth, 100];
leg.AutoUpdate = 'off';

% Make figure visble after adding all components
view(azimuth_ang, ele_ang);

    function adjust_theta(source, event)
        cla;
        theta = source.Value;
        % quaternions
        q = [e.*sin(theta/2); cos(theta/2)]; % quaternion operator (right-hand rule)
        v = [v; 0]; % pure quaternion
        v_alt = q(4)*v(1:3);
        v_e_cross = cross(q(1:3), v(1:3));
        proj = v_alt + v_e_cross;
        e_alt = dot(q(1:3), v(1:3))*q(1:3);
        proj_alt = q(4)*proj;
        proj_e_cross = cross(proj, -q(1:3));
        vp = e_alt + proj_alt + proj_e_cross;
        
        % vectarrow(zeros(3, 1),e, arrowColor, lineColor, lineWidth, arrowSize, arrowBase);
        
        % plot axes
        xplus1 = vectarrow(zeros(3, 1),[blength;0;0], 'k', 'k:', blineWidth, barrowSize, barrowBase);
        yplus1 = vectarrow(zeros(3, 1),[0;blength;0], 'k', 'k:', blineWidth, barrowSize, barrowBase);
        zplus1 = vectarrow(zeros(3, 1),[0;0;blength], 'k', 'k:', blineWidth, barrowSize, barrowBase);
        % actual vectors
        original1 = vectarrow(zeros(3, 1),v(1:3), 'b', 'b-', lineWidth, arrowSize, arrowBase);
        original_alt1 = vectarrow(zeros(3, 1),v_alt, 'c', 'c-', lineWidth, arrowSize, arrowBase);
        oalt_e_cross1 = vectarrow(zeros(3, 1),v_e_cross, 'c', 'c-', lineWidth, arrowSize, arrowBase);
        projection1 = vectarrow(zeros(3, 1),proj, 'c', 'c:', lineWidth, arrowSize, arrowBase);
        transformed1 = vectarrow(zeros(3, 1),vp, 'g', 'g-', lineWidth, arrowSize, arrowBase);
        saxis1 = vectarrow(zeros(3, 1),e, 'k', 'k-', lineWidth, arrowSize, arrowBase);
        saxis_alt1 = vectarrow(zeros(3, 1),e_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
        projection_alt1 = vectarrow(zeros(3, 1),proj_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
        projection_plus_saxis_alt1 = vectarrow(zeros(3, 1),proj_alt+e_alt, 'm', 'm-', lineWidth, arrowSize, arrowBase);
        projection_saxis_cross1 = vectarrow(zeros(3, 1),proj_e_cross, 'm', 'm-', lineWidth, arrowSize, arrowBase);

        % planes
        plane11 = createPlane(zeros(3,1), v_alt, zeros(3,1), v_e_cross, scale1, extension1, transparency1, color1);
        plane21 = createPlane(zeros(3,1), e_alt, zeros(3,1), proj_alt, scale2, extension2, transparency2, color2);
        plane31 = createPlane(zeros(3,1), proj_alt+e_alt, zeros(3,1), proj_e_cross, scale3, extension3, transparency3, color3);

        grid on; box on; axis equal; axis square;
        xlabel('X');
        ylabel('Y');
        zlabel('Z','Rotation',0);
        axis([-1, 1, -1, 1, -1, 1].*axes_length)
    end

    function adjust_az(source, event)
        azimuth_ang = source.Value;
        view(azimuth_ang, ele_ang);
    end

    function adjust_ele(source, event)
        ele_ang = source.Value;
        view(azimuth_ang, ele_ang);
    end
end