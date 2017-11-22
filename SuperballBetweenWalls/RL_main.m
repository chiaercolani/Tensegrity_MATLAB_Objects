% Simple dynamics example with 6-bar tensegrity.

clear all;
close all;
addpath('../tensegrityObjects')

%% Workspace Constants

% Wall 1 and 4 are positive values
wall1=0.8;
wall2=-0.8;
wall3=-0.8;
wall4=0.8;
barSpacing = 0.375;
barLength = 1.7;
bars = [1:2:11; 
        2:2:12];
strings = [1  1   1  1  2  2  2  2  3  3  3  3  4  4  4  4  5  5  6  6  7  7  8  8;
           7  8  10 12  5  6 10 12  7  8  9 11  5  6  9 11 11 12  9 10 11 12  9 10];
     
K = 1000;
c = 40;
stringStiffness = K*ones(24,1); % String stiffness (N/m)
barStiffness = 100000*ones(6,1); % Bar stiffness (N/m)
stringDamping = c*ones(24,1);  % String damping vector
nodalMass = 1.625*ones(12,1);
delT = 0.001;

bar_radius = 0.025; % meters
string_radius = 0.005;


%% Prepare environment
nodes = [-barSpacing     barLength*0.5  0;
         -barSpacing    -barLength*0.5  0;
          barSpacing     barLength*0.5  0;
          barSpacing    -barLength*0.5  0;
          0             -barSpacing     barLength*0.5;
          0             -barSpacing    -barLength*0.5;
          0              barSpacing     barLength*0.5;
          0              barSpacing    -barLength*0.5;        
          barLength*0.5  0             -barSpacing;
         -barLength*0.5  0             -barSpacing;
          barLength*0.5  0              barSpacing;
         -barLength*0.5  0              barSpacing];

% Rotate superball to the "reset" position
HH=makehgtform('zrotate',pi/4);
nodes = (HH(1:3,1:3)*nodes')';
HH=makehgtform('xrotate',11*pi/36);
nodes = (HH(1:3,1:3)*nodes')';

nodes(:,3) = nodes(:,3) + 1*barLength;
nodes(:,3) = nodes(:,3) - min(nodes(:,3)); % Make minimum node z=0 height.
nodes(:,1) = nodes(:,1) - mean(nodes(:,1)); % Center x, y, axis
nodes(:,2) = nodes(:,2) - mean(nodes(:,2));

stringRestLength = 0.9*ones(24,1)*norm(nodes(1,:)-nodes(7,:));

superBall = TensegrityStructure(nodes, strings, bars, zeros(12,3), stringStiffness,...
    barStiffness, stringDamping, nodalMass, delT, delT, stringRestLength,wall1,wall2,wall3,wall4);

superBallDynamicsPlot = TensegrityPlot(nodes, strings, bars, bar_radius, string_radius);
f = figure('units','normalized','outerposition',[0 0 1 1]);
% use a method within TensegrityPlot class to generate a plot of the
% structure
generatePlot(superBallDynamicsPlot,gca);
updatePlot(superBallDynamicsPlot);

%settings to make it pretty
axis equal
view(3)
grid on
light('Position',[0 0 10],'Style','local')
lighting flat
colormap([0.8 0.8 1; 0 1 1]);
lims = 1.2*barLength;
xlim([-lims lims])
ylim([-lims lims])
zlim(1.6*[-0.01 lims])
hold on

% Draw Wall 1
wall_x=[wall1 wall1;wall1 wall1];
wall_y=[wall4 wall3;wall4 wall3];
wall_z=[0 0;2.5 2.5];

wall=surf(wall_x,wall_y,wall_z);
%Set wall 1 to be RED
set(wall,'FaceColor',[1 0 0],'FaceAlpha',0.5);

% Draw Wall 2
wall_x=[wall2 wall2;wall2 wall2];
wall_y=[wall4 wall3;wall4 wall3];
wall_z=[0 0;2.5 2.5];

wall=surf(wall_x,wall_y,wall_z);
%Set wall 2 to be GREEN
set(wall,'FaceColor',[0 1 0],'FaceAlpha',0.5);

% Draw Wall 3
wall_x=[wall1 wall2;wall1 wall2];
wall_y=[wall3 wall3;wall3 wall3];
wall_z=[0 0;2.5 2.5];

wall=surf(wall_x,wall_y,wall_z);
%Set wall 3 to be YELLOW
set(wall,'FaceColor',[1 1 0],'FaceAlpha',0.5);

% Draw Wall 4
wall_x=[wall1 wall2;wall1 wall2];
wall_y=[wall4 wall4;wall4 wall4];
wall_z=[0 0;2.5 2.5];

wall=surf(wall_x,wall_y,wall_z);
%Set wall 4 to be BLUE
set(wall,'FaceColor',[0 0 1],'FaceAlpha',0.5);

xlabel('x'); ylabel('y'); zlabel('z');
    


%Policy gradient initialization
hiddenSize=10;
observations=superBall.simStruct.stringRestLengths;
actionSize=3;
gamma=0.99; %decay rate

agent=myPolicyGradient(hiddenSize,observations',actionSize,gamma);

episodeNumber=300;

%%Start Reinforcement Learning
while j<episodeNumber
    
    fprintf('EPISODE %d\n',j);

    %Reset superBall
    superBall = TensegrityStructure(nodes, strings, bars, zeros(12,3), stringStiffness,...
        barStiffness, stringDamping, nodalMass, delT, delT, stringRestLength,wall1,wall2,wall3,wall4);

    updatePlot(superBallDynamicsPlot);
    
    %Reset observations
    observations=superBall.simStruct.stringRestLengths;

    displayTimespan = 0.05; % 20fps. Increase display time interval if system can't keep up.
    
    %Execute episode
    
    myDynamicsUpdate(superBall, superBallDynamicsPlot, displayTimespan,agent,observations);
    
    j=j+1;

end
