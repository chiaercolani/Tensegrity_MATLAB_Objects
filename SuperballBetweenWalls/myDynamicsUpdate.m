function myDynamicsUpdate(tensStruct1, dynamicsPlot1, displayTimeInterval)
% This function will perform dynamics update each timestep.

%create some persistent variables for objects and structs
persistent tensStruct dynamicsPlot tspan i

if nargin>1
    i = 0;
    tensStruct = tensStruct1;
    dynamicsPlot = dynamicsPlot1;
    tspan = displayTimeInterval;
end

%%% Optional rest-length controller %%%
% i = i + 1;
% if i > 50 % Start after a certain time.
%     motorsToMove=[];
%     for j = 1:24
%         if round(rand)==1
%             motorsToMove=[motorsToMove;j]
%         end
%     end
%     tensStruct.simStruct.stringRestLengths(motorsToMove) = tensStruct.simStruct.stringRestLengths(motorsToMove)+0.2.*rand-0.1;
% end
i= i+1;
if i==50
    tensStruct.simStruct.stringRestLengths(1)=tensStruct.simStruct.stringRestLengths(1)+0.2;
end
%%% End controller %%%

% Update nodes:
dynamicsUpdate(tensStruct, tspan);
dynamicsPlot.nodePoints = tensStruct.ySim(1:end/2,:);
updatePlot(dynamicsPlot);

drawnow  %plot it up
end

