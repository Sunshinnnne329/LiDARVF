close all;
clear variables;

conditions.bShowVectorField=~true;
conditions.bRunUAV=true;
conditions.bPlotQuiverNorm=~true;
conditions.bAnimate=~true;
conditions.bNormVF=~true;
%bVaryUAV_Vel=true;
conditions.bUseVRel=true;
conditions.L=0;
p = gcp(); % get the current parallel pool

%res_noRel = RunVFLoiter(conditions);
fprintf('starting 1\n');
f(1) = parfeval(p,@RunVFLoiter,1,conditions);

conditions2=conditions;
conditions2.bUseVRel=true;
conditions2.L=1;
%res_Rel = RunVFLoiter(conditions);
fprintf('starting 2 - RelVel\n');
f(2) = parfeval(p,@RunVFLoiter,1,conditions2);
%return;

%value = fetchOutputs(f); % Blocks until complete
figure;hold all;
for idx = 1:2
  % fetchNext blocks until next results are available.
  [completedIdx,value] = fetchNext(f);
  errResults{completedIdx} = value;
  fprintf('Got result with index: %d.\n', completedIdx);
  if(completedIdx==1)
      clr = 'b-';
  else
      clr = 'r.';
  end
  plot(value.err,clr);
  %scatter(value.uav_pos(1,:),value.uav_pos(2,:));
end

figure;hold all;

for idx = 1:2
    scatter(errResults{idx}.uav_pos(1,:),errResults{idx}.uav_pos(2,:));
end