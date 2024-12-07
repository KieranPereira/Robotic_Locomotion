function penalty = simulateWalkingRobot(params,mdlName,scaleFactor,gait_period,actuatorType)
% Cost function for robot walking optimization
% Copyright 2017-2019 The MathWorks, Inc.

    % Load parameters into function workspace
    robotParameters;
    Assembly1_DataFile;

    % Trajectory sample time
    tsTraj = 0.01;          
    % Simulate air drag for stability 
    world_damping = 20;     % Translational damping
    world_rot_damping = 10; % Rotational damping
    
    % Apply variable scaling
    params = scaleFactor*params;
    
    % Extract simulation inputs from parameters
    N = numel(params)/4;
    hip_motion_F   = deg2rad([params(1:N), params(1)]);
    knee_motion_F  = deg2rad([params(N+1:2*N), params(N+1)]);
    hip_motion_H   = deg2rad([params(2*N+1:3*N), params(2*N+1)]);    
    knee_motion_H  = deg2rad([params(3*N+1:4*N), params(3*N+1)]);

    traj_times = linspace(0,gait_period,N+1);
       
    % Evaluate the trajectory at the start and halfway points for right and
    % left legs, respectively, to get initial conditions and trajectory
    % waypoint derivatives
    [q_F,hip_der_F,knee_der_F] = createSmoothTrajectory( ... 
        hip_motion_F,knee_motion_F,gait_period,[0 gait_period/2]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_FR = [-q_F(1,1) -q_F(2,1)];
    init_angs_FL = [-q_F(1,2) -q_F(2,2)];

    [q_H,hip_der_H,knee_der_H] = createSmoothTrajectory( ... 
        hip_motion_H,knee_motion_H,gait_period,[0 gait_period/2]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_HR = [-q_H(1,1) -q_H(2,1)];
    init_angs_HL = [-q_H(1,2) -q_H(2,2)];
    
    % Simulate the model
    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on');          

%dec 5 change, want travers in x direction instead
    % Unpack logged data
    measBody = get(simout.simout,'measBody').Values;
    yMax = max(abs(measBody.Y.Data));
    xEnd = measBody.X.Data(end);
    tEnd = simout.tout(end);

    % Calculate penalty from logged data
    

    %   Longitudinal (X) distance traveled without falling
    %   (ending simulation early) increases reward
    positiveReward = sign(xEnd)*xEnd^2 * tEnd;
    
    %   Lateral (X) displacement and trajectory aggressiveness 
    %   (number of times the derivative flips signs) decreases reward
    %   NOTE: Set lower limits to prevent divisions by zero
    aggressiveness_F = 0;
    diffs_F = [diff(hip_motion_F) diff(knee_motion_F)];
    aggressiveness_H = 0;
    diffs_H = [diff(hip_motion_H) diff(knee_motion_H)];
    for idx = 1:numel(diffs_F)-1
        if (sign(diffs_F(idx)/diffs_F(idx+1))<0) && mod(idx,N) 
             aggressiveness_F = aggressiveness_F + 1;            
        end
        if (sign(diffs_H(idx)/diffs_H(idx+1))<0) && mod(idx,N) 
             aggressiveness_H = aggressiveness_H + 1;            
        end
    end
    negativeReward = max(yMax,0.1) * (max(aggressiveness_F,1)+max(aggressiveness_H,1));
    
    %   Negative sign needed since the optimization minimizes cost function     
    penalty = -positiveReward/negativeReward;        
    
end

