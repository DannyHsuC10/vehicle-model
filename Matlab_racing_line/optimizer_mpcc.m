% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, u, new_b, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, x, u,  x0, u0)

    % The racing line is already fixed by TrackMPC.traj. Use the track
    % borders directly for the MPCC constraints.
    new_b = getFixedTrackBorders(TrackMPC.borders,x,MPC_vars,ModelParams);
    
    % formulate MPCC problem and solve it given the fixed track boundaries
    [xpred, upred,dupred,info] = getMPCmatrices(TrackMPC.traj,MPC_vars,ModelParams,new_b,x,u,x0,u0);

    if (info.exitflag == 0)
        exitflag = 0;
        x = xpred;
        u = upred;
    else
        exitflag = 1;
        % x and u stay identical to the initial guess
    end
end

function borders = getFixedTrackBorders(trackBorders,x,MPC_vars,ModelParams)

N = MPC_vars.N;
thetaIdx = ModelParams.stateindex_theta;
tl = trackBorders.pplx.breaks(end);
borders = zeros(N,4);

for i = 1:N
    theta = mod(x(thetaIdx,i+1),tl);
    borders(i,:) = [ppval(trackBorders.pplx,theta), ...
                    ppval(trackBorders.pply,theta), ...
                    ppval(trackBorders.pprx,theta), ...
                    ppval(trackBorders.ppry,theta)];
end

end
