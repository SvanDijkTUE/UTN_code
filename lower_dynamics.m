function x = lower_dynamics(state,input, disturbance, k, UTN)
     A = eye(length(UTN.Links)) - diag(UTN.Parking_rates - UTN.Merging_rates);
     if isa(input, 'sdpvar')
         Out = sdpvar(length(UTN.Links),1);
         In = sdpvar(length(UTN.Links),1);
     end
     if isa(input,'double') == true
         Out = zeros(length(UTN.Links),1);
         In = zeros(length(UTN.Links),1);
     end
     for m=1:length(UTN.Links)
        u = UTN.Links(m,1);
        d = UTN.Links(m,2);
        In(m) = 0; Out(m) = 0; 
     %% Determine a_enter at link m
        %find inbound indices
        idxt = find(UTN.Traffic_lights(:,2) == u & UTN.Traffic_lights(:,3) == d); %6 16    ... 6 7 13 14
        if isempty(idxt) ~= true
            idx = idxt;
            light_idx = UTN.Traffic_lights(idx,1);
            for i=1:length(light_idx)
                a_in = UTN.Turning_rates(light_idx(i),u,d)*UTN.Saturation_flow(light_idx(i),u)*input(idx(i))/UTN.Cycle(m);
                In(m) = In(m) + a_in;
            end
        end
     %% Determine a_leave at link m  
     idxt = find(UTN.Traffic_lights(:,1) == u & UTN.Traffic_lights(:,2) == d);
     if isempty(idxt) ~= true
            idx = idxt;
            light_idx = UTN.Traffic_lights(idx,3);
            for i=1:length(light_idx)
                a_out = UTN.Turning_rates(u,d,light_idx(i))*UTN.Saturation_flow(u,d)*input(idx(i))/UTN.Cycle(m);
                Out(m) = Out(m) + a_out;
            end
        end
     end
    
     x = A*state + (In-Out)*UTN.Cycle(1) + disturbance*UTN.Cycle(1);   
     end
     
     
 
     
% for m =1:length(UTN.Traffic_Lights)
%          u = UTN.Links(m,1);
%          d = UTN.Links(m,2);
%          i = UTN.Links(m,3);
%          for i=1:length(UTN.Intersections)
%              if nnz(ismember(UTN.Traffic_lights, [u,d,i], 'rows')) == 1
%                  [LIA,idx] = ismember(UTN.Traffic_lights, [u,d,i], 'rows');
%                  idx = find(idx == 1)
%                  a_leave(m) = UTN.Turning_rates(u,d,i)*UTN.Saturation_flow(u,d)*input(idx)/UTN.Cycle(m);
%                  Out(m) = Out(m) + a_leave(m);
%              end
%              
%          end
%                  
%      end
%              if nnz(ismember(UTN.Traffic_lights, [i,u,d], 'rows')) == 1
%                  UTN.Saturation_flow(i,u);
%                  UTN.Turning_rates(i,u,d);
%                  a_in(m) = UTN.Turning_rates(i,u,d)*UTN.Saturation_flow(i,u)/UTN.Cycle(m);%*input(m);
%                  In(m) = In(m) + a_in(m)
%              end
     %In component
% end