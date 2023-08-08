function x = lower_dynamics_expanded(state,input, disturbance, k, UTN)
%% TODO 
%Incorporate that at the external links cars leave at speed equal to
%saturation flow of that external link

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
        for i=1:length(UTN.Input_nodes{m})
            idxt = find(UTN.Traffic_lights(:,1) == UTN.Input_nodes{m}(i) & UTN.Traffic_lights(:,2) == u & UTN.Traffic_lights(:,3) == d);
            a_in = UTN.Turning_rates(UTN.Input_nodes{m}(i),u,d)*UTN.Saturation_flow(UTN.Input_nodes{m}(i),u)*input(idxt)/UTN.Cycle(m);
            In(m) = In(m) + a_in;
        end
     %% Determine a_leave at link m  
        for i=1:length(UTN.Output_nodes{m})
            idxt = find(UTN.Traffic_lights(:,1) == u & UTN.Traffic_lights(:,2) == d & UTN.Traffic_lights(:,3) == UTN.Output_nodes{m}(i));
            a_out = UTN.Turning_rates(u,d,UTN.Output_nodes{m}(i))*UTN.Saturation_flow(u,d)*input(idxt)/UTN.Cycle(m);
            Out(m) = Out(m) + a_out;
        end
        
        %assuming the exit Links empty out at a speed proportional to the
        %number of cars in the street.
        if ismember(m, UTN.External_Output_Links) == 1
            Out(m) = Out(m) + min(UTN.Saturation_flow(u,d), state(m)/UTN.Cycle(m));
        end
     end
      
     %Behavior on external links
%      for m = find(UTN.Links(:,1) > 6 | UTN.Links(:,2) > 6)
%          Out(m) = Out(m) + 1;
%      end
     x = A*state + (In-Out)*UTN.Cycle(1) + disturbance;   
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