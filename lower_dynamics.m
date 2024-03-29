function x = lower_dynamics(state,input, disturbance, k, UTN)
%% TODO 
%Incorporate that at the external links cars leave at speed equal to
%saturation flow of that external link  -- DONE 08/08/2023

%Incorporate the min condition

     A = eye(length(UTN.Links)) - diag(UTN.Parking_rates - UTN.Merging_rates);
     A = eye(length(UTN.Links));
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
        for i = UTN.Input_nodes{m}'
            idxt = find(UTN.Traffic_lights(:,1) == i & UTN.Traffic_lights(:,2) == u & UTN.Traffic_lights(:,3) == d);
            a_in = UTN.Turning_rates(i,u,d)*UTN.Saturation_flow(i,u)*input(idxt)/UTN.Cycle(m);
            In(m) = In(m) + a_in;
        end
     %% Determine a_leave at link m  
        for o = UTN.Output_nodes{m}'
            idxt = find(UTN.Traffic_lights(:,1) == u & UTN.Traffic_lights(:,2) == d & UTN.Traffic_lights(:,3) == o);
            a_out = UTN.Turning_rates(u,d,o)*UTN.Saturation_flow(u,d)*input(idxt)/UTN.Cycle(m);
            Out(m) = Out(m) + a_out;
        end
        
        %assuming the exit Links empty out at a speed proportional to the
        %number of cars in the street.
        if UTN.Options.Empty_output_links == true
            if ismember(m, UTN.External_Output_Links) == 1
                Out(m) = Out(m) + min(UTN.Saturation_flow(u,d), state(m)/UTN.Cycle(m));
            end
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