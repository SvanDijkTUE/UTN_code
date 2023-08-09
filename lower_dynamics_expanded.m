function [x q] = lower_dynamics_expanded(state,input, disturbance, queue, k, UTN)
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
        
        %Some queue calculations
        gamma  = UTN.Link_capacity(m) - sum(queue(find(UTN.Traffic_lights(:,1) == u & UTN.Traffic_lights(:,2) == d)))*UTN.Length_average_vehicle/(UTN.Free_flow(m)*UTN.Link_nr_of_lanes(m)*UTN.Cycle(m));
        tau = floor(gamma);
        gamma  = gamma-tau;
        
        In(m) = 0; Out(m) = 0;
     %% Determine a_enter at link m
        %find inbound indices
        for i = UTN.Input_nodes{m}'
            idxt = find(UTN.Traffic_lights(:,1) == i & UTN.Traffic_lights(:,2) == u & UTN.Traffic_lights(:,3) == d);
            a_in_saturation = UTN.Turning_rates(i,u,d)*UTN.Saturation_flow(i,u)*input(idxt)/UTN.Cycle(m);
            a_in_queue = 0;
            a_in_at_capacity = 0;
            In(m) = In(m) + min([a_in_saturation, a_in_queue, a_in_at_capacity]);
        end
     %% Determine a_leave at link m  
        for o = UTN.Output_nodes{m}'
            idxt = find(UTN.Traffic_lights(:,1) == u & UTN.Traffic_lights(:,2) == d & UTN.Traffic_lights(:,3) == o);
            %cars leaving if the saturation flow can be reached
            a_out_saturation = UTN.Turning_rates(u,d,o)*UTN.Saturation_flow(u,d)*input(idxt)/UTN.Cycle(m);
           
            %cars entering the leave queue
            %q(idxt) = queue(idx) + (a_arriv - a_leave)*UTN.Cycle(m);
            a_out_queue = 1000;
            
            %cars leaving when the target link is at capacity
            out_link = find(UTN.Links(:,1) == d & UTN.Links(:,2) == o);
            a_out_at_capacity = UTN.Turning_rates(u,d,o)/sum(UTN.Turning_rates(UTN.Input_nodes{out_link},d,o))*(UTN.Link_capacity(out_link) - state(out_link))/UTN.Cycle(out_link);
            
            %cars leaving
            Out(m) = Out(m) + min([a_out_saturation, a_out_queue, a_out_at_capacity]);
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
     q = queue;
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