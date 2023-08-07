function data_struct = UTN_setup(k)
%% TODO: Implement varying turning rates/parking rates based on k
% Implement sets of input and output links

% data_struct.Intersections = [1:16];
% data_struct.Internal_Links = {[1,2], [1,3], [1,5], [1,6], [3,2], [4,2], [4,5], [5,6]};
% data_struct.External_input_links = {[7,1], [8,1], [9,1], [14,4], [13,4]};
% data_struct.External_output_links = {[2,10], [2,11], [3,12], [6,15], [6,16]};
%data_struct.traffic_lights = {[1,2,10], [1,2,11], [1,3,2], [1,3,12], [1,5,6], [1,6,15], [1,6,16], [3,2,10], [3,2,11], 
s = [1 1 1 1 2 2 3 3 4 4 5 6 6 7 8 9 13 14];
t = [2 3 5 6 10 11 2 12 3 5 6 15 16 1 1 1 4 4];
G = digraph(s,t);
data_struct.Intersections = [1:16];
data_struct.Links = G.Edges{:,1}; %Find a way to rename the name of the table
nr_of_traffic_lights = 1;
for i=1:length(s)
    idx = find(s==t(i));
    if isempty(idx)
    else
        for j=idx(1):idx(end)
            data_struct.Traffic_lights{nr_of_traffic_lights} = [s(i),t(i),t(j)];
            nr_of_traffic_lights = nr_of_traffic_lights+1;
        end
    end
end
%data_struct.Traffic_lights = vertcat(data_Struct.Traffic_lights{:});
% data_struct.Traffic_lights = reshape(data_struct.Traffic_lights, [length(data_struct.Traffic_lights),1]);

%% Flow Initialization
data_struct.Saturation_flow = zeros(18,18);
for i=1:length(data_struct.Links)
    data_struct.Saturation_flow(data_struct.Links(i,1), data_struct.Links(i,2)) = 3600/3600; %in veh/h
end
data_struct.Saturation_flow(1,3) = 1800/3600;
data_struct.Saturation_flow(1,5) = 1800/3600;


data_struct.Nominal_inflow = zeros(18,1);
data_struct.Nominal_inflow(1) = 0; 
data_struct.Nominal_inflow(7) = 1000; data_struct.Nominal_inflow(8) = 1100; data_struct.Nominal_inflow(9) = 900; 
data_struct.Nominal_inflow(13) = 1800; data_struct.Nominal_inflow(14) = 1300; 
data_struct.Nominal_inflow = data_struct.Nominal_inflow/3600;
%% Turning Rates
data_struct.Turning_rates = zeros(18,18,18);
%self picked
data_struct.Turning_rates(1,2,10) = 0.25;
data_struct.Turning_rates(1,2,11) = 0.75;


data_struct.Turning_rates(1,5,6) = 1;

data_struct.Turning_rates(1,6,15) = 0.75;
data_struct.Turning_rates(1,6,16) = 0.25;

data_struct.Turning_rates(3,2,10) = 0.85;
data_struct.Turning_rates(3,2,11) = 0.15;

data_struct.Turning_rates(4,5,6) = 1; %mistake from losing a street

%from original paper
data_struct.Turning_rates(7,1,2) = 0.20;
data_struct.Turning_rates(7,1,3) = 0.05;
data_struct.Turning_rates(7,1,5) = 0.05;
data_struct.Turning_rates(7,1,6) = 0.70;

data_struct.Turning_rates(8,1,2) = 0.25;
data_struct.Turning_rates(8,1,3) = 0.30;
data_struct.Turning_rates(8,1,5) = 0.30;
data_struct.Turning_rates(8,1,6) = 0.15;

data_struct.Turning_rates(9,1,2) = 0.65;
data_struct.Turning_rates(9,1,3) = 0.05;
data_struct.Turning_rates(9,1,5) = 0.05;
data_struct.Turning_rates(9,1,6) = 0.25;

data_struct.Turning_rates(1,3,2) = 0.50;
data_struct.Turning_rates(1,3,12) = 0.50; %based on the 0.50 1,3,2

data_struct.Turning_rates(4,3,2) = 0.80;
data_struct.Turning_rates(4,3,12) = 0.20; %based on above info

data_struct.Turning_rates(5,6,15) = 0.30;
data_struct.Turning_rates(5,6,16) = 0.70;

data_struct.Turning_rates(13,4,3) = 0.40;
data_struct.Turning_rates(13,4,5) = 0.60;

data_struct.Turning_rates(14,4,3) = 0.60;
data_struct.Turning_rates(14,4,5) = 0.40;

%% Parking Rates
data_struct.Parking_rates = rand([1 length(data_struct.Links)])/10;
data_struct.Merging_rates = rand([1 length(data_struct.Links)])/10;

data_struct.Traffic_lights = vertcat(data_struct.Traffic_lights{:});

%% Cycle Times
data_struct.Cycle = 120*ones(18,1);

%% Link Capacities
 data_struct.Link_nr_of_lanes(1:length(data_struct.Links),1) = 1;
 data_struct.Link_capacity(1:length(data_struct.Links),1) = 2000; %Obtain accurate link capacity based on MFD
data_struct.Link_free_flow = data_struct.Saturation_flow;

%% Input and Output Links
 for m=1:length(data_struct.Links)
        u = data_struct.Links(m,1);
        d = data_struct.Links(m,2);
        In{m} = data_struct.Traffic_lights(find(data_struct.Traffic_lights(:,2) == u & data_struct.Traffic_lights(:,3) == d),1);
        Out{m} = data_struct.Traffic_lights(find(data_struct.Traffic_lights(:,1) == u & data_struct.Traffic_lights(:,2) == d),3);
 end
 data_struct.Input_nodes = In';
 data_struct.Output_nodes = Out';
 
 %% Free-flow
 data_struct.Free_flow(1:length(data_struct.Links),1) = 3600;
 data_struct.Free_flow(2) = 1800;
 data_struct.Free_flow(3) = 1800;
 
 %% Average Vehicle
 data_struct.Length_average_vehicle = 3.5;