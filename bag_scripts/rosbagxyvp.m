%% clear data
clear

%% choose a measurement
measurement_database = dir('*2020-02*.bag');
[chosenBag,~] = listdlg('PromptString','Select a mat file:','SelectionMode','single', 'ListString', {measurement_database.name}, 'ListSize', [500 200]);
if ~isempty(chosenBag)
    bag = rosbag((measurement_database(chosenBag).name));
    id = measurement_database(chosenBag).name;
else
    disp('Nothing is chosen');
    return;
end

%% /gps/current_pose
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/gps/current_pose');
sel_struct = readMessages(sel,'DataFormat','struct');
gps_kvh_x_pos = cellfun(@(m) double(m.Pose.Position.X), sel_struct);
gps_kvh_y_pos = cellfun(@(m) double(m.Pose.Position.Y), sel_struct);
tmp_gps_kvh_q_w = cellfun(@(m) double(m.Pose.Orientation.W), sel_struct);
tmp_gps_kvh_q_x = cellfun(@(m) double(m.Pose.Orientation.X), sel_struct);
tmp_gps_kvh_q_y = cellfun(@(m) double(m.Pose.Orientation.Y), sel_struct);
tmp_gps_kvh_q_z = cellfun(@(m) double(m.Pose.Orientation.Z), sel_struct);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), sel_struct);
sec = cellfun(@(m) double(m.Header.Stamp.Sec), sel_struct);
gps_kvh_time = sec + nsec / 1000000000;

%% /gps/duro/current_pose
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/gps/duro/current_pose');
sel_struct = readMessages(sel,'DataFormat','struct');
gps_duro_x_pos = cellfun(@(m) double(m.Pose.Position.X), sel_struct);
gps_duro_y_pos = cellfun(@(m) double(m.Pose.Position.Y), sel_struct);
tmp_gps_duro_q_w = cellfun(@(m) double(m.Pose.Orientation.W), sel_struct);
tmp_gps_duro_q_x = cellfun(@(m) double(m.Pose.Orientation.X), sel_struct);
tmp_gps_duro_q_y = cellfun(@(m) double(m.Pose.Orientation.Y), sel_struct);
tmp_gps_duro_q_z = cellfun(@(m) double(m.Pose.Orientation.Z), sel_struct);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), sel_struct);
sec = cellfun(@(m) double(m.Header.Stamp.Sec), sel_struct);
gps_duro_time = sec + nsec / 1000000000;

%% /gps/duro/imu >> accelerations
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/gps/duro/imu');
sel_struct = readMessages(sel,'DataFormat','struct');
gps_duro_x_acc = cellfun(@(m) double(m.X), sel_struct);
gps_duro_y_acc = cellfun(@(m) double(m.Y), sel_struct);
gps_duro_z_acc = cellfun(@(m) double(m.Z), sel_struct);

%% /current_velocity
%{
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/current_velocity');
sel_struct = readMessages(sel,'DataFormat','struct');
vehicle_velo = cellfun(@(m) double(m.Twist.Linear.X), sel_struct);
vehicle_steer = cellfun(@(m) double(m.Twist.Angular.Z), sel_struct);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), sel_struct);
sec = cellfun(@(m) double(m.Header.Stamp.Sec), sel_struct);
vehicle_time = sec + nsec / 1000000000;
%}

%% /vehicle_status
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/vehicle_status');
sel_struct = readMessages(sel,'DataFormat','struct');
vehicle_velo = cellfun(@(m) double(m.Speed), sel_struct);
vehicle_steer = cellfun(@(m) double(m.Angle), sel_struct);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), sel_struct);
sec = cellfun(@(m) double(m.Header.Stamp.Sec), sel_struct);
vehicle_time = sec + nsec / 1000000000;

%% quaternions
tmp_gps_kvh_quat = horzcat(tmp_gps_kvh_q_x, tmp_gps_kvh_q_x, tmp_gps_kvh_q_z, tmp_gps_kvh_q_w);
tmp_euler_kvh_gps = quat2eul(tmp_gps_kvh_quat);
gps_kvh_yaw_ori = tmp_euler_kvh_gps(:,3);

tmp_gps_duro_quat = horzcat(tmp_gps_duro_q_x, tmp_gps_duro_q_x, tmp_gps_duro_q_z, tmp_gps_duro_q_w);
tmp_euler_duro_gps = quat2eul(tmp_gps_duro_quat);
gps_duro_yaw_ori = tmp_euler_duro_gps(:,3);

%% xy to zero
%zero_x = gps_x_pos(1);
%zero_y = gps_y_pos(1);
%gps_x_pos = gps_x_pos - zero_x;
%gps_y_pos = gps_y_pos - zero_y;

%% time to zero
zero_time = gps_kvh_time(1);
gps_kvh_time = gps_kvh_time - zero_time;
vehicle_time = vehicle_time - zero_time;

%% horzcat
odom_velo = (horzcat(vehicle_velo, vehicle_steer))';
gps_kvh = (horzcat(gps_kvh_x_pos, gps_kvh_y_pos, gps_kvh_yaw_ori))';

%%
hold on
zoom on
grid on
legend on
axis equal
calculated_x = [];
calculated_y = [];
calculated_theta = [];
resample = 1; % 1 means no resample
x = gps_kvh_x_pos(1); y = gps_kvh_y_pos(1); theta = gps_kvh_yaw_ori(1); % initial orientation 
deltaT = max(vehicle_time) / size(vehicle_time,1) * resample; % Hz
plot(gps_kvh_x_pos, gps_kvh_y_pos, '.', 'DisplayName', 'gps kvh', 'MarkerSize', 6)
for i = 1:resample:size(vehicle_velo)
    vehicle_speed = vehicle_velo(i);
    steer_angle = vehicle_steer(i);    
    x = x +(deltaT * vehicle_speed * cos(theta));
    y = y +(deltaT * vehicle_speed * sin(theta));
    theta = wrapToPi(theta + (deltaT * vehicle_speed / 2.7 * tan(steer_angle)));
    calculated_x = [calculated_x x];
    calculated_y = [calculated_y y];
    calculated_theta = [calculated_theta theta];
end
hold on;
plot(calculated_x,calculated_y, '.', 'DisplayName', 'calculated odom');
plot(gps_duro_x_pos, gps_duro_y_pos, '.', 'DisplayName', 'gps duro', 'MarkerSize', 6)
xlabel('x [meters]')
ylabel('y [meters]')
title(id, 'Interpreter', 'none');
set(gca,'xticklabel',num2str(get(gca,'xtick')','%d'))
set(gca,'yticklabel',num2str(get(gca,'ytick')','%d'))

%%
figure
legend on; hold on;
plot(gps_duro_x_acc, 'DisplayName', 'x acceleration', 'MarkerSize', 6); 
plot(gps_duro_y_acc, 'DisplayName', 'y acceleration', 'MarkerSize', 6); 
plot(gps_duro_z_acc, 'DisplayName', 'z acceleration', 'MarkerSize', 6); 

%% save to mat file
% save(strcat('odom_', id(1:end-3), 'mat'), 'gps', 'odometry', 'id')

%% save img
saveas(gcf,strcat('v_', id(1:end-3),'png'))