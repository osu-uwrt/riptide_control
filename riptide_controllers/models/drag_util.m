%process simualtion data
close all;

%number of samples in evaluation data
samples_count = out.samples_count.Data(1);

time = out.time_buffer.Data(1:samples_count);
forces = out.forces_buffer.Data(:, 1:samples_count);
velocities = out.velocity_buffer.Data(:, 1:samples_count);
position = out.position_buffer.Data(:, 1:samples_count);
acceleration = out.acceleration_buffer.Data(:, 1:samples_count);

euler_angles = quat2eul([position(7,:)', position(4,:)',position(5,:)',position(6,:)']);
position_eul = [position(1:3, :); euler_angles'];

%vehicle masses and interias
mass = 31.988;
inertias = [mass, mass, mass, 0.72604692, 1.59322252, 1.65134701];


%process for angular accelerations - improve this
acceleration(4,2:end) = diff(velocities(4,:))' ./ diff(time);
acceleration(5,2:end) = diff(velocities(5,:))' ./ diff(time);
acceleration(6,2:end) = diff(velocities(6,:))' ./ diff(time);

%normalize time
time = time - time(1);
plot_axes_2(time, forces, velocities, "Force vs Axis Vel", ["Forces", "Velocities"])


%drag forces
drag_forces = forces - acceleration .* inertias';
plot_axes_2(time, drag_forces, velocities, "Drag vs on Axis Vel", ["drag forces", "Velocities"])

%drag force relative to linear velocites
plot_axes_2(time, position_eul, [velocities(1:3,:); velocities(1:3,:)], "Drag Vs on Linear Vel", ["Drag Forces", "Velocities"])

%suspect matrices [x,y,z,rx,ry,rz]
apply_mask = [1,0,0,0,0,0;
                0,1,0,0,0,0;
                0,0,1,0,0,0;
                0,1,1,0,0,0;
                1,0,1,0,0,0;
                1,1,0,0,0,0];

correction_matrix = apply_mask;
R_sqaure_matrix = apply_mask;

for correction_axis = 1:6
   %plot control signal
   drag_array = [drag_forces(correction_axis, :);drag_forces(correction_axis, :);drag_forces(correction_axis, :);drag_forces(correction_axis, :);drag_forces(correction_axis, :);drag_forces(correction_axis, :)];
   plot_axes_2(time, drag_array .* apply_mask(correction_axis,:)', velocities(1:6,:) .^ 2, "Control Signal vs Drag: " + num2str(correction_axis), ["Drag Force", "Velocities^2"])

    for motion_axis = 1:6
        %do least squares curve fitting

        [correction_matrix(correction_axis, motion_axis), R_sqaure_matrix(correction_axis, motion_axis)] = fit_cruve(velocities(motion_axis, :), drag_forces(correction_axis, :) * apply_mask(correction_axis, motion_axis));
    end

end

function plot_axes_2(time, forces, velocities, plot_title, legend_array)

    figure;
    for i=1:6
        subplot(6,1,i)

        yyaxis left
        plot(time, forces(i, :))

        yyaxis right
        plot(time, velocities(i,:))

        legend(legend_array)

        if(i == 1)
            title(plot_title)
        end

    end

end

function [coeffiecent, R_squared] = fit_cruve(vel_data, drag_data)

    x_matrix = vel_data' .^ 2;
    y_vector = drag_data';

    coeffiecent = inv(x_matrix' * x_matrix) * x_matrix' * y_vector;

    original_sum_of_sqaures = sum(drag_data .^ 2);
    adjusted_sum_of_sqaures = sum((drag_data - coeffiecent * vel_data .^ 2) .^ 2);

    R_squared = 1 - adjusted_sum_of_sqaures / original_sum_of_sqaures;
end
