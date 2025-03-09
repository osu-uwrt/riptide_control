%process simualtion data
close all;

%number of samples in evaluation data
samples_count = out.samples_count.Data(1);

time = out.time_buffer.Data(1:samples_count);
forces = out.forces_buffer.Data(:, 1:samples_count);
velocities = out.velocity_buffer.Data(:, 1:samples_count);
position = out.position_buffer.Data(:, 1:samples_count);
acceleration = out.acceleration_buffer.Data(:, 1:samples_count);

%normalize time
time = time - time(1);

plot_axes_2(time, forces, velocities, ["Forces", "Velocities"])

plot_axes_2(time, acceleration ./ forces, velocities, ["Acceleration / Forces", "Velocities"])



function plot_axes_2(time, forces, velocities, legend_array)

    figure;
    
    for i=1:6
        subplot(6,1,i)

        yyaxis left
        plot(time, forces(i, :))

        yyaxis right
        plot(time, velocities(i,:))

        legend(legend_array)
    end

end

