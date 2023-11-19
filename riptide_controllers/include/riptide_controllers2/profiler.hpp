#include <ruckig/ruckig.hpp>

class Profiler {
    const static int dof = 6;
    using dofArr = std::array<double, dof>;

    ruckig::Ruckig<dof> profiler;
    ruckig::InputParameter<dof> input;

    std::vector<dofArr> positionData;

public:
    Profiler(dofArr vmax, dofArr amax, dofArr jmax, float timeStep = 0.01f)
    : profiler(timeStep) {
        for(int i = 0; i < dof; i++) {
            input.max_velocity[i] = vmax[i];
            input.max_acceleration[i] = amax[i];
            input.max_jerk[i] = jmax[i];

            // Write default values
            input.current_position[i] = 0.0;
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;

            input.target_position[i] = 0.0;
            input.target_velocity[i] = 0.0;
            input.target_acceleration[i] = 0.0;
        }
    }

    Profiler(dofArr vmin, dofArr vmax, dofArr amin, dofArr doffArr amax, dofArr jmax, float timeStep = 0.01f) {
        Profiler(vmax, amax, jmax, timeStep);

        for(int i = 0; i < dof; i++) {
            input.min_velocity[i] = vmin[i];
            input.min_acceleration[i] = amin[i];
        }
    }

    void calculateNewProfile(dofArr currentPose, dofArr targetPose) {
        for(int i = 0; i < dof; i++) {
            input.current_position[i] = currentPose[i];
            input.current_velocity[i] = currentPose[i + dof];

            input.target_position[i] = targetPose[i];
            input.target_velocity[i] = targetPose[i + dof];
        }

        ruckig::OutputParameter<dof> output;

        while(profiler.update(input, output) == ruckig::Result::Working) {
            auto& p = output.new_position;

            dofArr posData;
            for(int i = 0; i < dof; i++) {
                posData[i] = p[i];
            }

            positionData.push_back(posData);

            output.pass_to_input(input);
        }
    }

    void setConstraints(dofArr vmax, dofArr amax, dofArr jmax) {
        for(int i = 0; i < dof; i++) {
            input.max_velocity[i] = vmax[i];
            input.max_acceleration[i] = amax[i];
            input.max_jerk[i] = jmax[i];

            input.min_velocity[i] = -vmax[i];
            input.min_acceleration[i] = -amax[i];
        }
    }

    void setConstraints(dofArr vmin, dofArr vmax, dofArr amin, dofArr amax, dofArr jmax) {
        for(int i = 0; i < dof; i++) {
            input.max_velocity[i] = vmax[i];
            input.max_acceleration[i] = amax[i];
            input.max_jerk[i] = jmax[i];

            input.min_velocity[i] = vmin[i];
            input.min_acceleration[i] = amin[i];
        }
    }

    std::vector<double> getProfile(int index) {
        std::vector<double> profile;

        for(auto& arr : positionData) {
            profile.push_back(arr[index]);
        }

        return profile;
    }
};