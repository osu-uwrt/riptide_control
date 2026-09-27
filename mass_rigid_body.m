function [MRB] = mass_rigid_body(mass, moment, r_bcg)
mass = mass * eye(3);
moment = moment * eye(3);
inertia_bcg = [0, m .* r_bcg(3), -m .* r_bcg(2);
                -m .* r_bcg(3), 0, m .* r_bcg(1);
                m .* r_bcg(2), -m .* r_bcg(1), 0];

MRB = [mass(1,:), inertia_bcg(1,:);
       mass(2,:), inertia_bcg(2,:);
       mass(3, :), inertia_bcg(3,:);
       -inertia_bcg(1,:), moment(1,:);
       -inertia_bcg(2,:), moment(2,:);
       -inertia_bcg(3,:), moment(3,:)];

end



