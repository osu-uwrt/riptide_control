// Re-fits a saved pool identification recording (e.g. against a different prior
// model, or after changing the fitter):
//   ros2 run riptide_mpc identify_fit --dir <session dir> [--model <prior yaml>] [--vehicle <vehicle yaml>]
// Writes report.yaml and model_identified.yaml into the session directory.
#include "riptide_mpc/identification.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>

using namespace riptide_mpc;

int main(int argc, char **argv) {
    std::string dir, model = ament_index_cpp::get_package_share_directory("riptide_mpc") + "/config/models/talos.yaml";
    std::string vehicle = ament_index_cpp::get_package_share_directory("riptide_descriptions2") + "/config/talos.yaml";
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--dir"))
            dir = argv[i + 1];
        else if (!std::strcmp(argv[i], "--model"))
            model = argv[i + 1];
        else if (!std::strcmp(argv[i], "--vehicle"))
            vehicle = argv[i + 1];
    }
    if (dir.empty()) {
        std::fprintf(stderr, "usage: identify_fit --dir <session dir> [--model yaml] [--vehicle yaml]\n");
        return 2;
    }
    std::vector<ident::Sample> samples;
    std::vector<ident::Segment> segments;
    ident::readRecording(dir, samples, segments);
    const YAML::Node prior = YAML::LoadFile(model);
    // The recording's per-thruster thrust was computed with the session's model; --model should be that one.
    const MatrixXd thruster_matrix = FossenModel::load(vehicle, model).thrusterMatrix();
    const auto result =
        ident::fit(prior, YAML::LoadFile(vehicle)["mass"].as<double>(), samples, segments, thruster_matrix,
                   YAML::LoadFile(vehicle));
    YAML::Node report = ident::report(prior, result);
    report["prior_model"] = model;
    std::ofstream(dir + "/report.yaml") << report << "\n";
    std::ofstream(dir + "/model_identified.yaml") << ident::identifiedModel(prior, result, "refit of " + dir + " from " + model) << "\n";
    std::printf("%s\n\nwrote %s/report.yaml and %s/model_identified.yaml\n", YAML::Dump(report).c_str(), dir.c_str(),
                dir.c_str());
    return 0;
}
