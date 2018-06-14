#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>
#include <data_tools/data_structures.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/binary_constraint_cost.h>
#include <gpgs_slam/unary_constraint_cost.h>
#include <gpgs_slam/visualization.h>
#include <gpgs_slam/multi_visualizer.h>
#include <gpgs_slam/igl_visualizer.h>

#include <ceres/ceres.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <random>
#include <thread>
#include <future>

using namespace std;

void subsample_cloud(Eigen::MatrixXd& points, int subsample)
{
    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (i % subsample == 0) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

void register_processes_ceres(gp_submaps& ss, bool with_rot)
{
    ceres::Problem problem;

    for (const tuple<int, int, Eigen::Vector3d, Eigen::Vector3d>& con : ss.binary_constraints) {
        int i, j;
        Eigen::Vector3d last_point1, first_point2;
        tie (i, j, last_point1, first_point2) = con;
        cout << "Adding binary constraint between " << i << " and " << j << endl;
        ceres::CostFunction* cost_function = BinaryConstraintCostFunctor::Create(last_point1, first_point2, 40.);
        ceres::LossFunction* loss_function = NULL;
        problem.AddResidualBlock(cost_function, loss_function, ss.trans[i].data(), ss.angles[i].data(),
                                                               ss.trans[j].data(), ss.angles[j].data());
    }

    for (const pair<int, int>& match : ss.matches) {
        int i, j;
        tie (i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;
        ceres::CostFunction* cost_function1 = new GaussianProcessCostFunction(ss.gps[i], ss.bounds[i], ss.points[j]);
        ceres::CostFunction* cost_function2 = new GaussianProcessCostFunction(ss.gps[j], ss.bounds[j], ss.points[i]);

        //ceres::LossFunction* loss_function = new ceres::SoftLOneLoss(5.);
        ceres::LossFunction* loss_function1 = new ceres::HuberLoss(.5);
        ceres::LossFunction* loss_function2 = new ceres::HuberLoss(5.);
        ceres::LossFunction* loss_function = NULL;
        problem.AddResidualBlock(cost_function1, loss_function, ss.trans[i].data(), ss.angles[i].data(),
                                                                ss.trans[j].data(), ss.angles[j].data());
        problem.AddResidualBlock(cost_function2, loss_function, ss.trans[j].data(), ss.angles[j].data(),
                                                                ss.trans[i].data(), ss.angles[i].data());
    }
    
    for (int i = 0; i < ss.trans.size(); ++i) {
        ceres::CostFunction* cost_function = UnaryConstraintCostFunctor::Create(ss.angles[i], 0.05);
        ceres::LossFunction* loss_function = NULL;
        problem.AddResidualBlock(cost_function, loss_function, ss.angles[i].data());

        problem.SetParameterLowerBound(ss.trans[i].data(), 0, ss.trans[i](0) - 20.);
        problem.SetParameterLowerBound(ss.trans[i].data(), 1, ss.trans[i](1) - 20.);
        problem.SetParameterUpperBound(ss.trans[i].data(), 0, ss.trans[i](0) + 20.);
        problem.SetParameterUpperBound(ss.trans[i].data(), 1, ss.trans[i](1) + 20.);

        problem.SetParameterLowerBound(ss.angles[i].data(), 2, ss.angles[i](2) - M_PI);
        problem.SetParameterUpperBound(ss.angles[i].data(), 2, ss.angles[i](2) + M_PI);

        if (!with_rot) {
            problem.SetParameterBlockConstant(ss.angles[i].data());
        }
        ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(3, {0, 1});
        problem.SetParameterization(ss.angles[i].data(), subset_parameterization);
    }
    
    //problem.SetParameterBlockConstant(trans[4].data());
    //problem.SetParameterBlockConstant(rots[4].data());

    ceres::Solver::Options options;
    //options.callbacks.push_back(new MultiVisCallback(points, gps, trans, rots));
	IglVisCallback* vis = new IglVisCallback(ss.points, ss.gps, ss.trans, ss.angles, ss.bounds);
    vis->set_matches(ss.matches);
    //vis->display(); // display initial conditions before starting optimization

    options.callbacks.push_back(vis);
    options.max_num_iterations = 200;
    options.update_state_every_iteration = true;
    options.num_threads = 8;
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    
	//vis->display();
	auto handle = std::async(std::launch::async, [&options, &problem, &summary]() {
		ceres::Solve(options, &problem, &summary);
	});
	vis->display();
	handle.get();
    delete vis; // it seems like memory of this is not handled by ceres

    std::cout << summary.FullReport() << '\n';

    std::cout << "Is usable?: " << summary.IsSolutionUsable() << std::endl;
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string file_str;
    string output_str = "gp_results.cereal";
    int subsample = 1;
    bool norot = false;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("file", "Input file", cxxopts::value(file_str))
      ("output", "Output file", cxxopts::value(output_str))
      ("norot", "No rotation?", cxxopts::value(norot))
      ("subsample", "Subsampling rate", cxxopts::value(subsample));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("file") == 0) {
		cout << "Please provide input file arg..." << endl;
		exit(0);
    }
	
    boost::filesystem::path path(file_str);
    boost::filesystem::path output(output_str);
	cout << "Input file : " << path << endl;
	cout << "Output file : " << output << endl;

    gp_submaps ss = read_data<gp_submaps>(path);
	
    ss.matches = compute_matches(ss.trans, ss.rots, ss.bounds);
    //ss.matches.resize(0);
    ss.binary_constraints = compute_binary_constraints(ss.trans, ss.rots, ss.points);
    
    ObsT original_points = ss.points;
    for (Eigen::MatrixXd& p : ss.points) {
        subsample_cloud(p, subsample);
    }

    for (Eigen::Vector3d& t : ss.trans) {
        //t = 1.5*t;
    }

    register_processes_ceres(ss, !norot);

    ss.points = original_points;
    write_data(ss, output);

    return 0;
}
