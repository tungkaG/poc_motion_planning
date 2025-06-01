#include <iostream>
#include <stdio.h>
#include <memory>
#include <vector>
#include <stdlib.h>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include "include/TrajectoryEvaluator.hpp"
#include "TrajectoryHandler.hpp"
#include "CoordinateSystemWrapper.hpp"

// Global params
static constexpr int num_samples_d = 9;
double sampling_d[num_samples_d + 1]; // +1 in case d is not present and needs to be added
static int actual_d_samples = 0;

static constexpr int num_samples_time = 7;
double sampling_time[num_samples_time + 1]; // +1 in case t is not present and needs to be added
static int actual_t_samples = 0;
static double N = 30; //  self.N = int(config_plan.planning.planning_horizon / config_plan.planning.dt)
static double dT = 0.1;
    
using SamplingMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, 13, Eigen::RowMajor>;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

SamplingMatrixXd generate_sampling_matrix_cstyle(
    double* t0_range, size_t t0_size,
    double* t1_range, size_t t1_size,
    double* s0_range, size_t s0_size,
    double* ss0_range, size_t ss0_size,
    double* sss0_range, size_t sss0_size,
    double* ss1_range, size_t ss1_size,
    double* sss1_range, size_t sss1_size,
    double* d0_range, size_t d0_size,
    double* dd0_range, size_t dd0_size,
    double* ddd0_range, size_t ddd0_size,
    double* d1_range, size_t d1_size,
    double* dd1_range, size_t dd1_size,
    double* ddd1_range, size_t ddd1_size,
    bool debug_mode = true
) {
    size_t total_combinations = t0_size * t1_size * s0_size * ss0_size * sss0_size *
                              ss1_size * sss1_size * d0_size * dd0_size * ddd0_size *
                              d1_size * dd1_size * ddd1_size;
    
    SamplingMatrixXd sampling_matrix(total_combinations, 13);
    
    size_t idx = 0;
    for (size_t i0 = 0; i0 < t0_size; ++i0)
    for (size_t i1 = 0; i1 < t1_size; ++i1)
    for (size_t i2 = 0; i2 < s0_size; ++i2)
    for (size_t i3 = 0; i3 < ss0_size; ++i3)
    for (size_t i4 = 0; i4 < sss0_size; ++i4)
    for (size_t i5 = 0; i5 < ss1_size; ++i5)
    for (size_t i6 = 0; i6 < sss1_size; ++i6)
    for (size_t i7 = 0; i7 < d0_size; ++i7)
    for (size_t i8 = 0; i8 < dd0_size; ++i8)
    for (size_t i9 = 0; i9 < ddd0_size; ++i9)
    for (size_t i10 = 0; i10 < d1_size; ++i10)
    for (size_t i11 = 0; i11 < dd1_size; ++i11)
    for (size_t i12 = 0; i12 < ddd1_size; ++i12) {
        sampling_matrix(idx, 0) = t0_range[i0];
        sampling_matrix(idx, 1) = t1_range[i1];
        sampling_matrix(idx, 2) = s0_range[i2];
        sampling_matrix(idx, 3) = ss0_range[i3];
        sampling_matrix(idx, 4) = sss0_range[i4];
        sampling_matrix(idx, 5) = ss1_range[i5];
        sampling_matrix(idx, 6) = sss1_range[i6];
        sampling_matrix(idx, 7) = d0_range[i7];
        sampling_matrix(idx, 8) = dd0_range[i8];
        sampling_matrix(idx, 9) = ddd0_range[i9];
        sampling_matrix(idx, 10) = d1_range[i10];
        sampling_matrix(idx, 11) = dd1_range[i11];
        sampling_matrix(idx, 12) = ddd1_range[i12];
        ++idx;
    }
    if (debug_mode) {
        std::cout << "<ReactivePlanner>: " << idx << " trajectories sampled" << std::endl;
    }
    
    return sampling_matrix;
}

int main() {
    // STATIC PART -------------------------------------------------------------------------------
    // Generate 9 linearly spaced samples between d_min and d_max
    static double d_min = -3.0;
    static double d_max = 3.0;

    static double d_step = (d_max - d_min) / (num_samples_d - 1);
    for (int i = 0; i < num_samples_d; ++i) {
        sampling_d[i] = d_min + i * d_step;
    }
    actual_d_samples = num_samples_d;

    // Generate 7 linearly spaced samples between t_min and t_max
    static double t_min = 1.1;
    static double step_size = 3;
    static double t_max = 3.0;

    sampling_time[actual_t_samples] = t_min;
    ++actual_t_samples;
    static double t_val = t_min + step_size * dT;
    while(t_val < t_max) {
        sampling_time[actual_t_samples] = t_val;
        ++actual_t_samples;
        t_val += step_size * dT;
    }

    // reference_path
    
    static double vehicle_a_max = 11.5;
    static double horizon = 3.0;
    static double vehicle_v_max = 50.8;
    static double v_limit = 36.0;
    // STATIC PART -------------------------------------------------------------------------------

    // DYNAMIC PART -------------------------------------------------------------------------------

    double s = 156.12;                   
    double ss = 5.62;                   
    double sss = 0.0521;               
    double d = 0.076;            
    double dd = -0.283;        
    double ddd = 0.623;

    double velocity = 5.634;
    // double timestep = msg.timestep;
    double desired_velocity = 8.243;

    // Generate 9 linearly spaced samples between min_v and max_v
    double min_v = std::max(0.001, velocity - vehicle_a_max * horizon);
    double max_v = std::min({velocity + (vehicle_a_max / 6.0) * horizon, v_limit, vehicle_v_max});
    static constexpr int num_samples_v = 9;
    double sampling_v[num_samples_v + 1]; // +1 in case ss is not present and needs to be added
    int actual_v_samples = 0;

    double step_v = (max_v - min_v) / (num_samples_v - 1);
    sampling_v[0] = ss; ++actual_v_samples;
    for (int i = 1; i < num_samples_v+1; ++i) {
        if(sampling_v[i] != ss) {
            ++actual_v_samples;
            sampling_v[i] = min_v + (i-1) * step_v;
        }
    }
    
    // Add d to sampling_d if not present
    bool add_d = true;
    for (int i = 0; i < num_samples_d; ++i) {
        if(sampling_d[i] == d) {
            add_d = false;
            break;  
        }
    }
    if (add_d) {
        sampling_d[actual_d_samples] = d;
        actual_d_samples++;
    }

    // Add N*dT to sampling_time if not present
    bool add_time = true;
    for (int i = 0; i < num_samples_time; ++i) {
        if(sampling_time[i] == N*dT) {
            add_time = false;
            break;
        }
    }
    if (add_time) {
        sampling_time[actual_t_samples] = N*dT;
        actual_t_samples++;
    }

    size_t t0_size = 1;
    size_t t1_size = actual_t_samples;
    size_t s0_size = 1;
    size_t ss0_size = 1;
    size_t sss0_size = 1;
    size_t ss1_size = actual_v_samples;
    size_t sss1_size = 1;
    size_t d0_size = 1;
    size_t dd0_size = 1;
    size_t ddd0_size = 1;
    size_t d1_size = actual_d_samples;
    size_t dd1_size = 1;
    size_t ddd1_size = 1;

    double t0_range[1] = {0.0};
    double s0_range[1] = {s};
    double ss0_range[1] = {ss};
    double sss0_range[1] = {sss};
    double sss1_range[1] = {0.0};
    double d0_range[1] = {d};
    double dd0_range[1] = {dd};
    double ddd0_range[1] = {ddd};
    double dd1_range[1] = {0.0};
    double ddd1_range[1] = {0.0};

    SamplingMatrixXd sampling_matrix = generate_sampling_matrix_cstyle(
        t0_range, t0_size,
        sampling_time, t1_size,
        s0_range, s0_size,
        ss0_range, ss0_size,
        sss0_range, sss0_size,
        sampling_v, ss1_size,
        sss1_range, sss1_size,
        d0_range, d0_size,
        dd0_range, dd0_size,
        ddd0_range, ddd0_size,
        sampling_d, d1_size,
        dd1_range, dd1_size,
        ddd1_range, ddd1_size
    );

    // // Print the sampling matrix
    // std::cout << "Sampling Matrix:" << std::endl;
    // for (int i = 0; i < sampling_matrix.rows(); ++i) {
    //     for (int j = 0; j < sampling_matrix.cols(); ++j) {
    //         std::cout << sampling_matrix(i, j) << " ";
    //     }
    //     std::cout << std::endl;
    // }

    TrajectoryHandler trajectory_handler(0.1, desired_velocity);
    trajectory_handler.generateTrajectories(sampling_matrix, false);
    // trajectory_handler.evaluateTrajectory(trajectory_handler.m_trajectories[0]);

    RowMatrixXd path(3, 2);  // 3 points (x,y)
    path << 1.0, 2.0,   // Point 1
            3.0, 4.0,   // Point 2
            5.0, 6.0;   // Point 3

    CoordinateSystemWrapper coordinate_system_wrapper(path);  // Pass the path to the wrapper
}


// t = array([1.7, 1.1, 2. , 2.3, 1.4, 2.6, 2.9, 3. ])
// v = array([1.42397132e+00, 2.84694265e+00, 4.26991398e+00, 5.69288530e+00,
//        5.61927802e+00, 7.11585663e+00, 8.53882795e+00, 9.96179927e+00,
//        1.13847706e+01, 1.00000000e-03])
// d = array([-0.75      ,  0.        ,  0.75      ,  1.5       ,  2.25      ,
//         3.        ,  0.07589202, -3.        , -2.25      , -1.5       ])