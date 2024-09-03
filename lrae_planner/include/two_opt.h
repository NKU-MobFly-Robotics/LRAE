//
// Created by hjl on 2022/6/7.
// Modified by bqc on 2023/10/24 
//

#ifndef ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H
#define ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H

#include <cmath>
#include <vector>

struct UandC
{
    double u;
    double c;
    std::vector<int> u_c_route;
};

class Two_Opt {
public:
    std::vector<double> points_gains_;
    std::vector<std::vector<double>> cost_matrix_;
    double lambda_;
    std::vector<int> init_route_;

    std::vector<int> best_route_;
    double best_J_ = DBL_MAX;
    UandC bestU_;
    UandC bestC_;

    int searched_route_num_;

    std::vector<UandC> u_cs_;

    Two_Opt(std::vector<int> &init_route, const std::vector<double> &point_gains,
            const std::vector<std::vector<double>> &cost_matrix, const double &lambda) {
        init_route_ = init_route;
        points_gains_ = point_gains;
        cost_matrix_ = cost_matrix;
        lambda_ = lambda;

        searched_route_num_=0;
    }

    void solve() {
        auto current_route = init_route_;
        auto best_route = current_route;
        UandC best_unity = computeUnity(best_route, points_gains_, cost_matrix_, lambda_);
        u_cs_.push_back(best_unity);
        auto new_route = best_route;
        UandC new_unity = best_unity;
        bool is_improved = true;
        while (is_improved) {
            is_improved = false;
            for (int i = 1; i < current_route.size() - 1; ++i) {
                for (int j = i + 1; j < current_route.size(); ++j) {
                    new_route = twoOptSwap(current_route, i, j);
                    new_unity = computeUnity(new_route, points_gains_, cost_matrix_, lambda_);
                    searched_route_num_++;
                    u_cs_.push_back(new_unity);
                    if (new_unity.u > best_unity.u) {
                        best_route = new_route;
                        best_unity = new_unity;
                        is_improved = true;
                    }
                }
            }
            current_route = best_route;
        }
        bestU_ = best_unity;

        current_route = init_route_;
        best_route = current_route;
        UandC best_cost = computeUnity(best_route, points_gains_, cost_matrix_, lambda_);
        u_cs_.push_back(best_cost);
        UandC new_cost = best_cost;
        bool is_improved2 = true;
        while (is_improved2) {
            is_improved2 = false;
            for (int i = 1; i < current_route.size() - 1; ++i) {
                for (int j = i + 1; j < current_route.size(); ++j) {
                    new_route = twoOptSwap(current_route, i, j);
                    new_cost = computeUnity(new_route, points_gains_, cost_matrix_, lambda_);
                    searched_route_num_++;
                    u_cs_.push_back(new_cost);
                    if (new_cost.c < best_cost.c) {
                        best_route = new_route;
                        best_cost = new_cost;
                        is_improved2 = true;
                    }
                }
            }
            current_route = best_route;
        }
        bestC_ = best_cost;

        for(int i = 0; i < u_cs_.size(); i++)
        {
            double eu = (bestU_.u - u_cs_[i].u) / bestU_.u; // G
            double ec = (u_cs_[i].c - bestC_.c) / bestC_.c; // L
            double J = 1 * (eu / (eu + 1)) + 1 * (ec / (ec + 1));
            if(best_J_ > J)
            {
                best_J_ = J;
                best_route_ = u_cs_[i].u_c_route;
            }
        }
    }

    UandC computeUnity(const std::vector<int> &route, const std::vector<double> &point_gains,
                        const std::vector<std::vector<double>> &cost_matrix, const double &lambda) {
        double current_cost = 0.0;
        double current_unity = 0.0;
        for (int i = 1; i < route.size(); i++) {
            current_cost = current_cost + cost_matrix[route[i - 1]][route[i]];
            // current_unity = current_unity + point_gains[route[i]] * exp(-lambda * current_cost);
            current_unity = current_unity + point_gains[route[i]] * (1 / (1 + current_cost));
        }
        UandC u_c;
        u_c.c = current_cost;
        u_c.u = current_unity;
        u_c.u_c_route = route;
        return u_c;
    }

    std::vector<int> twoOptSwap(const std::vector<int> &route, const int &i, const int &j) {
        std::vector<int> swap_route = route;
        std::reverse(swap_route.begin()+i,swap_route.begin()+j+1);

        return swap_route;
    }

};


#endif //ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H
