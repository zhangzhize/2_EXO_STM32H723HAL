#include "piecewise_Hermite_interp.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

int main()
{
    // input points
    float xs[]  = {0.0f, 20.0f, 55.0f, 70.0f, 100.0f};
    float ys[]  = {0.0f, 0.0f, 200.0f, 0.0f, 0.0f};
    float dys[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const uint16_t num_xs = 5;

    // create interpolator
    HermiteInterp interp(xs, ys, dys, num_xs);

    // perform interpolation with step = 1.0
    interp.Interp(1.0f);

    if (interp.ptr_y_interp_ == nullptr || interp.num_interp_ == 0)
    {
        std::cerr << "Interpolation failed or no output samples produced." << std::endl;
        return 1;
    }

    // write result to file in the same folder
    std::ofstream ofs("pchip_result.txt", std::ofstream::out | std::ofstream::trunc);
    if (!ofs)
    {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "x,y\n";
    for (uint16_t i = 0; i < interp.num_interp_; ++i)
    {
        float x = interp.x_interp_start_ + i * interp.x_interp_interval_;
        ofs << x << "," << interp.ptr_y_interp_[i] << "\n";
    }

    ofs.close();
    std::cout << "Wrote " << interp.num_interp_ << " samples to pchip_result.txt" << std::endl;
    return 0;
}
