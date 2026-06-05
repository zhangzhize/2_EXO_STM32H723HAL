/**
 * @file piecewise_Hermite_interp.hpp
 * @brief 分段三次 Hermite 插值器
 *
 * 给定 N 个数据点 (x_i, y_i) 及端点导数 dy_i，构造 N-1 段三次 Hermite 多项式。
 * 支持两种使用模式:
 *   1. 实时采样 Sample(x): 在插值表格中线性查找相邻点并线性插值 (O(log N))
 *   2. 预计算表格 Interp(): 以固定步长预计算均匀间隔的 y 值，后续采样 O(1)
 *
 * Hermite 多项式形式 (第 i 段区间 [x_i, x_{i+1}]):
 *   P(s) = a·s³ + b·s² + c·s + d,  其中 s = x - x_i
 *
 * 应用场景: 外骨骼力廓线生成器中的参考力矩曲线平滑插值
 */
#ifndef PIECEWISE_HERMLITE_INTERP_HPP
#define PIECEWISE_HERMLITE_INTERP_HPP

#include <cstdint>

class HermiteInterp
{
public:
    HermiteInterp();
    ~HermiteInterp();

    /**
     * @brief 计算所有分段的三次 Hermite 系数
     * @param ptr_xs  输入 x 坐标数组 (必须严格递增，相邻点间距 > 1e-6)
     * @param ptr_ys  输入 y 坐标数组
     * @param ptr_dys 各点的导数值 dy/dx
     * @param num_xs  数据点数量 (≥ 2)
     */
    void CalCoeffs(float* ptr_xs, float* ptr_ys, float *ptr_dys, uint16_t num_xs);

    /**
     * @brief 以固定步长预计算插值表格
     * @param x_interp_interval  采样间隔 (必须 > 1e-4)
     *
     * 从 x_0 到 x_{N-1}，以 x_interp_interval 为步长计算 y 值存入内部数组。
     * 后续 Sample() 可在预计算表格中 O(1) 查询。
     */
    void Interp(float x_interp_interval);

    /**
     * @brief 在预计算表格中采样 y 值 (需先调用 Interp())
     * @param x  查询点坐标
     * @return 插值结果。x 超出范围时返回最近端点的 y 值 (饱和外推)
     *
     * 内部使用二分查找定位，在相邻表项之间做线性插值。
     */
    float Sample(float x) const;

    /* 只读访问器 — 供外部查询插值表格属性 */
    uint16_t GetNumInterp() const;
    float GetXInterpStart() const;
    float GetXInterpInterval() const;
    const float* GetYInterp() const;

private:
    float *ptr_xs_;       /*!< 输入数据点 x 坐标数组 (内部副本) */
    float *ptr_ys_;       /*!< 输入数据点 y 坐标数组 (内部副本) */
    float *ptr_dys_;      /*!< 输入数据点导数值数组 (内部副本) */
    float *ptr_coef_a_;   /*!< 各段三次系数 a (s³ 项) */
    float *ptr_coef_b_;   /*!< 各段三次系数 b (s² 项) */
    float *ptr_coef_c_;   /*!< 各段三次系数 c (s¹ 项) */
    float *ptr_coef_d_;   /*!< 各段三次系数 d (s⁰ 项 = y_i) */
    uint16_t num_xs_;     /*!< 数据点数量 */

    float x_interp_start_;     /*!< 预计算表格的起始 x 值 (= x_0) */
    float x_interp_interval_;  /*!< 预计算表格的采样间隔 */
    uint16_t num_interp_;      /*!< 预计算表格的数据点数量 */
    float *ptr_y_interp_;      /*!< 预计算 y 值表格 */
};


#endif