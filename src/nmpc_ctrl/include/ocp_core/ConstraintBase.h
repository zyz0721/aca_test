#pragma once

#include <casadi/casadi.hpp>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <set>

// ============================================================
// 阶段选择器 — 约束作用于哪些阶段
// ============================================================

/**
 * 描述一个约束作用于 N+1 个阶段中的哪些。
 * 
 * 用法:
 *   StageSelector::all()          → 0,1,...,N
 *   StageSelector::initial()      → 0
 *   StageSelector::path()         → 1,...,N-1
 *   StageSelector::terminal()     → N
 *   StageSelector::pathAndTerminal() → 1,...,N
 *   StageSelector::range(3, 10)   → 3,4,...,10
 *   StageSelector::except0()      → 1,...,N  (同 pathAndTerminal)
 */
class StageSelector {
public:
    /// 是否作用于阶段 k (0-indexed, 总共 N+1 个阶段: 0..N)
    bool includes(int k, int N) const {
        if (custom_) return custom_(k, N);
        int lo = resolve(lo_raw_, N);
        int hi = resolve(hi_raw_, N);
        return k >= lo && k <= hi;
    }

    static StageSelector all()             { return {0, -1}; }         // 0..N
    static StageSelector initial()         { return {0, 0}; }          // 0
    static StageSelector path()            { return {1, -2}; }         // 1..N-1
    static StageSelector terminal()        { return {-1, -1}; }        // N
    static StageSelector pathAndTerminal() { return {1, -1}; }         // 1..N
    static StageSelector range(int lo, int hi) { return {lo, hi}; }    // lo..hi

    /// 自定义谓词
    static StageSelector custom(std::function<bool(int,int)> fn) {
        StageSelector s;
        s.custom_ = std::move(fn);
        return s;
    }

private:
    StageSelector() = default;
    StageSelector(int lo, int hi) : lo_raw_(lo), hi_raw_(hi) {}

    int lo_raw_ = 0;
    int hi_raw_ = -1;  // -1 = N, -2 = N-1, >=0 = literal
    std::function<bool(int,int)> custom_;

    // 统一进行解析：-1=N， -2=N-1, 其他 = 字面值
    int resolve(int raw, int N) const {
        if (raw == -1) return N;
        if (raw == -2) return N - 1;
        return raw;
    }
};

// ============================================================
// 约束描述 — 三种数学形式的统一容器
// ============================================================

/// 边界约束: lb[i] <= var[idx[i]] <= ub[i]
struct BoundConstraintData {
    enum VarType { STATE, INPUT };
    VarType var_type;
    std::vector<int>    idx;
    std::vector<double> lb;
    std::vector<double> ub;
    int dim() const { return static_cast<int>(idx.size()); }
};

/// 线性约束: lg <= C*x + D*u <= ug
struct LinearConstraintData {
    int ng = 0;
    std::vector<double> C;   // ng × nx, 列主序
    std::vector<double> D;   // ng × nu, 列主序
    std::vector<double> lg;
    std::vector<double> ug;
};

/// 非线性约束基类: lh <= h(x,u,p) <= uh
class NonlinearConstraintBase {
public:
    virtual ~NonlinearConstraintBase() = default;
    virtual casadi::MX computeConstraint(
        const casadi::MX& x, const casadi::MX& u, const casadi::MX& p) const = 0;
    virtual int getNumConstraints() const = 0;
    virtual std::string getName() const = 0;
    virtual void getBounds(std::vector<double>& lh, std::vector<double>& uh) const {
        int n = getNumConstraints();
        lh.assign(n, 0.0);
        uh.assign(n, 1e6);
    }
};


// 带阶段归属的约束条目
struct BoundEntry {
    StageSelector stages;
    BoundConstraintData data;
};

struct LinearEntry {
    StageSelector stages;
    LinearConstraintData data;
};

/// 软约束惩罚配置
struct SoftPenalty {
    bool is_soft = false;
    double Z = 1e4; // L2 惩罚权重 (二次): 偏离越远惩罚越大，强力推回可行域
    double z = 1e3; // L1 惩罚权重 (线性): 保证稀疏性，在可行域内时精确为0
};
struct NonlinearEntry {
    StageSelector stages;
    std::shared_ptr<NonlinearConstraintBase> constraint;
    SoftPenalty penalty; // 惩罚配置
};

// ============================================================
// OcpConstraints — 所有约束的聚合容器
// ============================================================

/**
 * @brief OCP 约束容器
 *
 * 所有约束都通过 addXxx(stages, data) 的统一形式添加，
 * 由 MPCSolver 在 setup 时按阶段聚合。
 *
 * @code
 *   OcpConstraints constr;
 *
 *   // 初始阶段：锁定所有状态
 *   constr.addBound(StageSelector::initial(), BoundConstraintData{STATE, {0,1,2,3,4,5}, ...});
 *
 *   // 路径+终端：速度限制
 *   constr.addBound(StageSelector::pathAndTerminal(), BoundConstraintData{STATE, {3,4,5}, ...});
 *
 *   // 全路径：输入限制
 *   constr.addBound(StageSelector::path(), BoundConstraintData{INPUT, {0,1,2}, ...});
 *
 *   // 非线性舵轮约束 (阶段1~N)
 *   constr.addNonlinear(StageSelector::pathAndTerminal(), steer_constr);
 *
 *   // 避障约束只在阶段 5~15 生效
 *   constr.addNonlinear(StageSelector::range(5, 15), obstacle_constr);
 *
 *   // 终端线性约束
 *   constr.addLinear(StageSelector::terminal(), terminal_lc);
 * @endcode
 */
class OcpConstraints {
public:
    // --- 添加约束 ---
    size_t getNumBoundGroups() const {return bounds_.size();}
    size_t getNumLinearGroups() const {return linears_.size();}
    size_t getNumNonlinearGroups() const {return nonlinears_.size();}
    void addBound(StageSelector stages, BoundConstraintData data) {
        bounds_.push_back({std::move(stages), std::move(data)});
    }

    void addLinear(StageSelector stages, LinearConstraintData data) {
        linears_.push_back({std::move(stages), std::move(data)});
    }
    // 将约束对象以及生效的阶段填入 nonlinears_列表中
    void addNonlinear(StageSelector stages, std::shared_ptr<NonlinearConstraintBase> c, SoftPenalty penalty = SoftPenalty{}) {
        nonlinears_.push_back({std::move(stages), std::move(c), penalty});
    }

    // 提取某阶段的松弛变量维度、索引及惩罚矩阵
    void getNonlinearSoftInfoForStage(int k, int N, int& nsh, std::vector<int>& idxsh, 
                                      std::vector<double>& Zl, std::vector<double>& Zu, 
                                      std::vector<double>& zl, std::vector<double>& zu) const {
        nsh = 0;
        idxsh.clear(); Zl.clear(); Zu.clear(); zl.clear(); zu.clear();
        int current_h_idx = 0;
        
        for (const auto& e : nonlinears_) {
            if (e.stages.includes(k, N)) {
                int dim = e.constraint->getNumConstraints();
                if (e.penalty.is_soft) {
                    // 如果被标记为软约束，将其在 h 向量中的相对索引压入 idxsh
                    for (int i = 0; i < dim; i++) {
                        idxsh.push_back(current_h_idx + i);
                        Zl.push_back(e.penalty.Z);
                        Zu.push_back(e.penalty.Z);
                        zl.push_back(e.penalty.z);
                        zu.push_back(e.penalty.z);
                    }
                    nsh += dim;
                }
                current_h_idx += dim;
            }
        }
    }

    // --- 按阶段查询（供 MPCSolver 使用）---

    /// 获取阶段 k 的所有状态边界约束
    void getStateBoundsForStage(int k, int N,
                                std::vector<int>& idx, std::vector<double>& lb, std::vector<double>& ub) const {
        idx.clear(); lb.clear(); ub.clear();
        for (const auto& e : bounds_) {
            if (e.data.var_type == BoundConstraintData::STATE && e.stages.includes(k, N)) {
                idx.insert(idx.end(), e.data.idx.begin(), e.data.idx.end());
                lb.insert(lb.end(), e.data.lb.begin(), e.data.lb.end());
                ub.insert(ub.end(), e.data.ub.begin(), e.data.ub.end());
            }
        }
    }

    /// 获取阶段 k 的所有输入边界约束
    void getInputBoundsForStage(int k, int N,
                                std::vector<int>& idx, std::vector<double>& lb, std::vector<double>& ub) const {
        idx.clear(); lb.clear(); ub.clear();
        for (const auto& e : bounds_) {
            if (e.data.var_type == BoundConstraintData::INPUT && e.stages.includes(k, N)) {
                idx.insert(idx.end(), e.data.idx.begin(), e.data.idx.end());
                lb.insert(lb.end(), e.data.lb.begin(), e.data.lb.end());
                ub.insert(ub.end(), e.data.ub.begin(), e.data.ub.end());
            }
        }
    }

    /// 获取阶段 k 的线性约束聚合（多个 LinearConstraintData 合并）
    LinearConstraintData getLinearForStage(int k, int N, int nx, int nu) const {
        LinearConstraintData merged;
        merged.ng = 0;
        for (const auto& e : linears_) {
            if (!e.stages.includes(k, N)) continue;
            int ng0 = merged.ng;
            merged.ng += e.data.ng;
            // 合并 lg/ug
            merged.lg.insert(merged.lg.end(), e.data.lg.begin(), e.data.lg.end());
            merged.ug.insert(merged.ug.end(), e.data.ug.begin(), e.data.ug.end());
            // 合并 C (列主序: 需要逐列拼接)
            merged.C.resize(merged.ng * nx, 0.0);
            for (int col = 0; col < nx; col++) {
                for (int row = 0; row < e.data.ng; row++) {
                    merged.C[(ng0 + row) + col * merged.ng] = e.data.C[row + col * e.data.ng];
                }
            }
            // 合并 D
            merged.D.resize(merged.ng * nu, 0.0);
            for (int col = 0; col < nu; col++) {
                for (int row = 0; row < e.data.ng; row++) {
                    merged.D[(ng0 + row) + col * merged.ng] = e.data.D[row + col * e.data.ng];
                }
            }
        }
        return merged;
    }

    /// 获取在阶段 k 生效的所有非线性约束
    std::vector<std::shared_ptr<NonlinearConstraintBase>>
    getNonlinearForStage(int k, int N) const {
        std::vector<std::shared_ptr<NonlinearConstraintBase>> result;
        for (const auto& e : nonlinears_) {
            if (e.stages.includes(k, N)) {
                result.push_back(e.constraint);
            }
        }
        return result;
    }

    /// 计算阶段 k 的非线性约束总维度
    int getNhForStage(int k, int N) const {
        int nh = 0;
        for (const auto& e : nonlinears_) {
            if (e.stages.includes(k, N)) nh += e.constraint->getNumConstraints();
        }
        return nh;
    }

    /// 获取全局非线性约束的并集（用于 ModelCompiler JIT 编译）
    /// 返回所有出现过的非线性约束（去重）
    std::vector<std::shared_ptr<NonlinearConstraintBase>> getAllNonlinearConstraints() const {
        // 保持插入顺序，用 set 去重指针
        std::vector<std::shared_ptr<NonlinearConstraintBase>> result;
        std::set<NonlinearConstraintBase*> seen;
        for (const auto& e : nonlinears_) {
            if (seen.insert(e.constraint.get()).second) {
                result.push_back(e.constraint);
            }
        }
        return result;
    }

    /// 是否存在任何非线性约束
    bool hasNonlinear() const { return !nonlinears_.empty(); }

    /// 检查所有阶段是否使用相同的非线性约束集合（简化当前实现的前提）
    bool isNonlinearUniformAcrossStages(int N) const {
        if (nonlinears_.empty()) return true;
        int nh0 = getNhForStage(1, N);
        for (int k = 2; k <= N; k++) {
            if (getNhForStage(k, N) != nh0) return false;
        }
        return true;
    }

private:
    std::vector<BoundEntry>     bounds_;
    std::vector<LinearEntry>    linears_;
    std::vector<NonlinearEntry> nonlinears_;
};