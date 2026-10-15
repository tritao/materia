#include "merge_cases.hpp"
int main() {
    merge_cases([](const std::vector<mk_segment> &segments, const motionkit::MergeOptions &options) {
        motionkit::Trajectory trajectory(1);
        for (const auto &s : segments) assert(trajectory.append(s));
        motionkit::PolynomialSource source{&trajectory, trajectory.segment_count(), 1,
            [](const void *context, uint32_t i) {
                const auto &t = *static_cast<const motionkit::Trajectory *>(context);
                return motionkit::PolynomialSegmentView{t.segment_start(i), t.segment_duration(i), t.segment_degree(i),
                    t.joint_count(), t.segment_coefficients(i), sizeof(mk_joint_coefficients)};
            }};
        std::vector<mk_segment> out;
        assert(motionkit::merge_polynomials(source, options, {&out,
            [](void *context, const mk_segment &s, uint32_t, uint32_t) -> mk_result {
                static_cast<std::vector<mk_segment> *>(context)->push_back(s); return MK_OK;
            }}) == MK_OK);
        return out;
    });
}
