#pragma once

#include "nativekit_scene.hpp"
#include <array>
#include <bit>
#include <unordered_map>

namespace nkscene::render_internal {

// GPU equivalence is independent of authored resource identity. Keep those IDs
// on the render items, so editing an occurrence never edits its neighbours.
class BatchMaterials {
    using Key = std::array<std::uint64_t, 19>;
    struct Hash {
        std::size_t operator()(const Key &key) const noexcept {
            std::size_t hash = 0;
            for (const auto field : key)
                hash ^= std::hash<std::uint64_t>{}(field) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
            return hash;
        }
    };
    static Key key(const MaterialState &s) {
        const auto bits = [](float value) { return std::bit_cast<std::uint32_t>(value); };
        // Encode fields, never padding. Bit equality is deliberately conservative,
        // and includes all texture and sampler IDs (not just surface colour).
        return {bits(s.base_color[0]), bits(s.base_color[1]), bits(s.base_color[2]), bits(s.base_color[3]),
                bits(s.opacity), s.flags, bits(s.metallic), bits(s.roughness),
                bits(s.emissive[0]), bits(s.emissive[1]), bits(s.emissive[2]), bits(s.alpha_cutoff),
                static_cast<std::uint64_t>(s.alpha_mode), s.base_color_texture.value,
                s.metallic_roughness_texture.value, s.normal_texture.value,
                s.emissive_texture.value, s.occlusion_texture.value, s.sampler.value};
    }
    std::unordered_map<MaterialId, MaterialId> aliases_;
    std::uint64_t revision_ = ~std::uint64_t{0};
public:
    // Revisit equivalence only on resource edits, including direct native callers
    // executing a retained plan against updated material resources.
    bool refresh(const SceneSnapshot &snapshot, bool force = false) {
        if (!force && revision_ == snapshot.material_resources_revision()) return false;
        std::unordered_map<Key, MaterialId, Hash> representatives;
        representatives.reserve(snapshot.materials().size());
        for (const auto &material : snapshot.materials()) {
            auto [found, inserted] = representatives.try_emplace(key(*material.state), material.id);
            if (!inserted && material.id.value < found->second.value) found->second = material.id;
        }
        std::unordered_map<MaterialId, MaterialId> aliases;
        aliases.reserve(snapshot.materials().size());
        for (const auto &material : snapshot.materials())
            aliases.emplace(material.id, representatives.at(key(*material.state)));
        revision_ = snapshot.material_resources_revision();
        // Adding an unused material doesn't disturb existing GPU groups. A
        // source-node assignment to it is handled by the ordinary sparse delta.
        bool regroup = false;
        for (const auto &[source, representative] : aliases_) {
            const auto found = aliases.find(source);
            if (found == aliases.end() || found->second != representative) {
                regroup = true;
                break;
            }
        }
        aliases_ = std::move(aliases);
        return regroup;
    }
    MaterialId canonical(MaterialId material) const {
        const auto found = aliases_.find(material);
        return found == aliases_.end() ? material : found->second;
    }
};
} // namespace nkscene::render_internal
