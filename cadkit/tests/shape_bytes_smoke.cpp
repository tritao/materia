// cad_shape_write_bytes / cad_shape_read_bytes: round trips, and bytes that are damaged in ways Haxe cannot forge
// (a valid checksum around a wrong version, a mismatched name section).
#include "cadkit.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes write_shape(cad_shape shape) {
    std::uint32_t size = 0;
    assert(cad_shape_write_bytes(shape, nullptr, &size) == CAD_ERROR_BUFFER_TOO_SMALL);
    assert(size > 32);
    Bytes bytes(size);
    std::uint32_t capacity = size;
    assert(cad_shape_write_bytes(shape, bytes.data(), &capacity) == CAD_OK);
    assert(capacity == size);
    return bytes;
}

std::string element_names(cad_shape shape, cad_shape_kind kind) {
    std::uint32_t size = 0;
    {
        const auto query = cad_shape_copy_element_names_bytes(shape, kind, nullptr, &size);
        assert(query == CAD_OK || query == CAD_ERROR_BUFFER_TOO_SMALL);  // the query answers with the size either way
    }
    std::string text(size, '\0');
    std::uint32_t capacity = size;
    if (size != 0) assert(cad_shape_copy_element_names_bytes(shape, kind, reinterpret_cast<std::uint8_t*>(&text[0]), &capacity) == CAD_OK);
    return text;
}

std::uint64_t checksum(const std::uint8_t* data, std::size_t size) {
    std::uint64_t hash = 1469598103934665603ull;
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= data[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

// The same bytes with a checksum that matches them again, so only the content is wrong.
void reseal(Bytes& bytes) {
    const auto sum = checksum(bytes.data(), bytes.size() - 8);
    for (int i = 0; i < 8; ++i) bytes[bytes.size() - 8 + i] = static_cast<std::uint8_t>((sum >> (8 * i)) & 0xffu);
}

std::uint32_t u32_at(const Bytes& bytes, std::size_t at) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(bytes[at + i]) << (8 * i);
    return value;
}

void put_u32_at(Bytes& bytes, std::size_t at, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) bytes[at + i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu);
}

bool refused(const Bytes& bytes, const char* expected_fragment = nullptr) {
    cad_shape shape = 0xdeadbeef;
    const auto result = cad_shape_read_bytes(bytes.empty() ? nullptr : bytes.data(), static_cast<std::uint32_t>(bytes.size()), &shape);
    if (result == CAD_OK) {
        cad_shape_destroy(shape);
        return false;
    }
    assert(shape == 0);
    assert(result == CAD_ERROR_INVALID_ARGUMENT || result == CAD_ERROR_OPERATION_FAILED);
    if (expected_fragment != nullptr) {
        const std::string message = cad_last_error();
        if (message.find(expected_fragment) == std::string::npos) {
            std::fprintf(stderr, "expected \"%s\" in \"%s\"\n", expected_fragment, message.c_str());
            return false;
        }
    }
    return true;
}

// Layout: magic(4) format(4) scheme(4) text-length(4) text names-size(4) brep-size(4) names brep checksum(8).
struct Sections {
    std::size_t text_length_at, names_size_at, names_at, brep_at;
    std::uint32_t names_size, brep_size;
};

Sections sections(const Bytes& bytes) {
    Sections s{};
    s.text_length_at = 12;
    const auto text_length = u32_at(bytes, s.text_length_at);
    s.names_size_at = 16 + text_length;
    s.names_size = u32_at(bytes, s.names_size_at);
    s.brep_size = u32_at(bytes, s.names_size_at + 4);
    s.names_at = s.names_size_at + 8;
    s.brep_at = s.names_at + s.names_size;
    assert(s.brep_at + s.brep_size + 8 == bytes.size());
    return s;
}

void round_trips() {
    cad_resource_counts before{};
    assert(cad_resource_counts_get(&before) == CAD_OK);

    cad_shape box = 0, cylinder = 0, hole = 0, drilled = 0, fused = 0;
    assert(cad_box(10, 20, 30, &box) == CAD_OK);
    assert(cad_cylinder(3, 40, &cylinder) == CAD_OK);
    assert(cad_shape_translate(cylinder, {4, 5, -5}, &hole) == CAD_OK);
    assert(cad_cut(box, hole, &drilled) == CAD_OK);
    assert(cad_fuse(box, hole, &fused) == CAD_OK);

    for (cad_shape original : {box, cylinder, hole, drilled, fused}) {
        const auto bytes = write_shape(original);
        assert(write_shape(original) == bytes);
        cad_shape restored = 0;
        assert(cad_shape_read_bytes(bytes.data(), static_cast<std::uint32_t>(bytes.size()), &restored) == CAD_OK);
        assert(restored != 0);
        for (cad_shape_kind kind : {CAD_SHAPE_FACE, CAD_SHAPE_EDGE, CAD_SHAPE_VERTEX, CAD_SHAPE_SOLID}) {
            assert(element_names(original, kind) == element_names(restored, kind));
        }
        double a = 0, b = 0;
        assert(cad_shape_volume(original, &a) == CAD_OK && cad_shape_volume(restored, &b) == CAD_OK);
        assert(a == b);
        cad_shape_destroy(restored);
    }

    // The capacity query and a short buffer.
    const auto bytes = write_shape(box);
    std::uint32_t small = 10;
    Bytes buffer(10);
    assert(cad_shape_write_bytes(box, buffer.data(), &small) == CAD_ERROR_BUFFER_TOO_SMALL);
    assert(small == bytes.size());
    assert(cad_shape_write_bytes(box, nullptr, nullptr) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_write_bytes(0, buffer.data(), &small) == CAD_ERROR_INVALID_HANDLE);
    cad_shape nothing = 5;
    assert(cad_shape_read_bytes(bytes.data(), static_cast<std::uint32_t>(bytes.size()), nullptr) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_read_bytes(nullptr, 0, &nothing) == CAD_ERROR_INVALID_ARGUMENT && nothing == 0);

    for (cad_shape shape : {box, cylinder, hole, drilled, fused}) cad_shape_destroy(shape);
    cad_resource_counts after{};
    assert(cad_resource_counts_get(&after) == CAD_OK);
    assert(after.shape_count == before.shape_count);
}

void damage() {
    cad_resource_counts before{};
    assert(cad_resource_counts_get(&before) == CAD_OK);
    cad_shape box = 0;
    assert(cad_box(10, 20, 30, &box) == CAD_OK);
    const auto bytes = write_shape(box);
    const auto layout = sections(bytes);

    assert(refused({}));
    assert(refused(Bytes(100, 0)));
    assert(refused(Bytes{'C', 'K', 'S', 'H'}));
    // Every truncation.
    for (std::size_t length = 0; length < bytes.size(); ++length) {
        assert(refused(Bytes(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(length))));
    }
    // Every flipped bit pattern in every byte (the checksum catches them all).
    for (std::size_t at = 0; at < bytes.size(); ++at) {
        auto damaged = bytes;
        damaged[at] ^= 0x01;
        assert(refused(damaged));
    }
    // Content that is wrong under a checksum that is right: the file is refused for what it says, before any B-rep is read.
    {
        auto wrong = bytes;
        put_u32_at(wrong, 4, 2);
        reseal(wrong);
        assert(refused(wrong, "format version"));
    }
    {
        auto wrong = bytes;
        put_u32_at(wrong, 8, u32_at(bytes, 8) + 1);
        reseal(wrong);
        assert(refused(wrong, "naming scheme"));
    }
    {
        auto wrong = bytes;
        wrong[16] = static_cast<std::uint8_t>(wrong[16] ^ 0x01);  // first character of the OCCT version text
        reseal(wrong);
        assert(refused(wrong, "OCCT version"));
    }
    {
        auto wrong = bytes;
        put_u32_at(wrong, layout.names_size_at, layout.names_size + 1);  // sections no longer add up
        reseal(wrong);
        assert(refused(wrong, "section sizes"));
    }
    {
        auto wrong = bytes;
        put_u32_at(wrong, layout.names_at, 1000000);  // the face count claims more names than there are bytes
        reseal(wrong);
        assert(refused(wrong, "name section"));
    }
    {
        auto wrong = bytes;
        wrong.insert(wrong.end() - 8, 0);  // a stray byte after the B-rep
        reseal(wrong);
        assert(refused(wrong));
    }
    // Names of one shape around the B-rep of another.
    {
        cad_shape sphere = 0;
        assert(cad_sphere(5, &sphere) == CAD_OK);
        const auto other = write_shape(sphere);
        const auto other_layout = sections(other);
        Bytes mixed(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(layout.brep_at));
        mixed.insert(mixed.end(), other.begin() + static_cast<std::ptrdiff_t>(other_layout.brep_at),
                     other.begin() + static_cast<std::ptrdiff_t>(other_layout.brep_at + other_layout.brep_size));
        mixed.resize(mixed.size() + 8);
        put_u32_at(mixed, layout.names_size_at + 4, other_layout.brep_size);
        reseal(mixed);
        assert(refused(mixed, "names do not match"));
        cad_shape_destroy(sphere);
    }
    // After all that the intact bytes still read.
    cad_shape restored = 0;
    assert(cad_shape_read_bytes(bytes.data(), static_cast<std::uint32_t>(bytes.size()), &restored) == CAD_OK);
    cad_shape_destroy(restored);
    cad_shape_destroy(box);
    cad_resource_counts after{};
    assert(cad_resource_counts_get(&after) == CAD_OK);
    assert(after.shape_count == before.shape_count);
}

void meshes_agree() {
    cad_shape cylinder = 0;
    assert(cad_cylinder(7, 25, &cylinder) == CAD_OK);
    const auto bytes = write_shape(cylinder);
    cad_shape restored = 0;
    assert(cad_shape_read_bytes(bytes.data(), static_cast<std::uint32_t>(bytes.size()), &restored) == CAD_OK);
    cad_mesh_options options{0.05, 0.3};
    cad_mesh a = 0, b = 0;
    assert(cad_shape_tessellate(cylinder, &options, &a) == CAD_OK && cad_shape_tessellate(restored, &options, &b) == CAD_OK);
    std::uint32_t na = 0, nb = 0;
    assert(cad_mesh_vertex_count(a, &na) == CAD_OK && cad_mesh_vertex_count(b, &nb) == CAD_OK && na == nb && na > 0);
    std::vector<cad_vec3> va(na), vb(nb), normals_a(na), normals_b(nb);
    assert(cad_mesh_copy_vertices(a, va.data(), na) == CAD_OK && cad_mesh_copy_vertices(b, vb.data(), nb) == CAD_OK);
    assert(cad_mesh_copy_normals(a, normals_a.data(), na) == CAD_OK && cad_mesh_copy_normals(b, normals_b.data(), nb) == CAD_OK);
    // Bit for bit: signed zeros included (they are written as plain zeros).
    assert(std::memcmp(va.data(), vb.data(), na * sizeof(cad_vec3)) == 0);
    assert(std::memcmp(normals_a.data(), normals_b.data(), na * sizeof(cad_vec3)) == 0);
    for (const auto& n : normals_a) assert(!std::signbit(n.x) || n.x != 0.0);
    cad_mesh_destroy(a);
    cad_mesh_destroy(b);
    cad_shape_destroy(cylinder);
    cad_shape_destroy(restored);
}

}  // namespace

int main() {
    round_trips();
    damage();
    meshes_agree();
    std::puts("shape bytes smoke passed");
    return 0;
}
