{{VERSION}}
{{VERTEX_PRECISION}}

layout(location=0) in vec2 position;
out vec2 vertex_uv;

void main() {
    vertex_uv = position * 0.5 + 0.5;
    gl_Position = vec4(position, 0.0, 1.0);
}
