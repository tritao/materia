{{VERSION}}
{{VERTEX_PRECISION}}
layout(location=0) in vec2 position;
out vec2 grid_ndc;
void main() {
    grid_ndc = position;
    gl_Position = vec4(position, 0.0, 1.0);
}
