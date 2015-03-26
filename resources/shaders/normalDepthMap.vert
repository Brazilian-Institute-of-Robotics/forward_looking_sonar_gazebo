#version 130

out vec3 pos;
out vec3 normal;
out mat3 TBN;

void main() {
    pos = (gl_ModelViewMatrix * gl_Vertex).xyz;
    normal = gl_NormalMatrix * gl_Normal;

    // Normal maps are built in tangent space, interpolating the vertex normal and a RGB texture.
    // TBN is the conversion matrix between Tangent Space -> World Space.
    vec3 n = normalize(normal);
    vec3 t = cross(n, vec3(-1,0,0));
    vec3 b = cross(n, t) + cross(t, n);
    TBN = mat3(t, b, n);

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0;
}
