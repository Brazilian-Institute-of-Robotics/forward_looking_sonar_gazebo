#version 130

in vec3 pos;
in vec3 normal;
in mat3 TBN;

uniform float farPlane;
uniform int drawNormal;
uniform int drawDepth;
uniform sampler2D normalTexture;
uniform float reflectance;
uniform float attenuationCoeff;

out vec4 out_data;

void main() {
    out_data = vec4(0, 0, 0, 0);

    vec3 normNormal;

    // // Normal for textured scenes (by normal mapping)
    if (textureSize(normalTexture, 0).x > 1) {
      vec3 normalRGB = texture2D(normalTexture, gl_TexCoord[0].xy).rgb;
      vec3 normalMap = (normalRGB * 2.0 - 1.0) * TBN;
      normNormal = normalize(normalMap);
    }

    // // Normal for untextured scenes
    // else
        normNormal = normalize(normal);

    // Material's reflectivity property
    if (reflectance > 0)
        normNormal = min(normNormal * reflectance, 1.0);

    vec3 normPosition = normalize(-pos);

    float linearDepth = sqrt(pos.z * pos.z + pos.x * pos.x + pos.y * pos.y);

    // Attenuation effect of sound in the water
    normNormal = normNormal * exp(-2 * attenuationCoeff * linearDepth);

    linearDepth = linearDepth / farPlane;

    if (!(linearDepth > 1)) {
        if (drawNormal==1){
            float value = dot(normPosition, normNormal);
            out_data.zw = vec2( abs(value), 1.0);
            //out_data.zw = vec2( 1.0, 1.0);
        }
        if (drawDepth==1)
            out_data.yw = vec2(linearDepth, 1.0);
    }
    if(linearDepth > 1.0 )
    {
        out_data = vec4(0,0,0,0);
    }
    //out_data = vec4(255,0,0,0);
    //gl_FragDepth = linearDepth;
}
