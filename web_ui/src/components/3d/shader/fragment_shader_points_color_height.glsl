varying vec4 PointWorld; // position in world coordinate
uniform float heightColor[2];
// https://stackoverflow.com/questions/15095909/from-rgb-to-hsv-in-opengl-glsl
// All components are in the range [0…1], including hue.
vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

#include <clipping_planes_pars_fragment>
void main() {
  #include <clipping_planes_fragment>
  float heigtRange = heightColor[0] - heightColor[1] + 0.1;
  vec3 hsv0 = vec3(heightColor[0] / heigtRange, 1, 1.0);
  vec3 hsv1 = vec3(heightColor[1] / heigtRange, 1, 1.0);
  float percent = PointWorld.z / heigtRange;
  gl_FragColor = vec4(hsv2rgb(mix(hsv0, hsv1, percent)), 1.0);
}