varying vec4 PointWorld; // position in world coordinate

// https://stackoverflow.com/questions/15095909/from-rgb-to-hsv-in-opengl-glsl
// All components are in the range [0…1], including hue.
vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

vec3 hsv0 = vec3(0, 1, 1);
vec3 hsv1 = vec3(0.9, 1, 1);

void main() {
  float depth = sqrt(PointWorld.x*PointWorld.x + PointWorld.y*PointWorld.y);
  if(depth > 20.0) depth = 20.0;
  if(depth < 0.5) depth = 0.5;
  gl_FragColor = vec4(hsv2rgb(mix(hsv0, hsv1, (depth-0.5)/18.0)), 1.0);
}