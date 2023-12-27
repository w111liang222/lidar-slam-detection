varying vec4 PointWorld;

uniform float pointSize;
#include <clipping_planes_pars_vertex>
void main() {
  #include <begin_vertex>
  PointWorld = modelMatrix * vec4(position.x, position.y, position.z, 1.0);
  gl_Position = projectionMatrix * viewMatrix * PointWorld;

  gl_PointSize = pointSize;
  #include <project_vertex>
  #include <clipping_planes_vertex>
}