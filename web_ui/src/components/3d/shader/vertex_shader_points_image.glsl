varying vec4 PointWorld;
uniform mat4 extraT;

uniform float pointSize;
#include <clipping_planes_pars_vertex>
void main() {
  #include <begin_vertex>
  PointWorld = extraT * vec4(position.x, position.y, position.z, 1.0);
  gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position.x, position.y, position.z, 1.0);

  gl_PointSize = pointSize;
  #include <project_vertex>
  #include <clipping_planes_vertex>
}