uniform sampler2D textureImage;
uniform float cx;
uniform float cy;
uniform float fx;
uniform float fy;
uniform float width;
uniform float height;

varying vec4 PointWorld; // position in camera coordinate
#include <clipping_planes_pars_fragment>
void main() {
  #include <clipping_planes_fragment>
  if(PointWorld.z > 0.1){
    float u = PointWorld.x/PointWorld.z * fx + cx;
    float v = PointWorld.y/PointWorld.z * fy + cy;
    if(u>width || u<0.0 || v>height || v<0.0){
      discard;
    }else{
      v = height - v;
      vec2 uv = vec2(
        u/ width,
        v/ height
      );
      gl_FragColor = texture(textureImage, uv );
    }
  }else{
    discard;
  }
}