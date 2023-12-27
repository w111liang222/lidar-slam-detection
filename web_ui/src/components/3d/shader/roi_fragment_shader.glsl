uniform vec3 color1;
uniform vec3 color2;

varying vec2 vUv;

void main() {
  
  float h = vUv[1];
  gl_FragColor = vec4(mix(color1, color2, vUv.y), 1.0-h);
  // if(h > 0.5)
  //     gl_FragColor = vec4(mix(color1, color2, vUv.y), 1.0-(h-0.5)*2.0);
  // else
  //     gl_FragColor = vec4(mix(color1, color2, vUv.y), h*2.0);
}