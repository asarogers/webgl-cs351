import { Box } from "@mui/material";
import { useEffect, useRef, useState } from "react";
import * as THREE from 'three';

const Canvas = () => {
  const canvasRef = useRef(null);
  const [pageHeight, setPageHeight] = useState(window.innerHeight);

  useEffect(() => {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000);
    const camera = new THREE.PerspectiveCamera(125, window.innerWidth / window.innerHeight, 0.1, 2000);
    camera.position.z = 2;

    const renderer = new THREE.WebGLRenderer({
      canvas: canvasRef.current,
      antialias: true
    });

    const updateSize = () => {
      const totalHeight = document.body.scrollHeight; // Get full height of the page
      setPageHeight(totalHeight);
      renderer.setSize(window.innerWidth, totalHeight);
      camera.aspect = window.innerWidth / totalHeight;
      camera.updateProjectionMatrix();
    };

    updateSize();

    window.addEventListener('resize', updateSize);

    const animate = () => {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };

    animate();

    // Create gradient shader material
    const createGlowLayer = (radius, opacity, x, y) => {
      const geometry = new THREE.CircleGeometry(radius, 64);
      // Define shader materials
      const vertexShader = `
        varying vec2 vUv;
        void main() {
          vUv = uv;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
      `;
      const fragmentShader = `
        varying vec2 vUv;
        uniform float opacity;
        void main() {
          vec3 color1 = vec3(1.0, 0.67, 0.0); // Warmer yellow #ffaa00
          vec3 color2 = vec3(0.8, 0.5, 0.0); // Darker orange
          // Create gradient based on y position
          vec3 finalColor = mix(color1, color2, vUv.y);
          // Add radial falloff
          float dist = length(vUv - vec2(0.5));
          float alpha = opacity * (1.0 - smoothstep(0.0, 0.5, dist));
          gl_FragColor = vec4(finalColor, alpha);
        }
      `;
      const material = new THREE.ShaderMaterial({
        vertexShader,
        fragmentShader,
        transparent: true,
        uniforms: {
          opacity: { value: opacity }
        }
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.x = x;
      mesh.position.y = y;
      return mesh;
    };

    // Calculate opacity values
    const division = 4 / 5;
    const opacities = Array(8).fill(0).map((_, i) => {
      if (i === 0) return 0.025;
      return 0.15 * Math.pow(division, i);
    });

    const layers = [
      { radius: 0.3, opacity: opacities[0] },
      { radius: 0.7, opacity: opacities[1] },
      { radius: 0.5, opacity: opacities[2] },
      { radius: 0.6, opacity: opacities[3] },
      { radius: 0.7, opacity: opacities[4] },
      { radius: 0.8, opacity: opacities[5] },
      { radius: 0.9, opacity: opacities[6] },
      { radius: 1.0, opacity: opacities[7] }
    ];

    // Create two arrays to hold all the meshes
    const allLayers = [];

    // Create and add all layers to the scene
    layers.forEach(({ radius, opacity }) => {
      const layer1 = createGlowLayer(radius, opacity, -1, 2.3);
      const layer2 = createGlowLayer(radius, opacity, 1, 1.7);
      const layer3 = createGlowLayer(radius, opacity, 0, 1);

      scene.add(layer1);
      scene.add(layer2);
      scene.add(layer3);

      // Store both layers for cleanup
      allLayers.push(layer1, layer2, layer3);
      // allLayers.push(layer1, layer2);
    });


    return () => {
      window.removeEventListener('resize', updateSize);
      renderer.dispose();
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      style={{
        position: 'absolute',  // Keep it absolute so elements scroll past it
        top: 0,
        left: 0,
        width: '100vw',
        height: `${pageHeight}px`,  // Make it match total page height
        display: 'block',
        zIndex: 2
      }}
    />
  );
};

export default Canvas;